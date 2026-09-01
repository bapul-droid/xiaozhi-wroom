#include "bt_discovery.h"

#include <cstring>

#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace {

constexpr const char* TAG = "WROOM_BT";
constexpr TickType_t kProtocolPollDelay = pdMS_TO_TICKS(10);
constexpr TickType_t kTlsPollDelay = pdMS_TO_TICKS(5);
constexpr TickType_t kTlsWindowTimeout = pdMS_TO_TICKS(6000);

// V5 does not mistake protocol object creation for TLS completion. First observe
// the TLS pressure dip, then wait for the internal heap to recover. The BT task
// runs at priority 4 while the activation task runs at priority 2, so once the
// recovery is observed it can take the table before AudioService::Start().
constexpr size_t kTlsPressureFreeInternal = 24 * 1024;
constexpr size_t kTlsPressureLargestInternal = 20 * 1024;
constexpr size_t kPostTlsFreeInternal = 30 * 1024;
constexpr size_t kPostTlsLargestInternal = 24 * 1024;

char* BdaToString(const esp_bd_addr_t bda, char* out, size_t size) {
    if (bda == nullptr || out == nullptr || size < 18) {
        return nullptr;
    }
    snprintf(out, size, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return out;
}

void LogHeap(const char* phase) {
    const size_t free_internal =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t min_internal =
        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    const size_t largest_dma =
        heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    const size_t free_spiram =
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t largest_spiram =
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    ESP_LOGW(TAG,
             "HEAP %s: internal=%u largest=%u min=%u | dma=%u largest=%u | spiram=%u largest=%u",
             phase,
             static_cast<unsigned>(free_internal),
             static_cast<unsigned>(largest_internal),
             static_cast<unsigned>(min_internal),
             static_cast<unsigned>(free_dma),
             static_cast<unsigned>(largest_dma),
             static_cast<unsigned>(free_spiram),
             static_cast<unsigned>(largest_spiram));
}

bool TlsPressureObserved() {
    const size_t free_internal =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return free_internal <= kTlsPressureFreeInternal ||
           largest_internal <= kTlsPressureLargestInternal;
}

bool PostTlsHeapRecovered() {
    const size_t free_internal =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return free_internal >= kPostTlsFreeInternal &&
           largest_internal >= kPostTlsLargestInternal;
}

bool WaitForPostTlsPreAudioWindow() {
    auto& app = Application::GetInstance();

    ESP_LOGI(TAG, "Bluetooth Discovery V5 waiting for protocol/TLS pressure cycle");
    LogHeap("v5_start");

    while (!app.IsProtocolReady()) {
        if (app.GetDeviceState() == kDeviceStateIdle) {
            ESP_LOGW(TAG, "V5 pre-audio window missed: application already idle");
            return false;
        }
        vTaskDelay(kProtocolPollDelay);
    }

    ESP_LOGI(TAG, "Protocol object ready; waiting for TLS pressure dip, not assuming TLS is done");
    LogHeap("protocol_ready_tls_pending");

    bool saw_tls_pressure = false;
    const TickType_t deadline = xTaskGetTickCount() + kTlsWindowTimeout;

    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        if (app.GetDeviceState() == kDeviceStateIdle) {
            ESP_LOGW(TAG, "V5 pre-audio window missed: activation already reached idle");
            LogHeap("pre_audio_window_missed");
            return false;
        }

        if (!saw_tls_pressure && TlsPressureObserved()) {
            saw_tls_pressure = true;
            ESP_LOGI(TAG, "TLS pressure dip observed; waiting for post-handshake heap recovery");
            LogHeap("tls_pressure_seen");
        }

        if (saw_tls_pressure && PostTlsHeapRecovered()) {
            ESP_LOGI(TAG, "Post-TLS heap recovery detected; Classic BT takes table before audio");
            LogHeap("post_tls_pre_audio_window");
            return true;
        }

        vTaskDelay(kTlsPollDelay);
    }

    ESP_LOGW(TAG, "V5 timed out waiting for TLS pressure/recovery cycle; BT skipped safely");
    LogHeap("pre_audio_window_timeout");
    return false;
}

void LogDiscoveryResult(esp_bt_gap_cb_param_t* param) {
    char address[18] = {};
    int32_t rssi = -129;
    uint32_t cod = 0;
    char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {};
    uint8_t* eir = nullptr;

    for (int i = 0; i < param->disc_res.num_prop; ++i) {
        auto* prop = param->disc_res.prop + i;
        switch (prop->type) {
            case ESP_BT_GAP_DEV_PROP_BDNAME: {
                size_t len = prop->len;
                if (len > ESP_BT_GAP_MAX_BDNAME_LEN) {
                    len = ESP_BT_GAP_MAX_BDNAME_LEN;
                }
                memcpy(name, prop->val, len);
                name[len] = '\0';
                break;
            }
            case ESP_BT_GAP_DEV_PROP_COD:
                cod = *static_cast<uint32_t*>(prop->val);
                break;
            case ESP_BT_GAP_DEV_PROP_RSSI:
                rssi = *static_cast<int8_t*>(prop->val);
                break;
            case ESP_BT_GAP_DEV_PROP_EIR:
                eir = static_cast<uint8_t*>(prop->val);
                break;
            default:
                break;
        }
    }

    if (name[0] == '\0' && eir != nullptr) {
        uint8_t len = 0;
        uint8_t* remote_name = esp_bt_gap_resolve_eir_data(
            eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
        if (remote_name == nullptr) {
            remote_name = esp_bt_gap_resolve_eir_data(
                eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
        }
        if (remote_name != nullptr) {
            if (len > ESP_BT_GAP_MAX_BDNAME_LEN) {
                len = ESP_BT_GAP_MAX_BDNAME_LEN;
            }
            memcpy(name, remote_name, len);
            name[len] = '\0';
        }
    }

    ESP_LOGI(TAG, "FOUND addr=%s rssi=%ld cod=0x%06lx name=%s",
             BdaToString(param->disc_res.bda, address, sizeof(address)),
             static_cast<long>(rssi), static_cast<unsigned long>(cod),
             name[0] ? name : "(unknown)");
}

void GapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            LogDiscoveryResult(param);
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                ESP_LOGI(TAG, "Discovery started - put Edifier in pairing mode now");
            } else {
                ESP_LOGI(TAG, "Discovery finished");
            }
            break;
        default:
            break;
    }
}

void DiscoveryTask(void*) {
    if (!WaitForPostTlsPreAudioWindow()) {
        vTaskDelete(nullptr);
        return;
    }

    LogHeap("before_ble_release");
    esp_err_t ble_release_err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ble_release_err == ESP_OK) {
        ESP_LOGI(TAG, "BLE controller memory released for Classic BT");
    } else {
        ESP_LOGW(TAG, "BLE controller memory release returned: %s",
                 esp_err_to_name(ble_release_err));
    }
    LogHeap("after_ble_release");

    ESP_LOGI(TAG, "Initializing Classic Bluetooth Discovery V5 BEFORE audio task startup");
    LogHeap("before_bt_controller_init");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        LogHeap("controller_init_failed");
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("controller_init_ok");

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        LogHeap("controller_enable_failed");
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("controller_enabled");

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
        LogHeap("bluedroid_init_failed");
        vTaskDelete(nullptr);
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
        LogHeap("bluedroid_enable_failed");
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("bluedroid_ready_pre_audio");

    err = esp_bt_gap_register_callback(GapCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    esp_bt_gap_set_device_name("XiaoZhi-WROOM");

    err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_start_discovery failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Classic BT has the table; audio may start after activation resumes");
    }

    vTaskDelete(nullptr);
}

}  // namespace

void StartBtDiscoveryV1() {
    BaseType_t ok = xTaskCreate(DiscoveryTask, "wroom_bt_scan", 4096, nullptr, 4, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Bluetooth discovery task");
    }
}
