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
constexpr TickType_t kTlsPollDelay = pdMS_TO_TICKS(2);
constexpr TickType_t kTlsWindowTimeout = pdMS_TO_TICKS(6000);

// V6: the first recovery after certificate verification is NOT enough. In V5
// that false recovery was ~34 KB free while MQTT still had CONNECT/TLS writes
// pending. Require a materially deeper recovery before letting Classic BT in.
constexpr size_t kTlsPressureFreeInternal = 24 * 1024;
constexpr size_t kTlsPressureLargestInternal = 20 * 1024;
constexpr size_t kDeepRecoveryFreeInternal = 40 * 1024;
constexpr size_t kDeepRecoveryLargestInternal = 28 * 1024;
constexpr int kRequiredRecoveryChecks = 3;

// Even after controller/Bluedroid init, do not launch inquiry if there is no
// useful allocation headroom left. This avoids repeating the V5 failure where
// Bluedroid left ~2 KB free / <1 KB largest and MQTT immediately collapsed.
constexpr size_t kMinFreeBeforeInquiry = 10 * 1024;
constexpr size_t kMinLargestBeforeInquiry = 6 * 1024;

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

bool DeepRecoveryObserved() {
    const size_t free_internal =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return free_internal >= kDeepRecoveryFreeInternal &&
           largest_internal >= kDeepRecoveryLargestInternal;
}

bool InquiryHeadroomSafe() {
    const size_t free_internal =
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return free_internal >= kMinFreeBeforeInquiry &&
           largest_internal >= kMinLargestBeforeInquiry;
}

bool WaitForDeepPostTlsRecovery() {
    auto& app = Application::GetInstance();

    ESP_LOGI(TAG, "Bluetooth Discovery V6 waiting for TLS pressure + deep recovery");
    LogHeap("v6_start");

    while (!app.IsProtocolReady()) {
        if (app.GetDeviceState() == kDeviceStateIdle) {
            ESP_LOGW(TAG, "V6 pre-audio window missed: application already idle");
            return false;
        }
        vTaskDelay(kProtocolPollDelay);
    }

    ESP_LOGI(TAG, "Protocol object ready; waiting for TLS pressure dip");
    LogHeap("protocol_ready_tls_pending");

    bool saw_tls_pressure = false;
    int recovery_checks = 0;
    const TickType_t deadline = xTaskGetTickCount() + kTlsWindowTimeout;

    while ((int32_t)(deadline - xTaskGetTickCount()) > 0) {
        if (app.GetDeviceState() == kDeviceStateIdle) {
            ESP_LOGW(TAG, "V6 window missed: activation/audio already reached idle");
            LogHeap("deep_recovery_missed");
            return false;
        }

        if (!saw_tls_pressure && TlsPressureObserved()) {
            saw_tls_pressure = true;
            ESP_LOGI(TAG, "TLS pressure dip observed; ignoring shallow certificate recovery");
            LogHeap("tls_pressure_seen");
        }

        if (saw_tls_pressure && DeepRecoveryObserved()) {
            ++recovery_checks;
            if (recovery_checks >= kRequiredRecoveryChecks) {
                ESP_LOGI(TAG,
                         "Deep post-TLS recovery stable (%d checks); BT may enter",
                         kRequiredRecoveryChecks);
                LogHeap("deep_recovery_open");
                return true;
            }
        } else {
            recovery_checks = 0;
        }

        vTaskDelay(kTlsPollDelay);
    }

    ESP_LOGW(TAG, "V6 timed out waiting for safe deep recovery; BT skipped");
    LogHeap("deep_recovery_timeout");
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

void RollBackBtForHeadroom() {
    ESP_LOGW(TAG, "BT headroom unsafe; rolling back controller to preserve MQTT");

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bluedroid_disable();
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
        esp_bluedroid_deinit();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_bt_controller_deinit();
    }

    LogHeap("after_bt_rollback");
}

void DiscoveryTask(void*) {
    if (!WaitForDeepPostTlsRecovery()) {
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

    ESP_LOGI(TAG, "Initializing Classic Bluetooth Discovery V6");
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
        RollBackBtForHeadroom();
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("controller_enabled");

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
        LogHeap("bluedroid_init_failed");
        RollBackBtForHeadroom();
        vTaskDelete(nullptr);
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
        LogHeap("bluedroid_enable_failed");
        RollBackBtForHeadroom();
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("bluedroid_ready");

    if (!InquiryHeadroomSafe()) {
        ESP_LOGW(TAG,
                 "Inquiry blocked: Bluedroid left insufficient MQTT-safe headroom");
        RollBackBtForHeadroom();
        vTaskDelete(nullptr);
        return;
    }

    err = esp_bt_gap_register_callback(GapCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %s", esp_err_to_name(err));
        RollBackBtForHeadroom();
        vTaskDelete(nullptr);
        return;
    }

    esp_bt_gap_set_device_name("XiaoZhi-WROOM");

    err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_start_discovery failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Classic BT inquiry started with MQTT-safe headroom gate passed");
        LogHeap("inquiry_started");
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
