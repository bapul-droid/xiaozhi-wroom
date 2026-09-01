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
constexpr TickType_t kStatePollDelay = pdMS_TO_TICKS(1000);
constexpr TickType_t kRetryDelay = pdMS_TO_TICKS(5000);
constexpr int kRequiredStableIdleChecks = 8;
constexpr int kMaxHeapChecks = 12;
// Discovery V2: activation/TLS gets first claim on internal RAM. Bluetooth
// discovery is allowed only after the application has remained idle for a
// continuous settling window, then it must still pass the heap gate.
constexpr size_t kMinFreeInternal = 90000;
constexpr size_t kMinLargestInternal = 50000;

char* BdaToString(const esp_bd_addr_t bda, char* out, size_t size) {
    if (bda == nullptr || out == nullptr || size < 18) {
        return nullptr;
    }
    snprintf(out, size, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return out;
}

void LogHeap(const char* phase) {
    const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t min_internal = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGW(TAG, "HEAP %s: free_internal=%u largest_internal=%u min_internal=%u",
             phase,
             static_cast<unsigned>(free_internal),
             static_cast<unsigned>(largest_internal),
             static_cast<unsigned>(min_internal));
}

bool HeapReadyForBt() {
    const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return free_internal >= kMinFreeInternal && largest_internal >= kMinLargestInternal;
}

void WaitForStableIdle() {
    auto& app = Application::GetInstance();
    int idle_checks = 0;
    DeviceState last_state = app.GetDeviceState();

    ESP_LOGI(TAG, "Bluetooth Discovery V2 waiting for activation/application idle");

    while (idle_checks < kRequiredStableIdleChecks) {
        const DeviceState state = app.GetDeviceState();
        if (state == kDeviceStateIdle) {
            ++idle_checks;
            if (idle_checks == 1) {
                ESP_LOGI(TAG, "Application idle; starting %ds settle window",
                         kRequiredStableIdleChecks);
            }
        } else {
            if (idle_checks > 0) {
                ESP_LOGW(TAG, "Idle settle interrupted by state=%d; restarting window",
                         static_cast<int>(state));
            } else if (state != last_state) {
                ESP_LOGI(TAG, "BT discovery deferred: application state=%d",
                         static_cast<int>(state));
            }
            idle_checks = 0;
        }

        last_state = state;
        if (idle_checks < kRequiredStableIdleChecks) {
            vTaskDelay(kStatePollDelay);
        }
    }

    ESP_LOGI(TAG, "Application idle stable for %ds; activation gate passed",
             kRequiredStableIdleChecks);
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
    WaitForStableIdle();

    for (int attempt = 1; attempt <= kMaxHeapChecks; ++attempt) {
        LogHeap("before_bt");
        if (HeapReadyForBt()) {
            ESP_LOGI(TAG, "Heap gate passed on check %d/%d", attempt, kMaxHeapChecks);
            break;
        }
        if (attempt == kMaxHeapChecks) {
            ESP_LOGW(TAG, "BT discovery skipped: internal heap never reached safe diagnostic gate");
            vTaskDelete(nullptr);
            return;
        }
        ESP_LOGW(TAG, "Heap gate not ready (%d/%d); retry in 5s", attempt, kMaxHeapChecks);
        vTaskDelay(kRetryDelay);
    }

    ESP_LOGI(TAG, "Initializing Classic Bluetooth Discovery V2");
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
        vTaskDelete(nullptr);
        return;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    LogHeap("bluedroid_ready");

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
