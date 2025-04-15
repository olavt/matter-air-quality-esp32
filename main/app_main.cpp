/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif
using namespace chip::DeviceLayer;
#endif

#include "MatterAirQuality.h"
#include "SensirionSEN66.h"

MatterAirQuality* matterAirQuality;
static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    //ESP_LOGI(TAG, "app_attribute_update_cb: type=%d endpoint_id=%d cluster_id=%lu, attribute_id=%lu", type, endpoint_id, cluster_id, attribute_id);

    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

static uint8_t GetMatterBootReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:
            return 0x00; // PowerOnReset
        case ESP_RST_SW:
            return 0x03; // SoftwareReset
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return 0x04; // WatchdogReset
        case ESP_RST_BROWNOUT:
            return 0x02; // BrownoutReset
        case ESP_RST_PANIC:
            return 0x05; // CrashReset
        case ESP_RST_EXT:
        default:
            return 0xFF; // Unknown (Matter allows vendor-specific values >= 0x80)
    }
}

void AddSoftwareDiagnosticsCluster(node_t* node)
{
    endpoint_t* root_endpoint = endpoint::get(node, 0);

    cluster::software_diagnostics::config_t config;
    uint32_t features = cluster::software_diagnostics::feature::watermarks::get_id();
    cluster_t* cluster = cluster::software_diagnostics::create(root_endpoint, &config, CLUSTER_FLAG_SERVER, features);

    // TBD: create functions does not exist for these attributes in the esp-matter SDK yet.
    // Check if implemented in later versions.
    attribute::create(cluster, SoftwareDiagnostics::Attributes::CurrentHeapFree::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY,
        esp_matter_uint64(0));
    attribute::create(cluster, SoftwareDiagnostics::Attributes::CurrentHeapUsed::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY,
        esp_matter_uint64(0));
    attribute::create(cluster, SoftwareDiagnostics::Attributes::ThreadMetrics::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY,
        esp_matter_array(nullptr, 0, 0));
}

void AddThreadNetworkDiagnosticsCluster(node_t* node)
{
    endpoint_t* root_endpoint = endpoint::get(node, 0);

    cluster::thread_network_diagnostics::config_t thread_network_config;
    cluster_t* cluster = cluster::thread_network_diagnostics::create(root_endpoint, &thread_network_config, CLUSTER_FLAG_SERVER);

    cluster::thread_network_diagnostics::feature::error_counts::add(cluster);

    // PacketCounts
    cluster::thread_network_diagnostics::feature::packets_counts::add(cluster);

    // MLECounts
    cluster::thread_network_diagnostics::feature::mle_counts::add(cluster);
    attribute::create(cluster, ThreadNetworkDiagnostics::Attributes::ChildRoleCount::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY, esp_matter_uint16(0));
    attribute::create(cluster, ThreadNetworkDiagnostics::Attributes::LeaderRoleCount::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY, esp_matter_uint16(0));
    attribute::create(cluster, ThreadNetworkDiagnostics::Attributes::RouterRoleCount::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY, esp_matter_uint16(0));

    // MACCounts
    cluster::thread_network_diagnostics::feature::mac_counts::add(cluster);
    attribute::create(cluster, ThreadNetworkDiagnostics::Attributes::RxTotalCount::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY, esp_matter_uint32(0));
    attribute::create(cluster, ThreadNetworkDiagnostics::Attributes::TxTotalCount::Id, ATTRIBUTE_FLAG_MANAGED_INTERNALLY, esp_matter_uint32(0));

    //attribute::thread_network_diagnostics::create_detached_role_count(cluster, 0);
    //esp_matter::attribute::thread_network_diagnostics::create_detached_role_count(cluster, 0);
}

void AddColorControlClusterFeatures(endpoint_t* endpoint)
{
    cluster_t *cluster = cluster::get(endpoint, ColorControl::Id);

    cluster::color_control::feature::hue_saturation::config_t config;
    cluster::color_control::feature::hue_saturation::add(cluster, &config);
}

void ConfigureGeneralDiagnosticsCluster(node_t* node)
{
    esp_matter::endpoint_t *rootEndpoint = esp_matter::endpoint::get(node, 0);
    cluster_t *cluster = cluster::get(rootEndpoint, GeneralDiagnostics::Id);

    uint8_t bootReason = GetMatterBootReason();
    cluster::general_diagnostics::attribute::create_boot_reason(cluster, bootReason);
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize driver */
    app_driver_handle_t light_handle = app_driver_light_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    ConfigureGeneralDiagnosticsCluster(node);

    extended_color_light::config_t light_config;
    light_config.on_off.on_off = DEFAULT_POWER;
    light_config.on_off.lighting.start_up_on_off = nullptr;
    light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.on_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;

    // endpoint handles can be used to add/modify clusters.
    endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create extended color light endpoint"));

    AddColorControlClusterFeatures(endpoint);
    AddSoftwareDiagnosticsCluster(node);

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

    AirQualitySensor* airQualitySensor = new SensirionSEN66();
    matterAirQuality = new MatterAirQuality(airQualitySensor, endpoint);
    matterAirQuality->CreateAirQualityEndpoint(node);

    /* Mark deferred persistence for some attributes that might be changed rapidly */
    attribute_t *current_level_attribute = attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::set_deferred_persistence(current_level_attribute);

    attribute_t *current_x_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
    attribute::set_deferred_persistence(current_x_attribute);
    attribute_t *current_y_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
    attribute::set_deferred_persistence(current_y_attribute);
    attribute_t *color_temp_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::set_deferred_persistence(color_temp_attribute);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    // Enable secondary network interface
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint = endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create secondary network interface endpoint"));
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);

    AddThreadNetworkDiagnosticsCluster(node);
#endif

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
    auto * dac_provider = get_dac_provider();
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
    static_cast<ESP32SecureCertDACProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
    static_cast<ESP32FactoryDataProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#endif
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    /* Starting driver with default values */
    app_driver_light_set_defaults(light_endpoint_id);

    matterAirQuality->StartMeasurements();

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

}
