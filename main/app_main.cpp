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

#include "MatterAirQualitySensor.h"
#include "MatterExtendedColorLight.h"
#include "MatterHumiditySensor.h"
#include "MatterTemperatureSensor.h"
#include "SensirionSEN66.h"

static constexpr uint32_t MEASUREMENT_SAMPLE_SECONDS = 60;

MatterAirQualitySensor* matterAirQualitySensor;
MatterExtendedColorLight* matterExtendedColorLight;
MatterTemperatureSensor* matterTemperatureSensor;
MatterHumiditySensor* matterHumiditySensor;
esp_timer_handle_t sensor_timer_handle;

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

    esp_matter::cluster::software_diagnostics::attribute::create_current_heap_free(cluster, 0);
    esp_matter::cluster::software_diagnostics::attribute::create_current_heap_used(cluster, 0);
    esp_matter::cluster::software_diagnostics::attribute::create_thread_metrics(cluster, nullptr, 0, 0);
}

void AddThreadNetworkDiagnosticsCluster(node_t* node)
{
    endpoint_t* root_endpoint = endpoint::get(node, 0);

    cluster::thread_network_diagnostics::config_t thread_network_config;
    cluster_t* cluster = cluster::thread_network_diagnostics::create(root_endpoint, &thread_network_config, CLUSTER_FLAG_SERVER);

    // ErrorCounts (ERRCNT)
    cluster::thread_network_diagnostics::feature::error_counts::add(cluster);

    // PacketCounts (PKTCNT)
    cluster::thread_network_diagnostics::feature::packets_counts::add(cluster);

    // MLECounts (MLECNT)
    cluster::thread_network_diagnostics::feature::mle_counts::add(cluster);
    esp_matter::cluster::thread_network_diagnostics::attribute::create_chlid_role_count(cluster, 0);
    esp_matter::cluster::thread_network_diagnostics::attribute::create_leader_role_count(cluster, 0);
    esp_matter::cluster::thread_network_diagnostics::attribute::create_router_role_count(cluster, 0);

    // MACCounts (MACCNT)
    cluster::thread_network_diagnostics::feature::mac_counts::add(cluster);
    esp_matter::cluster::thread_network_diagnostics::attribute::create_rx_total_count(cluster, 0);
    esp_matter::cluster::thread_network_diagnostics::attribute::create_tx_total_count(cluster, 0);

    esp_matter::cluster::thread_network_diagnostics::attribute::create_rloc16(cluster, 0);
}

void AddWiFidNetworkDiagnosticsCluster(node_t* node)
{
    endpoint_t* root_endpoint = endpoint::get(node, 0);

    cluster::wifi_network_diagnostics::config_t cluster_config;
    cluster_t* cluster = cluster::wifi_network_diagnostics::create(root_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // ErrorCounts (ERRCNT)
    cluster::wifi_network_diagnostics::feature::error_counts::add(cluster);

    // PacketCounts (PKTCNT)
    cluster::wifi_network_diagnostics::feature::packets_counts::add(cluster);
}

void ConfigureGeneralDiagnosticsCluster(node_t* node)
{
    esp_matter::endpoint_t *rootEndpoint = esp_matter::endpoint::get(node, 0);
    cluster_t *cluster = cluster::get(rootEndpoint, GeneralDiagnostics::Id);

    uint8_t bootReason = GetMatterBootReason();
    cluster::general_diagnostics::attribute::create_boot_reason(cluster, bootReason);
}

// Timer callback to measure air quality
void UpdateSensorsTimerCallback(void *arg)
{
    matterAirQualitySensor->UpdateMeasurements();
    matterTemperatureSensor->UpdateMeasurements();
    matterHumiditySensor->UpdateMeasurements();
}

void StartUpdateSensorsTimer()
{
    // Setup periodic timer to update sensor measurements

    esp_timer_create_args_t timer_args = {
        .callback = &UpdateSensorsTimerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK, // Run callback in a task (safer for I2C)
        .name = "update_sensors_timer",
        .skip_unhandled_events = true, // Skip if previous callback is still running
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &sensor_timer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return;
    }
    
    // Start the timer to trigger every 60 seconds (1 minute)
    err = esp_timer_start_periodic(sensor_timer_handle, MEASUREMENT_SAMPLE_SECONDS * 1000000ULL); // In microseconds
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Update sensors timer started.");
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
    AddSoftwareDiagnosticsCluster(node);

    matterExtendedColorLight = new MatterExtendedColorLight(node, light_handle);
    endpoint_t* light_endpoint = matterExtendedColorLight->CreateEndpoint();

    ABORT_APP_ON_FAILURE(light_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create extended color light endpoint"));

    light_endpoint_id = endpoint::get_id(light_endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

    AirQualitySensor* airQualitySensor = new SensirionSEN66(25.0f);
    airQualitySensor->Init();

    // Create Matter Air Quality Sensor Endpoint
    matterAirQualitySensor = new MatterAirQualitySensor(node, airQualitySensor, matterExtendedColorLight);
    matterAirQualitySensor->CreateEndpoint();

    // Create Matter Temperature Sensor Endpoint
    matterTemperatureSensor = new MatterTemperatureSensor(node, airQualitySensor);
    matterTemperatureSensor->CreateEndpoint();

    // Create Humidity Sensor Endpoint
    matterHumiditySensor = new MatterHumiditySensor(node, airQualitySensor);
    matterHumiditySensor->CreateEndpoint();

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

#if CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    AddWiFidNetworkDiagnosticsCluster(node);
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

    StartUpdateSensorsTimer();

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

