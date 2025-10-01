#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include <driver/gpio.h>

#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00
#define CHANNEL_1    ADC_CHANNEL_0
#define CHANNEL_2    ADC_CHANNEL_3
#define UNIT_1       ADC_UNIT_1

/* ------ FINISH HANDLING SUBSCRIPTIONS ------ */

/* Global Variables */
static const char *TAG = "ble_app";
static uint8_t ble_addr_type;
static uint8_t addr_val[6] = {0};
static uint16_t joystick_val_handle;
static int adc_raw[2][10];
static adc_oneshot_unit_handle_t handle1;
static adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = UNIT_1
};
static adc_oneshot_chan_cfg_t cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
};
static gpio_config_t gp0 = {
    .pin_bit_mask = (1ULL<<32),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFF0),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0xBEEF),
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &joystick_val_handle,
                .access_cb = joystick_subscribe_cb,
                .arg = NULL,
            },
            {0} /* terminator for characteristics */ 
        }
    },
    {0} /* terminator for services */
};

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *arg);
static void joystick_task(void *arg);
inline static void format_addr(char *addr_str, uint8_t addr[]);
static void start_advertising(void);
static void adc_init(adc_oneshot_unit_handle_t handle, adc_oneshot_unit_init_cfg_t init_cfg, adc_oneshot_chan_cfg_t cfg);

/* Public function declarations */
int gatt_svc_init(void);
void ble_store_config_init(void);


/* Configure Channels 0 and 3 to be read from ADC Unit 1*/
static void adc_init(adc_oneshot_unit_handle_t handle, adc_oneshot_unit_init_cfg_t init_cfg, adc_oneshot_chan_cfg_t cfg){
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, CHANNEL_1, &cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, CHANNEL_2, &cfg));
}

inline static void format_addr(char *addr_str, uint8_t addr[]){
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

/* CALLBACK FUNCTION */
static void on_stack_reset(int reason){
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

/* INIT and STORE BLE Address Type and Address Value */
/* Set GAP and begin advertising */
static void on_stack_sync(void){
    int rc = 0;
    char addr_str[18] = {0};
    rc = ble_hs_util_ensure_addr(0);
    if(rc != 0){
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }
    rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if(rc != 0){
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }
    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(ble_addr_type, addr_val, NULL);
    if(rc != 0){
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    /* format addr_val into String */
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);
    ble_app_advertise();
}

/* Handler for GAP events (when devices connect and disconnect) */
static int ble_gap_event(struct ble_gap_event *event, void *arg){
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

/* Set host callbacks and Store host configuration */
static void nimble_host_config_init(void){
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_store_config_init();
}

/* Set GAP struct to advertise explicitly declared data */
/* Set device address for GAP Scan Response fields */
/* START ADVERTISING */
static void ble_app_advertise(void){
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set member variables of adv_fields struct */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set adv_fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if(rc != 0){
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = ble_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if(rc != 0){
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set non-connetable and general discoverable mode */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Start advertising */
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if(rc != 0){
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started");
}


/* run on freeRTOS indefinitely */
static void nimble_host_task(void *arg) {
    ESP_LOGI(TAG, "nimBLE host task started");
    nimble_port_run();
    vTaskDelete(NULL);
}


/* Read each ADC channel 10 times per second */
/* Send raw ADC data over BLE */
static void joystick_task(void *arg){
    int x, y;
    uint8_t buf[4];

    /* Turn on Joystick Switch */
    gpio_set_level(GPIO_NUM_32, 1);

    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, CHANNEL_1, &x));
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, CHANNEL_2, &y));

        /* store data in buffer (little endian) */
        buf[0] = x & 0xFF;
        buf[1] = (x >> 8) & 0xFF;
        buf[2] = y & 0xFF;
        buf[3] = (y >> 8) & 0xFF;

        /* notify subscribed clients */
        ble_gatts_chr_updated(joystick_val_handle);

        /* Update the characteristic value */
        ble_gatts_set_attr_value(joystick_val_handle, sizeof(buf), buf);

        /* Read each ADC channel 10 times per second */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES ||
       ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    ret = nimble_port_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }
    ble_svc_gap_device_name_set("BLE-Server");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    adc_init(handle1, init_cfg, cfg);
    gpio_config(&gp0);

    /* ble_gatts_count_cfg(gatt_svcs) function counts how many services and 
    /* characteristics were defined. This info is used to allocate resources 
    /* appropriately. */    
    ble_gatts_count_cfg(gatt_svcs);

    /* register services into GATT data */
    ble_gatts_add_svcs(gatt_svcs);

    /* when stack and host sync ADVERTISING BEGINS*/
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
}
