#include <stdlib.h>
#include <string.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include "esp_gap_ble_api.h"
#include <esp_gatts_api.h>
#include <esp_log.h>

#include <map>

#include "ble.h"

static uint8_t stupid_service_uuid[ESP_UUID_LEN_128] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value ....... must be 0x00 0x00 for 128 uuid
    0x7E, 0xA5, 0x45, 0xDF, 0xA7, 0x25, 0x42, 0x66, 0xB9, 0x67, 0x9B, 0x19, 0x00, 0x00, 0x79, 0x53
};

static const uint16_t STUPID_CHARC_UUID = 0x0001;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = ESP_UUID_LEN_128,
    .p_service_uuid = stupid_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const uint16_t PRIM_SERVICE_UUID = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t CHARC_DECLARE_UUID = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t  CHARC_PROP =  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t CHARC_CONFIG_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static uint8_t charc_val = 20;
static uint16_t charc_ccc_val = 0;

#define GATTS_ATTR_COUNT 4

#define STUPID_SERVICE_IDX 0
#define STUPID_CHARC_DECL_IDX 1
#define STUPID_CHARC_VALUE_IDX 2
#define STUPID_CHARC_CONFIG_IDX 3

static const esp_gatts_attr_db_t gatt_db[GATTS_ATTR_COUNT] = {
    // Service Declaration
    [STUPID_SERVICE_IDX]        =
    {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&PRIM_SERVICE_UUID, .perm = ESP_GATT_PERM_READ,
      .max_length = ESP_UUID_LEN_128, .length = ESP_UUID_LEN_128, .value = stupid_service_uuid}},

    /* Characteristic Declaration */
    [STUPID_CHARC_DECL_IDX]     =
    {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&CHARC_DECLARE_UUID, .perm = ESP_GATT_PERM_READ,
      .max_length = 1, .length = 1, .value = (uint8_t *)&CHARC_PROP}},

    /* Characteristic Value */
    [STUPID_CHARC_VALUE_IDX] =
    {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&STUPID_CHARC_UUID, .perm = ESP_GATT_PERM_READ,
      .max_length = 1, .length = 1, .value = &charc_val}},

    /* Client Characteristic Configuration Descriptor */
    [STUPID_CHARC_CONFIG_IDX]  =
    {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&CHARC_CONFIG_UUID, 
        .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, .max_length = 2, .length = 2, .value = (uint8_t *)&charc_ccc_val}},
};

static uint16_t gatt_attr_handle[GATTS_ATTR_COUNT];

void gap_evt(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT\n");
        esp_ble_gap_start_advertising(&adv_params);
    }   break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");
    }   break;
    default:
        ESP_LOGI(__FILENAME__, "gap unhandled event=%d\n", event);
        break;
    }
}

std::map<uint16_t, esp_gatt_if_t> x = std::map<uint16_t, esp_gatt_if_t>();

void gatts_evt(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_REG_EVT\n");
        esp_ble_gap_set_device_name("ESP32");
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_ATTR_COUNT, 0);
    } break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_CREAT_ATTR_TAB_EVT\n");
        if(param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(gatt_attr_handle, param->add_attr_tab.handles, sizeof(gatt_attr_handle));
            esp_ble_gatts_start_service(gatt_attr_handle[STUPID_SERVICE_IDX]);
        }
    } break;
    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_CONNECT_EVT\n");
        x[param->connect.conn_id] = gatts_if;
    } break;
    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_DISCONNECT_EVT\n");
        x.erase(param->disconnect.conn_id);
        esp_ble_gap_start_advertising(&adv_params);
    } break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_READ_EVT\n");
        esp_ble_gatts_set_attr_value(gatt_attr_handle[STUPID_CHARC_VALUE_IDX], sizeof(charc_val), (uint8_t*)&charc_val);
    } break;
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_WRITE_EVT\n");
        if(gatt_attr_handle[STUPID_CHARC_CONFIG_IDX] == param->write.handle && param->write.len == 2) {
            charc_ccc_val = param->write.value[1]<<8 | param->write.value[0];
            if(charc_ccc_val == 0x0001) {
                ESP_LOGI(__FILENAME__, "ESP_GATTS_WRITE_EVT: notify enable\n");
            } else if (charc_ccc_val == 0x0002) {
                ESP_LOGI(__FILENAME__, "ESP_GATTS_WRITE_EVT: indicate enable\n");
            } else if (charc_ccc_val == 0x0000) {
                ESP_LOGI(__FILENAME__, "ESP_GATTS_WRITE_EVT: notify/indicate disable\n");
            }
        }
    } break;
    case ESP_GATTS_EXEC_WRITE_EVT: {
        ESP_LOGI(__FILENAME__, "ESP_GATTS_EXEC_WRITE_EVT\n");
    } break;
    case ESP_GATTS_SET_ATTR_VAL_EVT: {
    } break;
    case ESP_GATTS_MTU_EVT: {
    } break;
    case ESP_GATTS_CONF_EVT: {
    } break;
    default:
        ESP_LOGI(__FILENAME__, "gatts unhandled event=%d\n", event);
      break;
    }
}

void ble_init() {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_controller_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_controller_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_evt));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_evt));

    ESP_LOGI(__FILENAME__, "ble_init()");
}

void ble_start_app() {
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_LOGI(__FILENAME__, "ble_start_app()");
}

void ble_set_charc(uint8_t value) {
    if(!x.empty()) {
        charc_val = value;
        if(charc_ccc_val == 0x0001 || charc_ccc_val == 0x0002) {
            bool need_confirm = (charc_ccc_val == 0x0002);
            for (const auto& v : x) {
                esp_ble_gatts_send_indicate(v.second, v.first, gatt_attr_handle[STUPID_CHARC_VALUE_IDX], sizeof(charc_val), (uint8_t*)&charc_val, need_confirm);
            }
        } else {
            esp_ble_gatts_set_attr_value(gatt_attr_handle[STUPID_CHARC_VALUE_IDX], sizeof(charc_val), (uint8_t*)&charc_val);
        }
    }
}