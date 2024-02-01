#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>

#define TAG "GATTC_DEMO"

void handle_jbd_response(const uint8_t* inputBytes, const uint8_t inputBytesLen);

esp_bt_uuid_t JBD_MAIN_SERVICE_UUID = { //0000ff00-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = 0xFF00
};

esp_bt_uuid_t JDB_READABLE_CHAR_UUID = { //0000ff01-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = 0xFF01
};

esp_bt_uuid_t JBD_WRITEABLE_CHAR_UUID = { //0000ff02-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = 0xFF02
};

enum {
    PROFILE_BMS_READABLE,
    PROFILE_BMS_WRITEABLE,
    PROFILE_NUM
};

static bool connect    = false;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_BMS_READABLE] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_BMS_WRITEABLE] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

void sendBasicInfoRequest(){
        ESP_LOGI(TAG, "Sending request");
        extern uint8_t CMD_REQUEST_CELL_VOLTAGES_LEN;
        extern uint8_t CMD_REQUEST_CELL_VOLTAGES[];
        extern uint8_t CMD_REQUEST_BASIC_INFO_LEN;
        extern uint8_t CMD_REQUEST_BASIC_INFO[];
        esp_err_t err=esp_ble_gattc_write_char( gl_profile_tab[PROFILE_BMS_WRITEABLE].gattc_if,
                                  gl_profile_tab[PROFILE_BMS_WRITEABLE].conn_id,
                                  gl_profile_tab[PROFILE_BMS_WRITEABLE].char_handle,
                                  CMD_REQUEST_CELL_VOLTAGES_LEN,
                                  CMD_REQUEST_CELL_VOLTAGES,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        if(ESP_OK != err){
            ESP_LOGE(TAG, "Failed to write request");
        }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT: {
        if(PROFILE_BMS_READABLE==param->reg.app_id){
            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if (scan_ret){
                ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
            }
        }
        break;
    }
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_BMS_READABLE].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_BMS_READABLE].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        gl_profile_tab[PROFILE_BMS_WRITEABLE].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_BMS_WRITEABLE].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        ESP_LOGI(TAG, "REMOTE BDA:");
        esp_log_buffer_hex(TAG, gl_profile_tab[PROFILE_BMS_READABLE].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &JBD_MAIN_SERVICE_UUID);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d uuidlen=%d uuid16=%x", p_data->search_res.conn_id, p_data->search_res.is_primary, p_data->search_res.srvc_id.uuid.len, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        if (p_data->search_res.srvc_id.uuid.len == JBD_MAIN_SERVICE_UUID.len && p_data->search_res.srvc_id.uuid.uuid.uuid16 == JBD_MAIN_SERVICE_UUID.uuid.uuid16) {
            ESP_LOGI(TAG, "JBD service found");
            gl_profile_tab[PROFILE_BMS_READABLE].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_BMS_READABLE].service_end_handle = p_data->search_res.end_handle;
            gl_profile_tab[PROFILE_BMS_WRITEABLE].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_BMS_WRITEABLE].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(TAG, "Get service information from flash");
        } else {
            ESP_LOGE(TAG, "unknown service source");
        }
        ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    gl_profile_tab[PROFILE_BMS_READABLE].service_start_handle,
                                                                    gl_profile_tab[PROFILE_BMS_READABLE].service_end_handle,
                                                                    0,
                                                                    &count);
        if (status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
            break;
        }
        if (count != 2){
            ESP_LOGE(TAG, "Unexpected number of charactertics in service");
            break;
        }

        esp_gattc_char_elem_t writeableCharactertic;
        count=1;
        status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                    p_data->search_cmpl.conn_id,
                                                    gl_profile_tab[PROFILE_BMS_WRITEABLE].service_start_handle,
                                                    gl_profile_tab[PROFILE_BMS_WRITEABLE].service_end_handle,
                                                    JBD_WRITEABLE_CHAR_UUID,
                                                    &writeableCharactertic,
                                                    &count);
        if (status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid for writeable error");
            break;
        }
        if(count != 1){
            ESP_LOGE(TAG, "writeableCharactertic not found");
            break;
        }
        gl_profile_tab[PROFILE_BMS_WRITEABLE].char_handle = writeableCharactertic.char_handle;

        //-------
        esp_gattc_char_elem_t readableCharactertic;
        count=1;
        status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                    p_data->search_cmpl.conn_id,
                                                    gl_profile_tab[PROFILE_BMS_READABLE].service_start_handle,
                                                    gl_profile_tab[PROFILE_BMS_READABLE].service_end_handle,
                                                    JDB_READABLE_CHAR_UUID,
                                                    &readableCharactertic,
                                                    &count);
        if (status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid for readable error");
            break;
        }
        if(count != 1){
            ESP_LOGE(TAG, "readableCharactertic not found");
            break;
        }
        if ((readableCharactertic.properties&ESP_GATT_CHAR_PROP_BIT_NOTIFY)==0){
            ESP_LOGE(TAG, "readableCharactertic does not have notify flag");
            break;
        }
        gl_profile_tab[PROFILE_BMS_READABLE].char_handle = readableCharactertic.char_handle;
        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_BMS_READABLE].remote_bda, gl_profile_tab[PROFILE_BMS_READABLE].char_handle);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
            break;
        }

#if 1
        uint16_t count = 0;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                        gl_profile_tab[PROFILE_BMS_READABLE].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gl_profile_tab[PROFILE_BMS_READABLE].service_start_handle, //Ignored param
                                                                        gl_profile_tab[PROFILE_BMS_READABLE].service_end_handle, //Ignored param
                                                                        gl_profile_tab[PROFILE_BMS_READABLE].char_handle,
                                                                        &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
            break;
        }
        if (count == 0){
            ESP_LOGE(TAG, "No notification characteristic found for readable");
            break;
        }
        
        const esp_bt_uuid_t notify_descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
        };
        esp_gattc_descr_elem_t notificationDescriptor;
        ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                gl_profile_tab[PROFILE_BMS_READABLE].conn_id,
                                                                p_data->reg_for_notify.handle,
                                                                notify_descr_uuid,
                                                                &notificationDescriptor,
                                                                &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error");
            break;
        }
        if(count!=1){
            ESP_LOGE(TAG, "Can't find a good count of notification char count=%d", count);
            break;
        }
        if (!(notificationDescriptor.uuid.len == ESP_UUID_LEN_16 && notificationDescriptor.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)){
            ESP_LOGE(TAG, "Can't activate notification on this characteristic uuid=0x%04X", notificationDescriptor.uuid.uuid.uuid16);
            break;
        }
        
        uint16_t notify_en = 1;
        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                        gl_profile_tab[PROFILE_BMS_READABLE].conn_id,
                                                        notificationDescriptor.handle,
                                                        sizeof(notify_en),
                                                        (uint8_t *)&notify_en,
                                                        ESP_GATT_WRITE_TYPE_RSP,
                                                        ESP_GATT_AUTH_REQ_NONE);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
        }
        ESP_LOGI(TAG, "Requested notification");
#endif
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT");
        if (p_data->notify.is_notify){
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write descr success on connection %d, handle=%d. Notification is enabled.", p_data->write.conn_id, p_data->write.handle);
        sendBasicInfoRequest();
#if 0
        const esp_bt_uuid_t notify_descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
        };
        esp_gattc_descr_elem_t notificationDescriptor;
        count=1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                gl_profile_tab[PROFILE_BMS_READABLE].conn_id,
                                                                p_data->write.handle,
                                                                notify_descr_uuid,
                                                                &notificationDescriptor,
                                                                &count);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error");
            break;
        }
        if(count!=1){
            ESP_LOGE(TAG, "Can't find a good count of notification char count=%d", count);
            break;
        }
        if (!(notificationDescriptor.uuid.len == ESP_UUID_LEN_16 && notificationDescriptor.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)){
            ESP_LOGE(TAG, "Can't activate notification on this characteristic uuid=0x%04X", notificationDescriptor.uuid.uuid.uuid16);
            break;
        }
        
        ret_status = esp_ble_gattc_read_char_descr( gattc_if,
                                                        gl_profile_tab[PROFILE_BMS_READABLE].conn_id,
                                                        notificationDescriptor.handle,
                                                        ESP_GATT_AUTH_REQ_NONE);
        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
        }
#endif
        break;

    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write char success");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    case ESP_GATTC_READ_CHAR_EVT:        
        ESP_LOGI(TAG, "ESP_GATTC_READ_CHAR_EVT value len=%d", param->read.value_len);
        esp_log_buffer_hex(TAG, p_data->read.value, p_data->read.value_len);
        handle_jbd_response(p_data->read.value, p_data->read.value_len);
    default:
        break;
    }
}


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
//             esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
//             ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name == NULL || adv_name_len==0) {
                break;
            }
            ESP_LOGI(TAG, "Device name %.*s", adv_name_len, adv_name);

            uint8_t principal_service_uuid_len=0;
            uint8_t* principal_service_uuid = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_16SRV_PART, &principal_service_uuid_len);
            if(principal_service_uuid_len==0){
                break;
            }
            esp_log_buffer_hex(TAG, principal_service_uuid, principal_service_uuid_len);
            if(0xFF==principal_service_uuid[0] && 0x00==principal_service_uuid[1]){
                ESP_LOGW(TAG, "Found principal service");
            }

            uint8_t manufacturer_bytes_len=0;
            uint8_t* manufacturer_bytes = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manufacturer_bytes_len);
            if(6!=manufacturer_bytes_len){
                break;
            }
            if(! (0x38==manufacturer_bytes[3] && 0xc1==manufacturer_bytes[4] && 0xa4==manufacturer_bytes[5])){
                break;
            }
            ESP_LOGW(TAG, "Found manufacturer flags that match our BMS");
            esp_log_buffer_hex(TAG, manufacturer_bytes, manufacturer_bytes_len);
            if(connect == false) {
                connect = true;
                ESP_LOGI(TAG, "connect to %.*s", adv_name_len, adv_name);
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(gl_profile_tab[PROFILE_BMS_READABLE].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
            }

            ESP_LOGI(TAG, " ");
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "Search complete");
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void test_parser();
void app_main(void)
{
//     test_parser();
//     return;
    
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_BMS_READABLE);
    if (ret){
        ESP_LOGE(TAG, "%s gattc app register readable failed, error code = %x", __func__, ret);
    }
    ret = esp_ble_gattc_app_register(PROFILE_BMS_WRITEABLE);
    if (ret){
        ESP_LOGE(TAG, "%s gattc app register writeable failed, error code = %x", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
#if 1
    while(true){
//         vTaskDelay(5000/portTICK_PERIOD_MS);
//         sendBasicInfoRequest();
        vTaskDelay(1000/portTICK_PERIOD_MS);
//         esp_gatt_status_t ret_status = esp_ble_gattc_read_char(gl_profile_tab[PROFILE_BMS_READABLE].gattc_if,gl_profile_tab[PROFILE_BMS_READABLE].conn_id,gl_profile_tab[PROFILE_BMS_READABLE].char_handle,ESP_GATT_AUTH_REQ_NONE);
//         break;
    }
#endif
}
