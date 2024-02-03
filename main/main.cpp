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
#include "freertos/semphr.h"
#include <ctype.h>

#define TAG "JDB_MAIN"

extern "C" void handle_jbd_response(const uint8_t* inputBytes, const uint8_t inputBytesLen);
            esp_gattc_char_elem_t readableCharactertic;
            esp_gattc_char_elem_t writeableCharactertic;

class JBDConnection {
public:
    esp_bd_addr_t macAddress;
    esp_ble_addr_type_t addressType;
    uint16_t conn_id;
    SemaphoreHandle_t semaphore;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t readableCharacterticHandle;
    uint16_t writeableCharacterticHandle;
    
    JBDConnection(){
        semaphore=xSemaphoreCreateMutex();
    }
    
    void waitConnection();
    void onConnected(uint16_t connectionId){
        this->conn_id=connectionId;
        xSemaphoreGive(semaphore);
    }
    
    void setCharHandle(uint16_t readableCharacterticHandle, uint16_t writeableCharacterticHandle){
        ESP_LOGD(TAG, "setCharHandle: readableCharacterticHandle=%d, writeableCharacterticHandle=%d", readableCharacterticHandle, writeableCharacterticHandle);
        this->readableCharacterticHandle=readableCharacterticHandle;
        this->writeableCharacterticHandle=writeableCharacterticHandle;
    }
};


class JBDBLEStack {
    static const uint8_t JBD_APP_ID=0;
    static const uint8_t JBD_CONTROLLER_COUNT=2;
    
public:
    esp_gatt_if_t gattc_if;
    
    static esp_bt_uuid_t JBD_MAIN_SERVICE_UUID; //0000ff00-0000-1000-8000-00805f9b34fb
    static esp_bt_uuid_t JDB_READABLE_CHAR_UUID; //0000ff01-0000-1000-8000-00805f9b34fb
    static esp_bt_uuid_t JBD_WRITEABLE_CHAR_UUID;//0000ff02-0000-1000-8000-00805f9b34fb

    static JBDBLEStack* instance;
    static JBDBLEStack* getInstance(){
        if(NULL==instance){
            instance=new JBDBLEStack();
            configure_ble_stack();
        }
        return instance;
    }
    
    uint8_t jbdControllersCount=0;
    JBDConnection jbdControllers[2];
    
    void newControllerFound(esp_bd_addr_t bda, esp_ble_addr_type_t controllerAddressType){
        ESP_LOGI(TAG, "controllerAddressType=%d", controllerAddressType);
        memcpy(this->jbdControllers[jbdControllersCount].macAddress, bda, sizeof(esp_bd_addr_t));
        this->jbdControllers[jbdControllersCount].addressType=controllerAddressType;
        
        jbdControllersCount++;
        ESP_LOGI(TAG, "Have now found %d JBD controllers", jbdControllersCount);
    }
    
    JBDConnection* findConnectionByMAC(esp_bd_addr_t& macAddressToFind){
        for(int i=0; i<jbdControllersCount; i++){
            if(0==memcmp(jbdControllers[i].macAddress, macAddressToFind, sizeof(esp_bd_addr_t))){
                return &jbdControllers[i];
            }
        }
        return NULL;
    }
    
    JBDConnection* findConnectionByConnId(uint16_t connIdToFind){
        for(int i=0; i<jbdControllersCount; i++){
            if(jbdControllers[i].conn_id==connIdToFind){
                return &jbdControllers[i];
            }
        }
        return NULL;
    }
    
    
    void connectToControllers(){
        ESP_LOGI(TAG, "connectToControllers %d", jbdControllersCount);
        for(int i=0; i<jbdControllersCount; i++){
            jbdControllers[i].waitConnection();
        }
    }

    static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
        switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 7; //the unit of the duration is second
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT: {
    //             esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
    //             ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                uint8_t *adv_name = NULL;
                uint8_t adv_name_len = 0;
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
                JBDBLEStack::getInstance()->newControllerFound(scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type);
                
//                 ESP_LOGI(TAG, "connect to %.*s", adv_name_len, adv_name);
//                 esp_ble_gap_stop_scanning();
//                 esp_ble_gattc_open(gl_profile_tab[PROFILE_BMS_READABLE].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                break;
            }
            case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                ESP_LOGI(TAG, "Search complete");
                getInstance()->connectToControllers();

                break;
            default:
                ESP_LOGI(TAG, "Unhandeled GAP scan event %d", scan_result->scan_rst.search_evt);
                break;
            }
            break;
        }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:{
            break;
        }
        default:
            ESP_LOGI(TAG, "Unhandeled GAP event %d", event);
            break;
        }
    }

    static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param){
        esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
        switch(event){
        case ESP_GATTC_REG_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
            JBDBLEStack::getInstance()->gattc_if=gattc_if;
            if(JBD_APP_ID==param->reg.app_id){
                static esp_ble_scan_params_t ble_scan_params = {
                    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
                    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
                    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
                    .scan_interval          = 0x50,
                    .scan_window            = 0x30,
                    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
                };

                esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
                if (scan_ret){
                    ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
                }
            }
            break;
        }
        case ESP_GATTC_CONNECT_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
//             gl_profile_tab[PROFILE_BMS_READABLE].conn_id = p_data->connect.conn_id;
//             memcpy(gl_profile_tab[PROFILE_BMS_READABLE].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

            ESP_LOGI(TAG, "REMOTE BDA:");
//             esp_log_buffer_hex(TAG, gl_profile_tab[PROFILE_BMS_READABLE].remote_bda, sizeof(esp_bd_addr_t));
//             esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
//             if (mtu_ret){
//                 ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
//             }
            break;
        }
        case ESP_GATTC_OPEN_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_OPEN_EVT");
            if (param->open.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "open failed, status %d", p_data->open.status);
                break;
            }
            JBDBLEStack::getInstance()->findConnectionByMAC(param->open.remote_bda)->onConnected(param->open.conn_id);
            ESP_LOGI(TAG, "open success");
            break;
        }
        case ESP_GATTC_DIS_SRVC_CMPL_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT");
            esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &JBD_MAIN_SERVICE_UUID);
            break;
        }
        case ESP_GATTC_SEARCH_RES_EVT: {
            ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d uuidlen=%d uuid16=%x", p_data->search_res.conn_id, p_data->search_res.is_primary, p_data->search_res.srvc_id.uuid.len, p_data->search_res.srvc_id.uuid.uuid.uuid16);
            if (p_data->search_res.srvc_id.uuid.len == JBD_MAIN_SERVICE_UUID.len && p_data->search_res.srvc_id.uuid.uuid.uuid16 == JBD_MAIN_SERVICE_UUID.uuid.uuid16) {
                ESP_LOGI(TAG, "JBD service found");
                JBDConnection* conn=JBDBLEStack::getInstance()->findConnectionByConnId(param->search_res.conn_id);
                conn->service_start_handle = p_data->search_res.start_handle;
                conn->service_end_handle = p_data->search_res.end_handle;
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
            if (p_data->search_cmpl.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
                break;
            }
            JBDConnection* conn=JBDBLEStack::getInstance()->findConnectionByConnId(param->search_cmpl.conn_id);
            
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                        p_data->search_cmpl.conn_id,
                                                                        ESP_GATT_DB_CHARACTERISTIC,
                                                                        conn->service_start_handle,
                                                                        conn->service_end_handle,
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

            count=1;
            status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                        p_data->search_cmpl.conn_id,
                                                        conn->service_start_handle,
                                                        conn->service_end_handle,
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

            //-------
            count=1;
            status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                        p_data->search_cmpl.conn_id,
                                                        conn->service_start_handle,
                                                        conn->service_end_handle,
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
            conn->setCharHandle(readableCharactertic.char_handle, writeableCharactertic.char_handle);
            esp_ble_gattc_register_for_notify(gattc_if, conn->macAddress, conn->readableCharacterticHandle);
            break;
        }
        case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
            if (p_data->reg_for_notify.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
                break;
            }
            ESP_LOGD(TAG, "p_data->reg_for_notify.handle=%d", p_data->reg_for_notify.handle);
            JBDConnection* conn=JBDBLEStack::getInstance()->findConnectionByConnId(0);
#if 1
            uint16_t count = 0;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                            conn->conn_id,
                                                                            ESP_GATT_DB_DESCRIPTOR,
                                                                            conn->service_start_handle, //Ignored param
                                                                            conn->service_end_handle, //Ignored param
                                                                            conn->readableCharacterticHandle,
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
                                                                    conn->conn_id,
                                                                    conn->readableCharacterticHandle,
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
            esp_err_t write_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                            conn->conn_id,
                                                            notificationDescriptor.handle,
                                                            sizeof(notify_en),
                                                            (uint8_t *)&notify_en,
                                                            ESP_GATT_WRITE_TYPE_RSP,
                                                            ESP_GATT_AUTH_REQ_NONE);
            if (write_status != ESP_OK){
                ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
            }
            ESP_LOGI(TAG, "Requested notification");
            write_status = esp_ble_gattc_read_char_descr( gattc_if,
                                                            conn->conn_id,
                                                            notificationDescriptor.handle,
                                                            ESP_GATT_AUTH_REQ_NONE);
            if (write_status != ESP_OK){
                ESP_LOGE(TAG, "esp_ble_gattc_read_char_descr error");
            }
#endif
            break;
        }
        case ESP_GATTC_READ_DESCR_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_READ_DESCR_EVT");
            esp_log_buffer_hex(TAG, p_data->read.value, p_data->read.value_len);
            break;
        }
//         case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
//             ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
//             if(p_data->reg_for_notify.status != ESP_GATT_OK){
//                 ESP_LOGE(TAG, "p_data->reg_for_notify.status failed");
//                 break;
//             }
//             break;
//         }
        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT");
            if (p_data->notify.is_notify){
                ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
            }else{
                ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
            }
            esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
            break;
        case ESP_GATTC_WRITE_DESCR_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_WRITE_DESCR_EVT");
            if (p_data->write.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
                break;
            }
            ESP_LOGI(TAG, "write descr success on connection %d, handle=%d. Notification is enabled.", p_data->write.conn_id, p_data->write.handle);
//             sendBasicInfoRequest();
                    ESP_LOGI(TAG, "Sending request for basic info");
//             extern uint8_t CMD_REQUEST_CELL_VOLTAGES_LEN;
//             extern uint8_t CMD_REQUEST_CELL_VOLTAGES[];
            JBDConnection* conn=JBDBLEStack::getInstance()->findConnectionByConnId(param->write.conn_id);
            extern uint8_t CMD_REQUEST_BASIC_INFO_LEN;
            extern uint8_t CMD_REQUEST_BASIC_INFO[];
            esp_err_t err=esp_ble_gattc_write_char( gattc_if,
                                    p_data->write.conn_id,
                                    conn->writeableCharacterticHandle,
                                    CMD_REQUEST_BASIC_INFO_LEN,
                                    CMD_REQUEST_BASIC_INFO,
                                    ESP_GATT_WRITE_TYPE_NO_RSP,
                                    ESP_GATT_AUTH_REQ_NONE);
            if(ESP_OK != err){
                ESP_LOGE(TAG, "Failed to write request");
            }

            break;
        }
        case ESP_GATTC_WRITE_CHAR_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_WRITE_CHAR_EVT");
            if (p_data->write.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
                break;
            }
            ESP_LOGI(TAG, "write char success");
            break;
        }
        case ESP_GATTC_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            break;
        }
        case ESP_GATTC_READ_CHAR_EVT: {
            ESP_LOGI(TAG, "ESP_GATTC_READ_CHAR_EVT value len=%d", param->read.value_len);
            esp_log_buffer_hex(TAG, p_data->read.value, p_data->read.value_len);
            handle_jbd_response(p_data->read.value, p_data->read.value_len);
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandeled GATT event %d", event);
            break;
        }
    }
        
    static void configure_ble_stack(){
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

        ret = esp_ble_gattc_app_register(JBD_APP_ID);
        if (ret){
            ESP_LOGE(TAG, "%s gattc app register readable failed, error code = %x", __func__, ret);
        }

        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }
    }
    
    void search(){
    }
};
JBDBLEStack* JBDBLEStack::instance=NULL;
esp_bt_uuid_t JBDBLEStack::JBD_MAIN_SERVICE_UUID = { //0000ff00-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid={
        .uuid16 = 0xFF00
    }
};

esp_bt_uuid_t JBDBLEStack::JDB_READABLE_CHAR_UUID = { //0000ff01-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid={
        .uuid16 = 0xFF01
    }
};

esp_bt_uuid_t JBDBLEStack::JBD_WRITEABLE_CHAR_UUID = { //0000ff02-0000-1000-8000-00805f9b34fb
    .len = ESP_UUID_LEN_16,
    .uuid={
        .uuid16 = 0xFF02
    }
};

void JBDConnection::waitConnection(){
    esp_ble_gattc_open(JBDBLEStack::getInstance()->gattc_if, macAddress, addressType, true);
    xSemaphoreTake(semaphore, portMAX_DELAY);
}

extern "C" void app_main(void){
    JBDBLEStack* jbdBleStack=JBDBLEStack::getInstance();
    jbdBleStack->search();
}
