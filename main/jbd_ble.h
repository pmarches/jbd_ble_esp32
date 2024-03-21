#pragma once

#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <ctype.h>

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

#include <jbd_parser.h>

class JBDConnection {
public:
    esp_bd_addr_t macAddress;
    esp_ble_addr_type_t addressType;
    uint16_t conn_id;
    SemaphoreHandle_t connectSem;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t readableCharacterticHandle;
    uint16_t writeableCharacterticHandle;
    uint8_t defragbuffer[128];
    uint8_t defragbufferLen=0;
    JBDParser parser;
    
    char deviceName[32];
    JBDBasicInfo basicInfo;
    JBDPackInfo packInfo;
    JBDCellInfo cellInfo;

    JBDConnection();
    void connectAndWait();
    void waitUntilConnected();
    void onConnected(uint16_t connectionId);
    void onDisconnected();
    void setCharHandle(uint16_t readableCharacterticHandle, uint16_t writeableCharacterticHandle);
    void handle_jbd_notification(uint8_t* fragmentedBytes, uint8_t fragmentLen);    
    void requestBasicInfo();
    void requestPackInfo();
    void requestCellVoltages();
    void receiveBMSUpdate(const JBDParseResult& msg);
};


class JBDBLEStack {
    static const uint8_t JBD_APP_ID=0;
    static const uint8_t JBD_CONTROLLER_COUNT=2;
    static esp_gattc_char_elem_t readableCharactertic; //These need to be static for notification to work
    static esp_gattc_char_elem_t writeableCharactertic; //These need to be static for notification to work
    
public:
    esp_gatt_if_t gattc_if;
    
    static esp_bt_uuid_t JBD_MAIN_SERVICE_UUID; //0000ff00-0000-1000-8000-00805f9b34fb
    static esp_bt_uuid_t JBD_READABLE_CHAR_UUID; //0000ff01-0000-1000-8000-00805f9b34fb
    static esp_bt_uuid_t JBD_WRITEABLE_CHAR_UUID;//0000ff02-0000-1000-8000-00805f9b34fb
    uint8_t jbdControllersCount=0;
    JBDConnection jbdControllers[2];

    static JBDBLEStack* instance;
    static JBDBLEStack* getInstance();
        
    void newBMSFound(uint8_t* deviceName, uint8_t deviceNameLen, esp_bd_addr_t& bda, esp_ble_addr_type_t controllerAddressType);    
    JBDConnection* findConnectionByMAC(esp_bd_addr_t& macAddressToFind);
    JBDConnection* findConnectionByConnId(uint16_t connIdToFind);
    void connectToControllers();
    void waitForControllers();

    static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
    static void configure_ble_stack();
};
