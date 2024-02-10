#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

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

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <jbd_parser.h>
#include <jbd_ble.h>

#define TAG "JBD_MAIN"

void writefully(const int fd, const uint8_t* bytes, const size_t nbBytes){
    esp_log_buffer_hex(__FUNCTION__, bytes, nbBytes);
    ssize_t totalNbBytesWritten=0;
    while(totalNbBytesWritten!=nbBytes){
        ssize_t nbBytesWritten=write(fd, bytes+totalNbBytesWritten, nbBytes-totalNbBytesWritten);
        if(nbBytesWritten<0){
            ESP_LOGE(TAG, "write returned an error");
        }
        else{
            totalNbBytesWritten+=nbBytesWritten;
        }
    }
}

void writeStoredRegisterResponseUnsigned(uint8_t registerAddress, uint16_t unsignedValue){
    uint8_t responseBytes[32];
    uint8_t responseBytesLen;
    JBDParser::buildStoredRegisterResponseUnsigned(responseBytes, &responseBytesLen, registerAddress, unsignedValue);    
    writefully(STDOUT_FILENO, responseBytes, responseBytesLen);
}

class AggregateBMSModel {
public:
    static const uint8_t NB_BMS=2;
    JBDPackInfo packInfo[NB_BMS];
    JBDCellInfo cellInfo[NB_BMS];

    AggregateBMSModel(){
        JBDPackInfo packInfo;
        memset(&packInfo, 0, sizeof(packInfo));
        
        packInfo.packVoltage_cV=1323;
        packInfo.packCurrent_cA=-689;
        packInfo.balance_capacity_mAh=15906;
        packInfo.full_capacity_mAh=20000;
        packInfo.cycle_count=133;
        packInfo.manufacture_date=11350;
        packInfo.cell_balance_status=0;
        packInfo.cell_balance_status2=0;
        packInfo.bitset_errors=0;
        packInfo.softwareVersion=23;
        packInfo.state_of_charge=80;
        packInfo.fet_status=3;
        packInfo.cell_count=4;
        packInfo.temperature_sensor_count=2;
        packInfo.temperatures_deciK[0]=3031;
        packInfo.temperatures_deciK[1]=3039;

        update(0, packInfo);
        update(1, packInfo);
    }
    
    void update(uint8_t bmsIdx, JBDPackInfo &packInfoInput){
        packInfo[bmsIdx]=packInfoInput;
    }

    void getPackInfo(JBDPackInfo &packInfoOutput){
        packInfoOutput=packInfo[0];
    }
    
    void getDeviceName(char* outDeviceName){
        strcpy(outDeviceName, "LiFePO4");
    }

    void getCellInfo(JBDCellInfo &cellInfoOutput){
        cellInfoOutput.voltagesMv[0]=3374;
        cellInfoOutput.voltagesMv[1]=3371;
        cellInfoOutput.voltagesMv[2]=3371;
        cellInfoOutput.voltagesMv[3]=3371;
    }
};

AggregateBMSModel gModel;

//These message come from the GX
void handle_message_to_bms(const JBDParseResult& msg){
    if(JBDParseResult::REQUEST!=msg.payloadTypes){
        ESP_LOGE(TAG, "msg.payloadTypes=%d not yet supported", msg.payloadTypes);
        return;
    }
    if(JBDRequest::READ==msg.payload.request.operation){
        uint8_t buildBytes[128];
        uint8_t buildBytesLen=0;
        if(JBDRequest::BASIC_INFO_REGISTER==msg.payload.request.registerAddress){
            JBDPackInfo packInfo;
            gModel.getPackInfo(packInfo);
            JBDParser::buildFromPackInfo(buildBytes, &buildBytesLen, &packInfo);
            writefully(STDOUT_FILENO, buildBytes, buildBytesLen);
        }
        else if(JBDRequest::CELL_VOLTAGE_REGISTER==msg.payload.request.registerAddress){
            JBDCellInfo cellInfo;
            gModel.getCellInfo(cellInfo);
            JBDParser::buildFromCellInfo(buildBytes, &buildBytesLen, &cellInfo);
            writefully(STDOUT_FILENO, buildBytes, buildBytesLen);
        }
        else if(JBDRequest::DEVICE_NAME_REGISTER==msg.payload.request.registerAddress){
            char deviceName[32];
            gModel.getDeviceName(deviceName);
            JBDParser::buildFromDeviceName(buildBytes, &buildBytesLen, deviceName);
            writefully(STDOUT_FILENO, buildBytes, buildBytesLen);
        }
        else if(JBDRequest::CYCLE_CAP_REGISTER==msg.payload.request.registerAddress){
            writeStoredRegisterResponseUnsigned(JBDRequest::CYCLE_CAP_REGISTER, 400);
        }
        else if(JBDRequest::CHARGE_OVER_CURRENT_REGISTER==msg.payload.request.registerAddress){
            writeStoredRegisterResponseUnsigned(JBDRequest::CHARGE_OVER_CURRENT_REGISTER, 500);
        }
        else if(JBDRequest::DISCHARGE_OVER_CURRENT_REGISTER==msg.payload.request.registerAddress){
            writeStoredRegisterResponseUnsigned(JBDRequest::DISCHARGE_OVER_CURRENT_REGISTER, 500);
        }
        else if(JBDRequest::FUNCTIONAL_CONFIG_REGISTER==msg.payload.request.registerAddress){
            writeStoredRegisterResponseUnsigned(JBDRequest::FUNCTIONAL_CONFIG_REGISTER, 0x00);
        }
        else{
//             printf("UKN REGISTER %d", msg.payload.request.registerAddress);
            ESP_LOGE(TAG, "Unknown register %d", msg.payload.request.registerAddress);
            return;
        }
    }
    else if(JBDRequest::WRITE==msg.payload.request.operation){
//         uint8_t registerAddress=0x00;
        const uint8_t GENERIC_OK_RESPONSE[]="\xDD\x00\x00\x00\x00\x00\x77";
//         GENERIC_OK_RESPONSE[1]=registerAddress;
        writefully(STDOUT_FILENO, GENERIC_OK_RESPONSE, sizeof(GENERIC_OK_RESPONSE)-1);
    }
    else{
        ESP_LOGE(TAG, "Unknown operation %d", msg.payload.request.operation);
    }
}

void read_request_stdin_and_respond_stdout(){
    uart_set_baudrate(UART_NUM_0, 9600);
    ESP_LOGI(__FUNCTION__, "read_request_stdin_and_respond_stdout");

    uint8_t readBuffer[128];
    int readBufferLen=0;
    JBDParser parser;
    while(true){
        int nbBytesRead=read(STDIN_FILENO, readBuffer+readBufferLen, sizeof(readBuffer)-readBufferLen);
        if(nbBytesRead<=0){ //Nothing received
            vTaskDelay(300/portTICK_PERIOD_MS);
            continue;
        }
        readBufferLen+=nbBytesRead;
        
        while(readBufferLen>=7){
            JBDParseResult cmdToBMS;
            uint8_t const* parsedPosition=parser.parseBytesToBMS(readBuffer, readBufferLen, &cmdToBMS);
            int nbBytesParsed=parsedPosition-readBuffer;
            ESP_LOGI(TAG, "nbBytesParsed=%d", nbBytesParsed);

            readBufferLen-=nbBytesParsed;
            memmove(readBuffer, parsedPosition, readBufferLen);
                
            if(cmdToBMS.isSuccess){
                handle_message_to_bms(cmdToBMS);
            }
            else {
                ESP_LOGE(TAG, "Ignored some bytes because we failed to parse the message");
            }
        }
    }
}

void task_read_from_ble_bms(){
    JBDBLEStack* jbdBleStack=JBDBLEStack::getInstance();
    vTaskDelay(10000/portTICK_PERIOD_MS);
    while(true){
        for(uint8_t i=0; i<jbdBleStack->jbdControllersCount; i++){
            JBDConnection* conn=&jbdBleStack->jbdControllers[i];
            conn->requestCellVoltages();
            vTaskDelay(1000/portTICK_PERIOD_MS);
            conn->requestPackInfo();
            vTaskDelay(1000/portTICK_PERIOD_MS);
            conn->printState();
        }
    }
}

void configure_network_client_logging();
extern "C" void test_parser();

extern "C" void app_main(void){
    esp_log_level_set("*", ESP_LOG_ERROR);        // set all components to ERROR level
    esp_log_level_set("JBD_BLE", ESP_LOG_DEBUG);
    esp_log_level_set("NETWORK_LOGGING", ESP_LOG_DEBUG);
    esp_log_level_set("JDB_PARSER", ESP_LOG_DEBUG);
    
//     test_parser();
//     return;
    configure_network_client_logging();

    task_read_from_ble_bms();
    read_request_stdin_and_respond_stdout();
}

