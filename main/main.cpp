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
#include <jbd_model.h>

#define TAG "JBD_MAIN"

void writefully(const int fd, const uint8_t* bytes, const size_t nbBytes){
    esp_log_buffer_hex(TAG, bytes, nbBytes);
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
#if 0            
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
#endif
            JBDParser::buildFromPackInfo(buildBytes, &buildBytesLen, &packInfo);
            writefully(STDOUT_FILENO, buildBytes, buildBytesLen);
        }
        else if(JBDRequest::CELL_VOLTAGE_REGISTER==msg.payload.request.registerAddress){
            JBDCellInfo cellInfo;
            gModel.getCellInfo(cellInfo);
            for(int i=0; i<cellInfo.cell_count; i++){
                ESP_LOGI(TAG, "Generated cell info cellInfo%d =%d", i, cellInfo.voltagesMv[i]);
            }
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

void read_request_stdin_and_respond_stdout_jbd_protocol(){
    uart_set_baudrate(UART_NUM_0, 9600);
    ESP_LOGI(TAG, "read_request_stdin_and_respond_stdout_jbd_protocol");

    uint8_t readBuffer[128];
    int readBufferLen=0;
    JBDParser parser;
    while(true){
        int nbBytesRead=read(STDIN_FILENO, readBuffer+readBufferLen, sizeof(readBuffer)-readBufferLen);
        if(nbBytesRead<=0){ //Nothing received
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        readBufferLen+=nbBytesRead;
        
        while(readBufferLen>=7){
            JBDParseResult cmdToBMS;
            uint8_t const* parsedPosition=parser.parseBytesToBMS(readBuffer, readBufferLen, &cmdToBMS);
            int nbBytesParsed=parsedPosition-readBuffer;
            ESP_LOGI(TAG, "nbBytesParsed from SerialBattery to our controller %d", nbBytesParsed);
            esp_log_buffer_hex(TAG, readBuffer, nbBytesParsed);

            readBufferLen-=nbBytesParsed;
            memmove(readBuffer, parsedPosition, readBufferLen);
                
            if(cmdToBMS.isSuccess){
                handle_message_to_bms(cmdToBMS);
                ESP_LOGI(TAG, "Done with message. Response should have been sent");
            }
            else {
                ESP_LOGE(TAG, "Ignored some bytes because we failed to parse the message");
            }
        }
    }
}

void readOneHexLine(uint8_t* hexLine, uint8_t* hexLineLen, time_t timeoutMS){
    const uint8_t START_MARKER=(uint8_t) ':';
    const uint8_t END_MARKER=(uint8_t) '\n';
    *hexLineLen=0;
    time_t elapsed=0;
    while(true){
        if(read(STDIN_FILENO, hexLine+*hexLineLen, 1)<=0){ //Nothing received
            const time_t READ_WAIT_MS=300;
            elapsed+=READ_WAIT_MS;
            if(elapsed>timeoutMS){
                ESP_LOGI(TAG, "Read timeout. Discarding %d bytes already read", *hexLineLen);
                *hexLineLen=0;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(READ_WAIT_MS));
            continue;
        }
        if(0==*hexLineLen && START_MARKER!=hexLine[0]){
            continue; //Wait for start marker
        }

        *hexLineLen=*hexLineLen+1;
        ESP_LOGI(TAG, "hexLineLen=%d. hexline '%*s'", *hexLineLen, *hexLineLen, hexLine);
        if(END_MARKER==hexLine[*hexLineLen-1]){
            ESP_LOGI(TAG, "Got the END_MARKER");
            break;
        }
    }
}

bool isHexLineChecksumValid(uint8_t* hexLine, uint8_t hexLineLen){
    return true;
}

void handleHexLine(uint8_t* hexLine, uint8_t hexLineLen){
}

void read_request_stdin_and_respond_stdout_hex_protocol(){
    ESP_LOGI(TAG, "read_request_stdin_and_respond_stdout_hex_protocol");

    const uint16_t VEDIRECT_BAUDRATE=19200;
    ESP_LOGI(TAG, "Setting baud rate to %d", VEDIRECT_BAUDRATE);
    uart_set_baudrate(UART_NUM_0, VEDIRECT_BAUDRATE);

    uint8_t hexLine[128];
    uint8_t hexLineLen=0;
    while(true){
        const char* ASYNC_MGS=":A0102000543\n";
        writefully(STDOUT_FILENO, (uint8_t*) ASYNC_MGS, strlen(ASYNC_MGS));
        memset(hexLine, 0, sizeof(hexLine));
        readOneHexLine(hexLine, &hexLineLen, 5000);
        if(false==isHexLineChecksumValid(hexLine, hexLineLen)){
            ESP_LOGE(TAG, "Hexline failed checksum : %.*s", hexLineLen, hexLine);
            esp_log_buffer_hex(TAG, hexLine, hexLineLen);
            continue;
        }
        handleHexLine(hexLine, hexLineLen);
//         buildHexLineResponse(hexLine
    }
}

class Countdown {
public:
    TickType_t periodTicks;
    TickType_t nextTicks;
    Countdown(uint32_t periodMS){
        this->periodTicks=pdMS_TO_TICKS(periodMS);
        this->nextTicks=xTaskGetTickCount()+periodTicks;
    }
    
    bool hasElapsed(){
        TickType_t now=xTaskGetTickCount();
        if(now>nextTicks){
            nextTicks=now+periodTicks;
            return true;
        }
        return false;
    }
};

void task_read_from_ble_bms(void* arg){
    JBDBLEStack* jbdBleStack=JBDBLEStack::getInstance();
    
    Countdown packInfoCD(1000);
    Countdown cellInfoCD(5000);
    while(true){
        for(uint8_t i=0; i<jbdBleStack->jbdControllersCount; i++){
            JBDConnection* conn=&jbdBleStack->jbdControllers[i];
            if(packInfoCD.hasElapsed()){
                conn->requestPackInfo();
                vTaskDelay(300/portTICK_PERIOD_MS);
            }
            if(cellInfoCD.hasElapsed()){
                conn->requestCellVoltages();
                vTaskDelay(300/portTICK_PERIOD_MS);
            }
            vTaskDelay(300/portTICK_PERIOD_MS);
        }
    }
}

void configure_network_client_logging();
void init_network();
void initialise_mdns(void);

extern "C" void test_parser();

extern "C" void app_main(void){
//     test_parser();
//     return;
    esp_log_level_set("*", ESP_LOG_WARN); // set all components level
    esp_log_level_set("JBD_BLE", ESP_LOG_DEBUG);
    esp_log_level_set("NETWORK_LOGGING", ESP_LOG_DEBUG);
    esp_log_level_set("JBD_PARSER", ESP_LOG_DEBUG);
    esp_log_level_set("JBD_MAIN", ESP_LOG_DEBUG);
    esp_log_level_set("JBD_MODEL", ESP_LOG_DEBUG);
    
    configure_network_client_logging();
    init_network();
    initialise_mdns();

    JBDBLEStack* jbdBleStack=JBDBLEStack::getInstance();
    jbdBleStack->waitForControllers();

    TaskHandle_t xHandle = NULL;
//     xTaskCreate(task_read_from_ble_bms, "NAME", 2048, NULL, tskIDLE_PRIORITY, &xHandle );
//     read_request_stdin_and_respond_stdout_jbd_protocol();
    read_request_stdin_and_respond_stdout_hex_protocol();
}

