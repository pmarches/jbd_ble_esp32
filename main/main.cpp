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

#define TAG "JBD_MAIN"

extern "C" void handle_jbd_response(const uint8_t* inputBytes, const uint8_t inputBytesLen);

extern "C" void test_parser();
// #include <sys/select.h>

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
    JBDParser parser;
    parser.buildStoredRegisterResponseUnsigned(responseBytes, &responseBytesLen, registerAddress, unsignedValue);    
    writefully(STDOUT_FILENO, responseBytes, responseBytesLen);
}

//These message come from the GX
void handle_message_to_bms(const JBDParseResult& msg){
    if(JBDParseResult::REQUEST!=msg.payloadTypes){
        ESP_LOGE(TAG, "msg.payloadTypes=%d not yet supported", msg.payloadTypes);
        return;
    }
    if(JBDRequest::READ==msg.payload.request.operation){
        if(JBDRequest::BASIC_INFO_REGISTER==msg.payload.request.registerAddress){
            const uint8_t BASIC_INFO_RESPONSE[]="\xdd\x03\x00\x1b\x05\x2b\xfd\x4f\x3e\x22\x4e\x20\x00\x85\x2c\x56\x00\x00\x00\x00\x00\x00\x17\x50\x03\x04\x02\x0b\xd7\x0b\xdf\xfa\x58\x77";
            writefully(STDOUT_FILENO, BASIC_INFO_RESPONSE, sizeof(BASIC_INFO_RESPONSE)-1);
        }
        else if(JBDRequest::CELL_VOLTAGE_REGISTER==msg.payload.request.registerAddress){
            const uint8_t CELL_VOLTAGE_RESPONSE[]="\xdd\x04\x00\x08\x0d\x2e\x0d\x2b\x0d\x2b\x0d\x2b\xff\x15\x77";
            writefully(STDOUT_FILENO, CELL_VOLTAGE_RESPONSE, sizeof(CELL_VOLTAGE_RESPONSE)-1);
        }
        else if(JBDRequest::DEVICE_NAME_REGISTER==msg.payload.request.registerAddress){
            const uint8_t DEVICE_NAME_RESPONSE[]="\xdd\x05\x00\x09\x41\x42\x43\x44\x45\x46\x47\x48\x49\xFD\x8A\x77";
            writefully(STDOUT_FILENO, DEVICE_NAME_RESPONSE, sizeof(DEVICE_NAME_RESPONSE)-1);
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
#if 0
    while(true){
        ESP_LOGI("TEST", "read_request_stdin_and_respond_stdout");
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
#endif

    uint8_t readBuffer[128];
    int readBufferLen=0;
//     fd_set readfds;
//     FD_ZERO(&readfds);
//     FD_SET(STDIN_FILENO, &readfds);
    JBDParser parser;
    while(true){
//         int rc=select(STDIN_FILENO+1, &readfds, NULL, NULL, NULL);
//         if(rc<0){
//             perror("select returned an error\n");
//             vTaskDelay(300/portTICK_PERIOD_MS);
//             continue;
//         }
        
        int nbBytesRead=read(STDIN_FILENO, readBuffer+readBufferLen, sizeof(readBuffer)-readBufferLen);
        if(nbBytesRead>0){
            JBDParseResult cmdToBMS;
            readBufferLen+=nbBytesRead;
            uint8_t const* parsedPosition=parser.parseBytesToBMS(readBuffer, readBufferLen, &cmdToBMS);
            int nbBytesParsed=parsedPosition-readBuffer;
//             printf("nbBytesRead=%d nbBytesParsed=%d\n", nbBytesRead, nbBytesParsed);
            if(nbBytesParsed){
                memmove(readBuffer, parsedPosition, nbBytesParsed);
                readBufferLen-=nbBytesParsed;
                
                if(false==cmdToBMS.isSuccess){
                    ESP_LOGE(TAG, "Result shows failure");
                    continue;
                }
                handle_message_to_bms(cmdToBMS);
            }
        }
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}

void configure_network_client_logging();

extern "C" void app_main(void){
     configure_network_client_logging();
//     test_parser();
//     return;
#if 0    
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
#endif
    read_request_stdin_and_respond_stdout();
}

