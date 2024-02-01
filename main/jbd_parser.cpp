#include <cstring>
#include <ctype.h>

#include <esp_log.h>

#define TAG "JDB_PARSER"

uint8_t CMD_REQUEST_CELL_VOLTAGES_LEN=7;
uint8_t CMD_REQUEST_CELL_VOLTAGES[]={0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

uint8_t CMD_REQUEST_BASIC_INFO_LEN=7;
uint8_t CMD_REQUEST_BASIC_INFO[]={0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

class BaseJDBFromBMS {
public:
    uint8_t registerAddress;
    uint8_t commandStatus;
};

class PackVoltage: public BaseJDBFromBMS{
};

class CellVoltages : public BaseJDBFromBMS {
public :
    uint16_t voltagesMv[4];
};

class JDBParseResult {
public:
    enum {
        CELL_VOLTAGES,
        PAYLOAD_TYPES_MAX,
    } payloadTypes;
    
    union {
        PackVoltage packVoltage;
        CellVoltages cellVoltages;
    } payload;

    bool isSuccess;
    void dump(){
        ESP_LOGD(TAG, "Dump goes here");
    }
};

class JDBParser {
    const uint8_t START_BYTE=0xDD;
    const uint8_t END_BYTE=0x77;
    
public:
    static uint16_t computeChecksum(const uint8_t* inputBytes, const uint8_t inputBytesLen){
        uint16_t checksum=0;
        for(uint8_t i=0; i<inputBytesLen; i++){
            checksum+=inputBytes[i];
        }
        return (uint16_t)(0x10000U-checksum);
    }
    
    static uint16_t parseUShort(const uint8_t* inputBytes){
//         esp_log_buffer_hex(TAG, inputBytes, 2);
        uint16_t ret=(inputBytes[0]<<8)+inputBytes[1];
        return ret;
    }
    
    static int16_t parseShort(const uint8_t* inputBytes){
//         esp_log_buffer_hex(TAG, inputBytes, 2);
        int16_t ret=(inputBytes[0]<<8)+inputBytes[1];
        return ret;
    }

    bool parseBytesFromBMS(const uint8_t* inputBytes, const uint8_t inputBytesLen, JDBParseResult* result){
        if(NULL==result){
            return false;
        }
        result->isSuccess=false;
        
        if(inputBytesLen<7){
            return result->isSuccess;
        }
        uint8_t const* inputIt=inputBytes;
        if(START_BYTE!=inputIt[0]){
            return result->isSuccess;
        }
        inputIt++;
        
        
        if(0x03==inputIt[0]){ //Basic info
            inputIt++;
            uint8_t  commandStatus=inputIt[0]; inputIt++;
            if(0x80==commandStatus){
                ESP_LOGE(TAG, "Command error");
            }
            else{
                uint8_t  payloadLen=inputIt[0]; inputIt++;
                ESP_LOGD(TAG, "payloadLen=%d", payloadLen);
                uint16_t packVoltage_mV=parseUShort(inputIt); inputIt+=2;
                ESP_LOGD(TAG, "packVoltage_mV=%d", packVoltage_mV);
                int16_t packCurrent_mA=parseShort(inputIt); inputIt+=2;
                uint16_t balance_capacity_mAh=parseUShort(inputIt); inputIt+=2;
                uint16_t full_capacity_mAh=parseUShort(inputIt); inputIt+=2;
                uint16_t cycle_count=parseUShort(inputIt); inputIt+=2;
                uint16_t manufacture_date=parseUShort(inputIt); inputIt+=2;
                uint16_t cell_balance_status=parseUShort(inputIt); inputIt+=2;
                uint16_t cell_balance_status2=parseUShort(inputIt); inputIt+=2;
                uint16_t bitset_errors=parseUShort(inputIt); inputIt+=2;
                uint8_t softwareVersion=inputIt[0]; inputIt++;
                uint8_t state_of_charge=inputIt[0]; inputIt++;
                uint8_t fet_status=inputIt[0]; inputIt++;
                uint8_t cell_count=inputIt[0]; inputIt++;
                uint8_t temperature_sensor_count=inputIt[0]; inputIt++;
                ESP_LOGD(TAG, "temperature_sensor_count=%d", temperature_sensor_count);
                for(uint8_t i=0; i<temperature_sensor_count; i++){
                    uint8_t temperatures_deciK=parseUShort(inputIt); inputIt+=2;
                }
            }
        }
        else if(0x04==inputIt[0]){ //Cell voltages
            inputIt++;
            result->payload.cellVoltages.commandStatus=inputIt[0]; inputIt++;
            result->payloadTypes=JDBParseResult::CELL_VOLTAGES;
            result->payload.cellVoltages=CellVoltages();
            uint16_t nbCells=inputIt[0]/2;
            inputIt++;
            ESP_LOGD(TAG, "nbCells=%d", nbCells);
            for(uint16_t i=0; i<nbCells; i++){
                result->payload.cellVoltages.voltagesMv[i]=parseUShort(inputIt);
                inputIt+=2;
            }
        }
        else if(0x05==inputIt[0]){ //Hardware version
            inputIt++;
            uint8_t  commandStatus=inputIt[0]; inputIt++;
            if(0x80==commandStatus){
                ESP_LOGE(TAG, "Command error");
            }
            uint8_t deviceNameLen=inputIt[0]; inputIt++;
            const char* deviceName=(const char*) inputIt; inputIt+=deviceNameLen;
            ESP_LOGD(TAG, "device name %.*s", deviceNameLen, deviceName);
        }
        else{
            ESP_LOGW(TAG, "Unexpected payload type 0x%02X", inputIt[0]);
            return result->isSuccess;
        }
        const uint8_t payloadLen=inputIt-(inputBytes+2);
        const uint16_t computedChecksum=computeChecksum(inputBytes+2, payloadLen);
        const uint16_t inputChecksum=parseUShort(inputIt); inputIt+=2;
        if(computedChecksum!=inputChecksum){
            ESP_LOGW(TAG, "Expected checksum 0x%02X, but was 0x%02X", inputChecksum, computedChecksum);
        }
        
        if(END_BYTE!=inputIt[0]){
            ESP_LOGW(TAG, "Expected END_BYTE 0x%02X, but was 0x%02X", END_BYTE, inputIt[0]);
        }
        inputIt+=1;
        result->isSuccess=true;

        return result->isSuccess;
    }
};

extern "C" void test_parser(){
//     uint16_t cksum=computeChecksum(NULL, 0);
//     ESP_LOGE(TAG, "cksum=%d", cksum);
    //The BLE read pads the frame with nulls??
    const uint8_t FROM_BMS1[]="\xdd\x04\x00\x08\x0d\x2e\x0d\x2b\x0d\x2b\x0d\x2b\xff\x15\x77\x00\x00\x00\x00\x00";
    JDBParser parser;
    JDBParseResult msg;
    parser.parseBytesFromBMS(FROM_BMS1, sizeof(FROM_BMS1), &msg);
    for(int i=0; i<4; i++){
        ESP_LOGI(TAG, "Cell %d=%dmv", i, msg.payload.cellVoltages.voltagesMv[i]);
    }
    
    const uint8_t FROM_BMS_EXAMPLE1[]="\xDD\x03\x00\x1B\x17\x00\x00\x00\x02\xD0\x03\xE8\x00\x00\x20\x78\x00\x00\x00\x00\x00\x00\x10\x48\x03\x0F\x02\x0B\x76\x0B\x82\xFB\xFF\x77";
    JDBParseResult example1;
    parser.parseBytesFromBMS(FROM_BMS_EXAMPLE1, sizeof(FROM_BMS_EXAMPLE1), &example1);
    example1.dump();
    
    const uint8_t FROM_BMS_EXAMPLE2[]="\xDD\x05\x00\x0A\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\xFD\xE9\x77";
    JDBParseResult example2;
    parser.parseBytesFromBMS(FROM_BMS_EXAMPLE2, sizeof(FROM_BMS_EXAMPLE2), &example2);
}

extern "C" void handle_jbd_response(const uint8_t* inputBytes, const uint8_t inputBytesLen){
    JDBParser parser;
    JDBParseResult msg;
    parser.parseBytesFromBMS(inputBytes, inputBytesLen, &msg);
}
