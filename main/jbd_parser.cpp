#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <cstring>
#include <ctype.h>
#include <stdio.h>

#include <esp_log.h>

#include <jbd_parser.h>

#define TAG "JBD_PARSER"

uint8_t CMD_REQUEST_BASIC_INFO_LEN=7;
uint8_t CMD_REQUEST_BASIC_INFO[]={0xDD, 0xA5, 0x05, 0x00, 0xFF, 0xFB, 0x77};

uint8_t CMD_REQUEST_PACK_INFO_LEN=7;
uint8_t CMD_REQUEST_PACK_INFO[]={0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

uint8_t CMD_REQUEST_CELL_VOLTAGES_LEN=7;
uint8_t CMD_REQUEST_CELL_VOLTAGES[]={0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

uint16_t JBDParser::computeChecksum(const uint8_t* inputBytes, const uint8_t inputBytesLen){
    uint16_t checksum=0;
    for(uint8_t i=0; i<inputBytesLen; i++){
        checksum+=inputBytes[i];
    }
    return (uint16_t)(0x10000U-checksum);
}

uint16_t JBDParser::parseUShort(const uint8_t* inputBytes){
//     esp_log_buffer_hex(TAG, inputBytes, 2);
    uint16_t ret=(inputBytes[0]<<8)+inputBytes[1];
//     ESP_LOGD(TAG, "parseUShort=%d", ret);
    return ret;
}

int16_t JBDParser::parseShort(const uint8_t* inputBytes){
//     esp_log_buffer_hex(TAG, inputBytes, 2);
    int16_t ret=(inputBytes[0]<<8)+inputBytes[1];
//     ESP_LOGD(TAG, "parseShort=%d", ret);
    return ret;
}

void JBDParser::buildUShort(uint8_t* dest, uint16_t unsignedValue){
    dest[0]=(unsignedValue & 0xFF00)>>8;
    dest[1]=(unsignedValue & 0xFF);
}

uint8_t const* JBDParser::parseBytesToBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result){
    esp_log_buffer_hex(TAG, inputBytes, inputBytesLen);
    result->isSuccess=false;
    if(NULL==result){
        return inputBytes;
    }

    result->isSuccess=true;
    uint8_t const* inputIt=inputBytes;

    uint8_t magic=inputIt[0]; inputIt++;
    if(magic!=START_BYTE){
        ESP_LOGE(TAG, "Wrong start byte. got 0x%02X", magic);
        result->isSuccess=false;
        return inputIt;
    }
    
    JBDRequest request;
    const uint8_t REQUEST_WRITE_BYTE=0x5A;
    request.operation=inputIt[0]==REQUEST_WRITE_BYTE?JBDRequest::WRITE : JBDRequest::READ; inputIt++;
    request.registerAddress=inputIt[0]; inputIt++;
    request.payloadLen=inputIt[0]; inputIt++;
    result->payloadTypes=JBDParseResult::REQUEST;
    inputIt+=request.payloadLen;
    
    const uint8_t nbBytesToCksum=inputIt-(inputBytes+2);
    const uint16_t computedChecksum=computeChecksum(inputBytes+2, nbBytesToCksum);
    const uint16_t inputChecksum=parseUShort(inputIt); inputIt+=2;
    if(computedChecksum!=inputChecksum){
        ESP_LOGW(TAG, "Expected checksum 0x%02X, but was 0x%02X", inputChecksum, computedChecksum);
        result->isSuccess=false;
        return inputIt;
    }

    magic=inputIt[0]; inputIt++;
    if(END_BYTE!=magic){
        ESP_LOGW(TAG, "Expected END_BYTE 0x%02X, but was 0x%02X", END_BYTE, magic);
        result->isSuccess=false;
        return inputIt;
    }
    
    
    result->payload.request=request;
    return inputIt;
}

uint8_t const* JBDParser::parseBytesFromBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result){
    esp_log_buffer_hex(TAG, inputBytes, inputBytesLen);
    if(NULL==result){
        return inputBytes;
    }
    result->isSuccess=false;
    
    if(inputBytesLen<7){
        return inputBytes;
    }
    uint8_t const* inputIt=inputBytes;
    if(START_BYTE!=inputIt[0]){
        return inputIt;
    }
    inputIt++;
        
    if(JBDRequest::BASIC_INFO_REGISTER==inputIt[0]){ //Basic info
        inputIt++;
        uint8_t  commandStatus=inputIt[0]; inputIt++;
        if(0x80==commandStatus){
            ESP_LOGE(TAG, "Command error");
        }
        else{
            uint8_t  payloadLen=inputIt[0]; inputIt++;
            if(4+payloadLen+2+1 >inputBytesLen){
                ESP_LOGW(TAG, "Incomplete message, not parsing it");
                return inputBytes;
            }
            
            JBDPackInfo packInfo;
            packInfo.packVoltage_cV=parseUShort(inputIt); inputIt+=2;
            ESP_LOGD(TAG, "packInfo.packVoltage_cV=%u", packInfo.packVoltage_cV);
            packInfo.packCurrent_cA=parseShort(inputIt); inputIt+=2;
            ESP_LOGD(TAG, "packInfo.packCurrent_cA=%d", packInfo.packCurrent_cA);
            packInfo.balance_capacity_mAh=parseUShort(inputIt); inputIt+=2;
            packInfo.full_capacity_mAh=parseUShort(inputIt); inputIt+=2;
            packInfo.cycle_count=parseUShort(inputIt); inputIt+=2;
            packInfo.manufacture_date=parseUShort(inputIt); inputIt+=2;
            packInfo.cell_balance_status=parseUShort(inputIt); inputIt+=2;
            packInfo.cell_balance_status2=parseUShort(inputIt); inputIt+=2;
            packInfo.bitset_errors=parseUShort(inputIt); inputIt+=2;
            packInfo.softwareVersion=inputIt[0]; inputIt++;
            packInfo.state_of_charge=inputIt[0]; inputIt++;
            packInfo.fet_status=inputIt[0]; inputIt++;
            packInfo.cell_count=inputIt[0]; inputIt++;
            packInfo.temperature_sensor_count=inputIt[0]; inputIt++;
            for(uint8_t i=0; i<packInfo.temperature_sensor_count; i++){
                packInfo.temperatures_deciK[i]=parseUShort(inputIt); inputIt+=2;
            }
            
            result->payloadTypes=JBDParseResult::PACK_INFO;
            result->payload.packInfo=packInfo;
        }
    }
    else if(JBDRequest::CELL_VOLTAGE_REGISTER==inputIt[0]){ //Cell voltages
        inputIt++;
        JBDCellInfo cellInfo;
        cellInfo.commandStatus=inputIt[0]; inputIt++;
        cellInfo.cell_count=inputIt[0]/2;
        inputIt++;
        ESP_LOGD(TAG, "cellInfo.cell_count=%d", cellInfo.cell_count);
        for(uint16_t i=0; i<cellInfo.cell_count; i++){
            cellInfo.voltagesMv[i]=parseUShort(inputIt); inputIt+=2;
            ESP_LOGD(TAG, "cell %d=%d", i, cellInfo.voltagesMv[i]);
        }

        result->payloadTypes=JBDParseResult::CELL_INFO;
        result->payload.cellInfo=cellInfo;
    }
    else if(JBDRequest::DEVICE_NAME_REGISTER==inputIt[0]){
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
        return inputBytes;
    }
    
    const uint8_t payloadLen=inputIt-(inputBytes+2);
    const uint16_t computedChecksum=computeChecksum(inputBytes+2, payloadLen);
    const uint16_t inputChecksum=parseUShort(inputIt); inputIt+=2;
    if(computedChecksum!=inputChecksum){
        ESP_LOGW(TAG, "Expected checksum 0x%02X, but was 0x%02X", inputChecksum, computedChecksum);
        return inputBytes;
    }
    
    if(END_BYTE!=inputIt[0]){
        ESP_LOGW(TAG, "Expected END_BYTE 0x%02X, but was 0x%02X", END_BYTE, inputIt[0]);
        return inputBytes;
    }
    inputIt+=1;
    result->isSuccess=true;

    return inputIt;
}

void JBDParser::buildStoredRegisterResponseUnsigned(uint8_t* responseBytes, uint8_t* responseBytesLen, uint8_t registerAddress, uint16_t unsignedValue){
    responseBytes[0]=JBDParser::START_BYTE;
    responseBytes[1]=registerAddress;
    responseBytes[2]=0x00; //OK
    responseBytes[3]=2; //16 bits
    buildUShort(responseBytes+4, unsignedValue);
    buildUShort(responseBytes+6, computeChecksum(responseBytes+2, 4));
    responseBytes[8]=JBDParser::END_BYTE;
    *responseBytesLen=9;
    esp_log_buffer_hex(TAG, responseBytes, *responseBytesLen);
}

void JBDParser::buildFromPackInfo(uint8_t* buildBytes, uint8_t* buildBytesLen, JBDPackInfo *packInfo){
    uint8_t* it=buildBytes;
    *it=START_BYTE; it++;
    *it=JBDRequest::BASIC_INFO_REGISTER; it++;
    *it=0; it++;//Command status

    *it=0x17+(2*packInfo->temperature_sensor_count);  it++; //PayloadLen=27
    buildUShort(it, packInfo->packVoltage_cV); it+=2;
    buildUShort(it, packInfo->packCurrent_cA); it+=2;
    buildUShort(it, packInfo->balance_capacity_mAh); it+=2;
    buildUShort(it, packInfo->full_capacity_mAh); it+=2;
    buildUShort(it, packInfo->cycle_count); it+=2;
    buildUShort(it, packInfo->manufacture_date); it+=2;
    buildUShort(it, packInfo->cell_balance_status); it+=2;
    buildUShort(it, packInfo->cell_balance_status2); it+=2;
    buildUShort(it, packInfo->bitset_errors); it+=2;
    *it=packInfo->softwareVersion; it++;
    *it=packInfo->state_of_charge; it++;
    *it=packInfo->fet_status; it++;
    *it=packInfo->cell_count; it++;
    *it=packInfo->temperature_sensor_count; it++;
    for(int i=0; i<packInfo->temperature_sensor_count; i++){
        buildUShort(it, packInfo->temperatures_deciK[i]); it+=2;
    }

    uint8_t nbBytesToChecksum=it-buildBytes-2;
    buildUShort(it, computeChecksum(buildBytes+2, nbBytesToChecksum)); it+=2;
    *it=END_BYTE; it++;
    *buildBytesLen=it-buildBytes;
}

void JBDParser::buildFromDeviceName(uint8_t* buildBytes, uint8_t* buildBytesLen, const char* deviceName){
    uint8_t* it=buildBytes;
    *it=START_BYTE; it++;
    *it=JBDRequest::DEVICE_NAME_REGISTER; it++;
    *it=0; it++;//Command status

    size_t deviceNameLen=strlen(deviceName);
    *it=(uint8_t) deviceNameLen;  it++; //PayloadLen
    strncpy((char*) it, deviceName, deviceNameLen+1); it+=deviceNameLen;

    uint8_t nbBytesToChecksum=it-buildBytes-2;
    buildUShort(it, computeChecksum(buildBytes+2, nbBytesToChecksum)); it+=2;
    *it=END_BYTE; it++;
    *buildBytesLen=it-buildBytes;
}

void JBDParser::buildFromCellInfo(uint8_t* buildBytes, uint8_t* buildBytesLen, JBDCellInfo *cellInfo){
    uint8_t* it=buildBytes;
    *it=START_BYTE; it++;
    *it=JBDRequest::DEVICE_NAME_REGISTER; it++;
    *it=0; it++;//Command status

    *it=2*cellInfo->cell_count;  it++; //PayloadLen
    for(int i=0; i<cellInfo->cell_count; i++){
        buildUShort(it, cellInfo->voltagesMv[i]); it+=2;
    }

    uint8_t nbBytesToChecksum=it-buildBytes-2;
    buildUShort(it, computeChecksum(buildBytes+2, nbBytesToChecksum)); it+=2;
    *it=END_BYTE; it++;
    *buildBytesLen=it-buildBytes;
}


void JBDPackInfo::printJSON() const {
    printf("{\n");
    printf("  \"packVoltage_cV\": %u,\n", packVoltage_cV);
    printf("  \"packCurrent_cA\": %d,\n", packCurrent_cA);
    printf("  \"balance_capacity_mAh\": %u,\n", balance_capacity_mAh);
    printf("  \"full_capacity_mAh\": %u,\n", full_capacity_mAh);
    printf("  \"cycle_count\": %u,\n", cycle_count);
    printf("  \"manufacture_date\": %u,\n", manufacture_date);
    printf("  \"cell_balance_status\": %u,\n", cell_balance_status);
    printf("  \"cell_balance_status2\": %u,\n", cell_balance_status2);
    printf("  \"bitset_errors\": %u,\n", bitset_errors);
    printf("  \"softwareVersion\": %u,\n", softwareVersion);
    printf("  \"state_of_charge\": %u,\n", state_of_charge);
    printf("  \"fet_status\": %u,\n", fet_status);
    printf("  \"cell_count\": %u,\n", cell_count);
    printf("  \"temperature_sensor_count\": %u,\n", temperature_sensor_count);
    printf("  \"temperatures_deciK\": [%u, %u]\n", temperatures_deciK[0], temperatures_deciK[1]);
    printf("}\n");
}

void assertEquals(const char* tag, const uint8_t* gotBytes, size_t gotBytesLen, const uint8_t* expectedBytes, size_t expectedBytesLen){
    if(gotBytesLen!=expectedBytesLen){
        ESP_LOGE(TAG, "%s expected %d got %d", tag, expectedBytesLen, gotBytesLen);
    }
    if(memcmp(expectedBytes, gotBytes, expectedBytesLen)){
        ESP_LOGE(TAG, "%s has differing content", tag);
        esp_log_buffer_hex("EXPECTED", expectedBytes, expectedBytesLen);
        esp_log_buffer_hex("     GOT", gotBytes, gotBytesLen);
    }
}

extern "C" void test_parser(){
//     uint16_t cksum=computeChecksum(NULL, 0);
//     ESP_LOGE(TAG, "cksum=%d", cksum);
    //The BLE read pads the frame with nulls??
    const uint8_t FROM_BMS1[]="\xdd\x04\x00\x08\x0d\x2e\x0d\x2b\x0d\x2b\x0d\x2b\xff\x15\x77\x00\x00\x00\x00\x00";
    JBDParser parser;
    JBDParseResult msg;
    parser.parseBytesFromBMS(FROM_BMS1, sizeof(FROM_BMS1), &msg);
//     for(int i=0; i<4; i++){
//         ESP_LOGI(TAG, "Cell %d=%dmv", i, msg.payload.cellVoltages.voltagesMv[i]);
//     }
    
    const uint8_t SERIAL_STARTER1[]="\xdd\xa5\x03\x00\xff\xfd\x77";
    parser.parseBytesToBMS(SERIAL_STARTER1, sizeof(SERIAL_STARTER1), &msg);
    assert(msg.isSuccess);
    
    const uint8_t BASIC_INFO_FROM_BMS_EXAMPLE1[]="\xDD\x03\x00\x1B\x17\x00\x00\x00\x02\xD0\x03\xE8\x00\x00\x20\x78\x00\x00\x00\x00\x00\x00\x10\x48\x03\x0F\x02\x0B\x76\x0B\x82\xFB\xFF\x77";
    JBDParseResult example1;
    parser.parseBytesFromBMS(BASIC_INFO_FROM_BMS_EXAMPLE1, sizeof(BASIC_INFO_FROM_BMS_EXAMPLE1)-1, &example1);
    
    uint8_t buildBytes[128];
    uint8_t buildBytesLen=0;
    parser.buildFromPackInfo(buildBytes, &buildBytesLen, &example1.payload.packInfo);
    assertEquals("BASIC_INFO_FROM_BMS_EXAMPLE1", buildBytes, buildBytesLen, BASIC_INFO_FROM_BMS_EXAMPLE1, sizeof(BASIC_INFO_FROM_BMS_EXAMPLE1)-1);    
    
    const uint8_t FROM_BMS_EXAMPLE2[]="\xDD\x05\x00\x0A\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\xFD\xE9\x77";
    JBDParseResult example2;
    parser.parseBytesFromBMS(FROM_BMS_EXAMPLE2, sizeof(FROM_BMS_EXAMPLE2)-1, &example2);
    
    //Notice how these bytes are NOT null terminated, hence sizeof() is one less than the other examples
    const uint8_t BASIC_INFO_FROM_BMS_EXAMPLE3[]={0xdd, 0x03, 0x00, 0x1b, 0x05, 0x2b, 0xfd, 0x4f, 0x3e, 0x22, 0x4e, 0x20, 0x00, 0x85, 0x2c, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x50, 0x03, 0x04, 0x02, 0x0b, 0xd7, 0x0b, 0xdf, 0xfa, 0x58, 0x77};
    JBDParseResult example3;
    const uint8_t* end=parser.parseBytesFromBMS(BASIC_INFO_FROM_BMS_EXAMPLE3, sizeof(BASIC_INFO_FROM_BMS_EXAMPLE3), &example3);
    assert(example3.isSuccess);
    if(end!=BASIC_INFO_FROM_BMS_EXAMPLE3+sizeof(BASIC_INFO_FROM_BMS_EXAMPLE3)){
        ESP_LOGE(TAG, "endPtr=%p should be %p", end, BASIC_INFO_FROM_BMS_EXAMPLE3+sizeof(BASIC_INFO_FROM_BMS_EXAMPLE3));
    }    
//     example3.payload.packInfo.printJSON();
    JBDPackInfo packInfo;
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
    parser.buildFromPackInfo(buildBytes, &buildBytesLen, &packInfo);
    assertEquals("BASIC_INFO_FROM_BMS_EXAMPLE3", buildBytes, buildBytesLen, BASIC_INFO_FROM_BMS_EXAMPLE3, sizeof(BASIC_INFO_FROM_BMS_EXAMPLE3));    

    JBDParseResult example4;
    parser.parseBytesToBMS(CMD_REQUEST_BASIC_INFO, CMD_REQUEST_BASIC_INFO_LEN, &example4);
}

extern "C" void handle_jbd_response(const uint8_t* inputBytes, const uint8_t inputBytesLen){
    JBDParser parser;
    JBDParseResult msg;
    parser.parseBytesFromBMS(inputBytes, inputBytesLen, &msg);
}

