
class BaseJBDFromBMS {
public:
    uint8_t registerAddress;
    uint8_t commandStatus;
};

class JBDBasicInfo: public BaseJBDFromBMS{
public:
};

class JBDCellInfo : public BaseJBDFromBMS {
public :
    uint16_t voltagesMv[4];
};

class JBDPackInfo {
public:
    uint16_t packVoltage_cV;
    int16_t packCurrent_mA;
    uint16_t balance_capacity_mAh;
    uint16_t full_capacity_mAh;
    uint16_t cycle_count;
    uint16_t manufacture_date;
    uint16_t cell_balance_status;
    uint16_t cell_balance_status2;
    uint16_t bitset_errors;
    uint8_t softwareVersion;
    uint8_t state_of_charge;
    uint8_t fet_status;
    uint8_t cell_count;
    uint8_t temperature_sensor_count;
    uint8_t temperatures_deciK[2];

    void printJSON() const;
};

class JBDRequest {
public:
    enum {
        READ,
        WRITE,
    } operation;

    static const uint8_t ENTER_FACTORY_MODE_REGISTER=0x00;
    static const uint8_t EXIT_FACTORY_MODE_REGISTER=0x01;
    static const uint8_t BASIC_INFO_REGISTER=0x03;
    static const uint8_t CELL_VOLTAGE_REGISTER=0x04;
    static const uint8_t DEVICE_NAME_REGISTER=0x05;
    
    static const uint8_t CYCLE_CAP_REGISTER=0x11;
    static const uint8_t CHARGE_OVER_CURRENT_REGISTER=0x28;
    static const uint8_t DISCHARGE_OVER_CURRENT_REGISTER=0x29;
    static const uint8_t FUNCTIONAL_CONFIG_REGISTER=0x2D;
    
    uint8_t registerAddress;

    uint8_t payloadLen;
};

class JBDParseResult {
public:
    enum {
        BASIC_INFO,
        PACK_INFO,
        CELL_INFO,
        REQUEST,
        PAYLOAD_TYPES_MAX,
    } payloadTypes;
    
    union {
        JBDBasicInfo basicInfo;
        JBDPackInfo packInfo;
        JBDCellInfo cellInfo;
        JBDRequest request;
    } payload;

    bool isSuccess;
};

class JBDParser {
    const uint8_t START_BYTE=0xDD;
    const uint8_t END_BYTE=0x77;
    
public:
    static uint16_t computeChecksum(const uint8_t* inputBytes, const uint8_t inputBytesLen);
    static uint16_t parseUShort(const uint8_t* inputBytes);    
    static int16_t parseShort(const uint8_t* inputBytes);
    static void renderUShort(uint8_t* dest, uint16_t unsignedValue){
        dest[0]=(unsignedValue & 0xFF00)>>8;
        dest[1]=(unsignedValue & 0xFF);
    }
    uint8_t const* parseBytesFromBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result);
    uint8_t const* parseBytesToBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result);
    
    void buildStoredRegisterResponseUnsigned(uint8_t* responseBytes, uint8_t* responseBytesLen, uint8_t registerAddress, uint16_t unsignedValue){
        responseBytes[0]=JBDParser::START_BYTE;
        responseBytes[1]=registerAddress;
        responseBytes[2]=0x00; //OK
        responseBytes[3]=2; //16 bits
        renderUShort(responseBytes+4, unsignedValue);
        renderUShort(responseBytes+6, computeChecksum(responseBytes+2, 4));
        responseBytes[8]=JBDParser::END_BYTE;
        *responseBytesLen=9;
        esp_log_buffer_hex(__FUNCTION__, responseBytes, *responseBytesLen);
    }
};
