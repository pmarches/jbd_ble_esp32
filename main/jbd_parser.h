
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

    static const uint8_t BASIC_INFO_REGISTER=0x03;
    static const uint8_t CELL_VOLTAGE_REGISTER=0x04;
    static const uint8_t DEVICE_NAME_REGISTER=0x05;
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
    uint8_t const* parseBytesFromBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result);
    uint8_t const* parseBytesToBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result);
};
