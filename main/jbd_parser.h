
class BaseJBDFromBMS {
public:
    uint8_t registerAddress;
    uint8_t commandStatus;
};

class PackVoltage: public BaseJBDFromBMS{
};

class CellVoltages : public BaseJBDFromBMS {
public :
    uint16_t voltagesMv[4];
};

class JBDParseResult {
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
    void dump();
};

class JBDParser {
    const uint8_t START_BYTE=0xDD;
    const uint8_t END_BYTE=0x77;
    
public:
    static uint16_t computeChecksum(const uint8_t* inputBytes, const uint8_t inputBytesLen);
    static uint16_t parseUShort(const uint8_t* inputBytes);    
    static int16_t parseShort(const uint8_t* inputBytes);
    uint8_t const* parseBytesFromBMS(uint8_t const* inputBytes, const uint8_t inputBytesLen, JBDParseResult* result);
};
