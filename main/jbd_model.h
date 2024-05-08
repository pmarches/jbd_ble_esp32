#pragma once

#include <jbd_parser.h>

class AggregateBMSModel {
public:
    AggregateBMSModel();
    void getPackInfo(JBDPackInfo &packInfoOutput);
    void getDeviceName(char* outDeviceName);
    void getCellInfo(JBDCellInfo &cellInfoOutput);
};
