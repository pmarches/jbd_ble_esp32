#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <jbd_model.h>
#include <jbd_ble.h>
#include <esp_log.h>

#define TAG "JBD_MODEL"

AggregateBMSModel::AggregateBMSModel(){
}

void AggregateBMSModel::getPackInfo(JBDPackInfo &packInfoOutput){
    uint32_t packVoltage_cV=0; //Average
    int32_t packCurrent_cA=0;
    uint32_t full_capacity_mAh=0;
    uint32_t balance_capacity_mAh=0;
    uint16_t cell_count=0;
    uint16_t state_of_charge=0;
    
    packInfoOutput.cycle_count=0;
    packInfoOutput.temperature_sensor_count=0;
    packInfoOutput.bitset_errors=0;
    packInfoOutput.cell_balance_status=0;
    packInfoOutput.cell_balance_status2=0;
    
    JBDBLEStack* jbdBleStack = JBDBLEStack::getInstance();
    for(uint8_t i=0; i<jbdBleStack->jbdControllersCount; i++){
        JBDConnection* conn=&jbdBleStack->jbdControllers[i];
        if(i==0){
            packInfoOutput=conn->packInfo; //Init non-agregate fields
        }

        state_of_charge+=conn->packInfo.state_of_charge;
        packVoltage_cV+=conn->packInfo.packVoltage_cV;
        packCurrent_cA+=conn->packInfo.packCurrent_cA;
        full_capacity_mAh+=conn->packInfo.full_capacity_mAh;
        balance_capacity_mAh+=conn->packInfo.balance_capacity_mAh;
        cell_count+=conn->packInfo.cell_count;
        
        packInfoOutput.cycle_count+=conn->packInfo.cycle_count;
        packInfoOutput.manufacture_date=conn->packInfo.manufacture_date;
        packInfoOutput.cell_balance_status=conn->packInfo.cell_balance_status>>i*4;
//         packInfoOutput.cell_balance_status2=conn->packInfo.cell_balance_status2>>i*4;
        packInfoOutput.bitset_errors|=conn->packInfo.bitset_errors;
        packInfoOutput.softwareVersion=conn->packInfo.softwareVersion;
        packInfoOutput.fet_status=conn->packInfo.fet_status;
        packInfoOutput.temperature_sensor_count+=conn->packInfo.temperature_sensor_count;
        for(int j=0; j<conn->packInfo.temperature_sensor_count; j++){
            packInfoOutput.temperatures_deciK[i*2+j]=conn->packInfo.temperatures_deciK[j];
        }
    }
    
    packInfoOutput.state_of_charge=state_of_charge/jbdBleStack->jbdControllersCount;
    packInfoOutput.packVoltage_cV=packVoltage_cV/jbdBleStack->jbdControllersCount;
    packInfoOutput.packCurrent_cA=packCurrent_cA;
    packInfoOutput.full_capacity_mAh=full_capacity_mAh;
    packInfoOutput.balance_capacity_mAh=balance_capacity_mAh;
    packInfoOutput.cell_count=cell_count;
}

void AggregateBMSModel::getDeviceName(char* outDeviceName){
    strcpy(outDeviceName, "LiFePO4");
}

void AggregateBMSModel::getCellInfo(JBDCellInfo &cellInfoOutput){
    uint8_t allCellIdx=0;
    cellInfoOutput.cell_count=0;
    for(uint8_t i=0; i<JBDBLEStack::getInstance()->jbdControllersCount; i++){
        JBDConnection* conn=&JBDBLEStack::getInstance()->jbdControllers[i];
        cellInfoOutput.cell_count+=conn->packInfo.cell_count;
        for(uint8_t j=0; j<conn->packInfo.cell_count; j++){
            ESP_LOGD(TAG, "%s->cellInfo.voltagesMv[%d]=%d", conn->deviceName, j, conn->cellInfo.voltagesMv[j]);
            cellInfoOutput.voltagesMv[allCellIdx]=conn->cellInfo.voltagesMv[j];
            allCellIdx++;
        }
    }
}
