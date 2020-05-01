#include "MDR32Fx.h"
#include "defines.h"
#include "inits.h"
#include "math.h"
#include "L2G2IS.h"
#include "MPU-60x0.h"
#include "SPI.h"
#include "sensor_inits.h"
#include "CAN.h"
#include "SysTick.h"
#include "PID.h"

#define M_PI 3.14159265

float pwmden = 0.1;
bool serialInitSuccess = true;
float omega;
float angle = 0.0;
struct PID_t PID = {0, 0.01, 0, 0, 0};


int sign(float val) {
    if (val > 0) {
        return 1;
    }
    return -1;
}

float clip(float val, float thresh) {
    return fmin(fabs(val), thresh) * sign(val);
}

int main() {
    int del = 0;
    char data;
    
    initHSE();
    initMotorPWM();
    initSPI();
    initCS1();
    initCS2();
    initCS3();
    for (del = 0; del < 8000000; del++);  
    serialInitSuccess &= initL2G2IS();
    serialInitSuccess &= initMPU_60x0();
    serialInitSuccess &= initLSM6DS33();
    CAN1init();
    if (serialInitSuccess) {
        initDRV1_EN();//to light a LED
        initDRV2_EN();
        initSysTick();
        while (1) {
            omega = -(int16_t)((L2Gread(L2G_OUT_X_H) << 8 ) | L2Gread(L2G_OUT_X_L)) * 0.00763;//deg/sec
            int tick_elapsed = SysTickGetElapsed();
            float sec_elapsed = tick_elapsed / (float)CPU_FREQ;
            angle += omega *  sec_elapsed;
            float torq = update(&PID, angle, sec_elapsed);
            PWM_SIN_CCR = PWM_CCR_MAX * (sin((float)CNT_VAL / CNT_VAL_MAX * 2 * M_PI) + 1) / 2;
            PWM_COS_CCR = PWM_CCR_MAX * (clip(torq, 1) * cos((float)CNT_VAL / CNT_VAL_MAX * 2 * M_PI) + 1) / 2;
            static int candel = 0;
            candel++;
            if (candel == 100) {
                //CANwrite(*(int*)(&angle), 0);
                candel = 0;
            }
        }
    }
    
}

void regset(int* reg, int val, int msk) {
    int b = *reg;
    b |= b & (val | ~msk);
    *reg = b;
}

void CAN1_IRQHandler() {
    char reqid;
    bool iswrite;
    
    MDR_CAN1->BUF_CON[1] &= ~(1 << CAN_BUF_CON_RX_FULL_Pos);
    reqid = MDR_CAN1->CAN_BUF[1].DATAL;
    iswrite = (reqid & 128) > 0;
    reqid &= 127;
    switch (reqid) {
    case 0://sum
        if (iswrite) {
            (int)(PID.sum_sens) = MDR_CAN1->CAN_BUF[1].DATAH;
        }
        CANwrite(*(int*)(&(PID.sum_sens)), 0);
        break;
    case 1://lin
        if (iswrite) {
            (int)(PID.lin_sens) = MDR_CAN1->CAN_BUF[1].DATAH;
        }
        CANwrite(*(int*)(&(PID.lin_sens)), 0);
        break;
    }
}

bool flashtest() {
#define EEPROM_KEY 0x8AAA5551
    int i;
    uint32_t data = 0;
#define flush(cnt) for (i = 0; i < cnt; ++i)
    MDR_RST_CLK->PER_CLOCK |= 1 << 3;
    flush(10000);
 /*   //ifren =1
    //eeprom_adr = 0x08000000
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_NVSTR_Pos |
                      1 << EEPROM_CMD_PROG_Pos |
                      1 << EEPROM_CMD_IFREN_Pos |
                      1 << EEPROM_CMD_SE_Pos |
                      1 << EEPROM_CMD_YE_Pos |
                      1 << EEPROM_CMD_XE_Pos;
*/
//reading attempt
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_IFREN_Pos;
    MDR_EEPROM->ADR = 0x08000000;
    MDR_EEPROM->KEY = EEPROM_KEY;
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_PROG_Pos |
                       1 << EEPROM_CMD_XE_Pos |
                       1 << EEPROM_CMD_YE_Pos |
                       1 << EEPROM_CMD_SE_Pos;
    flush(3);
    data = MDR_EEPROM->DO;
    MDR_EEPROM->CMD &= !(1 << EEPROM_CMD_PROG_Pos);
    flush(500);
    MDR_EEPROM->CMD &= !(1 << EEPROM_CMD_XE_Pos |
                         1 << EEPROM_CMD_YE_Pos |
                         1 << EEPROM_CMD_SE_Pos);
    flush(100);    
    if (data == 0xABCDEF) {
        return true;
    }
//writing attempt
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_IFREN_Pos;
    MDR_EEPROM->ADR = 0x08000000;
    MDR_EEPROM->DI = 0xABCDEF;
    MDR_EEPROM->KEY = EEPROM_KEY;
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_PROG_Pos |
                       1 << EEPROM_CMD_XE_Pos;
    flush(500);
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_NVSTR_Pos;
    flush(1000);
    MDR_EEPROM->CMD |= 1 << EEPROM_CMD_YE_Pos;
    flush(4000);
    MDR_EEPROM->CMD &= !(1 << EEPROM_CMD_YE_Pos);
    flush(2);
    MDR_EEPROM->CMD &= !(1 << EEPROM_CMD_PROG_Pos);
    flush(500);
    MDR_EEPROM->CMD &= !(1 << EEPROM_CMD_XE_Pos |
                         1 << EEPROM_CMD_NVSTR_Pos |
                         1 << EEPROM_CMD_IFREN_Pos);
    flush(100);
    return false;
}

