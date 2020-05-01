#pragma once

#include "SPI.h"
#include "inits.h"
#include "math.h"

#define M_PI 3.14159265
#define SPI_CS_DEL_CNT 10

char L2Gread(char addr) {
    char buf;
    SET_CS1_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    buf = SPIread(addr);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS1_HI;
    return buf;
}

void L2Gread_mult(char addr, char cnt, char* data_arr) {
    SET_CS1_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIread_mult(addr, cnt, data_arr);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS1_HI;
}

void L2Gwrite(char addr, char val) {
    SET_CS1_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIwrite(addr, val);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS1_HI;
}

char MPUread(char addr) {
    char buf;
    SET_CS3_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    buf = SPIread(addr);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS3_HI;
    return buf;
}

void MPUread_mult(char addr, char cnt, char* data_arr) {
    SET_CS3_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIread_mult(addr, cnt, data_arr);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS3_HI;
}

void MPUwrite(char addr, char val) {
    SET_CS3_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIwrite(addr, val);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS3_HI;
}

char LSMread(char addr) {
    char buf;
    SET_CS2_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    buf = SPIrw((1 << 7) | addr, 0);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS2_HI;
    return buf;
}

void LSMread_mult(char addr, char cnt, char* data_arr) {
    SET_CS2_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIrw_mult(SPI_READ_FLAG | addr, cnt, data_arr);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS2_HI;
}

void LSMwrite(char addr, char val) {
    SET_CS2_LOW;
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SPIrw(addr, val);
    for (char i = 0; i < SPI_CS_DEL_CNT; i++);
    SET_CS2_HI;
}

int sign(float val) {
    if (val > 0) {
        return 1;
    }
    return -1;
}

float clip(float val, float thresh) {
    return fmin(fabs(val), thresh) * sign(val);
}

void setMotorTorq(float val) {
    PWM_SIN_CCR = PWM_CCR_MAX * (sin((float)CNT_VAL / CNT_VAL_MAX * 2 * M_PI) + 1) / 2;
    PWM_COS_CCR = PWM_CCR_MAX * (clip(val, 1) * cos((float)CNT_VAL / CNT_VAL_MAX * 2 * M_PI) + 1) / 2;
}

void setMotor1Torq(float val) {
    MDR_TIMER3->CCR3 = (PWM_CCR_MAX * (1 + val)) / 2;
}

void setMotor2Torq(float val) {
    MDR_TIMER3->CCR4 = (PWM_CCR_MAX * (1 + val)) / 2;
}

void CANsetInt(bool state) {
#define CAN_INT_ENABLE true
#define CAN_INT_DISABLE false
    MDR_CAN1->INT_EN = MDR_CAN1->INT_EN &  ~(1 << CAN_INT_EN_GLB_INT_EN_Pos) | (state << CAN_INT_EN_GLB_INT_EN_Pos);
}

#undef SPI_CS_DEL_CNT