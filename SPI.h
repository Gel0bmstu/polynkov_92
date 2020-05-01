#pragma once
#include "defines.h"

char SPIread(char addr) {
    char dummy;
    SPI->DR = (addr | (1 << 7)) << 8;
    while(SPI->SR & SSP_SR_BSY);
    while (SPI->SR & SSP_SR_RNE) {
        dummy = SPI->DR;
    }
    return dummy;
}

#define SPI_READ_FLAG 0x80  //(1 << 7)

char SPIrw(char addr, char val) {
    char dummy;
    SPI->DR = (addr << 8) | val;
    while(SPI->SR & SSP_SR_BSY);
    while (SPI->SR & SSP_SR_RNE) {
        dummy = SPI->DR;
    }
    return dummy;
}

void SPIrw_mult(char addr, char cnt, char* array) {
    char dummy;
    int dss_bckp = MDR_SSP1->CR0 & 0xF;
    MDR_SSP1->CR0 &= 0xFFF0;
    MDR_SSP1->CR0 |= 7 << SSP_CR0_DSS_Pos;
    SPI->DR = addr;
    while(SPI->SR & SSP_SR_BSY);
    while (SPI->SR & SSP_SR_RNE) {
           dummy = SPI->DR;
        }
    for (char i = 0; i < cnt; ++i) {
        SPI->DR = 0;
        while(SPI->SR & SSP_SR_BSY);
        while (SPI->SR & SSP_SR_RNE) {
            array[i] = SPI->DR;
        }
    }
    MDR_SSP1->CR0 &= 0xFFF0;
    MDR_SSP1->CR0 |= dss_bckp;
}

void SPIread_mult(char addr, char cnt, char* array) {
    char dummy;
    int dss_bckp = MDR_SSP1->CR0 & 0xF;
    MDR_SSP1->CR0 &= 0xFFF0;
    MDR_SSP1->CR0 |= 7 << SSP_CR0_DSS_Pos;
    SPI->DR = (addr | (1 << 7) | (1 << 6));
    while(SPI->SR & SSP_SR_BSY);
    while (SPI->SR & SSP_SR_RNE) {
            dummy = SPI->DR;
        }
    for (char i = 0; i < cnt; ++i) {
        SPI->DR = 0;
        while(SPI->SR & SSP_SR_BSY);
        while (SPI->SR & SSP_SR_RNE) {
            array[i] = SPI->DR;
        }
    }
    MDR_SSP1->CR0 &= 0xFFF0;
    MDR_SSP1->CR0 |= dss_bckp;
}

void SPIflushRX() {
    char dummy;
    while (SPI->SR & SSP_SR_RNE) {
        dummy = SPI->DR;
    }
}

void SPIflushTX() {
    while (SPI->SR & SSP_SR_BSY);
}

void SPIwrite(char addr, char val) {
    int del;
    for (del = 0; del < 10; del++);
    SPI->DR = (addr << 8) | val;
    while (SPI->SR & SSP_SR_BSY);
}
