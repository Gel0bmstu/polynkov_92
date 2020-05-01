#pragma once

#include "MDR32Fx.h"
#include "defines.h"
#include "stdbool.h"

void initMotorPWM() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 14) |  //TIM1 - Counter
                              (1 << 15) |  //TIM2 - PWM
                              (1 << 21);   //PORTA
    MDR_RST_CLK->TIM_CLOCK |= 1 << RST_CLK_TIM_CLOCK_TIM1_CLK_EN_Pos |
                              1 << RST_CLK_TIM_CLOCK_TIM2_CLK_EN_Pos;
    
    PWM_TIM->CNT = 0;
    PWM_TIM->CH1_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    PWM_TIM->CH1_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    PWM_TIM->CH1_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;    
    PWM_TIM->CH2_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    PWM_TIM->CH2_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    PWM_TIM->CH2_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;
    PWM_TIM->ARR = PWM_CCR_MAX;
    
    CNT_VAL = 0;
    CNT_TIM->ARR = CNT_VAL_MAX;
    CNT_TIM->PSG = 3;
    
    MDR_PORTA->FUNC |= 3 << PORT_FUNC_MODE1_Pos |
                       3 << PORT_FUNC_MODE2_Pos |
                       3 << PORT_FUNC_MODE3_Pos |
                       3 << PORT_FUNC_MODE4_Pos;
    MDR_PORTA->OE |= 1 << 1 |
                     1 << 2 |
                     1 << 3 |
                     1 << 4;
    MDR_PORTA->ANALOG |= 1 << 1 |
                         1 << 2 |
                         1 << 3 |
                         1 << 4;
    MDR_PORTA->PWR |= 3 << PORT_PWR1_Pos |
                      3 << PORT_PWR2_Pos |
                      3 << PORT_PWR3_Pos |
                      3 << PORT_PWR4_Pos;
    
    PWM_TIM->CNTRL |= 1 << TIMER_CNTRL_CNT_EN_Pos;
    CNT_TIM->CNTRL |= 1 << TIMER_CNTRL_CNT_EN_Pos;
}

void initSampeTim() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 14) |  //TIM1 - Counter
                              (1 << 15) |  //TIM2 - PWM
                              (1 << 21);   //PORTA
    MDR_RST_CLK->TIM_CLOCK |= 1 << RST_CLK_TIM_CLOCK_TIM1_CLK_EN_Pos |
                              1 << RST_CLK_TIM_CLOCK_TIM2_CLK_EN_Pos;
    
    PWM_TIM->CNT = 0;
    PWM_TIM->CH1_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    PWM_TIM->CH1_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    PWM_TIM->CH1_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;    
    PWM_TIM->CH2_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    PWM_TIM->CH2_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    PWM_TIM->CH2_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;
    PWM_TIM->ARR = PWM_CCR_MAX;
    
    CNT_VAL = 0;
    CNT_TIM->ARR = CNT_VAL_MAX;
    CNT_TIM->PSG = 3;
    
    MDR_PORTA->FUNC |= 3 << PORT_FUNC_MODE1_Pos |
                       3 << PORT_FUNC_MODE2_Pos |
                       3 << PORT_FUNC_MODE3_Pos |
                       3 << PORT_FUNC_MODE4_Pos;
    MDR_PORTA->OE |= 1 << 1 |
                     1 << 2 |
                     1 << 3 |
                     1 << 4;
    MDR_PORTA->ANALOG |= 1 << 1 |
                         1 << 2 |
                         1 << 3 |
                         1 << 4;
    MDR_PORTA->PWR |= 3 << PORT_PWR1_Pos |
                      3 << PORT_PWR2_Pos |
                      3 << PORT_PWR3_Pos |
                      3 << PORT_PWR4_Pos;
    
    PWM_TIM->CNTRL |= 1 << TIMER_CNTRL_CNT_EN_Pos;
    CNT_TIM->CNTRL |= 1 << TIMER_CNTRL_CNT_EN_Pos;
}

void initMotorPWM1() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 16) |//PWM
                              (1 << 22);//PORTB
    MDR_RST_CLK->TIM_CLOCK |= 1 << RST_CLK_TIM_CLOCK_TIM3_CLK_EN_Pos;
    MDR_TIMER3->CNT = 0;
    MDR_TIMER3->CH3_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    MDR_TIMER3->CH3_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    MDR_TIMER3->CH3_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;    
    MDR_TIMER3->CH4_CNTRL = 0 << TIMER_CH_CNTRL_CAP_NPWM_Pos |
                         6 << TIMER_CH_CNTRL_OCCM_Pos;
    MDR_TIMER3->CH4_CNTRL1 = 2 << TIMER_CH_CNTRL1_SELO_Pos |
                          1 << TIMER_CH_CNTRL1_SELOE_Pos |
                          2 << TIMER_CH_CNTRL1_NSELO_Pos |
                          1 << TIMER_CH_CNTRL1_NSELOE_Pos;
    MDR_TIMER3->CH4_CNTRL2 = 1 << TIMER_CH_CNTRL2_CCRRLD_Pos;
    MDR_TIMER3->ARR = PWM_CCR_MAX;
    
    MDR_PORTB->FUNC |= 3 << PORT_FUNC_MODE5_Pos |
                       3 << PORT_FUNC_MODE6_Pos |
                       3 << PORT_FUNC_MODE7_Pos |
                       3 << PORT_FUNC_MODE8_Pos;
    MDR_PORTB->OE |= 1 << 5 |
                     1 << 6 |
                     1 << 7 |
                     1 << 8;
    MDR_PORTB->ANALOG |= 1 << 5 |
                         1 << 6 |
                         1 << 7 |
                         1 << 8;
    MDR_PORTB->PWR |= 3 << PORT_PWR5_Pos |
                      3 << PORT_PWR6_Pos |
                      3 << PORT_PWR7_Pos |
                      3 << PORT_PWR8_Pos;
    
    MDR_TIMER3->CNTRL |= 1 << TIMER_CNTRL_CNT_EN_Pos;
}

void initHSE() {
    // Задаем коэффициент умножения для CPU PLL:
    MDR_RST_CLK->PLL_CONTROL |= ((CPU_FREQ / HSE_FREQ) - 1) << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos;
    // Включаем PLL
    MDR_RST_CLK->PLL_CONTROL |= 1 << RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos;
    // Проверяем, включен ли PLL
    while (!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));
    // Параметры тактирования 
    MDR_RST_CLK->CPU_CLOCK |= 1 << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos | // Источник тактирования - CPU_C3
                              1 << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos; // Источник для CPU_C2 - PLLCPUo
    
    // Включаем внешнее тактирование
    MDR_RST_CLK->HS_CONTROL |= 1 << RST_CLK_HS_CONTROL_HSE_ON_Pos;
    // Проверяем, что внешнее тактирование включено
    while (!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY));
    // Источник для CPU_C1 - HSE
    MDR_RST_CLK->CPU_CLOCK |= 2 << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos;
}

void initDRV1_EN() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 25);//PE
    MDR_PORTE->FUNC |= 0 << PORT_FUNC_MODE3_Pos;
    MDR_PORTE->OE |= 1 << 3;
    MDR_PORTE->ANALOG |= 1 << 3;
    MDR_PORTE->PWR |= 3 << PORT_PWR3_Pos;
    MDR_PORTE->RXTX |= 1 << 3;
}

void initDRV2_EN() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 25);//PE
    MDR_PORTE->FUNC |= 0 << PORT_FUNC_MODE2_Pos;
    MDR_PORTE->OE |= 1 << 2;
    MDR_PORTE->ANALOG |= 1 << 2;
    MDR_PORTE->PWR |= 3 << PORT_PWR2_Pos;
    MDR_PORTE->RXTX |= 1 << 2;
}

void initDRV3_EN() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 25);//PE
    MDR_PORTE->FUNC |= 0 << PORT_FUNC_MODE6_Pos;
    MDR_PORTE->OE |= 1 << 6;
    MDR_PORTE->ANALOG |= 1 << 6;
    MDR_PORTE->PWR |= 3 << PORT_PWR6_Pos;
    MDR_PORTE->RXTX |= 1 << 6;
}

void DRV3_OFF() {
    MDR_PORTE->RXTX &= ~(1 << 6);
}

void initDRV4_EN() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 25);//PE
    MDR_PORTE->FUNC |= 0 << PORT_FUNC_MODE7_Pos;
    MDR_PORTE->OE |= 1 << 7;
    MDR_PORTE->ANALOG |= 1 << 7;
    MDR_PORTE->PWR |= 3 << PORT_PWR7_Pos;
    MDR_PORTE->RXTX |= 1 << 7;
}

void initSPI() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 8) |//SPI1
                              (1 << 29);//PORTF
    MDR_RST_CLK->SSP_CLOCK = 1 << RST_CLK_SSP_CLOCK_SSP1_CLK_EN_Pos |
                             3 << RST_CLK_SSP_CLOCK_SSP1_BRG_Pos;
/*PF0 - TXD
  PF1 - CLK
  PF2 - FSS (global SPI on, no hardware control)
  PF3 - RXD*/
    MDR_PORTF->FUNC |= 2 << PORT_FUNC_MODE0_Pos |
                       2 << PORT_FUNC_MODE1_Pos |
                       0 << PORT_FUNC_MODE2_Pos |
                       2 << PORT_FUNC_MODE3_Pos;
    MDR_PORTF->OE |= 1 << 0 |
                     1 << 1 |
                     1 << 2 |
                     0 << 3;
    MDR_PORTF->ANALOG |= 1 << 0 |
                         1 << 1 |
                         1 << 2 |
                         1 << 3;
    MDR_PORTF->PWR |= 3 << PORT_PWR0_Pos |
                      3 << PORT_PWR1_Pos |
                      3 << PORT_PWR2_Pos |
                      3 << PORT_PWR3_Pos;
    MDR_PORTF->RXTX |= 1 << 2;
		
    MDR_SSP1->CR0 = 4   << SSP_CR0_SCR_Pos |
                    1   << SSP_CR0_SPH_Pos |
                    1   << SSP_CR0_SPO_Pos |
                    0   << SSP_CR0_FRF_Pos |
                    15  << SSP_CR0_DSS_Pos;
    MDR_SSP1->CPSR = 2;
    MDR_SSP1->CR1 = 1 << SSP_CR1_SSE_Pos;
}

void initSPI_EN() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 29);//PF2
    MDR_PORTF->FUNC |= 0 << PORT_FUNC_MODE2_Pos;
    MDR_PORTF->OE |= 1 << 2;
    MDR_PORTF->ANALOG |= 1 << 2;
    MDR_PORTF->PWR |= 3 << PORT_PWR2_Pos;
    MDR_PORTF->RXTX |= (1 << 2);
}

void initCS1() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 24);//PD4
    MDR_PORTD->FUNC |= 0 << PORT_FUNC_MODE4_Pos;
    MDR_PORTD->OE |= 1 << 4;
    MDR_PORTD->ANALOG |= 1 << 4;
    MDR_PORTD->PWR |= 3 << PORT_PWR4_Pos;
#define SET_CS1_LOW MDR_PORTD -> RXTX &= ~(1 << 4)
#define SET_CS1_HI MDR_PORTD -> RXTX |= (1 << 4)
    SET_CS1_HI;
}

void initCS2() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 23);//PC2
    MDR_PORTC->FUNC |= 0 << PORT_FUNC_MODE2_Pos;
    MDR_PORTC->OE |= 1 << 2;
    MDR_PORTC->ANALOG |= 1 << 2;
    MDR_PORTC->PWR |= 3 << PORT_PWR2_Pos;
#define SET_CS2_LOW MDR_PORTC -> RXTX &= ~(1 << 2)
#define SET_CS2_HI MDR_PORTC -> RXTX |= (1 << 2)
    SET_CS2_HI;
}

void initCS3() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 23);//PC1
    MDR_PORTC->FUNC |= 0 << PORT_FUNC_MODE1_Pos;
    MDR_PORTC->OE |= 1 << 1;
    MDR_PORTC->ANALOG |= 1 << 1;
    MDR_PORTC->PWR |= 3 << PORT_PWR1_Pos;
#define SET_CS3_LOW MDR_PORTC->RXTX&=~(1<<1)
#define SET_CS3_HI MDR_PORTC->RXTX|=(1<<1)
    SET_CS3_HI;
}

void initCS4() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 23);//PC1
    MDR_PORTC->FUNC |= 0 << PORT_FUNC_MODE0_Pos;
    MDR_PORTC->OE |= 1 << 0;
    MDR_PORTC->ANALOG |= 1 << 0;
    MDR_PORTC->PWR |= 3 << PORT_PWR0_Pos;
#define SET_CS4_LOW MDR_PORTC->RXTX&=~(1<<0)
#define SET_CS4_HI MDR_PORTC->RXTX|=(1<<0)
    SET_CS4_HI;
}

void initDRDY1() {
    // Включаем тактирование порта A
    MDR_RST_CLK->PER_CLOCK |= (1 << 21);
    // Настраеваем порт A
    MDR_PORTA->FUNC |= 0 << PORT_FUNC_MODE0_Pos;
    MDR_PORTA->OE |= 0 << 0;
    MDR_PORTA->ANALOG |= 1 << 0;
    MDR_PORTA->PWR |= 3 << PORT_PWR0_Pos;
 }

bool getDRDY1() {
    return (MDR_PORTA->RXTX & 1);
}

void initDRDY2() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 22);
    MDR_PORTB->FUNC |= 0 << PORT_FUNC_MODE10_Pos;
    MDR_PORTB->OE |= 0 << 10;
    MDR_PORTB->ANALOG |= 1 << 10;
    MDR_PORTB->PWR |= 3 << PORT_PWR10_Pos;
}

bool getDRDY2() {
    return (MDR_PORTB->RXTX & (1 << 10));
}

void initDRDY3() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 21);
    MDR_PORTA->FUNC |= 0 << PORT_FUNC_MODE5_Pos;
    MDR_PORTA->OE |= 0 << 5;
    MDR_PORTA->ANALOG |= 1 << 5;
    MDR_PORTA->PWR |= 3 << PORT_PWR5_Pos;
}

bool getDRDY3() {
    return (MDR_PORTA->RXTX & (1 << 5));
}

void CAN1init() {
    MDR_RST_CLK->PER_CLOCK |= (1 << 0) |
                              (1 << 21);//PA
    //PA6 - TX
    //PA7 - RX
    MDR_PORTA->FUNC |= 2 << PORT_FUNC_MODE6_Pos |
                       2 << PORT_FUNC_MODE7_Pos;
    MDR_PORTA->OE |= 1 << 6 |
                     0 << 7;
    MDR_PORTA->ANALOG |= 1 << 6 |
                         1 << 7;
    MDR_PORTA->PWR |= 3 << PORT_PWR6_Pos |
                      3 << PORT_PWR7_Pos;
    
    MDR_RST_CLK->CAN_CLOCK = 1 << RST_CLK_CAN_CLOCK_CAN1_CLK_EN_Pos |
                             3 << RST_CLK_CAN_CLOCK_CAN1_BRG_Pos;
  
    MDR_CAN1->CONTROL = (0 << CAN_CONTROL_SAP_Pos) | (0 << CAN_CONTROL_ROP_Pos) | (1  << CAN_CONTROL_CAN_EN_Pos);
  
    MDR_CAN1->BITTMNG = 0 << CAN_BITTMNG_SB_Pos |
                        1 << CAN_BITTMNG_SJW_Pos |
                        0 << CAN_BITTMNG_BRP_Pos |
                        1  << CAN_BITTMNG_SEG2_Pos |
                        2  << CAN_BITTMNG_SEG1_Pos |
                        3  << CAN_BITTMNG_PSEG_Pos;
    //tx buf
    for (char i = 0; i < 9; ++i) {
    MDR_CAN1->BUF_CON[i] = 1 << CAN_BUF_CON_EN_Pos;
    MDR_CAN1->CAN_BUF[i].DLC = (1 << 11) | (1 << 9) | (5 << 0);// | (1 << ) | (1 << ) | (1 << ) | (1 << ) | (1 << );
    MDR_CAN1->CAN_BUF[i].ID = 0 << 18;
    }
    //rx buf
    __enable_irq();
    NVIC_EnableIRQ(CAN1_IRQn);
    MDR_CAN1->BUF_CON[9] = 1 << CAN_BUF_CON_RX_TXN_Pos | 1 << CAN_BUF_CON_EN_Pos;
    MDR_CAN1->INT_RX |= 1 << 9; 
    MDR_CAN1->INT_EN = 1 << CAN_INT_EN_GLB_INT_EN_Pos | 1 << CAN_INT_EN_RX_INT_EN_Pos;
    MDR_CAN1->CAN_BUF[9].DLC = (1 << 11) | (1 << 9) | (5 << 0);// | (1 << ) | (1 << ) | (1 << ) | (1 << ) | (1 << );
    MDR_CAN1->CAN_BUF_FILTER[9].MASK = 0;
    MDR_CAN1->CAN_BUF_FILTER[9].FILTER = 0;
}