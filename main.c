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
#include "sensor.h"
#include "MDR32F9Qx_uart.h"             // Keil::Drivers:UART

#define USE_L2G
#include "sensor_type_defines.h"


#define M_PI 3.14159265

float pwmden = 0.1;
bool mpuInitSuccess, l2gInitSuccess, lsmInitSuccess;
char mpu_omega_x_L, mpu_omega_x_H, mpu_omega_y_L, mpu_omega_y_H, 
		 mpu_temp_L, mpu_temp_H, 
		 mpu_ox_data[8],
		 mpu_oy_data[8],
		 mpu_temp_data[8];
float mpu_omega_x,
      mpu_omega_y,
      mpu_temp;

char l2g_omega_x_L, l2g_omega_x_H, l2g_omega_y_L, l2g_omega_y_H, 
		 l2g_temp_L, l2g_temp_H, 
		 l2g_ox_data[8],
		 l2g_oy_data[8],
		 l2g_temp_data[8];
float l2g_omega_x,
      l2g_omega_y,
      l2g_temp;

char lsm_omega_x_L, lsm_omega_x_H, lsm_omega_y_L, lsm_omega_y_H, 
		 lsm_temp_L, lsm_temp_H, 
		 lsm_ox_data[8],
		 lsm_oy_data[8],
		 lsm_temp_data[8];
float lsm_omega_x,
      lsm_omega_y,
      lsm_temp;

int mode = 0,
		i = 0;

char receive_data[8];

void uartInit();
void sendMpuData();
void sendL2gData();
void sendLsmData();
void sendEndOfPacketSymbol();
void sendError();

void sendMode2() {
	sendMpuData();
	sendL2gData();	
}

void sendMode3() {
	sendMpuData();
	sendL2gData();
	sendLsmData();			
}

int main() {
	// initHSE();

	// Инициализируем выходы с SPI и светодиоды
	initCS1();
	initCS2();
	initCS3();

	// Инициализируем сам SPI интрефейс и UART
	// интерфейс
	initSPI();
	uartInit();

	// Инициализируем "перефирию" (порты A и B)
	initDRDY1();
	initDRDY2();
	initDRDY3();
	
	// Посылаем сигнал о начале работы программы
	while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
	UART_SendData(MDR_UART2, 'G');
	
	// Инициализируем чувствительные элементы
	mpuInitSuccess = initMPU_60x0();
	l2gInitSuccess = initL2G2IS();
	lsmInitSuccess = initLSM6DS33();
	
	// Если инициализация прошла успешно, запускаем
	// Главный цикл
	if (mpuInitSuccess && l2gInitSuccess && lsmInitSuccess) {
		while (1) {
			char pckg_data[6];
			
			L2Gread_mult(L2G_TEMP_OUT_L, 6, pckg_data);
			l2g_omega_x_L = pckg_data[2];
			l2g_omega_x_H = pckg_data[3];
			l2g_omega_y_L = pckg_data[4];
			l2g_omega_y_H = pckg_data[5];
			l2g_temp_L = pckg_data[0];
			l2g_temp_H = pckg_data[1];
			l2g_temp = (int16_t)((l2g_temp_H << 8) | l2g_temp_L) * 0.0625 / 16  + 25;
			l2g_omega_x = ((int16_t)((l2g_omega_x_H << 8) | l2g_omega_x_L) /(float)262) - get_polynom_zero_bias(l2g_temp, l2g_x_c);
			l2g_omega_y = (int16_t)((l2g_omega_y_H << 8) | l2g_omega_y_L) /(float)262;
			
			sprintf(l2g_ox_data, "%f", l2g_omega_x);
			sprintf(l2g_oy_data, "%f", l2g_omega_y);
			sprintf(l2g_temp_data, "%f", l2g_temp);
			
			MPUread_mult(MPU_60x0_TEMP_OUT_H, 6, pckg_data);
			mpu_omega_x_L = pckg_data[3];
			mpu_omega_x_H = pckg_data[2];
			mpu_omega_y_L = pckg_data[5];
			mpu_omega_y_H = pckg_data[4];
			mpu_temp_L = pckg_data[1];
			mpu_temp_H = pckg_data[0];
		
			mpu_omega_x = (int16_t)(((mpu_omega_x_H << 8) | mpu_omega_x_L) / (float)131);
			mpu_omega_y = (int16_t)((mpu_omega_y_H << 8) | mpu_omega_y_L) / (float)131;
			mpu_temp = (int16_t)((mpu_temp_H << 8) | mpu_temp_L) / (float)340  + 36.63;
			
			sprintf(mpu_ox_data, "%f", mpu_omega_x);
			sprintf(mpu_oy_data, "%f", mpu_omega_y);
			sprintf(mpu_temp_data, "%f", mpu_temp);
			
			LSMread_mult(LSM_TEMP_OUT_L, 6, pckg_data);
			lsm_omega_x_L = pckg_data[2];
			lsm_omega_x_H = pckg_data[3];
			lsm_omega_y_L = pckg_data[4];
			lsm_omega_y_H = pckg_data[5];
			lsm_temp_L = pckg_data[0];
			lsm_temp_H = pckg_data[1];
			lsm_omega_x = (float)(int16_t)((lsm_omega_x_H << 8) | lsm_omega_x_L) * 0.004375;
			lsm_omega_y = (float)(int16_t)((lsm_omega_y_H << 8) | lsm_omega_y_L) * 0.004375;
			lsm_temp = (float)(int16_t)((lsm_temp_H << 8) | lsm_temp_L) / 16  + 25;
			
			sprintf(lsm_ox_data, "%f", lsm_omega_x);
			sprintf(lsm_oy_data, "%f", lsm_omega_y);
			sprintf(lsm_temp_data, "%f", lsm_temp);
			
			if (MDR_UART2->RIS & (1 << UART_RIS_RXRIS_Pos)) {
				for (int i = 0; i < 8; i++) {
						receive_data[i] = (char)UART_ReceiveData(MDR_UART2);
				}
				
				if (!strcmp(receive_data, "get1data")) {
					mode = 1;
				} else if (!strcmp(receive_data, "get2data")) {
					mode = 2;
				} else if (!strcmp(receive_data, "get3data")) {
					mode = 3;
				} else {
					mode = 0;
				};
				
				i = 0;
				strcpy(receive_data, " ");
				
			};
			
			switch (mode) {
				case 1:
					sendMpuData();
					break;
				case 2:
					sendMode2();
					break;
				case 3:
					sendMode3();
					break;
				default:
					sendError();
					break;
			}
			
			sendEndOfPacketSymbol();
		}
	} else {
		while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
		UART_SendData(MDR_UART2, 'E');
	}
}

void sendEndOfPacketSymbol() {
	char *EOP = "\r\n";
	for (int i = 0; i < strlen(EOP); i++) {
		while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
		UART_SendData(MDR_UART2, EOP[i]);
	}
}

void sendError() {
	char* err = " ErrMsg ";
	for (int i = 0; i < 8; i++) {
		while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
		UART_SendData(MDR_UART2, err[i]);
	}
}

void uartSendWord(char* command) {
	for (int i = 0; i < 8; i++) {
		while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
		UART_SendData(MDR_UART2, command[i]);
	}
}

void sendDelimer() {
	char del = ' ';
	while (UART_GetFlagStatus(MDR_UART2, UART_FLAG_TXFE ) != SET);
	UART_SendData(MDR_UART2, del);
}

void sendMpuData() {
	uartSendWord(mpu_ox_data);
	sendDelimer();
	uartSendWord(mpu_oy_data);
	sendDelimer();
	uartSendWord(mpu_temp_data);
	sendDelimer();
}

void sendL2gData() {
	uartSendWord(l2g_ox_data);
	sendDelimer();
	uartSendWord(l2g_oy_data);
	sendDelimer();
	uartSendWord(l2g_temp_dataa);
	sendDelimer();
}

void sendLsmData() {
	uartSendWord(lsm_ox_data);
	sendDelimer();
	uartSendWord(lsm_oy_data);
	sendDelimer();
	uartSendWord(lsm_temp_dataa);
	sendDelimer();
}

void uartInit()
{
  MDR_RST_CLK->PER_CLOCK |= (1UL << 24); //тактирование порта D

  MDR_PORTD->FUNC |= ((2 << 1*2) | (2 << 0*2)); //режим работы порта
  MDR_PORTD->ANALOG |= ((1 << 1) | (1 << 0)); //цифровой
  MDR_PORTD->PWR |= ((3 << 1*2) | (3 << 0*2)); //максимально быcтрый

  MDR_RST_CLK->PER_CLOCK |= (1UL << 7); //тактирование UART2
  MDR_RST_CLK->UART_CLOCK = (0 /*установка делителя для UART1 = undefined*/
  |(0 << 8) /*установка делителя для UART2 = 1*/
  |(0 << 24) /*разрешение тактовой частоты UART1*/
  |(1 << 25)); /*разрешение тактовой частоты UART2*/ 

  //Параметры делителя при частоте = 8000000Гц и скорости = 115200 бит/сек
  MDR_UART2->IBRD = 0x4; //целая часть делителя скорости
  MDR_UART2->FBRD = 0x16; //дробная часть делителя скорости
  MDR_UART2->LCR_H = ((0 << 1) /*разрешение проверки четности*/
  |(0 << 2) /*четность/нечетность (нет контроля)*/
  |(0 << 3) /*стоп-бит = 1 бит*/
  |(3 << 5) /*длина слова = 8 бит*/
  |(0 << 7)); /*передача бита четности*/

	MDR_UART2->LCR_H |= (1 << UART_LCR_H_FEN_Pos); // Включаем FIFO буфер
  MDR_UART2->IFLS |= (0 << UART_IFLS_RXIFLSEL_Pos);
	MDR_UART2->CR = ((1 << 8)|(1 << 9)|1); //передачик и приемник разрешен, разрешение приемопередатчика UART2
}