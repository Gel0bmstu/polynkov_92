#pragma once

#include "board.h"
#include "SPI.h"
#include "L2G2IS.h"
#include "MPU-60x0.h"
#include "LSM6DS33.h"

bool initL2G2IS() {
    while(L2Gread(L2G_WHO_AM_I) != L2G_WHO_AM_I_VAL);
  
    // Ребутаем память (отчищаем)
		L2Gwrite(L2G_CTRL_REG1,
						 1 << L2G_CTRL_REG1_BOOT_Pos);
		L2Gwrite(L2G_CTRL_REG1,
						 0 << L2G_CTRL_REG1_ODU_Pos | // Обновляем выходные данные
						 L2G_CTRL_REG1_PW_NORM);      // Устанавливаем обычный режим работы
		
    // Устанавливаем порядок выбора ФНЧ - 1 очередь
    L2Gwrite(L2G_CTRL_REG2,
            1 << L2G_CTRL_REG2_LPF_O_Pos);
    // Включаем режим синхронного чтения
		while (L2Gread(L2G_CTRL_REG3) != (
								1 << L2G_CTRL_REG3_DRDY_EN_Pos)) {
							L2Gwrite(L2G_CTRL_REG3,
						 1 << L2G_CTRL_REG3_DRDY_EN_Pos);			
					}
    // Устанавливаем рабочий диапазон ±100 °/c
		L2Gwrite(L2G_CTRL_REG4,
						 0 << L2G_CTRL_REG4_FS_Pos);
    return true;
}

bool initMPU_60x0() {
		// Проверяем наличие подключения к ЧЭ через SPI
    while (MPUread(MPU_60x0_WHO_AM_I) != MPU_60x0_WHO_AM_I_VAL);
    MPUwrite(MPU_60x0_INT_PIN_CFG,
             1 << 5 | 1 << 4 | 0 << 2 | 0 << 0);
    // Включаем прирывание по завершению операции записи во все 
    // регистры датчиков
    MPUwrite(MPU_60x0_INT_ENABLE,
             1 << 0);
    // Устанавливаем рабочий диапазон датчика ±250 °/c
    MPUwrite(MPU_60x0_GYRO_CONFIG,
             MPU_60x0_GYRO_CONFIG_FS_SEL_250);
    // Устанавливаем источник тактирования:
    // Выбираем встроенный 8МГц осцилятор
    MPUwrite(MPU_60x0_PWR_MGMT_1,
             0);
    // Запрещаем использование акселерометра ?
    MPUwrite(MPU_60x0_PWR_MGMT_2,
             0);
    return true;
}

bool initLSM6DS33() {
    while (LSMread(LSM_WHO_AM_I) != LSM_WHO_AM_I_VAL);
		while(LSMread(LSM_CTRL10_C) != (
             1 << LSM_CTRL10_C_XEN_G_Pos | // Включаем Х ось
             1 << LSM_CTRL10_C_YEN_G_Pos | // Включаем Y ось
             1 << LSM_CTRL10_C_ZEN_G_Pos)) { // Включаем Z ось
								LSMwrite(LSM_CTRL10_C,
										 1 << LSM_CTRL10_C_XEN_G_Pos |
										 1 << LSM_CTRL10_C_YEN_G_Pos |
										 1 << LSM_CTRL10_C_ZEN_G_Pos); 
						 }
    LSMwrite(LSM_CTRL2_G,
             // Устанавливаем частоту выдачи сигнала 1.66 МГц
             8 << LSM_CTRL2_G_ODR_G_Pos | 
             // Устанавливаем рабочий диапазон гироскопа ±125 1
             1 << LSM_CTRL2_G_FS_125_Pos);
    LSMwrite(LSM_INT2_CTRL,
             1 << 1);
    // Выходные регистры не обновляются, пока MSB и LSB не будут
    // прочитаны
    while(LSMread(LSM_CTRL3_C) != (1 << 6 | 1 << 2)) {
			    LSMwrite(LSM_CTRL3_C,
             1 << LSM_CTRL3_C_BDU_Pos | 1 << 2);
		}
    return true;
}
