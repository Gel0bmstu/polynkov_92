#pragma once

#define L2G_WHO_AM_I      0x00
#define L2G_WHO_AM_I_VAL  0xD9
#define L2G_TEMP_OUT_L    0x01
#define L2G_TEMP_OUT_H    0x02
#define L2G_OUT_X_L       0x03
#define L2G_OUT_X_H       0x04
#define L2G_OUT_Y_L       0x05
#define L2G_OUT_Y_H       0x06
#define L2G_STATUS_REG    0x09
#define L2G_CTRL_REG1     0x0B
#define L2G_CTRL_REG2     0x0C
#define L2G_CTRL_REG3     0x0D
#define L2G_ORIENT_CONFIG 0x10
#define L2G_OFF_X         0x11
#define L2G_OFF_Y         0x12
#define L2G_CTRL_REG4     0x1F

#define L2G_CTRL_REG1_BOOT_Pos 7
#define L2G_CTRL_REG1_P_DRDY_Pos 6
#define L2G_CTRL_REG1_BLE_Pos 5
#define L2G_CTRL_REG1_SIM_Pos 4
#define L2G_CTRL_REG1_ODU_Pos 3
#define L2G_CTRL_REG1_PW_Pos 0
#define L2G_CTRL_REG1_PW_NORM 3 << L2G_CTRL_REG1_PW_Pos

#define L2G_CTRL_REG2_LPF_O_Pos 7

#define L2G_CTRL_REG3_DRDY_EN_Pos 1

#define L2G_CTRL_REG4_FS_Pos 3