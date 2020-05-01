#ifdef USE_L2G
    #define SENSOR_READ L2G_read
    #define SENSOR_READ_MULT L2Gread_mult
    #define GYRO_X_L L2G_OUT_X_L
    #define GYRO_X_H L2G_OUT_X_H
    #define GYRO_Y_L L2G_OUT_Y_L
    #define GYRO_Y_H L2G_OUT_Y_H
    #define TEMP_L L2G_TEMP_OUT_L
    #define TEMP_H L2G_TEMP_OUT_H
    
    #define X_L_POS 2
    #define X_H_POS 3
    #define Y_L_POS 4
    #define Y_H_POS 5
    #define T_L_POS 0
    #define T_H_POS 1
    #define SENSOR_OFFSET_REG TEMP_L
#endif

#ifdef USE_MPU
    #define SENSOR_READ MPU_read
    #define SENSOR_READ_MULT MPUread_mult
    #define GYRO_X_L MPU_60x0_GYRO_XOUT_L
    #define GYRO_X_H MPU_60x0_GYRO_XOUT_H
    #define GYRO_Y_L MPU_60x0_GYRO_YOUT_L
    #define GYRO_Y_H MPU_60x0_GYRO_YOUT_H
    #define TEMP_L MPU_60x0_TEMP_OUT_L
    #define TEMP_H MPU_60x0_TEMP_OUT_H
    
    #define X_L_POS 3
    #define X_H_POS 2
    #define Y_L_POS 5
    #define Y_H_POS 4
    #define T_L_POS 1
    #define T_H_POS 0
    #define SENSOR_OFFSET_REG TEMP_H
#endif