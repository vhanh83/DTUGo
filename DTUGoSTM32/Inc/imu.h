#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "l3gd20.h"

/* Địa chỉ I2C của LSM303DLHC */
#define LSM303_ACC_ADDR         (0x19 << 1) // 8-bit address (0x32)
#define LSM303_MAG_ADDR         (0x1E << 1) // 8-bit address (0x3C)

/* Cấu trúc lưu góc quay/nghiêng (Roll, Pitch, Yaw) */
typedef struct {
    float yaw;   // Gyro Z (dps)
    float roll;  // Góc Roll từ Accel (độ)
    float pitch; // Góc Pitch từ Accel (độ)
} IMU_OrientationTypeDef;

/* Cấu trúc dữ liệu Vector 3 trục */
typedef struct {
    float x;
    float y;
    float z;
} IMU_Vector3f_t;

/**
 * @brief Khởi tạo IMU STM32F4 Discovery (L3GD20 qua SPI1 + LSM303 Accel qua I2C1)
 */
HAL_StatusTypeDef IMU_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

/**
 * @brief Đọc vận tốc góc từ L3GD20 Gyroscope (đơn vị: dps)
 */
HAL_StatusTypeDef IMU_ReadGyro(IMU_Vector3f_t *gyro);

/**
 * @brief Đọc gia tốc từ LSM303 Accelerometer (đơn vị: g)
 */
HAL_StatusTypeDef IMU_ReadAccel(I2C_HandleTypeDef *hi2c, IMU_Vector3f_t *accel);

/**
 * @brief Đọc góc nghiêng Roll/Pitch tính từ LSM303 và Yaw từ L3GD20
 */
HAL_StatusTypeDef IMU_ReadOrientation(I2C_HandleTypeDef *hi2c, IMU_OrientationTypeDef *orientation);

#ifdef __cplusplus
}
#endif

#endif // IMU_H