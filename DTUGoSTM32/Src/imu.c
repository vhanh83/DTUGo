#include "imu.h"
#include <stdio.h>
#include <math.h>

/* Thanh ghi LSM303 Accelerometer */
#define LSM303_WHO_AM_I_A       0x0F
#define LSM303_CTRL_REG1_A      0x20
#define LSM303_OUT_X_L_A        0x28
#define LSM303_ACC_SENSITIVITY  0.001f // 1 mg/LSB (Thang ±2g, 12-bit)
#define ALPHA 0.98f // Hệ số lọc bù (98% Gyro, 2% Accel)
static L3GD20_HandleTypeDef hgyro;
static float current_yaw = 0.0f;
static uint32_t last_time = 0;
// Biến lưu Bias (Offset) tĩnh của Gyroscope
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
HAL_StatusTypeDef IMU_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    char msg[80];

    // 1. Khởi tạo L3GD20 Gyro
    if (L3GD20_Init(&hgyro, hspi, GPIOE, GPIO_PIN_3, L3GD20_FULLSCALE_250DPS) != HAL_OK) {
        return HAL_ERROR;
    }

    // 2. Khởi tạo LSM303 Accel (0x19)
    uint8_t ctrl1_a = 0x57; // 100Hz ODR, Enable X,Y,Z
    if (HAL_I2C_Mem_Write(hi2c, LSM303_ACC_ADDR, LSM303_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &ctrl1_a, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // 3. THỰC HIỆN CALIB GYRO BIAS (Lấy 200 mẫu khi đứng yên trong 1 giây)
    if (huart) {
        int n = snprintf(msg, sizeof(msg), "[IMU] Calibrating Gyro Offset... Keep board STILL!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)msg, (uint16_t)n, 100);
    }

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    L3GD20_Data_t gyro_raw;
    const int calib_samples = 200;

    for (int i = 0; i < calib_samples; i++) {
        if (L3GD20_ReadDPS(&hgyro, &gyro_raw) == HAL_OK) {
            sum_x += gyro_raw.X;
            sum_y += gyro_raw.Y;
            sum_z += gyro_raw.Z;
        }
        HAL_Delay(5); // Lấy mẫu mỗi 5ms
    }

    gyro_bias_x = sum_x / (float)calib_samples;
    gyro_bias_y = sum_y / (float)calib_samples;
    gyro_bias_z = sum_z / (float)calib_samples;

    if (huart) {
        int n = snprintf(msg, sizeof(msg), "[IMU OK] Calib Done! GyroZ Offset = %.3f dps\r\n", gyro_bias_z);
        HAL_UART_Transmit(huart, (uint8_t*)msg, (uint16_t)n, 100);
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU_ReadGyro(IMU_Vector3f_t *gyro) {
    L3GD20_Data_t gyro_data;
    if (L3GD20_ReadDPS(&hgyro, &gyro_data) != HAL_OK) {
        return HAL_ERROR;
    }
    // Trừ đi sai số tĩnh Bias
    gyro->x = gyro_data.X - gyro_bias_x;
    gyro->y = gyro_data.Y - gyro_bias_y;
    gyro->z = gyro_data.Z - gyro_bias_z;
    return HAL_OK;
}
HAL_StatusTypeDef IMU_ReadAccel(I2C_HandleTypeDef *hi2c, IMU_Vector3f_t *accel) {
    uint8_t buf[6];
    // Set bit 0x80 (MSB) để bật Auto-Increment đọc nhiều thanh ghi liên tiếp trên LSM303
    if (HAL_I2C_Mem_Read(hi2c, LSM303_ACC_ADDR, LSM303_OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Ghép dữ liệu 12-bit (căn lề trái trong 16-bit) -> Shift right 4 bits
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) >> 4;

    accel->x = (float)raw_x * LSM303_ACC_SENSITIVITY;
    accel->y = (float)raw_y * LSM303_ACC_SENSITIVITY;
    accel->z = (float)raw_z * LSM303_ACC_SENSITIVITY;

    return HAL_OK;
}

HAL_StatusTypeDef IMU_ReadOrientation(I2C_HandleTypeDef *hi2c, IMU_OrientationTypeDef *orientation) {
    IMU_Vector3f_t acc;
    IMU_Vector3f_t gyro;

    if (IMU_ReadAccel(hi2c, &acc) != HAL_OK) return HAL_ERROR;
    if (IMU_ReadGyro(&gyro) != HAL_OK) return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (last_time == 0 || dt <= 0.0f) dt = 0.02f;
    last_time = now;

    // 1. Tính góc Roll & Pitch từ Gia tốc
    float accel_roll  = atan2f(acc.y, acc.z) * 180.0f / (float)M_PI;
    float accel_pitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * 180.0f / (float)M_PI;

    // 2. Khởi tạo biến lọc bù
    static float filtered_roll = 0.0f;
    static float filtered_pitch = 0.0f;
    static uint8_t is_first_run = 1;

    // Lần đọc đầu tiên: gán trực tiếp góc từ gia tốc để không bị trôi/hội tụ chậm
    if (is_first_run) {
        filtered_roll  = accel_roll;
        filtered_pitch = accel_pitch;
        is_first_run = 0;
    } else {
        // Các lần tiếp theo: chạy bộ lọc bù bình thường
        filtered_roll  = ALPHA * (filtered_roll + gyro.x * dt) + (1.0f - ALPHA) * accel_roll;
        filtered_pitch = ALPHA * (filtered_pitch + gyro.y * dt) + (1.0f - ALPHA) * accel_pitch;
    }

    // 3. Tích phân góc Yaw (đã trừ Bias)
    current_yaw += gyro.z * dt;
    if (current_yaw > 180.0f)  current_yaw -= 360.0f;
    if (current_yaw < -180.0f) current_yaw += 360.0f;

    orientation->roll  = filtered_roll;
    orientation->pitch = filtered_pitch;
    orientation->yaw   = current_yaw;

    return HAL_OK;
}