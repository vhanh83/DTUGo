#ifndef __L3GD20_H
#define __L3GD20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ====================================================================
 *                 L3GD20 REGISTER MAP & DEFINITIONS
 * ==================================================================== */

extern SPI_HandleTypeDef hspi1;

#define L3GD20_WHO_AM_I_ADDR       0x0F
#define L3GD20_CTRL_REG1_ADDR      0x20
#define L3GD20_CTRL_REG2_ADDR      0x21
#define L3GD20_CTRL_REG3_ADDR      0x22
#define L3GD20_CTRL_REG4_ADDR      0x23
#define L3GD20_CTRL_REG5_ADDR      0x25
#define L3GD20_OUT_X_L_ADDR        0x28

/* Identification Codes */
#define L3GD20_WHO_AM_I_ID1        0xD4  /* L3GD20 */
#define L3GD20_WHO_AM_I_ID2        0xD3  /* L3G4200D */
#define L3GD20_WHO_AM_I_ID3        0xD7  /* I3G4250D / L3GD20H */

/* Full Scale Selection (dps: degrees per second) */
typedef enum {
  L3GD20_FULLSCALE_250DPS  = 0x00,
  L3GD20_FULLSCALE_500DPS  = 0x10,
  L3GD20_FULLSCALE_2000DPS = 0x20
} L3GD20_FullScale_t;

/* Cấu trúc dữ liệu raw và dữ liệu thực (dps) */
typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} L3GD20_RawData_t;

typedef struct {
  float X;
  float Y;
  float Z;
} L3GD20_Data_t;

/* Cấu trúc quản lý Device */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *CS_Port;
  uint16_t           CS_Pin;
  float              Sensitivity; // dps/LSB
} L3GD20_HandleTypeDef;

/* ====================================================================
 *                         FUNCTION PROTOTYPES
 * ==================================================================== */

/**
  * @brief  Khởi tạo cảm biến L3GD20.
  * @param  dev: Con trỏ tới cấu trúc L3GD20_HandleTypeDef.
  * @param  hspi: Con trỏ tới SPI handle của STM32 (ví dụ: &hspi1).
  * @param  CS_Port: Port GPIO của chân Chip Select (ví dụ: GPIOE).
  * @param  CS_Pin: Chân GPIO của Chip Select (ví dụ: GPIO_PIN_3).
  * @param  scale: Thang đo FSR (250DPS, 500DPS, 2000DPS).
  * @retval HAL_StatusTypeDef: HAL_OK nếu khởi tạo thành công.
  */
HAL_StatusTypeDef L3GD20_Init(L3GD20_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, 
                              GPIO_TypeDef *CS_Port, uint16_t CS_Pin, 
                              L3GD20_FullScale_t scale);

/**
  * @brief  Đọc ID của thiết bị (Kiểm tra xem chip có kết nối không).
  * @param  dev: Con trỏ tới cấu trúc L3GD20_HandleTypeDef.
  * @param  id: Con trỏ lưu ID trả về.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef L3GD20_ReadID(L3GD20_HandleTypeDef *dev, uint8_t *id);

/**
  * @brief  Đọc dữ liệu vận tốc góc thô (Raw int16_t).
  * @param  dev: Con trỏ tới cấu trúc L3GD20_HandleTypeDef.
  * @param  raw_data: Con trỏ lưu kết quả raw X, Y, Z.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef L3GD20_ReadRaw(L3GD20_HandleTypeDef *dev, L3GD20_RawData_t *raw_data);

/**
  * @brief  Đọc dữ liệu vận tốc góc đã đổi ra đơn vị dps (Degrees Per Second).
  * @param  dev: Con trỏ tới cấu trúc L3GD20_HandleTypeDef.
  * @param  data: Con trỏ lưu kết quả float X, Y, Z.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef L3GD20_ReadDPS(L3GD20_HandleTypeDef *dev, L3GD20_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __L3GD20_H */