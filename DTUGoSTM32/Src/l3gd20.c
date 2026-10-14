#include "l3gd20.h"

/* Bit điều khiển giao tiếp SPI trên L3GD20 */
#define L3GD20_READ_BIT      0x80
#define L3GD20_MULTIPLE_BIT  0x40

/* Macro điều khiển chân Chip Select */
#define CS_LOW(dev)  HAL_GPIO_WritePin((dev)->CS_Port, (dev)->CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH(dev) HAL_GPIO_WritePin((dev)->CS_Port, (dev)->CS_Pin, GPIO_PIN_SET)

/* Helper: Đọc 1 thanh ghi */
static HAL_StatusTypeDef L3GD20_ReadRegister(L3GD20_HandleTypeDef *dev, uint8_t reg, uint8_t *data) {
  uint8_t tx[2] = { reg | L3GD20_READ_BIT, 0xFF }; // tx[1] là Dummy byte
  uint8_t rx[2] = { 0 };
  HAL_StatusTypeDef status;

  CS_LOW(dev);
  status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
  CS_HIGH(dev);

  if (status == HAL_OK) {
    *data = rx[1]; // Dữ liệu L3GD20 trả về nằm ở byte thứ 2
  }

  return status;
}

/* Helper: Ghi 1 thanh ghi */
static HAL_StatusTypeDef L3GD20_WriteRegister(L3GD20_HandleTypeDef *dev, uint8_t reg, uint8_t data) {
  uint8_t tx[2] = { reg & ~L3GD20_READ_BIT, data };
  HAL_StatusTypeDef status;

  CS_LOW(dev);
  status = HAL_SPI_Transmit(dev->hspi, tx, 2, 100);
  CS_HIGH(dev);

  return status;
}

/* Helper: Đọc nhiều thanh ghi liên tiếp (Auto-increment) */
static HAL_StatusTypeDef L3GD20_ReadRegisters(L3GD20_HandleTypeDef *dev, uint8_t reg, uint8_t *data, uint16_t size) {
  uint8_t tx[7] = {0}; // Tối đa đọc 6 byte dữ liệu XYZ + 1 byte địa chỉ
  uint8_t rx[7] = {0};
  
  if (size > 6) return HAL_ERROR; // Bảo vệ vượt dung lượng buffer

  tx[0] = reg | L3GD20_READ_BIT | L3GD20_MULTIPLE_BIT;
  // Các byte tx[1..6] giữ nguyên 0x00 / 0xFF làm dummy bytes

  CS_LOW(dev);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, size + 1, 100);
  CS_HIGH(dev);

  if (status == HAL_OK) {
    // Copy phần dữ liệu thực sự (bỏ qua byte đầu tiên là rác khi gửi địa chỉ)
    for (uint16_t i = 0; i < size; i++) {
      data[i] = rx[i + 1];
    }
  }

  return status;
}

/* --- HÀM KHỞI TẠO --- */
HAL_StatusTypeDef L3GD20_Init(L3GD20_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, 
                              GPIO_TypeDef *CS_Port, uint16_t CS_Pin, 
                              L3GD20_FullScale_t scale) {
  uint8_t id = 0;

  dev->hspi = hspi;
  dev->CS_Port = CS_Port;
  dev->CS_Pin = CS_Pin;

  // Đảm bảo chân CS ban đầu ở mức HIGH
  CS_HIGH(dev);

  // Kiểm tra ID
  if (L3GD20_ReadID(dev, &id) != HAL_OK) {
    return HAL_ERROR;
  }

  if (id != L3GD20_WHO_AM_I_ID1 && id != L3GD20_WHO_AM_I_ID2 && id != L3GD20_WHO_AM_I_ID3) {
    return HAL_ERROR; // ID không khớp
  }

  // 1. Cấu hình CTRL_REG1: Enable ODR 95Hz, Cut-off 12.5Hz, Normal Mode, Enable cả 3 trục X, Y, Z
  uint8_t ctrl1 = 0x0F; // 0000 1111 -> Normal mode, X,Y,Z enabled
  if (L3GD20_WriteRegister(dev, L3GD20_CTRL_REG1_ADDR, ctrl1) != HAL_OK) {
    return HAL_ERROR;
  }

  // 2. Cấu hình CTRL_REG4: Chọn Full Scale và tính Sensitivity
  uint8_t ctrl4 = scale;
  if (L3GD20_WriteRegister(dev, L3GD20_CTRL_REG4_ADDR, ctrl4) != HAL_OK) {
    return HAL_ERROR;
  }

  // Cập nhật độ nhạy (Sensitivity) tương ứng theo Datasheet
  switch (scale) {
    case L3GD20_FULLSCALE_250DPS:
      dev->Sensitivity = 0.00875f; // 8.75 mdps/LSB
      break;
    case L3GD20_FULLSCALE_500DPS:
      dev->Sensitivity = 0.01750f; // 17.5 mdps/LSB
      break;
    case L3GD20_FULLSCALE_2000DPS:
      dev->Sensitivity = 0.07000f; // 70 mdps/LSB
      break;
    default:
      dev->Sensitivity = 0.00875f;
      break;
  }

  return HAL_OK;
}

/* --- ĐỌC ID CHIP --- */
HAL_StatusTypeDef L3GD20_ReadID(L3GD20_HandleTypeDef *dev, uint8_t *id) {
  return L3GD20_ReadRegister(dev, L3GD20_WHO_AM_I_ADDR, id);
}

/* --- ĐỌC DỮ LIỆU RAW --- */
HAL_StatusTypeDef L3GD20_ReadRaw(L3GD20_HandleTypeDef *dev, L3GD20_RawData_t *raw_data) {
  uint8_t buffer[6];

  // Đọc 6 byte liên tiếp từ OUT_X_L đến OUT_Z_H
  if (L3GD20_ReadRegisters(dev, L3GD20_OUT_X_L_ADDR, buffer, 6) != HAL_OK) {
    return HAL_ERROR;
  }

  // Ghép byte Little-Endian
  raw_data->X = (int16_t)((buffer[1] << 8) | buffer[0]);
  raw_data->Y = (int16_t)((buffer[3] << 8) | buffer[2]);
  raw_data->Z = (int16_t)((buffer[5] << 8) | buffer[4]);

  return HAL_OK;
}

/* --- ĐỌC DỮ LIỆU DPS --- */
HAL_StatusTypeDef L3GD20_ReadDPS(L3GD20_HandleTypeDef *dev, L3GD20_Data_t *data) {
  L3GD20_RawData_t raw;

  if (L3GD20_ReadRaw(dev, &raw) != HAL_OK) {
    return HAL_ERROR;
  }

  data->X = (float)raw.X * dev->Sensitivity;
  data->Y = (float)raw.Y * dev->Sensitivity;
  data->Z = (float)raw.Z * dev->Sensitivity;

  return HAL_OK;
}