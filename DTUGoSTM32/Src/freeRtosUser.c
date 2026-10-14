#include "FreeRTOS.h"
#include "uart.h"
#include "freeRtosUser.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "i2c.h"
#include "l3gd20.h"
#include "LegMotion.h"
#include "BodyIK.h"
#include "imu.h"
#include "ina3221.h"
#include "string.h"
L3GD20_HandleTypeDef hgyro;
IMU_OrientationTypeDef current_imu_orientation; // Global variable to hold the latest IMU orientation
INA3221_ReadingsTypeDef current_battery_readings; // Global variable to hold the latest battery readings

// Struct matching the Jetson packet (1 byte type + 36 bytes floats)
// Packed to ensure no padding bytes are added by the compiler, which would break parsing
struct __attribute__((packed)) CmdPayload {
    uint8_t cmd_type;
    float lin_x;
    float lin_y;
    float ang_z;
    float roll;
    float pitch;
    float yaw;
    float z_offset;
    float x_offset;
    float y_offset;
    float pivot_x;
    float pivot_y;
};
#define CMD_NORMAL 0x01
#define CMD_RESET 0x02
#define CMD_WAVE 0x03
#define CMD_HEEL 0x04
#define CMD_DANCE 0x05
#define CMD_EXTRAS 0x06
#define CMD_EYES 0x07
#define CMD_BOOTUP 0x08
// DMA Buffer and tracking
#define DMA_RX_BUFFER_SIZE 64 // Larger than the expected command size to ensure no bytes are missed
uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE]; // Circular Queue for DMA reception of UART data
uint16_t old_pos = 0;
//55 AA 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05
// RX Parser state machine variables
#define PAYLOAD_SIZE 45
uint8_t payload_buffer[PAYLOAD_SIZE]; // Queue to hold incoming payload bytes
uint8_t payload_index = 0;
int parser_state = 0;

// Parsed command storage
struct CmdPayload last_cmd; // Struct to hold the payload data cleanly
volatile int new_cmd_ready = 0; // Flag to indicate a new command is ready // Not used right now
uint32_t last_cmd_timestamp_ms = 0; // Extra safety to prevent stale commands

// TX telemetry
#define TELEM_PKT_SIZE 99  // 2 header + 96 payload + 1 checksum
static uint8_t telem_tx_buf[TELEM_PKT_SIZE]; // Must be static/global for DMA

/* Definitions for cmdMutex */
osMutexId_t cmdMutexHandle;
const osMutexAttr_t cmdMutex_attributes = {
  .name = "cmdMutex"
};
osMutexId_t imuMutexHandle;
const osMutexAttr_t imuMutex_attributes = {
  .name = "imuMutex"
};
/* Definitions for batteryMutex */
osMutexId_t batteryMutexHandle;
const osMutexAttr_t batteryMutex_attributes = {
  .name = "batteryMutex"
};
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for rxCommTask */
osThreadId_t rxCommTaskHandle;
const osThreadAttr_t rxCommTask_attributes = {
  .name = "rxCommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for txCommTask */
osThreadId_t txCommTaskHandle;
const osThreadAttr_t txCommTask_attributes = {
  .name = "txCommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
void MX_FREERTOS_Init(void) {
    cmdMutexHandle = osMutexNew(&cmdMutex_attributes);
    imuMutexHandle = osMutexNew(&imuMutex_attributes);
    batteryMutexHandle = osMutexNew(&batteryMutex_attributes);
    controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
    imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);
    rxCommTaskHandle = osThreadNew(StartRxCommTask, NULL, &rxCommTask_attributes);
    txCommTaskHandle = osThreadNew(StartTxCommTask, NULL, &txCommTask_attributes);
    batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);
}
void StartControlTask(void *argument)
{
  osDelay(1000); // Wait peripherals up

  SerialPrintfU2("I2C3 PCA9685 Scan...\r\n");
  I2C_Scan(&hi2c3, &huart2);

  PCA9685_Init(&hi2c3, PCA9685_I2C_ADDRESS, 0);
  PCA9685_SetOscillatorFrequency(27000000);
  PCA9685_SetPWMFreq(50.0f);

  uint32_t last_toggle_time = HAL_GetTick();
  const uint32_t toggle_interval = 100; // 100 ms
    LegIK_HardwareInit(); // Init the IK leg structs 
  struct CmdPayload active_cmd = {0}; // Struct to hold the currently active command
  static uint8_t prev_cmd_type = CMD_RESET;
  
    osDelay(1000);
    SerialPrintfU2("DTU GO Ready\r\n");
  for(;;)
  {
    // Toggle LED PD12
    if ((HAL_GetTick() - last_toggle_time) >= toggle_interval)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      last_toggle_time = HAL_GetTick();
    }
    // Safely copy the latest command from the commTask
    osMutexAcquire(cmdMutexHandle, osWaitForever);
    active_cmd = last_cmd;
    osMutexRelease(cmdMutexHandle);

    // If more than 500 milliseconds have passed since the last valid command,
    // stop the robot for safety
    if ((HAL_GetTick() - last_cmd_timestamp_ms) > 500) {
        active_cmd.cmd_type = CMD_RESET;
        active_cmd.lin_x = 0.0f;
        active_cmd.lin_y = 0.0f;
        active_cmd.ang_z = 0.0f;
        active_cmd.roll = 0.0f;
        active_cmd.pitch = 0.0f;
        active_cmd.yaw = 0.0f;
        active_cmd.z_offset = 0.0f;
    }

    // Smooth the target height, tilt, and translation values using an Exponential Moving Average (EMA)
    static float smooth_z = 0.0f;
    static float smooth_roll = 0.0f;
    static float smooth_pitch = 0.0f;
    static float smooth_yaw = 0.0f;
    static float smooth_x_offset = 0.0f;
    static float smooth_y_offset = 0.0f;
    // Filter coefficient (0.0 < alpha <= 1.0)
    // 0.15 at 50Hz gives a smooth transition over ~130ms while remaining responsive
    const float alpha = 0.15f;
    smooth_z        += alpha * (-active_cmd.z_offset - smooth_z); // Jetson sends + to lift body up. Inverse IK matrix requires -z_offset to pull body up.
    smooth_roll     += alpha * (active_cmd.roll - smooth_roll);
    smooth_pitch    += alpha * (active_cmd.pitch - smooth_pitch);
    smooth_yaw      += alpha * (active_cmd.yaw - smooth_yaw);
    smooth_x_offset += alpha * (active_cmd.x_offset - smooth_x_offset);
    smooth_y_offset += alpha * (active_cmd.y_offset - smooth_y_offset);

    // Process Kinematics and Motion state machine based on cmd_type
    switch(active_cmd.cmd_type) 
    {
        case CMD_NORMAL:
            {
                SerialPrintfU2("NORMAL\r\n");
                float target_feet[4][3];

                // Calculate raw gait foot trajectory paths relative to default stance
                calculateTrotGaitPositions(active_cmd.lin_x, active_cmd.lin_y, active_cmd.ang_z, target_feet);

                // Stream footprints through orientation matrix to apply Roll/Pitch/Yaw (using smoothed parameters)
                updateBodyPostureWithFeet(target_feet, 0.0f, 0.0f, smooth_z,
                                          smooth_roll, smooth_pitch, smooth_yaw,
                                          active_cmd.pivot_x, active_cmd.pivot_y, 0.0f);
                // Non-smoothing code:
                // // Relative Height Conversion. 
                // // Jetson sends + to lift body up. Inverse IK matrix requires -transZ to pull body up.
                // float transZ = -active_cmd.z_offset; 

                // // Stream footprints through orientation matrix to apply Roll/Pitch/Yaw
                // updateBodyPostureWithFeet(target_feet, 0.0f, 0.0f, transZ,
                //                           active_cmd.roll, active_cmd.pitch, active_cmd.yaw,
                //                           active_cmd.pivot_x, active_cmd.pivot_y, 0.0f);
            }
            break;

        case CMD_EXTRAS:
            {
                SerialPrintfU2("EXTRA\r\n");
                // In extras mode, handle relative height translation and body shifts (using smoothed parameters)
                updateBodyPosture(smooth_x_offset, smooth_y_offset, smooth_z,
                                  smooth_roll, smooth_pitch, smooth_yaw,
                                  active_cmd.pivot_x, active_cmd.pivot_y, 0.0f);
            }
            break;

        case CMD_WAVE:
            {
               SerialPrintfU2("WAVE\r\n");
                float target_feet[4][3];
                
                // Tell the function to restart the animation if we JUST switched to wave mode
                bool just_started = (prev_cmd_type != CMD_WAVE);
                
                float shiftX = 0.0f;
                float shiftY = 0.0f;
                float shiftZ = 0.0f;
                float roll = 0.0f;
                float pitch = 0.0f;
                float tibia_angle = -1.0f;
                calculateWavePositions(target_feet, &shiftX, &shiftY, &shiftZ, &roll, &pitch, &tibia_angle, just_started);
                float transZ = 0.0f; // -active_cmd.z_offset // Ignore initial Z offset for wave
                updateBodyPostureWithFeet(target_feet, shiftX, shiftY, transZ + shiftZ,
                                          active_cmd.roll + roll, active_cmd.pitch + pitch, active_cmd.yaw,
                                          0.0f, 0.0f, 0.0f);

                // If Phase 3 is active, override the Tibia servo angle with the waving angle
                if (tibia_angle >= 0.0f) {
                    setServoAngle(CH_FR_TIBIA, tibia_angle);
                }
            }
            break;

        case CMD_HEEL:
            SerialPrintfU2("heelingPose\r\n");
            heelingPose();
            break;

        case CMD_DANCE:
            SerialPrintfU2("Dance\r\n");
            for (int i = 0; i <= 30; i++) {
                SerialPrintfU2("Dance Step %d\r\n", i);
                sineStepGait();
        }
        // TODO: Implement dance
            break;

        case CMD_RESET:
        default:
            standingPose(); // Safely return to flat, neutral stance
            break;

        case CMD_EYES:
            break;
        
        case CMD_BOOTUP:
          // TODO  
          break;
    }

    // Update previous command tracker for the next loop iteration
    prev_cmd_type = active_cmd.cmd_type;
    
    osDelay(20); // 50Hz Loop Rate
  }
}
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  osDelay(1000); // Chờ ngoại vi và bus SPI/I2C ổn định

  SerialPrintfU2("I2C1 Detect...\r\n");
  I2C_Scan(&hi2c1, &huart2);
  
  // Khởi tạo IMU (L3GD20 trên SPI1 + LSM303 trên I2C1)
  if (IMU_Init(&hspi1, &hi2c1, &huart2) == HAL_OK)
  {
    SerialPrintfU2("[IMU Task] Discovery IMU Init SUCCESS!\r\n");
  }
  else
  {
    SerialPrintfU2("[IMU Task] Discovery IMU Init FAILED!\r\n");
  }

  for(;;)
  {
    // Đọc liên tục và cập nhật góc vào biến toàn cục g_imu_orientation
    if (IMU_ReadOrientation(&hi2c1, &current_imu_orientation) == HAL_OK)
    {
      // Có thể log ra màn hình debug nếu cần
       //SerialPrintfU2("Roll: %.2f | Pitch: %.2f | Yaw: %.2f\r\n", 
                    // current_imu_orientation.roll, current_imu_orientation.pitch, current_imu_orientation.yaw);
    }

    osDelay(10); // Chu kỳ cập nhật 50Hz (20ms)
  }
}
void StartRxCommTask(void *argument)
{
  HAL_UART_Receive_DMA(&huart6, dma_rx_buffer, DMA_RX_BUFFER_SIZE);

  /* Infinite loop */
  for(;;)
  {
    // Check where the DMA currently is writing.
    // __HAL_DMA_GET_COUNTER returns the number of bytes REMAINING in the buffer before 
    // it wraps around, so subtract from total size to get the current position.
    uint16_t current_pos = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

    // current_pos is the REAR of the queue where new bytes are being written, 
    // old_pos is the FRONT of the queue where we are reading from.

    // Process all new bytes that have arrived since we last checked
    // (i.e. until our REAR catches up to the FRONT)
    while (old_pos != current_pos) {
        uint8_t rx_byte = dma_rx_buffer[old_pos];
        
        // State machine to find headers, read payload, and check checksum
        switch(parser_state) {
            // This order of 0x55 and 0xAA represents tranmission from jetson to STM  
            // Looking for Header 1 (0x55)
            // If 1st header byte was found, set state to 1 to look for 2nd header byte
            case 0:
                if (rx_byte == 0x55) parser_state = 1;
                
                 // Debug: print received byte in hex
                break;
            
            // Looking for Header 2 (0xAA). 
            // If found, set state to 2 to start reading payload bytes
            // If not found, look for header 1 again
            case 1: 
                if (rx_byte == 0xAA) {
                    parser_state = 2;
                    payload_index = 0;
                } else if (rx_byte != 0x55) {
                    parser_state = 0;
                }
                break;
                
            // Reading Payload (PAYLOAD_SIZE bytes)
            // Once payload is full, verify checksum (state 3)
            case 2:
                payload_buffer[payload_index++] = rx_byte;
                if (payload_index >= PAYLOAD_SIZE) {
                    parser_state = 3;
                }
                break;
            
            // - Verify Checksum (parity)
            // - Safely copy bytes from payload_buffer directly into the last_cmd struct to 
            //   avoid alignment HardFaults. Use mutex to prevent race conditions.
            //   Payload_buffer is just a byte array, no gaurantee of correct alignment for
            //   struct access, so use memcpy which can handle unaligned access.
            // - Set a flag to indicate data is ready for use
            // - Reset state machine for next packet
            case 3: ;        
                uint8_t received_checksum = rx_byte;
                uint8_t calculated_checksum = 0;   
                for (int i = 0; i < PAYLOAD_SIZE; i++) {
                    calculated_checksum ^= payload_buffer[i];
                }
                if (calculated_checksum == received_checksum) {
                    osMutexAcquire(cmdMutexHandle, osWaitForever);
                    memcpy(&last_cmd, payload_buffer, sizeof(struct CmdPayload));
                    new_cmd_ready = 1;
                    last_cmd_timestamp_ms = HAL_GetTick();
                    osMutexRelease(cmdMutexHandle);
                }else{
                    // checksum_errors++;
                }
                parser_state = 0;
                break;
        }

        // Advance our read pointer, wrapping around if needed
        old_pos = (old_pos + 1) % DMA_RX_BUFFER_SIZE;
    }

    // Let FreeRTOS give CPU time to other tasks
    osDelay(10); // 100Hz command processing rate
  }
}
void StartTxCommTask(void *argument)
{
  /* USER CODE BEGIN StartTxCommTask */
  /* Infinite loop */
  for(;;)
  {
      // Wait for previous DMA transfer to complete
      if (huart6.gState == HAL_UART_STATE_READY) {
          // Snapshot battery readings
          INA3221_ReadingsTypeDef batt_snap;
          osMutexAcquire(batteryMutexHandle, osWaitForever);
          batt_snap = current_battery_readings;
          osMutexRelease(batteryMutexHandle);

          // Snapshot imu reading
          IMU_OrientationTypeDef imu_snap;
          osMutexAcquire(imuMutexHandle, osWaitForever);
          imu_snap = current_imu_orientation;
          osMutexRelease(imuMutexHandle);

          // Snapshot joint angles
          float joint_angles[4][3];
          getJointAngles(joint_angles);

          // Build packet: [0xAA][0x55][3 floats = 12 bytes][9 floats = 36 bytes][12 floats = 48 bytes][XOR checksum]
          // This particular order represents transmission from STM to jetson
          telem_tx_buf[0] = 0xAA;
          telem_tx_buf[1] = 0x55;
          float telem_floats[24] = {
              imu_snap.roll,
              imu_snap.pitch,
              imu_snap.yaw,
              batt_snap.bus_voltage_V[0],
              batt_snap.bus_voltage_V[1],
              batt_snap.bus_voltage_V[2],
              batt_snap.bus_voltage_V[3],
              batt_snap.bus_voltage_V[4],
              batt_snap.bus_voltage_V[5],
              batt_snap.bus_voltage_V[6],
              batt_snap.bus_voltage_V[7],
              batt_snap.bus_voltage_V[8],
              joint_angles[0][0],
              joint_angles[0][1],
              joint_angles[0][2],
              joint_angles[1][0],
              joint_angles[1][1],
              joint_angles[1][2],
              joint_angles[2][0],
              joint_angles[2][1],
              joint_angles[2][2],
              joint_angles[3][0],
              joint_angles[3][1],
              joint_angles[3][2],
          };

          memcpy(&telem_tx_buf[2], telem_floats, sizeof(telem_floats));
          
          // Checksum only the payload
          uint8_t cksum = 0;
          for (int i = 2; i < TELEM_PKT_SIZE - 1; i++) {
              cksum ^= telem_tx_buf[i];
          }
          telem_tx_buf[TELEM_PKT_SIZE - 1] = cksum;
          
          // Fire-and-forget DMA transmit (non-blocking, ~0.9ms at 460800 baud)
          // Needs USART1 global interrupt enabled
          HAL_UART_Transmit_DMA(&huart6, telem_tx_buf, TELEM_PKT_SIZE);
          
      //     // Debug: Transmit succeeded
      //     char msg[] = "TX Task: Sent via DMA\r\n";
      //     HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
      // } else {
      //     // Debug: UART is stuck busy!
      //     char msg[] = "TX Task: UART1 BUSY!\r\n";
      //     HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
      }
      osDelay(100);
  }
}
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  osDelay(2000);
  INA3221_Init(&hi2c3, &huart2);
  INA3221_ReadingsTypeDef readings;

  /* Infinite loop */
  for(;;)
  {
    if (INA3221_ReadAll(&hi2c3, &readings) == HAL_OK) {
      osMutexAcquire(batteryMutexHandle, osWaitForever);
      current_battery_readings = readings;
      osMutexRelease(batteryMutexHandle);
    }
    osDelay(1000); // 1 Hz as battery voltage does not change fast
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        // uart_errors++;
        
        // Clear overrun and noise flags so DMA reception doesn't lock up
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        // Restart circular DMA reception
        HAL_UART_Receive_DMA(huart, dma_rx_buffer, DMA_RX_BUFFER_SIZE);
    }
}