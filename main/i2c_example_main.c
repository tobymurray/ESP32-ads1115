/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ADS1115_SENSOR_ADDRESS ADS1115_ADDRESS   /*!< slave address for BH1750 sensor */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADS1X15_ADDRESS (0x48)  // 1001 000 (ADDR = GND)
                                /*=========================================================================*/

/*=========================================================================
    Default thresholds for comparator
    -----------------------------------------------------------------------*/
#define ADS1X15_LOW_THRESHOLD_DEFAULT (0x8000)
#define ADS1X15_HIGH_THRESHOLD_DEFAULT (0x7FFF)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_POINTER_MASK (0x03)
#define ADS1X15_REG_POINTER_CONVERT (0x00)
#define ADS1X15_REG_POINTER_CONFIG (0x01)
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02)
#define ADS1X15_REG_POINTER_HITHRESH (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_CONFIG_OS_MASK (0x8000)
#define ADS1X15_REG_CONFIG_OS_SINGLE (0x8000)   // Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY (0x0000)     // Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000)  // +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200)  // +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600)  // +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800)  // +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000)  // Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100)  // Power-down single-shot mode (default)

#define ADS1015_REG_CONFIG_DR_MASK (0x00E0)
#define ADS1015_REG_CONFIG_DR_128SPS (0x0000)   // 128 samples per second
#define ADS1015_REG_CONFIG_DR_250SPS (0x0020)   // 250 samples per second
#define ADS1015_REG_CONFIG_DR_490SPS (0x0040)   // 490 samples per second
#define ADS1015_REG_CONFIG_DR_920SPS (0x0060)   // 920 samples per second
#define ADS1015_REG_CONFIG_DR_1600SPS (0x0080)  // 1600 samples per second (default)
#define ADS1015_REG_CONFIG_DR_2400SPS (0x00A0)  // 2400 samples per second
#define ADS1015_REG_CONFIG_DR_3300SPS (0x00C0)  // 3300 samples per second

#define ADS1115_REG_CONFIG_DR_8SPS (0x0000)    // 8 samples per second
#define ADS1115_REG_CONFIG_DR_16SPS (0x0020)   // 16 samples per second
#define ADS1115_REG_CONFIG_DR_32SPS (0x0040)   // 32 samples per second
#define ADS1115_REG_CONFIG_DR_64SPS (0x0060)   // 64 samples per second
#define ADS1115_REG_CONFIG_DR_128SPS (0x0080)  // 128 samples per second (default)
#define ADS1115_REG_CONFIG_DR_250SPS (0x00A0)  // 250 samples per second
#define ADS1115_REG_CONFIG_DR_475SPS (0x00C0)  // 475 samples per second
#define ADS1115_REG_CONFIG_DR_860SPS (0x00E0)  // 860 samples per second

#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010)
#define ADS1X15_REG_CONFIG_CMODE_TRAD (0x0000)    // Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008)
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI (0x0008)   // ALERT/RDY pin is high when active

#define ADS1X15_REG_CONFIG_CLAT_MASK (0x0004)    // Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT (0x0000)  // Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004)   // Latching comparator

#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003)
#define ADS1X15_REG_CONFIG_CQUE_1CONV (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE (0x0003)   // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/*=========================================================================
    GAIN VOLTAGES
    -----------------------------------------------------------------------*/
#define ADS1115_VOLTS_PER_BIT_GAIN_TWOTHIRDS 0.0001875F
#define ADS1115_VOLTS_PER_BIT_GAIN_ONE 0.000125F
#define ADS1115_VOLTS_PER_BIT_GAIN_TWO 0.0000625F
#define ADS1115_VOLTS_PER_BIT_GAIN_FOUR 0.00003125F
#define ADS1115_VOLTS_PER_BIT_GAIN_EIGHT 0.000015625F
#define ADS1115_VOLTS_PER_BIT_GAIN_SIXTEEN 0.0000078125F

#define ADS1015_VOLTS_PER_BIT_GAIN_TWOTHIRDS 0.003F
#define ADS1015_VOLTS_PER_BIT_GAIN_ONE 0.002F
#define ADS1015_VOLTS_PER_BIT_GAIN_TWO 0.001F
#define ADS1015_VOLTS_PER_BIT_GAIN_FOUR 0.0005F
#define ADS1015_VOLTS_PER_BIT_GAIN_EIGHT 0.00025F
#define ADS1015_VOLTS_PER_BIT_GAIN_SIXTEEN 0.000125F
/*=========================================================================*/

/*=========================================================================
    CHIP BASED BIT SHIFT
    -----------------------------------------------------------------------*/
#define ADS1015_CONV_REG_BIT_SHIFT_4 4
#define ADS1115_CONV_REG_BIT_SHIFT_0 0

/*=========================================================================*/

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l) {
  // Start with default values
  uint16_t config = ADS1X15_REG_CONFIG_CQUE_NONE |     // Disable the comparator (default val)
                    ADS1X15_REG_CONFIG_CLAT_NONLAT |   // Non-latching (default val)
                    ADS1X15_REG_CONFIG_CPOL_ACTVLOW |  // Alert/Rdy active low   (default val)
                    ADS1X15_REG_CONFIG_CMODE_TRAD |    // Traditional comparator (default val)
                    ADS1X15_REG_CONFIG_MODE_SINGLE;    // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1X15_REG_CONFIG_PGA_6_144V;

  // Set Samples per Second
  config |= 0x0080;  // 128

  // Set channels
  config |= ADS1X15_REG_CONFIG_MUX_DIFF_0_1;  // set P and N inputs for differential

  // Set 'start single-conversion' bit
  config |= ADS1X15_REG_CONFIG_OS_SINGLE;

  ESP_LOGI(TAG, "Read is %d and write is %d", READ_BIT, WRITE_BIT);

  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDRESS << 1 | WRITE_BIT, ACK_VAL);
  i2c_master_write_byte(cmd, ADS1X15_REG_POINTER_CONFIG, ACK_VAL);
  i2c_master_write_byte(cmd, (uint8_t)(config >> 8), ACK_VAL);
  i2c_master_write_byte(cmd, (uint8_t)config, ACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  ESP_LOGI(TAG, "Return value from pointing to the config register: %d", ret);

  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data_h, ACK_VAL);
  i2c_master_read_byte(cmd, data_l, ACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  ESP_LOGI(TAG, "Return value from trying to read from the config register: %d", ret);

  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDRESS << 1 | WRITE_BIT, ACK_VAL);
  i2c_master_write_byte(cmd, ADS1X15_REG_POINTER_CONVERT, ACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  ESP_LOGI(TAG, "Return value from pointing to the conversion register: %d", ret);

  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data_h, ACK_VAL);
  i2c_master_read_byte(cmd, data_l, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  ESP_LOGI(TAG, "Return value from trying to read from the conversion register: %d", ret);

  return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_test_task(void *arg) {
  int ret;
  uint32_t task_idx = (uint32_t)arg;
  uint8_t sensor_data_h, sensor_data_l;
  int cnt = 0;
  while (1) {
    ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
    ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
    xSemaphoreTake(print_mux, portMAX_DELAY);

    // Convert the two u_int8_t sensor reading into a single u_int16_t
    u_int16_t raw_measurement = (sensor_data_h << 8 | sensor_data_l);

    // Convert the unsigned u_int16_t into a signed integer
    int raw_measurement_as_signed_int = (0x8000 & raw_measurement ? (int)(0x7FFF & raw_measurement) - 0x8000 : raw_measurement);

    // Undo the effect of the gain
    float millivolts_before_gain = raw_measurement_as_signed_int * 0.1875;

    // Convert to volts
    float voltage = millivolts_before_gain / 1000;

    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      printf("*******************\n");
      printf("The voltage is %.02f\n", voltage);
      printf("*******************\n");
    } else {
      ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
    }
    xSemaphoreGive(print_mux);
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
  }
  vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

void app_main(void) {
  print_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_master_init());
  xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
