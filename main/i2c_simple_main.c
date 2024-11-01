
// #include <stdio.h>
// #include <string.h>
// #include "unity.h"
// #include "driver/i2c.h"
// #include "driver/uart.h"
// #include "sdkconfig.h"
// #include "mpu6050.h"
// #include "esp_system.h"
// #include "esp_log.h"


// #define I2C_MASTER_SCL_IO 18      /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO 19      /*!< gpio number for I2C master data  */
// #define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
// #define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

// static const char *TAG = "mpu6050 test";
// static mpu6050_handle_t mpu6050 = NULL;
// static uart_port_t uart_num = UART_NUM_0;

// /**
//  * @brief i2c master initialization
//  */
// static void i2c_bus_init(void)
// {
//     i2c_config_t conf;
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//     conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

//     esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "I2C config returned error");
//         return;
//     }

//     ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "I2C install returned error");
//         return;
//     }
// }

// /**
//  * @brief i2c master initialization
//  */
// static void i2c_sensor_mpu6050_init(void)
// {
//     esp_err_t ret;

//     i2c_bus_init();
//     mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
//     if (mpu6050 == NULL)
//     {
//         ESP_LOGE(TAG, "MPU6050 create returned NULL");
//         return;
//     }

//     ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "MPU6050 config error");
//         return;
//     }

//     ret = mpu6050_wake_up(mpu6050);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "MPU6050 wake up error");
//         return;
//     }
// }

// /*
//  * Initialize uart port
//  */
// static void uart_setup()
// {
//     uart_num = UART_NUM_0;
//     const int uart_buffer_size = (1024 * 2);
//     QueueHandle_t uart_queue;

//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .rx_flow_ctrl_thresh = 122,
//         // .use_ref_tick        = false,
//     };

//     // Configure UART parameters
//     ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
//     // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
//     // ESP_ERROR_CHECK(uart_set_pin(uart_num, 13, 26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
//     ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
// }

// void app_main()
// {
//     esp_err_t ret;
//     uint8_t mpu6050_deviceid;
//     mpu6050_acce_value_t acce;
//     mpu6050_gyro_value_t gyro;
//     mpu6050_temp_value_t temp;

//     i2c_sensor_mpu6050_init();
//     uart_setup();

//     for (int i = 0; i < 5; i++)
//     {
//         char *test_str = "This is a test string.\n";
//         ret = uart_write_bytes(uart_num, (const char *)test_str, strlen(test_str));
//         if (ret != ESP_OK)
//         {
//             ESP_LOGE(TAG, "Fail to send uart message");
//         }
//     }

//     ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to get MPU6050 device ID");
//     }

//     ret = mpu6050_get_acce(mpu6050, &acce);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to get accelerometer data");
//     }

//     ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

//     ret = mpu6050_get_gyro(mpu6050, &gyro);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to get gyroscope data");
//     }

//     ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

//     ret = mpu6050_get_temp(mpu6050, &temp);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to get temperature data");
//     }

//     ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

//     mpu6050_delete(mpu6050);
//     ret = i2c_driver_delete(I2C_MASTER_NUM);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to delete I2C driver");
//     }

    
// }


#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include "unity.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "mpu6050.h"

#define I2C_MASTER_SCL_IO 18      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 19      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

static void initUart();
static void Receive_callback(void *argument);
static void i2c_bus_init(void);
static void i2c_sensor_mpu6050_init(void);

void app_main(void)
{

    initUart();
    i2c_sensor_mpu6050_init();

    // printf("hello world after init Uart\n");

    xTaskCreate(Receive_callback, "UART receive callback", 4096, NULL, 5, NULL); // receiving commands from main uart
}

static void initUart()
{

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config;
    usb_serial_jtag_config.rx_buffer_size = 1024;
    usb_serial_jtag_config.tx_buffer_size = 1024;

    esp_err_t ret = ESP_OK;
    /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK)
    {
        return;
    }

    /* Tell vfs to use usb-serial-jtag driver */
    esp_vfs_usb_serial_jtag_use_driver();
    
}

static void Receive_callback(void *argument)
{
    // esp_err_t ret;
    // uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    for (;;)
    {
        uint8_t ch;
        ch = fgetc(stdin);
        if (ch != 0xFF)
        {
            printf("  acce_x  acce_y  acce_z  gyro_x  gyro_y  gyro_z   temp\n");
            for (int i = 0; i < 500; i++){
                mpu6050_get_acce(mpu6050, &acce);
                mpu6050_get_gyro(mpu6050, &gyro);
                mpu6050_get_temp(mpu6050, &temp);
                // printf("acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
                // printf("gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
                // printf("t:%.2f \n", temp.temp);
                printf("%8.2f%8.2f%8.2f%8.2f%8.2f%8.2f%8.2f\n", acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z, temp.temp);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to get MPU6050 device ID");
    // }

    // ret = mpu6050_get_acce(mpu6050, &acce);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to get accelerometer data");
    // }

    // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    // ret = mpu6050_get_gyro(mpu6050, &gyro);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to get gyroscope data");
    // }

    // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    // ret = mpu6050_get_temp(mpu6050, &temp);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to get temperature data");
    // }

    // ESP_LOGI(TAG, "t:%.2f \n", temp.temp);

    // mpu6050_delete(mpu6050);
    // ret = i2c_driver_delete(I2C_MASTER_NUM);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to delete I2C driver");
    // }
}

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C config returned error");
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C install returned error");
        return;
    }
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL)
    {
        ESP_LOGE(TAG, "MPU6050 create returned NULL");
        return;
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 config error");
        return;
    }

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 wake up error");
        return;
    }
}
