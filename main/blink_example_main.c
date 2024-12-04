/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
//#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050_app";
static mpu6050_handle_t mpu6050 = NULL;

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

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C configuration failed!");
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed!");
        return;
    }

    ESP_LOGI(TAG, "I2C initialized successfully.");
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
        if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "Failed to create MPU6050 handle!");
        return;
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050!");
        return;
    }

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050!");
        return;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully.");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MPU6050 application...");

    // Initialize MPU6050
    i2c_sensor_mpu6050_init();

    // Check MPU6050 device ID
    uint8_t mpu6050_deviceid;
    esp_err_t ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    if (ret == ESP_OK && mpu6050_deviceid == MPU6050_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "MPU6050 detected successfully!");
    } else {
        ESP_LOGE(TAG, "Failed to detect MPU6050. Check connections!");
        return;
    }

    // Read and log sensor data
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Accel: x=%.2f, y=%.2f, z=%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data.");
        }

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Gyro: x=%.2f, y=%.2f, z=%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        } else {
            ESP_LOGE(TAG, "Failed to read gyroscope data.");
        }

        ret = mpu6050_get_temp(mpu6050, &temp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f°C", temp.temp);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature.");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}


