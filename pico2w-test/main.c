#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// -----------------------------
// CONFIG
// -----------------------------
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15

#define MPU_ADDR 0x68

// MPU6050 registers
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_CONFIG   0x1C
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_XOUT_H   0x3B

// --------------------------------------------
// GLOBAL OFFSETS
// --------------------------------------------
int16_t accel_offset[3] = {0, 0, 0};
int16_t gyro_offset[3]  = {0, 0, 0};

// -----------------------------
// MPU INIT
// -----------------------------
void mpu6050_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

void mpu6050_init() {

    // Wake up MPU6050
    mpu6050_write_reg(REG_PWR_MGMT_1, 0x00);
    sleep_ms(50);

    // Accelerometer scale: ±4g (FS_SEL = 01 → 0x08)
    // bit4:3 = 01, tutto il resto deve essere 0
    mpu6050_write_reg(REG_ACCEL_CONFIG, 0x00);

    // Gyroscope scale: ±500°/s (FS_SEL = 01 → 0x08)
    mpu6050_write_reg(REG_GYRO_CONFIG, 0x00);

    sleep_ms(10);
}

// -----------------------------
// READ RAW SENSOR VALUES
// -----------------------------
void mpu6050_read_raw(int16_t *accel, int16_t *gyro) {

    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t data[14];

    // Select starting register
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);

    // Read 14 bytes: accel(6), temp(2), gyro(6)
    i2c_read_blocking(I2C_PORT, MPU_ADDR, data, 14, false);

    // Accelerometer
    accel[0] = (int16_t)((data[0] << 8) | data[1]);
    accel[1] = (int16_t)((data[2] << 8) | data[3]);
    accel[2] = (int16_t)((data[4] << 8) | data[5]);

    // Gyroscope
    gyro[0] = (int16_t)((data[8] << 8) | data[9]);
    gyro[1] = (int16_t)((data[10] << 8) | data[11]);
    gyro[2] = (int16_t)((data[12] << 8) | data[13]);
}

// --------------------------------------------
// CALIBRAZIONE DI 5 SECONDI
// --------------------------------------------
void calibrate_mpu6050() {

    printf("Calibrazione in corso... tieni fermo il sensore.\n");

    const float duration = 5.0f;   // 5 secondi
    const int samples = (int)(duration * 1000.0f / 5.0f);  
    // leggo ogni 5 ms → ~1000 samples

    int32_t acc_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};

    int16_t acc[3], gyro[3];

    for (int i = 0; i < samples; i++) {
        mpu6050_read_raw(acc, gyro);

        acc_sum[0] += acc[0];
        acc_sum[1] += acc[1];
        acc_sum[2] += acc[2];

        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];

        sleep_ms(5);
    }

    // MEDIA
    accel_offset[0] = (acc_sum[0] / samples);
    accel_offset[1] = (acc_sum[1] / samples);
    accel_offset[2] = (acc_sum[2] / samples);

    gyro_offset[0] = (gyro_sum[0] / samples);
    gyro_offset[1] = (gyro_sum[1] / samples);
    gyro_offset[2] = (gyro_sum[2] / samples);

    // ACCELEROMETRO: compensiamo la gravità di 1g sull’asse Z
    // 1 g = 8192 RAW a ±4g
    accel_offset[2] -= 8192;

    printf("Calibrazione completata.\n");
    printf("Offset accelerometro: X=%d  Y=%d  Z=%d\n",
           accel_offset[0], accel_offset[1], accel_offset[2]);
    printf("Offset giroscopio:    X=%d  Y=%d  Z=%d\n",
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

// -----------------------------
// MAIN
// -----------------------------
int main() {

    stdio_init_all();
    sleep_ms(2000);

    // Init I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Init MPU6050
    mpu6050_init();
    printf("MPU6050 inizializzato.\n");

    // Calibrazione
    calibrate_mpu6050();

    int16_t accel[3], gyro[3];

    while (1) {

        mpu6050_read_raw(accel, gyro);

        // Applichiamo gli offset
        int16_t ax_r = accel[0] - accel_offset[0];
        int16_t ay_r = accel[1] - accel_offset[1];
        int16_t az_r = accel[2] - accel_offset[2];

        int16_t gx_r = gyro[0] - gyro_offset[0];
        int16_t gy_r = gyro[1] - gyro_offset[1];
        int16_t gz_r = gyro[2] - gyro_offset[2];

        // Conversione
        float ax = ax_r / 8192.0f;    // ±4g
        float ay = ay_r / 8192.0f;
        float az = az_r / 8192.0f;

        float gx = gx_r / 65.5f;      // ±500°/s
        float gy = gy_r / 65.5f;
        float gz = gz_r / 65.5f;

        printf("%.3f %.3f %.3f %.3f %.3f %.3f\n", ax, ay, az, gx, gy, gz);

        sleep_ms(200);
    }
}



