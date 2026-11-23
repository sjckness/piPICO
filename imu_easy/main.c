#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// -----------------------------
// CONFIG
// -----------------------------
#define I2C_PORT i2c1
#define I2C_SDA  14
#define I2C_SCL  15

#define MPU_ADDR 0x68

// MPU6050 registers
#define REG_PWR_MGMT_1     0x6B
#define REG_GYRO_FILTER    0x1A
#define REG_ACCEL_CONFIG   0x1C
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_XOUT_H   0x3B

// --------------------------------------------
// GLOBAL OFFSETS (in unità fisiche: g e °/s)
// --------------------------------------------
float accel_offset[3] = {0.0f, 0.0f, 0.0f};
float gyro_offset[3]  = {0.0f, 0.0f, 0.0f};


/**
 * Aggiorna il filtro di Kalman.
 * newAngle: angolo da accelerometro (deg)
 * newRate:  velocità angolare da gyro (deg/s)
 * dt:       intervallo di campionamento (s)
 */

// -----------------------------
// MPU6050 LOW LEVEL
// -----------------------------
void mpu6050_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

void mpu6050_init() {
    // Wake up MPU6050
    mpu6050_write_reg(REG_PWR_MGMT_1, 0x00);
    sleep_ms(50);

    // Accelerometer scale: ±4g → 0x08
    mpu6050_write_reg(REG_ACCEL_CONFIG, 0x08);

    // Gyroscope scale: ±500°/s → 0x08
    mpu6050_write_reg(REG_GYRO_CONFIG, 0x08);

    // Gyroscope/LPF config (a tua scelta, qui esempio 0x05)
    mpu6050_write_reg(REG_GYRO_FILTER, 0x05);

    sleep_ms(10);
}

/**
 * Legge accelerometro (g) e gyro (°/s)
 * accel[0..2] = ax, ay, az
 * gyro[0..2]  = gx, gy, gz
 */
void mpu6050_read(float *accel, float *gyro) {
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t data[14];
    int16_t accel_raw[3], gyro_raw[3];

    // Select starting register
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);

    // Read 14 bytes: accel(6), temp(2), gyro(6)
    i2c_read_blocking(I2C_PORT, MPU_ADDR, data, 14, false);

    // Accelerometer raw
    accel_raw[0] = (int16_t)((data[0] << 8) | data[1]);
    accel_raw[1] = (int16_t)((data[2] << 8) | data[3]);
    accel_raw[2] = (int16_t)((data[4] << 8) | data[5]);

    // Gyroscope raw
    gyro_raw[0] = (int16_t)((data[8] << 8) | data[9]);
    gyro_raw[1] = (int16_t)((data[10] << 8) | data[11]);
    gyro_raw[2] = (int16_t)((data[12] << 8) | data[13]);

    // Conversione in unità fisiche
    // ±4g → 8192 LSB/g
    accel[0] = (float)accel_raw[0] / 8192.0f;
    accel[1] = (float)accel_raw[1] / 8192.0f;
    accel[2] = (float)accel_raw[2] / 8192.0f;

    // ±500°/s → 65.5 LSB/(°/s)
    gyro[0] = (float)gyro_raw[0] / 65.5f;
    gyro[1] = (float)gyro_raw[1] / 65.5f;
    gyro[2] = (float)gyro_raw[2] / 65.5f;
}

// --------------------------------------------
// CALIBRAZIONE DI 5 SECONDI
// --------------------------------------------
void calibrate_mpu6050() {
    printf("Calibrazione in corso... tieni fermo il sensore.\n");

    const float duration_s = 5.0f;
    const float sample_dt_ms = 5.0f;
    const int samples = (int)(duration_s * 1000.0f / sample_dt_ms);

    float acc[3], gyro[3];
    double acc_sum[3]  = {0.0, 0.0, 0.0};
    double gyro_sum[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < samples; i++) {
        mpu6050_read(acc, gyro);

        acc_sum[0]  += acc[0];
        acc_sum[1]  += acc[1];
        acc_sum[2]  += acc[2];

        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];

        sleep_ms((uint32_t)sample_dt_ms);
    }

    accel_offset[0] = (float)(acc_sum[0] / samples);
    accel_offset[1] = (float)(acc_sum[1] / samples);
    accel_offset[2] = (float)(acc_sum[2] / samples);

    gyro_offset[0]  = (float)(gyro_sum[0] / samples);
    gyro_offset[1]  = (float)(gyro_sum[1] / samples);
    gyro_offset[2]  = (float)(gyro_sum[2] / samples);

    // L'IMU è ferma: ci aspettiamo ~1g su Z
    accel_offset[2] -= 1.0f; // togli la gravità dall'offset

    printf("Calibrazione completata.\n");
    printf("Offset accelerometro (g): X=%.4f  Y=%.4f  Z=%.4f\n",
           accel_offset[0], accel_offset[1], accel_offset[2]);
    printf("Offset giroscopio (deg/s): X=%.4f  Y=%.4f  Z=%.4f\n",
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

    float accel[3], gyro[3];

    // Per calcolare dt
    absolute_time_t last_time = get_absolute_time();

    // Inizializzazione Kalman usando l'angolo da accelerometro
    // (aspettiamo una prima lettura valida)
    mpu6050_read(accel, gyro);
    // Compensa offset
    accel[0] -= accel_offset[0];
    accel[1] -= accel_offset[1];
    accel[2] -= accel_offset[2];



    // Yaw solo integrato (senza magnetometro → deriva nel tempo)
    float yaw = 0.0f;

    while (1) {
        absolute_time_t now = get_absolute_time();
        float dt = (float)absolute_time_diff_us(last_time, now) / 1e6f;
        last_time = now;

        // Lettura IMU
        mpu6050_read(accel, gyro);

        // Applichiamo gli offset (g e °/s)
        float ax = accel[0] - accel_offset[0];
        float ay = accel[1] - accel_offset[1];
        float az = accel[2] - accel_offset[2];

        float gx = gyro[0] - gyro_offset[0];
        float gy = gyro[1] - gyro_offset[1];
        float gz = gyro[2] - gyro_offset[2];

        // Stampa su seriale: qw qx qy qz
        printf("%.4f %.4f %.4f %.4f %.4f %.4f\n", ax, ay, az, gx, gy, gz);

        // Se vuoi ~100Hz, evita di dormire troppo a lungo.
        // Puoi anche mettere sleep_ms(5) per alleggerire un po' la CPU.
        sleep_ms(5);
    }
}




