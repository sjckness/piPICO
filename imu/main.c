#include <stdio.h>
#include <math.h>
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

// -----------------------------
// KALMAN FILTER
// -----------------------------
typedef struct {
    float angle;   // angolo stimato (deg)
    float bias;    // bias stimato del gyro (deg/s)
    float P[2][2]; // matrice di covarianza
} Kalman;

Kalman kalman_roll  = {0};
Kalman kalman_pitch = {0};

// Parametri Kalman (puoi ritoccarli)
const float Q_angle   = 0.001f; // varianza processo per angolo
const float Q_bias    = 0.003f; // varianza processo per bias
const float R_measure = 0.03f;  // varianza misura (accelerometro)

/**
 * Aggiorna il filtro di Kalman.
 * newAngle: angolo da accelerometro (deg)
 * newRate:  velocità angolare da gyro (deg/s)
 * dt:       intervallo di campionamento (s)
 */
float kalman_update(Kalman *k, float newAngle, float newRate, float dt) {
    // --- PREDICTION ---
    float rate = newRate - k->bias;
    k->angle += dt * rate;

    // Aggiorna P
    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += Q_bias * dt;

    // --- UPDATE ---
    float S = k->P[0][0] + R_measure; // innovazione
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    float y = newAngle - k->angle;
    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    float P00 = k->P[0][0];
    float P01 = k->P[0][1];

    k->P[0][0] -= K[0] * P00;
    k->P[0][1] -= K[0] * P01;
    k->P[1][0] -= K[1] * P00;
    k->P[1][1] -= K[1] * P01;

    return k->angle;
}

// -----------------------------
// QUATERNION (da roll, pitch, yaw)
// -----------------------------
typedef struct {
    float w, x, y, z;
} Quaternion;

// roll, pitch, yaw in gradi
Quaternion euler_to_quaternion(float roll_deg, float pitch_deg, float yaw_deg) {
    float roll  = roll_deg  * (float)3.14f / 180.0f;
    float pitch = pitch_deg * (float)3.14f / 180.0f;
    float yaw   = yaw_deg   * (float)3.14f / 180.0f;

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * cp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

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

    // Angoli da accelerometro (in gradi) – convenzione classica
    float rollAcc  = atan2f(accel[1], accel[2]) * 180.0f / (float)3.14;
    float pitchAcc = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0f / (float)3.14;

    kalman_roll.angle  = rollAcc;
    kalman_pitch.angle = pitchAcc;

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

        // Angoli da accelerometro (gradi)
        rollAcc  = atan2f(ay, az) * 180.0f / (float)3.14;
        pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)3.14;

        // Filtro di Kalman su roll e pitch
        float roll  = kalman_update(&kalman_roll,  rollAcc,  gx, dt);
        float pitch = kalman_update(&kalman_pitch, pitchAcc, gy, dt);

        // Yaw: integriamo solo il gyro (occhio alla deriva)
        yaw += gz * dt;

        // Converte in quaternione
        Quaternion q = euler_to_quaternion(roll, pitch, yaw);

        // Stampa su seriale: qw qx qy qz
        printf("%.6f %.6f %.6f %.6f\n", q.w, q.x, q.y, q.z);

        // Se vuoi ~100Hz, evita di dormire troppo a lungo.
        // Puoi anche mettere sleep_ms(5) per alleggerire un po' la CPU.
        sleep_ms(5);
    }
}




