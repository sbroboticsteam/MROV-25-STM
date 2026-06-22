#ifndef IMU_H
#define IMU_H

#define BNO055_ADDR 0x28<<1

typedef struct Gyro{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} Gyro;

typedef struct Accel{
    float accel_x;
    float accel_y;
    float accel_z;
} Accel;

typedef struct Mag{
    float mag_x;
    float mag_y;
    float mag_z;
} Mag;

typedef struct IMU_Raw{
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;

    float mag_x;
    float mag_y;
    float mag_z;
} IMU_Raw;

typedef struct IMU_Fused{
    float euler_heading;
    float euler_roll;
    float euler_pitch;

    // QUATERNION
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
} IMU_Fused;

typedef union IMU_Data{
    IMU_Raw raw;
    IMU_Fused fused;
}


int imu_read_nb(uint8_t * buffer, uint16_t n, uint8_t addr, I2C_HandleTypeDef* hi2c);

int imu_write_1b(uint8_t addr, uint8_t value, I2C_HandleTypeDef* hi2c);

// ========= NON FUSION =========
//GYRO

float get_gyro_x(I2C_HandleTypeDef* hi2c);

float get_gyro_y(I2C_HandleTypeDef* hi2c);

float get_gyro_z(I2C_HandleTypeDef* hi2c);

// ACCEL

float get_accel_x(I2C_HandleTypeDef* hi2c);

float get_accel_y(I2C_HandleTypeDef* hi2c);

float get_accel_z(I2C_HandleTypeDef* hi2c);

// MAG

float get_mag_x(I2C_HandleTypeDef* hi2c);

float get_mag_y(I2C_HandleTypeDef* hi2c);

float get_mag_z(I2C_HandleTypeDef* hi2c);

// ========= FUSION =========

// EULER 

float get_euler_heading(I2C_HandleTypeDef* hi2c);

float get_euler_roll(I2C_HandleTypeDef* hi2c);

float get_euler_pitch(I2C_HandleTypeDef* hi2c);

// QUATERNION

float get_quat_w(I2C_HandleTypeDef* hi2c);

float get_quat_x(I2C_HandleTypeDef* hi2c);

float get_quat_y(I2C_HandleTypeDef* hi2c);

float get_quat_z(I2C_HandleTypeDef* hi2c);

// PACKETS
IMU_Raw get_packet(I2C_HandleTypeDef* hi2c);
IMU_Fused get_packet_fused(I2C_HandleTypeDef* hi2c);


#endif