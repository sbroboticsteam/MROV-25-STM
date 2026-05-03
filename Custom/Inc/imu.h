#ifndef IMU_H
#define IMU_H

#define BNO085_ADDR 0x4A<<1
// #define BNO085_ADDR 0x4A

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

typedef struct IMU_Packet{
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;

    float mag_x;
    float mag_y;
    float mag_z;
} IMU_Packet;

typedef struct IMU_Packet_Fused{
    float euler_heading;
    float euler_roll;
    float euler_pitch;

    // QUATERNION
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
} IMU_Packet_Fused;


int imu_read_nb(uint8_t * buffer, uint16_t n, uint8_t addr, I2C_HandleTypeDef* hisc1);

int imu_write_1b(uint8_t addr, uint8_t value, I2C_HandleTypeDef* hisc1);

// ========= NON FUSION =========
//GYRO

float get_gyro_x(I2C_HandleTypeDef* hisc1);

float get_gyro_y(I2C_HandleTypeDef* hisc1);

float get_gyro_z(I2C_HandleTypeDef* hisc1);

// ACCEL

float get_accel_x(I2C_HandleTypeDef* hisc1);

float get_accel_y(I2C_HandleTypeDef* hisc1);

float get_accel_z(I2C_HandleTypeDef* hisc1);

// MAG

float get_mag_x(I2C_HandleTypeDef* hisc1);

float get_mag_y(I2C_HandleTypeDef* hisc1);

float get_mag_z(I2C_HandleTypeDef* hisc1);

// ========= FUSION =========

// EULER 

float get_euler_heading(I2C_HandleTypeDef* hisc1);

float get_euler_roll(I2C_HandleTypeDef* hisc1);

float get_euler_pitch(I2C_HandleTypeDef* hisc1);

// QUATERNION

float get_quat_w(I2C_HandleTypeDef* hisc1);

float get_quat_x(I2C_HandleTypeDef* hisc1);

float get_quat_y(I2C_HandleTypeDef* hisc1);

float get_quat_z(I2C_HandleTypeDef* hisc1);



#endif