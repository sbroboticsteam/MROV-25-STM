#include "main.h"
#include "../../Custom/Inc/imu.h"
#include <stdio.h>

int imu_read_nb(uint8_t * buffer, uint16_t n, uint8_t addr, I2C_HandleTypeDef* hi2c){
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Read(
    hi2c,
    BNO055_ADDR, 
    addr, 
    I2C_MEMADD_SIZE_8BIT, 
    buffer, 
    n, 
    HAL_MAX_DELAY
  );
  return status == HAL_OK;
}


int imu_write_1b(uint8_t addr, uint8_t value, I2C_HandleTypeDef* hi2c) 
{
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(
    hi2c,
    BNO055_ADDR,
    addr,
    I2C_MEMADD_SIZE_8BIT,
    &value,
    1,
    HAL_MAX_DELAY
  );
  return (status == HAL_OK);
}

// ========= NON FUSION =========
//GYRO

float get_gyro_x(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x14, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_gyro_y(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x16, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_gyro_z(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x18, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

// ACCEL

float get_accel_x(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x8, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_accel_y(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0xA, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_accel_z(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0xC, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

// MAG

float get_mag_x(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x10, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_mag_y(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x12, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_mag_z(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x14, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

// ========= FUSION =========

// EULER 

float get_euler_heading(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x1A, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_euler_roll(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x1C, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_euler_pitch(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x1E, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

// QUATERNION

float get_quat_w(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x20, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_quat_x(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x22, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_quat_y(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x24, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }
}

float get_quat_z(I2C_HandleTypeDef* hi2c){
  uint8_t buffer[2];
  if (imu_read_nb(buffer, 2, 0x26, hi2c)){
    int16_t raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    return raw/16.0f;
  } else {
    return 0.0f;
  }

}

// FUSED PACKET
IMU_Packet get_packet(I2C_HandleTypeDef* hi2c){
  IMU_Packet out = {
    get_gyro_x(hi2c),
    get_gyro_y(hi2c),
    get_gyro_z(hi2c),
    get_accel_x(hi2c),
    get_accel_y(hi2c),
    get_accel_z(hi2c),
    get_mag_x(hi2c),
    get_mag_y(hi2c),
    get_mag_z(hi2c)
  };
  return out;
}

IMU_Packet_Fused get_packet_fused(I2C_HandleTypeDef* hi2c){
  IMU_Packet_Fused out = {
    get_euler_heading(hi2c),
    get_euler_roll(hi2c),
    get_euler_pitch(hi2c),
    get_quat_w(hi2c),
    get_quat_x(hi2c),
    get_quat_y(hi2c),
    get_quat_z(hi2c),

  };
  return out;
}