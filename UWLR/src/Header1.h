#pragma once
#include "VL53L0X_1.h"
#include "LSM9DS1.h"

void inline check(int);

void Seattle(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function, int seed);

void Follow_Wall(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function);

void Follow_Object(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function);

void Object_Avoidance(VL53L0X Sensor1, VL53L0X Sensor2, VL53L0X Sensor3, VL53L0X Sensor4, VL53L0X Sensor5, VL53L0X Sensor6, LSM9DS1 imu, int function, int seed);