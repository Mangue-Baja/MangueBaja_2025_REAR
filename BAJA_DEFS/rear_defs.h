#ifndef REAR_DEFS_H
#define REAR_DEFS_H

#include "mbed.h"
#include "defs.h"
#include "FIR.h"
#include <cstdint>

#define CAN_BPS_1000K       1000000

/* Moving Average Definitions */
#define ADCVoltageLimit     3.3
#define R_TERM              1000
#define sample              40
#define CVTsample           25
//#define DENSITY             1.3565

/* Wheel Definitions */
#define PI                        3.1416
#define WHEEL_DIAMETER            0.5842      // m
//#define WHEEL_HOLES_NUMBER_MB1  24
#define WHEEL_HOLES_NUMBER_REAR   12
#define WHEEL_HOLES_NUMBER_FRONT  24

/* Servo definitions */
#define MID_MODE            0x00
#define RUN_MODE            0x01
#define CHOKE_MODE          0x02
#define SERVO_MID           1180
#define SERVO_RUN           800
#define SERVO_CHOKE         1550

typedef enum
{
    IDLE_ST,            // Wait
    TEMP_MOTOR_ST,      // Measure temperature of motor
    TEMP_CVT_ST,        // Measure temperature of CVT
    FUEL_ST,            // Proccess fuel data sampling
    SPEED_ST,           // Calculate speed
    VOLTAGE_ST,         // Calculate State of Charge and battery voltage
    SYSTEM_CURRENT_ST,  // Measure the current of the system
    THROTTLE_ST,        // Write throttle position (PWM)
    DEBUG_ST            // Send data for debug

} state_t;

typedef struct
{
    // MPU_Bluetooth (sent by physical serial connection)
    //  String config_bluetooth_enabled;
    //  String config_bluedroid_enabled;
    //  String config_bt_spp_enabled;

    // MPU
    uint8_t lora_init;

    // SCU
    uint8_t internet_modem;       
    uint8_t mqtt_client_connection; 
    uint8_t sd_start;               
    uint8_t check_sd;               

    // FRONT
    uint8_t accel_begin;

    // REAR
    uint8_t termistor;
    uint8_t cvt_temperature;
    uint8_t measure_volt;
    uint8_t speed_pulse_counter;
    uint16_t servo_state;
    /* Servo states by number:
        * 3 = MID 
        * 2 = RUN
        * 4 = CHOKE
        * 0 = ERRO
    */

} bluetooth;

#endif
