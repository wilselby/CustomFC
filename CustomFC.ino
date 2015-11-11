

/* Includes */
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_ADC.h>
#include <StorageManager.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Compass.h>
#include <AP_NavEKF.h>
#include <PID.h>

/* Defines */

// Motor Definitions
#define MOTOR_F   2    // Front   
#define MOTOR_R   0    // Right
#define MOTOR_B   3    // Back
#define MOTOR_L   1    // Left

// PID array 
PID pids[6];
#define PID_PITCH_STAB 0
#define PID_ROLL_STAB 1
#define PID_YAW_STAB 2
#define PID_PITCH_RATE 3
#define PID_ROLL_RATE 4
#define PID_YAW_RATE 5

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

/* Variables */

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

AP_InertialSensor ins;
AP_InertialSensor_MPU6000 imu(ins);
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;
AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, baro, gps);

/* Functions */
float map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() 
{

  //ESC Frequency to 490 Hz
  hal.rcout->set_freq(0xF, 490);  

  // Enable output channels
  for (uint8_t i=0; i<8; i++) {
    hal.rcout->enable_ch(i);
  }

  // we need to stop the barometer from holding the SPI bus
  hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
  hal.gpio->write(40, 1);

  // Initialise MPU6050 sensor
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

  ins.init_accel();

  ahrs.init();

  if( compass.init() ) {
    hal.console->printf("Enabling compass\n");
    ahrs.set_compass(&compass);
  } 
  else {
    hal.console->printf("No compass detected\n");
  }

  gps.init(NULL);

  // PID values
  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_PITCH_STAB].kI(0);
  pids[PID_PITCH_STAB].kD(0);
  pids[PID_PITCH_STAB].imax(8);

  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kI(0);
  pids[PID_ROLL_STAB].kD(0);
  pids[PID_ROLL_STAB].imax(8);

  pids[PID_YAW_STAB].kP(10);
  pids[PID_YAW_STAB].kI(0);
  pids[PID_YAW_STAB].kD(0);
  pids[PID_YAW_STAB].imax(8);

  pids[PID_PITCH_RATE].kP(0.7);
  pids[PID_PITCH_RATE].kI(1);
  pids[PID_PITCH_RATE].kD(0);
  pids[PID_PITCH_RATE].imax(50);

  pids[PID_ROLL_RATE].kP(0.7);
  pids[PID_ROLL_RATE].kI(1);
  pids[PID_ROLL_RATE].kD(0);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(2.7);
  pids[PID_YAW_RATE].kI(1);
  pids[PID_YAW_RATE].kD(0);
  pids[PID_YAW_RATE].imax(50);


}

void loop() 
{
  uint16_t channels[8];  // array for raw channel values
  float rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
  Vector3f gyro;
  float gyroroll, gyropitch, gyroyaw;
  float roll, pitch, yaw;
  static uint16_t counter;
  static uint32_t last_t, last_print, last_compass;
  static float yaw_target = 0; 
  float pitch_stab_output;
  float roll_stab_output;
  float yaw_stab_output;
  float pitch_output;
  float roll_output;
  float yaw_output;

  uint32_t now = hal.scheduler->micros();
  if (last_t == 0) {
    last_t = now;
    return;
  }

  last_t = now;  

  // Read RC channels and store in channels array
  hal.rcin->read(channels, 8);

  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  rcthr = channels[2];
  rcyaw = -map(channels[3], 1124, 1920, -150, 150);
  rcpit = -map(channels[1], 1124, 1918, -45, 45);
  rcroll = -map(channels[0], 1124, 1918, -45, 45);

  ins.wait_for_sample();

  ins.update();
  gyro = ins.get_gyro();
  gyropitch = ToDeg(gyro.y);
  gyroroll = ToDeg(gyro.x);
  gyroyaw = ToDeg(gyro.z);

  ahrs.update();
  //roll = ahrs.roll_sensor/100;
  //pitch = ahrs.pitch_sensor/100;
  //yaw = ahrs.yaw_sensor/100;
  roll = ToDeg(ahrs.roll);
  pitch = ToDeg(ahrs.pitch);
  yaw = ToDeg(ahrs.yaw);
  counter++;

  if(rcthr > 1120 + 100){

    // Stablise PIDS
    pitch_stab_output = constrain_float(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    roll_stab_output = constrain_float(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    yaw_stab_output = constrain_float(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

    // Yaw input is rate control, check if pilot is changing yaw rate
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }

    // rate PIDS
    pitch_output =  constrain_float(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyropitch, 1), - 500, 500);  
    roll_output =  constrain_float(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroroll, 1), -500, 500);  
    yaw_output =  -constrain_float(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroyaw, 1), -500, 500);  

    // mix pid outputs and send to the motors.
    hal.rcout->write(MOTOR_F, rcthr + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_R, rcthr - roll_output  - yaw_output);
    hal.rcout->write(MOTOR_B, rcthr - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_L, rcthr + roll_output  - yaw_output);
  } 
  else {
    // motors off
    hal.rcout->write(MOTOR_F, 1000);
    hal.rcout->write(MOTOR_R, 1000);
    hal.rcout->write(MOTOR_B, 1000);
    hal.rcout->write(MOTOR_L, 1000);

    yaw_target = yaw;

    for(int i=0; i<6; i++) // reset PID integrals whilst on the ground
      pids[i].reset_I();

  }

  if (now - last_print >= 100000 /* 100ms : 10hz */) {
    hal.console->printf_P(PSTR("P:%4.1f  R:%4.1f Y:%4.1f rate=%.1f\n"),pitch,roll,yaw,(1.0e6*counter)/(now-last_print));
    //hal.console->printf_P(PSTR("THR %.0f YAW %.0f PIT %.0f ROLL %.0f\r\n"),rcthr, rcyaw, rcpit, rcroll);
    //hal.console->printf_P(PSTR("F %.2f R %.2f B %.2f L %.2f \r\n"),pitch_stab_output, -roll_stab_output, -pitch_stab_output, roll_stab_output);
    last_print = now;
    counter = 0;
  }

}

AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup









