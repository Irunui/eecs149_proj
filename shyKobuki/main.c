// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
#include "app_util.h"
#include "simple_ble.h"
#include "app_error.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// INITIAL POSITION
float initial_x = 0;
float initial_y = 0;
float initial_angle = 0;

// MAIN COMPUTER DATA CURRENT STATE
float current_angle = 0;
float current_x = 0;
float current_y = 0;
int master = 0;
int current_rx_id = 0;
bool inside = true;

// VARIABLES DEFINITION
float deviation = 0;
float distance = 0;
bool left = false;

// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:02
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x0002,  // used as the 5th and 6th octet in the device BLE address, you will need to change this for each device you have
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(50, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  RANDOM,
  ALIGNING,
  GOING_BACK,
  ORIENTING,
  OBSTACLE_TURN,
  OBSTACLE_BACK_UP
} robot_state_t;

robot_state_t state = OFF;
robot_state_t previous_state = OFF;

//  BASIC CALCUL FUNCTIONS and constants

float Pi = M_PI;

static float measure_distance (uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.00008529;
  float result;
  if (current_encoder < previous_encoder) {
    result = (0xFFFF - previous_encoder + current_encoder) * CONVERSION;
  } else {
    result = (current_encoder - previous_encoder) * CONVERSION;
  }

  if( result > 5){
    return 0;
  }
  return result;
}

static float measure_negative_distance (uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.00008529;
  float result;
  if (previous_encoder < current_encoder) {
    result = (0xFFFF - current_encoder + previous_encoder) * CONVERSION;
  } else {
    result = (previous_encoder - current_encoder) * CONVERSION;
  }

  if( result > 5){
    return 0;
  }
  return result;
}

static float measure_angle (float angle) {
  const float CONVERSION = 0.056;
  float result;
  result = angle + (angle * CONVERSION);
  return result;
}

static float angle_to_target(float x, float y, float alpha, float x_target, float y_target) {
    float temp_alpha; // angle relative to origin to turn
    if (x != x_target) {
        temp_alpha = 180/3.1415*atan((y_target-y)/(x_target-x));
        if ((x_target-x)<0) {// if cosinus is negative
            if (temp_alpha >0) {
                temp_alpha = -180 + temp_alpha;
            } else {
                temp_alpha = 180+temp_alpha;
            }
         }
    } else {
       if (y_target-y>0) {
          temp_alpha = +90;
       } else {
           temp_alpha = -90;
       }
    }

    float res = temp_alpha-alpha; // angle to turn relative to current orientation
    if (res>180) {
       res -= 360;
    } else if(res<-180) {
       res += 360;
    }
    return res;
}

// global variables to store info sent via BLE
float distance_to_target = 0; // distance to reach the target
float angle_to_turn = 0;// angle to turn to go to target
float final_alpha=0; // final angle to have relative to current

static void go_to_target(float x, float y, float alpha, float x_target, float y_target, float alpha_target) {
    // goes from current position (x,y,alpha) to target position (x_target, y_target, alpha_target
    // Basically, we have to initialize variables and change state
    // then movement is handled by the state machine
    distance_to_target = sqrt((x_target-x)*(x_target-x)+(y_target-y)*(y_target-y));
    angle_to_turn = angle_to_target(x,y,alpha,x_target,y_target);// angle to turn to go to target
    final_alpha=alpha_target-alpha; // final angle to have relative to current

    state = ALIGNING;
    mpu9250_stop_gyro_integration();
    mpu9250_start_gyro_integration();
}

void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {

  //extract the fields we care about
  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  uint8_t const* ble_addr = adv_report->peer_addr.addr;
  uint8_t* adv_buf = adv_report->data.p_data;
  uint16_t adv_len = adv_report->data.len;
  uint32_t temp;

  if ((ble_addr[5] == 0xF8 && ble_addr[4] == 0x59 && ble_addr[3] == 0x71 && ble_addr[2] == 0x99)
      || (ble_addr[5] == 0xCC && ble_addr[4] == 0x2F && ble_addr[3] == 0x71 && ble_addr[2] == 0x58)) {
  if (ble_addr[5] == 0xCC && ble_addr[4] == 0x2F && ble_addr[3] == 0x71 && ble_addr[2] == 0x58) {
    printf("Kobuki ID Received: %d\n", adv_buf[2]);
    if (adv_buf[2] == 1){     //ID of the Kobuki
      if(current_rx_id != adv_buf[3]) {
        //display_write("BLE Received", DISPLAY_LINE_0);
        current_rx_id = adv_buf[3];
        master = adv_buf[4];

        temp = (adv_buf[5]<<24) + (adv_buf[6]<<16) + (adv_buf[7]<<8) + adv_buf[8];
        memcpy(&current_x, &temp, 4);
        temp = (adv_buf[9]<<24) + (adv_buf[10]<<16) + (adv_buf[11]<<8) + adv_buf[12];
        memcpy(&current_y, &temp, 4);
        current_angle = adv_buf[13]*2;
        //printf("Current x: %f\n", current_x);
        //printf("Current y: %f\n", current_y);
        temp = (adv_buf[14]<<24) + (adv_buf[15]<<16) + (adv_buf[16]<<8) + adv_buf[17];
        memcpy(&initial_x, &temp, 4);
        temp = (adv_buf[18]<<24) + (adv_buf[19]<<16) + (adv_buf[20]<<8) + adv_buf[21];
        memcpy(&initial_y, &temp, 4);
        initial_angle = adv_buf[22]*2;

        if (adv_buf[23] == 1) {
          inside = true;
        } else {
          inside = false;
        }
        //printf("Initial x: %f\n", initial_x);
        //printf("Initial y: %f\n", initial_y);
      }
    }
  }
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  //nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  //nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  //nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  float random = 0;
  int count = 0;
  int time = 0;

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // INITIAL CONDITITONS
  KobukiSensors_t sensors = {0};
  uint16_t prev_encoder = 0;

  float deviation = 0;
  float distance = 0;
  float distanceObstacle = 0;

  float speed_correction=0, min_rotation_speed = 50;

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();

  // Start scanning
  //scanning_start();

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    sd_ble_gap_scan_stop();
    kobukiSensorPoll(&sensors);
    scanning_start();
    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);
    //printf("Master: %d\n", master);

    // HANDLE STATES
    switch(state) {
      case OFF: {
        //sd_ble_gap_scan_stop();
        kobukiDriveDirect(0,0);
        state = OFF;

        if (is_button_pressed(&sensors) || master != 0) {
          state = RANDOM;
          //printf("ALIGNING\n");
        } else {
          display_write("OFF", DISPLAY_LINE_0);
        }
        //scanning_start();
        break; // each case needs to end with break!
      }

      case RANDOM: {
        if (master==2) {
          go_to_target(current_x, current_y, current_angle, initial_x, initial_y, initial_angle);
          //printf("ALIGNING\n");
        } else if (master==0) {
            state = OFF;
        } else if (sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight || sensors.bumps_wheelDrops.bumpCenter) {
          previous_state = RANDOM;
          state = OBSTACLE_BACK_UP;
          distanceObstacle = 0;
        } else {
          display_write("I SEE NO ONE", DISPLAY_LINE_0);
          //kobukiDriveDirect(0,0);
          state = RANDOM;
          count++;
          printf("Random: %f\n", random);
          if(count >= 30) {
            random = (float)((double)rand())/((double) RAND_MAX);
            if(random <= 0.35) {
              //printf("Forward\n");
              kobukiDriveDirect(50,50);
            } else if (random <= 0.55 && random > 0.35) {
              //printf("Turn Left\n");
              kobukiDriveDirect(40,-40);
            } else if (random <= 0.75 && random > 0.55){
              //printf("Turn Right\n");
              kobukiDriveDirect(-40, 40);
            } else {
              kobukiDriveDirect(0, 0);
            }
            count = 0;
          }
        }
        break; // each case needs to end with break!
      }

      case ALIGNING: { // THIS CASE CORRESPONDS TO GOING TO AN ASSIGNED POSITION (turning in direction of target)
          // transition logic
          deviation = mpu9250_read_gyro_integration().z_axis;

          // displaying angle remaining to turn
          display_write("SOMEONES THERE!", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf,16,"%f", angle_to_turn-deviation);
          display_write(buf, DISPLAY_LINE_1);

          if (is_button_pressed(&sensors) || master==0) { // if we stop manually
            state = OFF;
            display_write("", DISPLAY_LINE_1);
          } else if (sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight || sensors.bumps_wheelDrops.bumpCenter) {
            previous_state = ALIGNING;
            state = OBSTACLE_BACK_UP;
            distanceObstacle = 0;
            prev_encoder = sensors.rightWheelEncoder;
          } else if (abs(deviation) >= abs(angle_to_turn)) { // if we turned enough
            state = GOING_BACK;
            prev_encoder = sensors.rightWheelEncoder;
          } else { // if we need to keep turning
              // speed correction is used to adjust speed to distance remaining
              // basically, the closer the we are from target, the slower we go
              speed_correction = angle_to_turn-deviation;
              float s = min_rotation_speed + abs(speed_correction);

              // this determines which direction to turn
              if (angle_to_turn>0) {
                  s = s; // good direction
              } else {
                  s = -s; // other direction
              }

              // turn wheels
              kobukiDriveDirect(-s,s);
              state = ALIGNING;
          }
          break; // each case needs to end with break!
      }
      case GOING_BACK: {
        uint16_t next_encoder = sensors.rightWheelEncoder;

        distance = measure_distance(next_encoder, prev_encoder);
        deviation = mpu9250_read_gyro_integration().z_axis;

        display_write("GOING FORWARD", DISPLAY_LINE_0);
        char buf[16];
        snprintf(buf,16,"%f", distance_to_target-distance);
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors) || master==0) {
          state = OFF;
          display_write("", DISPLAY_LINE_1);
        } else if (sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight || sensors.bumps_wheelDrops.bumpCenter) {
          previous_state = GOING_BACK;
          state = OBSTACLE_BACK_UP;
          distanceObstacle = 0;
          prev_encoder = sensors.rightWheelEncoder;
        } else if (distance >= distance_to_target) {
          state = ORIENTING;
          deviation = 0;
        } else {
          float correction = 0;
          if (angle_to_turn-deviation>0) {
              correction = 3;
          } else {
              correction = -3;
          }

          kobukiDriveDirect(100-correction,100+correction);

          state = GOING_BACK;
        }
        break;
      }

      case ORIENTING: {

        deviation = mpu9250_read_gyro_integration().z_axis;

        display_write("ORIENTING", DISPLAY_LINE_0);
        char buf[16];
        snprintf(buf,16,"%f", final_alpha-deviation);
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors) || master==0) {
          state = OFF;
          display_write("", DISPLAY_LINE_1);
        } else if (sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight || sensors.bumps_wheelDrops.bumpCenter) {
          previous_state = ORIENTING;
          state = OBSTACLE_BACK_UP;
          distanceObstacle = 0;
          prev_encoder = sensors.rightWheelEncoder;
        } else if (abs(deviation-angle_to_turn) >= abs(final_alpha-angle_to_turn)) {
            state = OFF;
            display_write("", DISPLAY_LINE_1);
            display_write("STOP", DISPLAY_LINE_0);
            master = 0;
        } else {
            float s;

            // adjusting speed to distance to target
            speed_correction = final_alpha-deviation;
            s = min_rotation_speed + abs(speed_correction);
            // determining direction to turn
            if (final_alpha > angle_to_turn) {
                s = s; // good direction
            } else {
                s = -s; // other direction
            }
            // turning
            kobukiDriveDirect(-s,s);
            state = ORIENTING;
        }
        break;
      }

      case OBSTACLE_BACK_UP: {
        uint16_t next_encoder = sensors.rightWheelEncoder;
        distanceObstacle = measure_negative_distance(next_encoder, prev_encoder);
        char buf[16];
        snprintf(buf,16,"%f", distanceObstacle);
        display_write(buf, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          display_write("", DISPLAY_LINE_1);
        } else if (distanceObstacle >= 0.05) {
          kobukiDriveDirect(0,0);
          if(time > 5){
            state = previous_state;
            prev_encoder = sensors.rightWheelEncoder;
          }
        } else {
          // perform state-specific actions here
          display_write("OBSTACLE", DISPLAY_LINE_0);
          kobukiDriveDirect(-50,-50);
          state = OBSTACLE_BACK_UP;
        }
        break;
      }
    }
  }
}
