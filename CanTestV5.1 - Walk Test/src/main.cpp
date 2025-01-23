#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <bits/stdc++.h>
#include <map>
#include <string>

// =======================================================================================================
// How to add the custom 'all_motor_pos_to_zero()' function:
// 1) Copy the following
//      void motor_pos_to_zero();
//    Navigate to the project > .pio > libdeps > Xiaomi_CyberGear_Arduino > xioami_cybergear_driver.h
//    Paste anywhere in the 'public' section of class XiaomiCyberGearDriver
//
// 2) Copy the following
//      void XiaomiCyberGearDriver::motor_pos_to_zero(){
//      uint8_t data[8] = {0x01};
//      _send_can_package(_cybergear_can_id, CMD_SET_MECH_POSITION_TO_ZERO,_master_can_id, 8, data);
//      }
//    Navigate to the project > .pio > libdeps > Xiaomi_CyberGear_Arduino > xioami_cybergear_driver.cpp
//    Paste at the very bottom of the file
// =======================================================================================================

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool stall = false;

static int incomingByte = -1;
static bool driver_installed = false;
static bool rotate_clockwise = false;
unsigned long previousMillis = 0;  // will store last time a message was send
static int readCanID = 6; // ie 0 = read motor 1 (the motor to print status to serial)

// For microcontroller manual PID tuning
// Ankle ascillation at 1.7KP with 6.0 current limit
float Kp = 3.3, Ki = 0.1, Kd = 0.3; // 0.85, 0.6, 0.08
float maxIntegral = 6.0, maxOutput = 10000;
float desiredPosition = 0.0, currentPosition = 0.0;
float integral = 0.0;
float previousError = 0.0;

// for threading
unsigned long lastStartTime = 0;
const unsigned long groupDelay = 1000; // 1 second

// Define the CAN ID's for each motor
uint8_t CYBERGEAR_CAN_ID1 = 0x65;
uint8_t CYBERGEAR_CAN_ID2 = 0x66;
uint8_t CYBERGEAR_CAN_ID3 = 0x67;
uint8_t CYBERGEAR_CAN_ID4 = 0x68;
uint8_t CYBERGEAR_CAN_ID5 = 0x69;
uint8_t CYBERGEAR_CAN_ID6 = 0x6A;
uint8_t CYBERGEAR_CAN_ID7 = 0x6B;
uint8_t CYBERGEAR_CAN_ID8 = 0x6C;
uint8_t CYBERGEAR_CAN_ID9 = 0x6D;
uint8_t CYBERGEAR_CAN_ID10 = 0x6E;
uint8_t CYBERGEAR_CAN_ID11 = 0x6f; 
uint8_t CYBERGEAR_CAN_ID12 = 0x70;
uint8_t MASTER_CAN_ID = 0x00;

// array of all 12 cybergears
XiaomiCyberGearDriver gearArr[12] = {
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID1, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID2, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID3, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID4, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID5, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID6, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID7, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID8, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID9, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID10, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID11, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID12, MASTER_CAN_ID)
};

// Create the maps (dictionaries) and array
std::map<std::string, XiaomiCyberGearDriver> dictFR; // Front Right Leg
std::map<std::string, XiaomiCyberGearDriver> dictFL; // Front Left Leg
std::map<std::string, XiaomiCyberGearDriver> dictBR; // Back Right Leg
std::map<std::string, XiaomiCyberGearDriver> dictBL; // Back Left Leg

XiaomiCyberGearStatus statusArr[12];


// function declarations
static void initialize_all_motors();
static void all_motor_pos_to_zero();
static void handle_rx_message(twai_message_t& message);
static void check_alerts();
static void walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict);
static void updated_walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict);
static void pid_control(float dt, float desired, float current, XiaomiCyberGearDriver cybergear);
static void normalize();

void setup() {
  // Define Dictionary of Cybergears
  dictFR["Hip"] = gearArr[11];
  dictFR["Knee"] = gearArr[10];
  dictFR["Ankle"] = gearArr[9];

  dictFL["Hip"] = gearArr[8];
  dictFL["Knee"] = gearArr[7];
  dictFL["Ankle"] = gearArr[6];

  dictBR["Hip"] = gearArr[5];
  dictBR["Knee"] = gearArr[4];
  dictBR["Ankle"] = gearArr[3];

  dictBL["Hip"] = gearArr[2];
  dictBL["Knee"] = gearArr[1];
  dictBL["Ankle"] = gearArr[0];

  delay(1000);
  initialize_all_motors(); // All initializing for all motors
  delay(2000);
  //all_motor_pos_to_zero(); // Power all motors and set positions to zero
  delay(1000);
}

void loop() {
  if (!driver_installed) {
    delay(5000);
    Serial.printf("Driver not installed - bool is false");
    return;
  }

  // if (!stall)
  // {
  //   delay(2000);
  //   stall = true;
  // }

  delay(30); // 30/1000 = 0.03 seconds
  check_alerts();
   
  // Get status for serial output
  // for (int i = 0; i < 12; i++)
  // {
  //   statusArr[i] = gearArr[i].get_status();
  // }
  // ----------------------------
  XiaomiCyberGearStatus cybergear_status = gearArr[readCanID].get_status();
  Serial.printf("Motor: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    gearArr[readCanID].request_status();
  }

  // Manual PID tuning block - TO BE TESTED (initialize motors to MODE_CURRENT)
  // For MODE_CURRENT, current KP values are extreme and should be tested with caution
  // float currentPos = cybergear_status.position;
  // float desiredPos = -1.30;
  // pid_control(0.03, desiredPos, currentPos, gearArr[readCanID]);

  // ==================================================================================
  // START AT STRAIGHT LEG ORIENTATION IF EVER USING WALK CYCLE (all legs straight out)
  // ==================================================================================

  // std::thread a(updated_walk_cycle, -1, dictFR);
  // std::thread b(updated_walk_cycle, 1, dictBL);
  // a.detach();
  // b.detach();
  // delay(1012.5);
  // std::thread c(updated_walk_cycle, 1, dictFL);
  // std::thread d(updated_walk_cycle, -1, dictBR);
  // c.detach();
  // d.detach();
  // delay(982.5);
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == gearArr[readCanID].get_motor_can_id()){
    gearArr[readCanID].process_message(message);
  }
}

// Sets all motor current positions to zero
static void all_motor_pos_to_zero()
{
  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} zero pos\n", i);
    gearArr[i].motor_pos_to_zero();
    gearArr[i].set_position_ref(0.0f);
  }
}

// Initializes all motors
static void initialize_all_motors()
{
  gearArr[0].init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} init\n", i);
    gearArr[i].init_motor(MODE_CURRENT); 
    gearArr[i].set_limit_speed(2.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
    gearArr[i].set_limit_current(6.0); /* current limit allows faster operation */ // was set to 6.0
    gearArr[i].set_limit_torque(6.0f); // lowered from 1.5
    //gearArr[i].set_position_kp(5000.0f);
    gearArr[i].enable_motor(); /* turn on the motor */
  }
  driver_installed = true;
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}

static void updated_walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict)
{
  // Assuming the motors are set to position 0, legs are straight out/down
  
  XiaomiCyberGearDriver Knee = dict["Knee"];
  XiaomiCyberGearDriver Ankle = dict["Ankle"];
  int delaySlice = 75;

  Knee.set_position_ref(-1.21 * polarity);
  Ankle.set_position_ref(1.50 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.29 * polarity);
  Ankle.set_position_ref(1.61 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.38 * polarity);
  Ankle.set_position_ref(1.79 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.39 * polarity);
  Ankle.set_position_ref(1.90 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.38 * polarity);
  Ankle.set_position_ref(1.99 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.34 * polarity);
  Ankle.set_position_ref(2.05 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.28 * polarity);
  Ankle.set_position_ref(2.08 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.18 * polarity);
  Ankle.set_position_ref(2.09 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.07 * polarity);
  Ankle.set_position_ref(2.07 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.97 * polarity);
  Ankle.set_position_ref(2.03 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.86 * polarity);
  Ankle.set_position_ref(1.96 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.74 * polarity);
  Ankle.set_position_ref(1.86 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.64 * polarity);
  Ankle.set_position_ref(1.70 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.64 * polarity);
  Ankle.set_position_ref(1.64 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.67 * polarity);
  Ankle.set_position_ref(1.65 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.72 * polarity);
  Ankle.set_position_ref(1.66 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.77 * polarity);
  Ankle.set_position_ref(1.67 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.82 * polarity);
  Ankle.set_position_ref(1.67 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.88 * polarity);
  Ankle.set_position_ref(1.68 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.93 * polarity);
  Ankle.set_position_ref(1.67 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-0.97 * polarity);
  Ankle.set_position_ref(1.67 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.01 * polarity);
  Ankle.set_position_ref(1.66 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.05 * polarity);
  Ankle.set_position_ref(1.65 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.08 * polarity);
  Ankle.set_position_ref(1.63 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.12 * polarity);
  Ankle.set_position_ref(1.61 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.15 * polarity);
  Ankle.set_position_ref(1.58 * polarity);
  delay(delaySlice);

  Knee.set_position_ref(-1.18 * polarity);
  Ankle.set_position_ref(1.54 * polarity);
  delay(delaySlice);
}

static void walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict)
{
  // Assuming the motors are set to position 0, legs are straight out/down

  XiaomiCyberGearDriver Knee = dict["Knee"];
  XiaomiCyberGearDriver Ankle = dict["Ankle"];

  Knee.set_position_ref(-1.33 * polarity);
  Ankle.set_position_ref(1.39 * polarity);
  delay(100);

  Knee.set_position_ref(-1.41 * polarity);
  Ankle.set_position_ref(1.46 * polarity);
  delay(100);

  Knee.set_position_ref(-1.63 * polarity);
  Ankle.set_position_ref(1.92 * polarity);
  delay(100);

  Knee.set_position_ref(-1.65 * polarity);
  Ankle.set_position_ref(2.24 * polarity);
  delay(100);

  Knee.set_position_ref(-1.52 * polarity);
  Ankle.set_position_ref(2.37 * polarity);
  delay(100);

  Knee.set_position_ref(-1.34 * polarity);
  Ankle.set_position_ref(2.42 * polarity);
  delay(100);

  Knee.set_position_ref(-1.12 * polarity);
  Ankle.set_position_ref(2.42 * polarity);
  delay(100);

  Knee.set_position_ref(-0.86 * polarity);
  Ankle.set_position_ref(2.35 * polarity);
  delay(100);

  Knee.set_position_ref(-0.6 * polarity);
  Ankle.set_position_ref(2.19 * polarity);
  delay(100);

  Knee.set_position_ref(-0.35 * polarity);
  Ankle.set_position_ref(1.95 * polarity);
  delay(100);

  Knee.set_position_ref(-0.17 * polarity);
  Ankle.set_position_ref(1.70 * polarity);
  delay(100);

  Knee.set_position_ref(-0.11 * polarity);
  Ankle.set_position_ref(1.53 * polarity);
  delay(100);

  Knee.set_position_ref(-0.13 * polarity);
  Ankle.set_position_ref(1.40 * polarity);
  delay(100);

  Knee.set_position_ref(-0.2 * polarity);
  Ankle.set_position_ref(1.45 * polarity);
  delay(100);

  Knee.set_position_ref(-0.3 * polarity);
  Ankle.set_position_ref(1.53 * polarity);
  delay(100);

  Knee.set_position_ref(-0.42 * polarity);
  Ankle.set_position_ref(1.61 * polarity);
  delay(100);

  Knee.set_position_ref(-0.53 * polarity);
  Ankle.set_position_ref(1.67 * polarity);
  delay(100);

  Knee.set_position_ref(-0.65 * polarity);
  Ankle.set_position_ref(1.72 * polarity);
  delay(100);

  Knee.set_position_ref(-0.76 * polarity);
  Ankle.set_position_ref(1.75 * polarity);
  delay(100);

  Knee.set_position_ref(-0.87 * polarity);
  Ankle.set_position_ref(1.77 * polarity);
  delay(100);

  Knee.set_position_ref(-0.97 * polarity);
  Ankle.set_position_ref(1.77 * polarity);
  delay(100);

  Knee.set_position_ref(-1.1 * polarity);
  Ankle.set_position_ref(1.76 * polarity);
  delay(100);

  Knee.set_position_ref(-1.12 * polarity);
  Ankle.set_position_ref(1.73 * polarity);
  delay(100);

  Knee.set_position_ref(-1.19 * polarity);
  Ankle.set_position_ref(1.68 * polarity);
  delay(100);

  Knee.set_position_ref(-1.23 * polarity);
  Ankle.set_position_ref(1.63 * polarity);
  delay(100);

  Knee.set_position_ref(-1.28 * polarity);
  Ankle.set_position_ref(1.56 * polarity);
  delay(100);

  Knee.set_position_ref(-1.3 * polarity);
  Ankle.set_position_ref(1.48 * polarity);
  delay(100);
}


void pid_control(float dt, float desired, float current, XiaomiCyberGearDriver cybergear) // dt = change in time slice (ie, delay(30))
{
    desiredPosition = desired;
    currentPosition = current;
    float error = desiredPosition - currentPosition;

    // proportional term
    float proportionalTerm = Kp * error;

    // Integral term
    integral += error * dt;
    if (integral > maxIntegral) 
      integral = maxIntegral;

    if (integral < -maxIntegral) 
      integral = -maxIntegral;

    Serial.println(integral);

    float integralTerm = Ki * integral;

    // Derivative term
    float derivative = (error - previousError) / dt;
    float derivativeTerm = Kd * derivative;

    // Compute output
    float output = proportionalTerm + integralTerm + derivativeTerm;
    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;

    // Set and move on
    cybergear.set_current_ref(output);
    previousError = error;
}

static void normalize()
{
  for (int i = 0; i < 12; i++)
  {
    gearArr[i].set_position_ref(0.0);
  }
}
