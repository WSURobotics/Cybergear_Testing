#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <map>
#include <string>

// =======================================================================================================
// How to add the custom 'all_motor_pos_to_zero()' function (only works for MODE_POSITION):
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

// Intervals:
#define TRANSMIT_RATE_MS 1000 // 30 for MODE_CURRENT
#define POLLING_RATE_MS 1000 // 100 for MODE_CURRENT

// Static globals
static bool initStall = false; // If we want an initial stall before starting the loop
static bool driver_installed = false; // Checks if the motor has been initialized
unsigned long previousMillis = 0;  // will store last time a message was sent

static int readCanID = 9; // ID of the motor to print status to serial

// Define the CAN IDs for each motor
uint8_t CYBERGEAR_CAN_ID1 = 0x65;
uint8_t CYBERGEAR_CAN_ID2 = 0x66;
uint8_t CYBERGEAR_CAN_ID3 = 0x67;
uint8_t CYBERGEAR_CAN_ID4 = 0x68;
uint8_t CYBERGEAR_CAN_ID5 = 0x69;
uint8_t CYBERGEAR_CAN_ID6 = 0x6A;
uint8_t CYBERGEAR_CAN_ID7 = 0x71; // SWAPPED 7 FOR 13 - MOTOR IS NOW 13!
uint8_t CYBERGEAR_CAN_ID8 = 0x6C;
uint8_t CYBERGEAR_CAN_ID9 = 0x6D;
uint8_t CYBERGEAR_CAN_ID10 = 0x6E;
uint8_t CYBERGEAR_CAN_ID11 = 0x6f;
uint8_t CYBERGEAR_CAN_ID12 = 0x70;
uint8_t MASTER_CAN_ID = 0x00;

// Array of all 12 cybergears
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

// The 'knee' values for a step, in order
float kneeArray[27] = {
    -1.21, -1.29, -1.38, -1.39, -1.38, -1.34, -1.28, -1.18, -1.07, -0.97,
    -0.86, -0.74, -0.64, -0.64, -0.67, -0.72, -0.77, -0.82, -0.88, -0.93,
    -0.97, -1.01, -1.05, -1.08, -1.12, -1.15, -1.18
};

// The 'ankle' values for a step, in order
float ankleArray[27] = {
    1.50, 1.61, 1.79, 1.90, 1.99, 2.05, 2.08, 2.09, 2.07, 2.03,
    1.96, 1.86, 1.70, 1.64, 1.65, 1.66, 1.67, 1.67, 1.68, 1.67,
    1.67, 1.66, 1.65, 1.63, 1.61, 1.58, 1.54
};

// FL Polarity: 1
// FR Polarity: -1
// BL Polarity: 1
// BR Polarity: -1

// For setting walk positions in loop
static int group1 = 0; // Tracks index of group 1
static int group2 = 0; // Tracks index of group 2
static bool group2Start = false; // If group 2 has started or not
static unsigned long startMillis = -1; // Holds the start time
static unsigned long endMillis = 0; // Holds the end/current time

// Function declarations
static void initialize_all_motors(); // Initializes all motors
static void all_motor_pos_to_zero(); // Sets all motor positions to zero (MODE_POSITION ONLY)
static void handle_rx_message(twai_message_t& message); // Configures status messages
static void check_alerts(); // Checks for status messages
static void pid_control(float dt, float desired, float current, XiaomiCyberGearDriver cybergear); // Adjusts and assigns PID values

void setup() {
  delay(1000);
  initialize_all_motors();
  delay(1000);
  all_motor_pos_to_zero();
  delay(1000);
}

void loop() {
  if (!driver_installed) { // Checks for motor initialization
    delay(5000);
    Serial.printf("Driver not installed - bool is false");
    return;
  }

  delay(60);
  check_alerts(); // Read incoming CAN messages

  XiaomiCyberGearStatus cybergear_status = gearArr[readCanID].get_status(); // Read and print readCanID motor status
  Serial.printf("Motor: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    gearArr[readCanID].request_status();
  }

  // If we want a single delay before the loop begins (optional)
  if (!initStall)
  {
    delay(5000);
    // Set to a sit
    gearArr[10].set_position_ref(-1.21 * -1);
    gearArr[9].set_position_ref(1.50 * -1);
    gearArr[7].set_position_ref(-1.21 * 1);
    gearArr[6].set_position_ref(1.50 * 1);
    gearArr[4].set_position_ref(-1.21 * -1);
    gearArr[3].set_position_ref(1.50 * -1);
    gearArr[1].set_position_ref(-1.21 * 1);
    gearArr[0].set_position_ref(1.50 * 1);

    delay(5000);
    initStall = true;
  }

  // ==================================================================================
  // START AT STRAIGHT LEG ORIENTATION IF EVER USING WALK CYCLE (all legs straight out)
  // ==================================================================================

  // TIMER STARTS (run once first time)
  // if (startMillis == -1)
  // {
  //   startMillis = millis();
  // }

  gearArr[10].set_position_ref(kneeArray[group1] * -1);
  gearArr[9].set_position_ref(ankleArray[group1] * -1);
  gearArr[1].set_position_ref(kneeArray[group1] * 1);
  gearArr[0].set_position_ref(ankleArray[group1] * 1);
  group1++;
  if (group1 == 27) // reset if at max
    group1 = 0;

  // IF HAS BEEN 1026ms, good 2 go (if end - start => 1026)
  // if (!group2Start)
  //   endMillis = millis();
  if (!group2Start && group1 == 13) // If group1 is halfway through cycle, give group2 the OK
    group2Start = true;
  if (group2Start) // If given the OK
  {
    gearArr[7].set_position_ref(kneeArray[group2] * 1);
    gearArr[6].set_position_ref(ankleArray[group2] * 1);
    gearArr[4].set_position_ref(kneeArray[group2] * -1);
    gearArr[3].set_position_ref(ankleArray[group2] * -1);
    group2++;
  }
  if (group2 == 27) // reset if at max
    group2 = 0;

  // ==================================================================================
  // EDIT FOR SERIAL INPUT ACTIONS AS REQUIRED
  // ==================================================================================

  // if (Serial.available()) {  // Check if data is available
  //       String command = Serial.readStringUntil('\n');  // Read input until newline
  //       command.trim();  // Remove any whitespace or newlines

  //       int motor;
  //       float position;

  //       if (sscanf(command.c_str(), "%d: %f", &motor, &position) == 2) {  // Parse input
  //           Serial.print("Motor: ");
  //           Serial.print(motor);
  //           Serial.print(", Position: ");
  //           Serial.println(position);

  //           if (motor == 1) {
  //               //gearArr[9].set_position_ref(position);
  //               gearArr[9].enable_motor();
  //           } else if (motor == 2) {
  //               //gearArr[10].set_position_ref(position);
  //               gearArr[6].enable_motor();
  //           } else {
  //               Serial.println("Invalid motor ID");
  //           }
  //       } else {
  //           Serial.println("Invalid command format");
  //       }
  //   }
}

/**
 * @brief Handles received CAN messages and processes them.
 *
 * @param message The received CAN message.
 */
static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == gearArr[readCanID].get_motor_can_id()){
    gearArr[readCanID].process_message(message);
  }
}

/**
 * @brief Sets all motor current positions to zero.
 */
static void all_motor_pos_to_zero()
{
  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} zero pos\n", i);
    gearArr[i].motor_pos_to_zero();
    gearArr[i].set_position_ref(0.0f);
    delay(10);
  }
}

/**
 * @brief Initializes all motors.
 */
static void initialize_all_motors()
{
  gearArr[0].init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} init\n", i);
    gearArr[i].init_motor(MODE_POSITION);
    gearArr[i].set_limit_speed(6.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
    gearArr[i].set_limit_current(20.0f); /* current limit allows faster operation */ // was set to 6.0
    gearArr[i].set_limit_torque(12.0f); // lowered from 1.5
    //gearArr[i].set_position_kp(500.0f);
    gearArr[i].enable_motor(); /* turn on the motor */
    delay(10);
  }
  driver_installed = true;
}

/**
 * @brief Checks for alerts and handles them.
 */
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