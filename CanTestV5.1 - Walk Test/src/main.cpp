#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <bits/stdc++.h>
#include <map>
#include <string>

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
static bool rotate_clockwise = false;
unsigned long previousMillis = 0;  // will store last time a message was send

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
uint8_t CYBERGEAR_CAN_ID11 = 0x6f; // was 6f!! changed to motor 13
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


// function declarations
static void initialize_all_motors();
static void all_motor_pos_to_zero();
static void handle_rx_message(twai_message_t& message);
static void check_alerts();
static void walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict);

void setup() {
  // Define Dictionary of Cybergears
  dictFR["Hip"] = gearArr[11];
  dictFR["Knee"] = gearArr[10];
  dictFR["Ankle"] = gearArr[9];

  delay(1000);
  initialize_all_motors(); // All initializing for all motors
  delay(2000);
  all_motor_pos_to_zero(); // Power all motors and set positions to zero

  // Set up our walk cycle (testing block)
  // dictFR["Knee"].set_position_ref(-1.33 * -1);
  // dictFR["Ankle"].set_position_ref(1.39 * -1);
  // delay(2000);
  // gearArr[9].set_limit_speed(3.0f);
  // gearArr[10].set_limit_speed(3.0f);
  // delay(1000);
}

void loop() {
  if (!driver_installed) {
    delay(5000);
    Serial.printf("Driver not installed - bool is false?");
    return;
  }

  delay(30);
  //check_alerts();

  // Looping the walk cycle ===================================
  //walk_cycle(-1, dictFR);
  // Looping the walk cycle ===================================

  // Get status for serial output
  XiaomiCyberGearStatus cybergear_status = gearArr[0].get_status();
  Serial.printf("Motor 1: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    gearArr[0].request_status();
  }
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID1){
    gearArr[0].process_message(message);
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
    gearArr[i].init_motor(MODE_POSITION); 
    gearArr[i].set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
    gearArr[i].set_limit_current(6.0); /* current limit allows faster operation */
    gearArr[i].set_limit_torque(1.5f); // lowered from 1.5
    gearArr[i].set_position_kp(0.0f);
    gearArr[i].enable_motor(); /* turn on the motor */
  }
  driver_installed = true;
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS)); // problem child??
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

static void walk_cycle(int polarity, std::map<std::string, XiaomiCyberGearDriver> dict)
{
  // Assuming the motors are set to position 0 straight down...

  XiaomiCyberGearDriver Knee = dict["Knee"];
  XiaomiCyberGearDriver Ankle = dict["Ankle"];

  //delay(2000);
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