#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was send

// 1) Set CYBERGEAR_CAN_ID to the motor's current canID. 
// 1) Set cybergear.set_motor_can_id() to desired canID.
// 1) Run.

// 2) Set CYBERGEAR_CAN_ID to motor's new canID (what you just set).
// 2) Comment out cybergear.set_motor_can_id().
// 2) Run.

// 3) If status values are 0, CYBERGEAR_CAN_ID is incorrect for the motor.

// CanID default is 127 (0x7F) [other tested values: 0x11, 0x22]
// CanID's have been set to the hex of motor number (1-12) + 100 [101-112];

// Motor CanIDs
// Back Left: Wrist = #1 (101 = 0x65), Elbow = #2 (102 = 0x66), Shoulder = #3 (103 = 0x67)
// Back Right: Wrist = #4 (104 = 0x68), Elbow = #5 (105 = 0x69), Shoulder = #6 (106 = 0x6A)
// Front Left: Wrist = #7 (107 = 0x6B), Elbow = #8 (108 = 0x6C), Shoulder = #9 (109 = 0x6D)
// Front Right: Wrist = #10 (110 = 0x6E), Elbow = #11 (111 = 0x6F), Shoulder = #12 (112 = 0x70)

// Documented By Kenan Anderson and Sam Palatnikov 

// ===================================== SET CAN ID =================================
uint8_t CYBERGEAR_CAN_ID = 0x70; 
// ===================================== SET CAN ID =================================

uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

// function declarations
static void handle_rx_message(twai_message_t& message);
static void check_alerts();

void setup() {
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  cybergear.init_motor(MODE_POSITION); 
  cybergear.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear.set_limit_current(12.0); /* current limit allows faster operation */
  cybergear.set_limit_torque(0.8f); // lowered from 1.5

  // ===================================== SET CAN ID =================================
  //cybergear.set_motor_can_id(0x70);
  // ===================================== SET CAN ID =================================

  cybergear.enable_motor(); /* turn on the motor */

  driver_installed = true;
  delay(3000);
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  delay(20);
  check_alerts();

  // Prints cybergear status
  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
  Serial.println(cybergear.get_motor_can_id());

  // Clock cycle
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear.request_status();
  }
}

// Below functions for reading and printing can errors
static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }
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