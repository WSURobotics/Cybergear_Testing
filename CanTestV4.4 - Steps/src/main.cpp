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
static bool rotate_clockwise = false;
unsigned long previousMillis = 0;  // will store last time a message was send

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
uint8_t CYBERGEAR_CAN_ID11 = 0x71; // was 6f!! changed to motor 13
uint8_t CYBERGEAR_CAN_ID12 = 0x70;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear1 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID1, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear2 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID2, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear3 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID3, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear4 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID4, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear5 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID5, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear6 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID6, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear7 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID7, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear8 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID8, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear9 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID9, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear10 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID10, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear11 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID11, MASTER_CAN_ID);
XiaomiCyberGearDriver cybergear12 = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID12, MASTER_CAN_ID);

// function declarations
static void initialize_all_motors();
static void all_motor_pos_to_zero();
static void handle_rx_message(twai_message_t& message);
static void check_alerts();

void setup() {
  initialize_all_motors(); // All initializing for all motors!
  delay(2000);
  all_motor_pos_to_zero(); // Power all motors and set positions to zero

  // delay(1000); // Elbow to position
  // cybergear2.set_position_ref(-0.5f);
  // cybergear5.set_position_ref(0.5f);
  // cybergear8.set_position_ref(-0.5f);
  // cybergear11.set_position_ref(0.5f);

  // delay(1000); // Shoulder to raise
  // cybergear3.set_position_ref(-0.6f);
  // cybergear6.set_position_ref(0.6f);
  // cybergear9.set_position_ref(0.6f);
  // cybergear12.set_position_ref(-0.6f);

  // delay(2000); // Wrist and Elbow to extend
  // // Want to push up the wrist and lower elbow to adjust
  // cybergear1.set_position_ref(-0.5f);
  // cybergear2.set_position_ref(-0.3f);

  // cybergear4.set_position_ref(0.5f);
  // cybergear5.set_position_ref(0.3f);

  // cybergear7.set_position_ref(-0.5f);
  // cybergear8.set_position_ref(-0.3f);

  // cybergear10.set_position_ref(0.5f);
  // cybergear11.set_position_ref(0.3f);
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  delay(30);
  //check_alerts();

  XiaomiCyberGearStatus cybergear_status = cybergear1.get_status();
  Serial.printf("Motor 1: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear1.request_status();
  }
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID1){
    cybergear1.process_message(message);
  }
}

static void all_motor_pos_to_zero()
{
  cybergear1.motor_pos_to_zero();
  cybergear1.set_position_ref(0.0f);
  cybergear2.motor_pos_to_zero();
  cybergear2.set_position_ref(0.0f);
  cybergear3.motor_pos_to_zero();
  cybergear3.set_position_ref(0.0f);
  
  cybergear4.motor_pos_to_zero();
  cybergear4.set_position_ref(0.0);
  cybergear5.motor_pos_to_zero();
  cybergear5.set_position_ref(0.0f);
  cybergear6.motor_pos_to_zero();
  cybergear6.set_position_ref(0.0f);
  
  cybergear7.motor_pos_to_zero();
  cybergear7.set_position_ref(0.0f);
  cybergear8.motor_pos_to_zero();
  cybergear8.set_position_ref(0.0f);
  cybergear9.motor_pos_to_zero();
  cybergear9.set_position_ref(0.0f);
  
  cybergear10.motor_pos_to_zero();
  cybergear10.set_position_ref(0.0f);
  cybergear11.motor_pos_to_zero();
  cybergear11.set_position_ref(0.0f);
  cybergear12.motor_pos_to_zero();
  cybergear12.set_position_ref(0.0f);
}

static void initialize_all_motors()
{
  cybergear1.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  cybergear1.init_motor(MODE_POSITION); 
  cybergear1.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear1.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear1.set_limit_torque(1.5f); // lowered from 1.5
  cybergear1.enable_motor(); /* turn on the motor */
  
  cybergear2.init_motor(MODE_POSITION); 
  cybergear2.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear2.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear2.set_limit_torque(1.5f); // lowered from 1.5
  cybergear2.enable_motor(); /* turn on the motor */
  
  cybergear3.init_motor(MODE_POSITION); 
  cybergear3.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear3.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear3.set_limit_torque(1.5f); // lowered from 1.5
  cybergear3.enable_motor(); /* turn on the motor */
  
  cybergear4.init_motor(MODE_POSITION); 
  cybergear4.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear4.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear4.set_limit_torque(1.5f); // lowered from 1.5
  cybergear4.enable_motor(); /* turn on the motor */
  
  cybergear5.init_motor(MODE_POSITION); 
  cybergear5.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear5.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear5.set_limit_torque(1.5f); // lowered from 1.5
  cybergear5.enable_motor(); /* turn on the motor */
  
  cybergear6.init_motor(MODE_POSITION); 
  cybergear6.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear6.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear6.set_limit_torque(1.5f); // lowered from 1.5
  cybergear6.enable_motor(); /* turn on the motor */
  
  cybergear7.init_motor(MODE_POSITION); 
  cybergear7.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear7.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear7.set_limit_torque(1.5f); // lowered from 1.5
  cybergear7.enable_motor(); /* turn on the motor */
  
  cybergear8.init_motor(MODE_POSITION); 
  cybergear8.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear8.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear8.set_limit_torque(1.5f); // lowered from 1.5
  cybergear8.enable_motor(); /* turn on the motor */
  
  cybergear9.init_motor(MODE_POSITION); 
  cybergear9.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear9.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear9.set_limit_torque(1.5f); // lowered from 1.5
  cybergear9.enable_motor(); /* turn on the motor */
  
  cybergear10.init_motor(MODE_POSITION); 
  cybergear10.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear10.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear10.set_limit_torque(1.5f); // lowered from 1.5
  cybergear10.enable_motor(); /* turn on the motor */
  
  cybergear11.init_motor(MODE_POSITION); 
  cybergear11.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear11.set_limit_current(4.0); /* current limit allows faster operation */
  cybergear11.set_limit_torque(1.5f); // lowered from 1.5
  cybergear11.enable_motor(); /* turn on the motor */
  
  cybergear12.init_motor(MODE_POSITION); 
  cybergear12.set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
  cybergear12.set_limit_current(6.0); /* current limit allows faster operation */
  cybergear12.set_limit_torque(1.5f); // lowered from 1.5
  cybergear12.enable_motor(); /* turn on the motor */

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