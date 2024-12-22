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

#define MASTER_CAN_ID 0x00
#define CYBERGEAR_COUNT 12

//                                      1     2     3     4     5     6     7     8     9    10    11    12
const uint8_t CYBERGEAR_CAN_IDS[] = {0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x71, 0x70};
// 0x71 was was 0x6F!! changed to motor 13

XiaomiCyberGearDriver *cybergears[CYBERGEAR_COUNT];

// function declarations
static void initialize_all_motors();
static void all_motor_pos_to_zero();
static void handle_rx_message(twai_message_t& message);
static void check_alerts();

void setup() {
  for (int i=0; i<CYBERGEAR_COUNT; i++) {
    cybergears[i] = new XiaomiCyberGearDriver(CYBERGEAR_CAN_IDS[i], MASTER_CAN_ID);
  }

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

  XiaomiCyberGearStatus cybergear_status = cybergears[0]->get_status();
  Serial.printf("Motor 1: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergears[0]->request_status();
  }
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_IDS[0]){
    cybergears[0]->process_message(message);
  }
}

static void all_motor_pos_to_zero()
{
  for (auto &gd : cybergears) {
    gd->set_position_ref(0.0f);
  }
}

static void initialize_all_motors()
{
  cybergears[0]->init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear

  //                                 1    2    3    4    5    6    7    8    9   10   11   12
  const double CURRENT_LIMITS[] = {6.0, 4.0, 6.0, 6.0, 4.0, 6.0, 6.0, 4.0, 6.0, 6.0, 4.0, 6.0};
  for (int i=0; i<CYBERGEAR_COUNT; i++) {
    cybergears[i]->init_motor(MODE_POSITION);
    cybergears[i]->set_limit_speed(1.0f); /* set the maximum speed of the motor */ // was set to 10.0f!
    cybergears[i]->set_limit_current(CURRENT_LIMITS[i]); /* current limit allows faster operation */
    cybergears[i]->set_limit_torque(1.5f); // lowered from 1.5
    cybergears[i]->enable_motor(); /* turn on the motor */
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
