#include "ble_config.h"

/*
 * Apurv Suman
 * UW PMP CSEP590 A4
 * Heavily uses scaffolding code at https://github.com/jonfroehlich/CSE590Sp2018/tree/master/A04-FaceTrackerBLE
 */

/*
 * Provides skeleton code to interact with the Android FaceTrackerBLE app 
 * 
 * Created by Jon Froehlich, May 7, 2018
 * 
 * Based on previous code by Liang He, Bjoern Hartmann, 
 * Chris Dziemborowicz and the RedBear Team. See: 
 * https://github.com/jonfroehlich/CSE590Sp2018/tree/master/A03-BLEAdvanced
 */

#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

#define RECEIVE_MAX_LEN  5
#define SEND_MAX_LEN    3

// Must be an integer between 1 and 9 and and must also be set to len(BLE_SHORT_NAME) + 1
#define BLE_SHORT_NAME_LEN 8 
#define BLE_SHORT_NAME 'a','p','s','u','m','a','n'  

#define HAPPINESS_ANALOG_OUT_PIN D2

#define MAX_SERVO_ANGLE  180
#define MIN_SERVO_ANGLE  0

#define BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN D7

// happiness meter (servo)
Servo _happinessServo;

/* ultrasonic setup */
  // Pins
const int TRIG_PIN = D0;
const int ECHO_PIN = D1;

  // Constants
  // Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 400;
const unsigned int WARNING_DIST = 50;

  // Smoothing variables for ultrasonic sensor
uint16_t const SMOOTHING_WINDOW = 3;
uint16_t smoothDistance[SMOOTHING_WINDOW];
int smoothCurrentPosition = 0;
uint16_t lastSmoothDistanceValue = MAX_DIST; // starting value default

  // helper functions
void reactToUltrasonicDistance(uint16_t);
uint16_t getUltrasonicValue();
void updateUltrasonicLastValue(uint16_t cm);
uint16_t smoothIntArray(uint16_t arr[]);

  // sound and LED and photo indicators
const int SOUND_OUTPUT_PIN = A4;
const int LED_OUTPUT_PIN = A0;
const int PHOTO_INPUT_PIN = A2;
const unsigned int MAX_HUMAN_FREQ = 5000;
const unsigned int MIN_HUMAN_FREQ = 2000;
const unsigned int MAX_PHOTO_VAL = 4096;
const float DARKNESS_MULTIPLIER_MAX = 2.0;

/*
 * Creative Feature: Blinking Password
 * In a giving Blink Window (i.e. smoothing) determine which eyes were blinked
 * The sequence of blinks gives you a password
 * If the password attempt matches the hard-coded password
 * Then disable the alarm
 */
/* creative feature variables */
bool alarmOn = true;
int const CORRECT_PASSWORD[] = { 1, 2 }; // hard-coded password
int const PASSWORD_SIZE = 2;
int password[PASSWORD_SIZE];
int currentPasswordPosition = 0;
int const BLINK_WINDOW_SIZE = 10;
int leftBlink[BLINK_WINDOW_SIZE];
int rightBlink[BLINK_WINDOW_SIZE];
int currentBlinkPosition = 0;

// Device connected and disconnected callbacks
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle);
void deviceDisconnectedCallback(uint16_t handle);

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000; // send

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size); // function declaration for receiving data callback
static uint8_t send_data[SEND_MAX_LEN] = { 0x00 };

// Define the configuration data
static uint8_t adv_data[] = {
  0x02,
  BLE_GAP_AD_TYPE_FLAGS,
  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE, 
  
  BLE_SHORT_NAME_LEN,
  BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
  BLE_SHORT_NAME, 
  
  0x11,
  BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71 
};

static btstack_timer_source_t send_characteristic;
static void bleSendDataTimerCallback(btstack_timer_source_t *ts); // function declaration for sending data callback
int _sendDataFrequency = 200; // 200ms (how often to read the pins and transmit the data to Android)

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Face Tracker BLE Demo.");

  // Initialize ble_stack.
  ble.init();
  
  // Register BLE callback functions
  ble.onConnectedCallback(bleConnectedCallback);
  ble.onDisconnectedCallback(bleDisconnectedCallback);

  //lots of standard initialization hidden in here - see ble_config.cpp
  configureBLE(); 
  
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);
  
  // Register BLE callback functions
  ble.onDataWriteCallback(bleReceiveDataCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");

  // Setup servo motor pin
  pinMode(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, OUTPUT);
  _happinessServo.attach(HAPPINESS_ANALOG_OUT_PIN);
  _happinessServo.write( (int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0) );

  // sound and led and photo pin setup
  pinMode(LED_OUTPUT_PIN, OUTPUT);
  pinMode(SOUND_OUTPUT_PIN, OUTPUT);
  pinMode(PHOTO_INPUT_PIN, INPUT);
  
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Start a task to check status of the pins on your RedBear Duo
  // Works by polling every X milliseconds where X is _sendDataFrequency
  send_characteristic.process = &bleSendDataTimerCallback;
  ble.setTimer(&send_characteristic, _sendDataFrequency); 
  ble.addTimer(&send_characteristic);
}

void loop() 
{
  
}

/**
 * @brief Connect handle.
 *
 * @param[in]  status   BLE_STATUS_CONNECTION_ERROR or BLE_STATUS_OK.
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("BLE device connected!");
      digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, HIGH);
      break;
    default: break;
  }
}

/**
 * @brief Disconnect handle.
 *
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleDisconnectedCallback(uint16_t handle) {
  Serial.println("BLE device disconnected.");
  digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, LOW);
}

/**
 * @brief Callback for receiving data from Android (or whatever device you're connected to).
 *
 * @param[in]  value_handle  
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.   
 *
 * @retval 
 */
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Received data: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++) {
      Serial.print(receive_data[index]);
      Serial.print(" ");
    }
    Serial.println(" ");
    
    // process the data received from Android 
    if (receive_data[0] == 0x01) { //receive the face data 
      // blinking values for creative password
      leftBlink[currentBlinkPosition] = receive_data[1];
      rightBlink[currentBlinkPosition] = receive_data[2];

      // check if you have enough right and left vals to update password
      if (currentBlinkPosition == BLINK_WINDOW_SIZE - 1) {
        updatePassword();
        currentBlinkPosition = 0;

        // check if you have you enough password vals to evaluate password
        if (currentPasswordPosition == PASSWORD_SIZE - 1) {
          evaluatePassword();
          currentPasswordPosition = 0;
        } else {
          currentPasswordPosition += 1;
        }
      } else {
        currentBlinkPosition += 1;
      }
      
      //servo angling
      int servoVal = map(receive_data[4], 0, 255, 135, 45);

      _happinessServo.write( servoVal );
    }
  }
  return 0;
}

/**
 * @brief Timer task for sending status change to client.
 * @param[in]  *ts   
 * @retval None
 * 
 * Send the data from either analog read or digital read back to 
 * the connected BLE device (e.g., Android)
 */
static void bleSendDataTimerCallback(btstack_timer_source_t *ts) {
  
  uint16_t cm = getUltrasonicValue();
  updateUltrasonicLastValue(cm);
  reactToUltrasonicDistance();
    
  /* Restart timer */
  ble.setTimer(ts, _sendDataFrequency);
  ble.addTimer(ts);
}

/*
 *  Helper functions for ultrasonic
 *  Reading the value
 *  Smoothing the value
 *  Sending and reacting to value (alarms, back to Android, etc)
 *  
*/
uint16_t getUltrasonicValue() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  uint16_t cm;
  uint16_t inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  // of sound in air at sea level (~340 m/s).
  // Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  cm = pulse_width / 58;

  return cm;
}

void updateUltrasonicLastValue(uint16_t cm) {

  smoothDistance[smoothCurrentPosition] = cm;
  if (smoothCurrentPosition == SMOOTHING_WINDOW - 1) {
    lastSmoothDistanceValue = smoothIntArray(smoothDistance);
    smoothCurrentPosition = 0;
  } else {
    smoothCurrentPosition = smoothCurrentPosition + 1;
  }
}

uint16_t smoothIntArray(uint16_t arr[]) {
  uint16_t sum = 0;
  for (uint16_t i = 0; i < SMOOTHING_WINDOW; i++) {
    sum = sum + arr[i];
  }

  return sum / SMOOTHING_WINDOW;
  
}

void reactToUltrasonicDistance() {
  // sound alarm and LED if within 0.5m
  if (lastSmoothDistanceValue < WARNING_DIST) {
    if (alarmOn) {
      analogWrite(LED_OUTPUT_PIN, 255);
      // higher frequency as the person gets closer
      int distanceToSound = map(lastSmoothDistanceValue, WARNING_DIST, 0, MIN_HUMAN_FREQ, MAX_HUMAN_FREQ/DARKNESS_MULTIPLIER_MAX);

      // higher frequency if its darker
      distanceToSound = distanceToSound * getDarknessMultiplier();
      
      tone(SOUND_OUTPUT_PIN, distanceToSound);
    }
  } else {
    analogWrite(LED_OUTPUT_PIN, 0);
    noTone(SOUND_OUTPUT_PIN);
  }

  // send distance over to Android app
  send_data[0] = lastSmoothDistanceValue;
  if (ble.attServerCanSendPacket())
        ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
}

/* creative feature helpers */
void updatePassword() {
  // left and right blinks are determined based on what amount of the blink window had that eye blinking
  bool leftBlinkStatus = majorityInArray(leftBlink);
  bool rightBlinkStatus = majorityInArray(rightBlink);

  int passwordVal;
  if (!leftBlinkStatus && !rightBlinkStatus) {
    passwordVal = 0;
  } else if (leftBlinkStatus && !rightBlinkStatus) {
    passwordVal = 1;
  } else if (!leftBlinkStatus && rightBlinkStatus) {
    passwordVal = 2;
  } else if (leftBlinkStatus && rightBlinkStatus) {
    passwordVal = 3;
  }

  password[currentPasswordPosition] = passwordVal;
}

void evaluatePassword() {
  for (int i = 0; i < PASSWORD_SIZE; i++) {
    // password fails so move on
    if (CORRECT_PASSWORD[i] != password[i]) {
      return;
    }
  }

  // passwords match so disable alarm
  alarmOn = false;
}

bool majorityInArray(int arr[]) {
  int sum = 0;
  int arrLen = sizeof(arr)/sizeof(arr[0]);
  for (int i = 0; i < arrLen; i++) {
    sum = sum + arr[i];
  }

  return sum > (arrLen / 4.0);
}

/* function creates a multiplier based on how dark it is */
float getDarknessMultiplier() {
  int photoVal = analogRead(PHOTO_INPUT_PIN);
  float multiplier = 1.0;
  if (photoVal < MAX_PHOTO_VAL/DARKNESS_MULTIPLIER_MAX) {
    multiplier = DARKNESS_MULTIPLIER_MAX;
  }
  return multiplier;
}


