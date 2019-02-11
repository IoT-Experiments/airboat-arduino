#include <Arduino.h>
#include <Servo.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define NAME "AIRBOAT"
#define PIN_MOTOR_1A 5
#define PIN_MOTOR_1B 6
#define PIN_SERVO 9

#define PIN_DEBUG_RX 7
#define PIN_DEBUG_TX 8

#define EEPROM_SCHEMA 0x01
#define EEPROM_ADDRESS_SERVO_MIDDLE_POSITION 1

#define SERVO_MAX_DEVIATION 25
#define SERVO_MIDDLE_POSITION_DEFAULT 90

//#define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(x)  g_debugSerial.println(x)
#else
 #define DEBUG_PRINT(x)
#endif

//////////

void watchdogSetup();
void sendATCommand(String command);
bool waitForResponse(String value, unsigned int timeout);
void resetBluetoothModule();
void setup_eeprom();

//////////

int16_t g_throttleValue = 0;
int16_t g_servoValue = 0;
uint8_t g_servoAngle;
uint8_t g_servoMiddlePosition;
Servo g_servoCtrl;
bool g_connected = false;
SoftwareSerial g_debugSerial(PIN_DEBUG_RX, PIN_DEBUG_TX);

//////////

void setup() {
  g_debugSerial.begin(115200);
  Serial.begin(9600);
  // We want to control the timeout with the delay() function into the main loop
  Serial.setTimeout(50); // Should be enough at 9600bauds = 1200 bytes / sec

  setup_eeprom();
  g_servoMiddlePosition = EEPROM.read(EEPROM_ADDRESS_SERVO_MIDDLE_POSITION);

  pinMode(PIN_MOTOR_1A, OUTPUT);
  pinMode(PIN_MOTOR_1B, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);

  g_servoCtrl.attach(PIN_SERVO);
  g_servoAngle = g_servoMiddlePosition;

  //  Init values
  g_servoCtrl.write(g_servoAngle);
  analogWrite(PIN_MOTOR_1A, LOW);
  analogWrite(PIN_MOTOR_1B, LOW);

  //watchdogSetup();

  DEBUG_PRINT("Begin BLE setup");

  // Waits for bluetooth module
  resetBluetoothModule();

  // Setup name
  sendATCommand(String("AT+NAME") + NAME);
  delay(1000);
  // Set slave mode
  sendATCommand("AT+ROLE0"); // seems to cause a reset
  delay(1000);
  // Set System LED
  sendATCommand("AT+PIO10");
  delay(1000);
  // Setup mode and type
  sendATCommand("AT+MODE0"); // Transmission mode (serial after connection)
  delay(1000);
  sendATCommand("AT+TYPE0"); // Disable pin code
  delay(1000);
  // Set password
  //sendATCommand("AT+PASS000000");
  //delay(1000);
  // Activate "connected" notifications
  sendATCommand("AT+NOTI1");
  delay(1000);
  // Disable address in "connected" notifications
  sendATCommand("AT+NOTP0");
  delay(1000);
  // Reset to apply configuration
  //resetBluetoothModule();
  DEBUG_PRINT("End of BLE setup");
}

void loop() {
  // Wait connection
  if(Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    DEBUG_PRINT("BLE serial RX: " + line);

    if(line.indexOf("OK+LOST") >= 0) {
      g_connected = false;
      DEBUG_PRINT("-> Disconnected");
      analogWrite(PIN_MOTOR_1A, LOW);
      analogWrite(PIN_MOTOR_1B, LOW);
      g_servoCtrl.write(g_servoMiddlePosition);
    } else if(line.indexOf("OK+CONN") >= 0) {
      g_connected = true;
      DEBUG_PRINT("-> Connected");
      Serial.println("OK:CONN");
    } else if(line.indexOf("AT$TEST") >= 0) {
      Serial.println("OK:TEST_START");
      g_servoCtrl.write(g_servoMiddlePosition - SERVO_MAX_DEVIATION);
      analogWrite(PIN_MOTOR_1A, 255);
      analogWrite(PIN_MOTOR_1B, LOW);
      delay(2000);
      //wdt_reset();
      g_servoCtrl.write(g_servoMiddlePosition + SERVO_MAX_DEVIATION);
      analogWrite(PIN_MOTOR_1A, LOW);
      analogWrite(PIN_MOTOR_1B, 255);
      delay(2000);
      //wdt_reset();
      g_servoCtrl.write(g_servoMiddlePosition);
      analogWrite(PIN_MOTOR_1A, LOW);
      analogWrite(PIN_MOTOR_1B, LOW);
      Serial.println("OK:TEST_STOP");
    } else if(line.startsWith("AT$PARAMS:")) {
      // Remote AT command to change the parameters
      String parameters = line.substring(10);
      // motor speed / stepper (°)
      if(sscanf(parameters.c_str(), "%d;%d", &g_throttleValue, &g_servoValue) != EOF) {
        if(g_throttleValue == 0) {
            analogWrite(PIN_MOTOR_1A, LOW);
            analogWrite(PIN_MOTOR_1B, LOW);
        } else if(g_throttleValue > 0) {
            analogWrite(PIN_MOTOR_1A, round(g_throttleValue / 100.0 * 255));
            analogWrite(PIN_MOTOR_1B, LOW);
        } else if(g_throttleValue < 0) {
            analogWrite(PIN_MOTOR_1A, LOW);
            analogWrite(PIN_MOTOR_1B, round(abs(g_throttleValue) / 100.0 * 255));
        }

        float stepperAngle = g_servoMiddlePosition + SERVO_MAX_DEVIATION * g_servoValue / 100.0;
        DEBUG_PRINT(String("Angle: ") + stepperAngle);

        g_servoAngle = (uint8_t)round(stepperAngle);
        g_servoCtrl.write(g_servoAngle);
        Serial.println("OK");
      } else {
        Serial.println("ERR");
      }
    } else if(line.startsWith("AT$TRIM:")) {
      String parameters = line.substring(8, 9);
      if(parameters == "+") {
        g_servoMiddlePosition++;
      } else if(parameters == "-") {
        g_servoMiddlePosition--;
      } else if(parameters == "R") {
        g_servoMiddlePosition = SERVO_MIDDLE_POSITION_DEFAULT;
      }
      EEPROM.write(EEPROM_ADDRESS_SERVO_MIDDLE_POSITION, g_servoMiddlePosition);
      g_servoAngle = g_servoMiddlePosition;
      g_servoCtrl.write(g_servoMiddlePosition);
      Serial.println("OK");
    }
  }

  //wdt_reset();
}

void sendATCommand(String command) {
  Serial.print(command);
  DEBUG_PRINT("BLE serial TX: " + command);
}

void resetBluetoothModule() {
  sendATCommand("AT+RESET");
  String serialData;
  do {
    delay(500);
    if(Serial.available()) {
      serialData = Serial.readString();
      DEBUG_PRINT("Received " + serialData);
    }
  } while(serialData.indexOf("www.jnhuamao.cn") == -1);
}

bool waitForResponse(String value, unsigned int timeout) {
  unsigned long startTime = millis();
  String line;
  do {
    line = Serial.readString();
    delay(500);
    if(millis() - startTime > timeout) {
      return false;
    }
  } while(line.indexOf(value) == -1);
  return true;
}

void setup_eeprom() {
  uint8_t eepromSchema = EEPROM.read(0);
  if(eepromSchema != EEPROM_SCHEMA) {
    EEPROM.write(0, EEPROM_SCHEMA);
    EEPROM.write(EEPROM_ADDRESS_SERVO_MIDDLE_POSITION, 90);
  }
}

void watchdogSetup(void) {
  cli(); // disable all interrupts
  wdt_reset(); // reset the WDT timer
  // Enter Watchdog Configuration mode
  WDTCSR |= _BV(WDCE);
  // Set Watchdog settings (8s)
  WDTCSR |= _BV(WDE); // Enables watchdog
  WDTCSR |= _BV(WDP3);
  WDTCSR &= ~_BV(WDP2);
  WDTCSR &= ~_BV(WDP1);
  WDTCSR |= _BV(WDP0);
  WDTCSR &= ~_BV(WDIE); // Disables interrupt
  WDTCSR &= ~_BV(WDCE); // Exit configuration mode
  sei();
}

// Vector for watchdog
ISR(WDT_vect) {
  // N/A
}
