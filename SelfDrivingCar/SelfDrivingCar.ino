/**
 * Self driving car project.
 *
 * Pre-requisits
 * ***************
 *
 * Libraries (Optional):
 *   > LiquidCrystal new: https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
 *      Install it in ~/Arduino/libraries/ to override the IDE one
 *   > IRremote: http://github.com/shirriff/Arduino-IRremote
 *      remove IDE's libraries/RobotIRremote and add this one to ~/Arduino/libraries instead
 *
 * Board-specific tweaks:
 *   Mega (ATmega2560)
 *   -----------------
 *     * You will get errors compiling with the default libraries, to remove timer conflicts beween them you need to
 *         > ~/Arduino/IRremote/IRremoteInt.h
 *             uncomment "#define IR_USE_TIMER3" instead of the default "#define IR_USE_TIMER2" for __AVR_ATmega2560__
 *         > IDE libraries/Servo/src/avr/ServoTimers.h
 *             leave only "#define _useTimer5" and shorten the timer16_Sequence_t list accordingly for __AVR_ATmega2560__
 *             (one timer can drive up to 12 servos)
 *
 *  Uno and Nano (ATmega328)
 *  ------------------------
 *    * These boards only have 3 timers, one for delay(), one for servos and the other one for either IR or buzzer, so
 *       you have to choose between being able to remote control it or buzz when the battery is low, cannot do both...
 *
 *  Edison (x86 Intel)
 *  ------------------
 *    * IRremote and LiquidCrystal new are not x86-ready
 *
 *  Constants and specs:
 *    > Mega board          : __AVR_ATmega2560__ (256KB of code available to the user, 8KB of RAM, 4KB EEPROM)
 *    > Micro board         : __AVR_ATmega32U4__ ( 28KB of code available to the user, 2.5KB of RAM, 1KB EEPROM)
 *    > Uno and Nano boards : __AVR_ATmega328P__ ( 31.5KB Uno / 30KB Nano of code available to the user, 2KB of RAM, 1KB EEPROM)
 */

// Whether or not to use the Serial as debugging tool
// Bluetooth to Serial dongle can be connected on pins D0 (TX -> dongle RX) and D1 (RX -> dongle TX) to help with remote debugging
#define __DEBUG__

//Uncomment the lines below if the respective equipment is connected
#ifndef __ARDUINO_X86__
// #define LCD_CONNECTED
// #define IR_PIN 2
#endif

// #define BATT_PIN A3
// #define TILT_PIN 3

#if defined(BATT_PIN) && ((not defined(IR_PIN)) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__))
// only Mega and Micro support both the IR receiver and the tone() generator, Uno and Nano do not
// #define PIEZO_PIN 6
#endif

// All conditional includes have to be commented out manually, because of the preprocessor bug in the Arduino IDE
#if defined(LCD_CONNECTED)
//#include <Wire.h>
// LCD is connected to the I2C bus (Uno/Nano board: SDA=A4, SCL=A5; Mega2560 board: SDA=20, SCL=21; Micro/Leonardo: SDA=2, SCL=3)
//#include <LiquidCrystal_I2C.h>
#endif

#if defined(IR_PIN)
//#include <IRremote.h>
#endif

#if defined(BATT_PIN)
#include "Battery.h"
#endif

#include <Servo.h>
#include "ServoExt.h"
#include "SR04.h"

// Analog pins
#define STEERING_SERVO A0
#define THROTTLE_SERVO A1
#define SCANNING_SERVO A2
#define IR_RANGE A6

// Front-facing range finder HC SR04 module
SR04 frontRanger(4, 5);

#if defined(IR_RANGE)
#include "Sharp.h"

Sharp irRanger(IR_RANGE);
#endif

#if defined(BATT_PIN)
// R1 = 10Kohm, R2 = 3.3Kohm
#define R1 10000.
#define R2 3300.
// Car is powered by a LiPo 2s
Battery carBattery(BATT_PIN, R2 / (R1 + R2), 4.93, 2, CHEMISTRY_LIPO);
#endif

// Digital pins

// Serial is connected to the Bluetooth dongle, 0 and 1

#if defined(IR_PIN)
IRrecv irrecv(IR_PIN);
decode_results results;

#define CAR_STOP 1
#define CAR_RUN 0

// "OFF" key on the LED remote
#define IR_CODE_PAUSE 0xFFF807
// "ON" key on the LED remote
#define IR_CODE_PLAY  0xFFB04F
#endif

#if defined(LCD_CONNECTED)
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

// Futaba S3003
// 100% of the remote control travel is 30&deg; only
// Time to reach the above max travel, doc says 230ms for 60&deg; at 4.8V (no load!)
ServoExt steeringServo(30, 200);

// Tamiya ESC, 100% of the remote control travel is 30&deg;
ServoExt throttleServo(30, 3000);

// car top speed, in cm/s
// TT-01 specs give a top speed of 29km/h at full throttle with 2S LiPo. That is 800cm/s.
#define CAR_TOP_SPEED 200

// Kit: TowerPro 9g SG90
// Current: Turnigy TGY-930, it can travel max to 75&deg;
// Time to reach max travel, doc says 140ms for 60&deg; at 4.8V, so 175ms at 75&deg;
ServoExt scanningServo(75, 175);

void debug(String message) {
#if defined(__DEBUG__)
  Serial.print(millis() / 1000);
  Serial.print(" : ");
  Serial.println(message);
#endif
}

/**
 * Set up the board pins and initialize the available hardware, as per the defined options above
 */
void setup() {
#if defined(__DEBUG__)
  Serial.begin(115200);

  delay(5000);
#endif

#if defined(LCD_CONNECTED)
  lcd.begin(16, 2);
  lcd.backlight();
  setLCD("Car booting");
#endif

#if defined(PIEZO_PIN)
  pinMode(PIEZO_PIN, OUTPUT);
#endif

#if defined(TILT_PIN)
  pinMode(TILT_PIN, INPUT_PULLUP);
#endif

  randomSeed(analogRead(A6));

  pinMode(13, OUTPUT);

  // attach the servos, also setting the pin mode
  steeringServo.attach(STEERING_SERVO);
  throttleServo.attach(THROTTLE_SERVO);
  scanningServo.attach(SCANNING_SERVO, 600, 2225);

  fullStop();

  // internal led: off
  digitalWrite(13, LOW);

#if defined(IR_PIN)
  irrecv.enableIRIn(); // Start the receiver
#endif

#if defined(LCD_CONNECTED)
  setLCD("Opening eyes");
#endif

  steeringServo.setAngle(90);

  lookAround();

  //  Timer1.initialize(50000);  // call the driver() function every 50ms
  //  Timer1.attachInterrupt(driver);
}

/**
 * Stop the car and center the steering wheel
 */
void fullStop() {
  // no movement
  goForward(0);

  // center direction
  steeringServo.setAngle(90, true);

  throttleServo.waitForAngle();
}

/**
 * Get the range to the nearest object using the available sensors
 * @return range to object in cm, zero or negative if the range cannot be determined
 */
int getRange(){
#if defined(IR_RANGE)
  const int range = irRanger.getRange();
  
  if (range<150)
    return range;
  
  const int sonicRange = frontRanger.getRange();
  
  if (sonicRange==0)
    return range;
  
  if (sonicRange>=150)
    return sonicRange;
  
  return min(sonicRange, range);
#else
  return frontRanger.getRange();
#endif
}

/**
 * <code>true</code> if the car was going forward and should first brake before reversing
 */
volatile boolean wasForward = false;

/**
 * Updated with the last moment when the car was set to go forward
 */
volatile unsigned long int runningSince = 0;

/**
 * Estimated running speed set by the last command, in cm/s
 */
volatile int runningSpeed;

/**
 * Tell the car to go forward (throttle > 0), backwards (throttle < 0) or stop (throttle=0).
 * When reversing or stopping from going forward the car will actually call for a brake, setting the ESC in reverse then putting it back to neutral.
 * @param throttle percentage of throttle to ask for, in the range [-100, 100]
 */
void goForward(const int throttle) {
  if (throttle <= 0 && wasForward) {
    // simulate a braking
    throttleServo.write(75);
    // TODO: consider for how long to brake, function of the estimated current speed
    delay(200);
    // back to neutral
    throttleServo.write(90);
    delay(50);

    runningSince = 0;
    runningSpeed = 0;
  }

  throttleServo.setPercentage(throttle, false);

  if (throttle < 0) {
    wasForward = false;
  }
  else if (throttle > 0) {
    runningSince = millis();
    runningSpeed = throttle * CAR_TOP_SPEED / 100;
    wasForward = true;
  }
}

#if defined(LCD_CONNECTED)
// whether or not the previous iteration it was in the default state (showing the distance) or was a different message and in that case the content has to be reset
boolean wasCleared = true;

/**
 * Clear the LCD and set this message on the first line
 */
void setLCD(const char* message) {
  lcd.clear();
  lcd.home();
  lcd.print(message);
  wasCleared = true;
}
#endif

#if defined(BATT_PIN) && defined(PIEZO_PIN)
/**
 * Generate an alarm tone
 */
void alarmTone() {
  tone(PIEZO_PIN, 262, 250);
  delay(250);
}
#endif

#if defined(BATT_PIN)
/**
 * Check battery level and if it's below 10% then stop the car
 *
 * @return <code>true</code> if the car can still run, <code>false</code> if it should stop and save the battery
 */
boolean checkBattStatus() {
  float gas = carBattery.getGas(true);

#if defined(__DEBUG__)
  Serial.print("Battery gas left: ");
  Serial.print(gas);
  Serial.print(", V: ");
  Serial.println(carBattery.getVoltage(true));
#endif

  if (gas < 10) {
    fullStop();
    digitalWrite(13, HIGH);

#if defined(PIEZO_PIN)
    alarmTone();
#else
    delay(500);
#endif
    digitalWrite(13, LOW);
#if defined(PIEZO_PIN)
    alarmTone();
#else
    delay(500);
#endif

    return false;
  }

  return true;
}
#endif

#if defined(TILT_PIN)
// flag set to <code>true</code> when the car is upside down
volatile boolean wasTurned = false;

/**
 * Check if the tilt sensor is triggered (car is turned over) and stop the car if so.
 *
 * @return <code>true</code> if the car can run or <code>false</code> if it was stopped and should not run
 */
boolean checkTiltStatus() {
  if (digitalRead(TILT_PIN) == LOW) {  // Using internal pullup resistor the value will be HIGH by default, LOW when the circuit is closed (tilted)
    if (!wasTurned) {
      // car is turned upside down, full stop

#if defined(LCD_CONNECTED)
      setLCD("Put me down");
#endif

      fullStop();

      wasTurned = true;
    }

    return false;
  }

  if (wasTurned) {
#if defined(LCD_CONNECTED)
    setLCD("Thank you");
#endif
    // wait a couple of seconds for the car to stand still

    for (byte i = 0; i < 20; i++) {
      if (digitalRead(TILT_PIN) == HIGH) {
#if defined(LCD_CONNECTED)
        setLCD("Unstable ");
#endif
        return false;
      }
      delay(100);
    }

    wasTurned = false;

    scanForOptions(true);
  }

  return true;
}
#endif

#if defined(IR_PIN)
volatile byte remoteState = CAR_RUN;

/**
 * Check if the STOP command was received. If it was received (now or in previous iterations) then do a full stop.
 *
 * @return <code>true</code> if the car can continue running, <code>false</code> if it should stop.
 */
boolean checkIRStatus() {
  if (irrecv.decode(&results)) {
#if defined(__DEBUG__)
    Serial.print("Got remote command: ");
    Serial.println(results.value, HEX);
#endif
    irrecv.resume(); // Receive the next value

    switch (results.value) {
      case IR_CODE_PAUSE:
        remoteState = CAR_STOP;
        break;
      case IR_CODE_PLAY:
        if (remoteState == CAR_STOP)
          scanForOptions(true);

        remoteState = CAR_RUN;
        break;
    }
  }

  if (remoteState == CAR_STOP) {
#if defined(LCD_CONNECTED)
    setLCD("Cooling tires...");
#endif

    fullStop();
    delay(100);
    return false;
  }

  return true;
}
#endif

// signed
char scanDir = 1;

// initial angle = (index+1) * 15 degrees
// min angle = 15
// max angle = 165
#define SCAN_ANGLES 11
#define SCAN_ANGLE_INCREMENT 15

volatile unsigned int distanceAtAngle[SCAN_ANGLES];
// actual angle of the distance, updated as the car travels
volatile float angle[SCAN_ANGLES];

// angle at which the car currently travels
volatile byte runningAngle = 90;

/**
 * After a critical event clear the distance vectors to force a full update
 */
void clearDistanceVectors() {
  for (byte i = 0; i < SCAN_ANGLES; i++) {
    distanceAtAngle[i] = 0;
    angle[i] = 0;
  }
}

void dumpDistance() {
#if defined(__DEBUG__)
  Serial.print("[Distance, Angle] = ");

  for (byte i = 0; i < SCAN_ANGLES; i++) {
    Serial.print("[");
    Serial.print(distanceAtAngle[i]);
    Serial.print(", ");
    Serial.print(angle[i]);
    Serial.print("] ");
  }

  Serial.println();
#endif
}

/**
 * This is the car driver code. It is called every 50ms to decide what to do next.
 */
boolean driver() {
#if defined(TILT_PIN)
  if (digitalRead(TILT_PIN) == LOW) {
    wasTurned = true;
  }
#endif

#if defined(IR_PIN)
  if (wasTurned || remoteState == CAR_STOP || scanning) {
    goForward(0);
    steeringServo.setAngle(90, false);
    return false;
  }
#endif

#if defined(__DEBUG__)
  Serial.print("After having travelled since ");
  Serial.print(runningSince);
  Serial.print(" ms (for ");
  Serial.print(millis() - runningSince);
  Serial.print(" ms) at ");
  Serial.print(runningSpeed);
  Serial.println(" cm/s");
  dumpDistance();
#endif

  unsigned int bestDistance = 0;
  byte bestAngle = 0;

  // update distance array and angles of previously known objects function of how much and in which direction the car has travelled to them
  if (runningSince > 0) {
    const long travelled = (runningSpeed * (millis() - runningSince)) / 1000;

#if defined(__DEBUG__)
    Serial.print("Travelled: ");
    Serial.print(travelled);
    Serial.println(" ");
#endif

    for (byte i = 0; i < SCAN_ANGLES; i++) {
      const byte angleDiff = angle[i] > runningAngle ? angle[i] - runningAngle : runningAngle - angle[i];

      const float z = distanceAtAngle[i] * cos(angleDiff * PI / 180);

      if (z <= travelled) {
        distanceAtAngle[i] = 0;
        angle[i] = 0;
        continue;
      }

      const float y = distanceAtAngle[i] * sin(angleDiff * PI / 180);

      const float q = sqrt( (z - travelled) * (z - travelled) + y * y );

      distanceAtAngle[i] = round(q);

      const float newAngle = asin(y / q) * 180 / PI;

      if (angle[i] < runningAngle)
        angle[i] = 90 - newAngle;
      else
        angle[i] = 90 + newAngle;

      if (distanceAtAngle[i] > bestDistance) {
        bestDistance = distanceAtAngle[i];
        bestAngle = i;
      }
    }
  }
  else {
    for (byte i = 0; i < SCAN_ANGLES; i++) {
      if (distanceAtAngle[i] > bestDistance) {
        bestDistance = distanceAtAngle[i];
        bestAngle = i;
      }
    }
  }

#if defined(__DEBUG__)
  Serial.println("After recomputing the distances:");
  dumpDistance();
#endif

  if (bestDistance >= 60) {
    // go in that direction
    runningAngle = angle[bestAngle];
    steeringServo.setAngle(runningAngle, false);

    // if the obstacle is at 2.5m or more, go full blast
    // otherwise slow down with the square root of the distance
    // absolute throttle max is 40% of the top speed, 20% is the minimum for the car to go forward at all
    const long fwSpeed =  20 + min(20, sqrt(bestDistance));

#if defined(__DEBUG__)
    Serial.print("Going towards ");
    Serial.print(angle[bestAngle]);
    Serial.print(" at ");
    Serial.print(fwSpeed);
    Serial.println("% throttle");
#endif

    goForward(fwSpeed);
  }
  else {
#if defined(__DEBUG__)
    Serial.println("Driver didn't have any option");
#endif

    return false;
  }

  return true;
}

/**
 * Main loop, waiting patiently for the ultrasound sensor to turn and get the range.
 */
void loop() {
  debug("1. enter loop");
  driver();
  debug("2. called driver");

#if defined(BATT_PIN)
  if (!checkBattStatus()) {
    delay(1000);
    return;
  }
#endif

#if defined(TILT_PIN)
  if (!checkTiltStatus()) {
    delay(1000);
    return;
  }
#endif

#if defined(IR_PIN)
  if (!checkIRStatus()) {
    return;
  }
#endif

  // continue from where the scanning servo was sitting last
  byte newScanAngle = scanningServo.getAngle();

  if (newScanAngle >= 120)
    newScanAngle = 120 - SCAN_ANGLE_INCREMENT;
  else if (newScanAngle <= 60)
    newScanAngle = 60 + SCAN_ANGLE_INCREMENT;
  else
    newScanAngle += scanDir * SCAN_ANGLE_INCREMENT;

  if (newScanAngle > scanningServo.getAngle())
    scanDir = 1;
  else
    scanDir = -1;

  scanningServo.setAngle(newScanAngle, true);

  debug("3. set scanning angle");

  const long distance = getRange();

  debug("4. got range");

#if defined(__DEBUG__)
  Serial.print("Range to the object at ");
  Serial.print(newScanAngle);
  Serial.print(" degrees: ");
  Serial.println(distance);
#endif

  int bestDistance = 0;
  byte bestAngle = 0;

  distanceAtAngle[(newScanAngle / SCAN_ANGLE_INCREMENT) - 1] = distance;
  angle[(newScanAngle / SCAN_ANGLE_INCREMENT) - 1] = newScanAngle;

  for (byte i = 0; i < SCAN_ANGLES; i++) {
    if (distanceAtAngle[i] > bestDistance) {
      bestDistance = distanceAtAngle[i];
      bestAngle = i;
    }
  }

  debug("5. got best direction");

  if (bestDistance < 60) {
    debug("5.1. Collision alert");

    // at 60cm from an obstacle stop and scan for alternative routes
    fullStop();

#if defined(LCD_CONNECTED)
    setLCD("Collision alert");
#endif

    for (byte i = 0; i < 6; i++) {
      const byte idx = scanForOptions(i % 2 == 0);

      if (idx < SCAN_ANGLES) {
        bestDistance = distanceAtAngle[idx];
        bestAngle = idx;
        break;
      }
    }

    if (bestDistance < 60) {
      // still didn't find a way out, give the user a chance to put the car back in some open space
      debug("Waiting 5 seconds for human intervention");

      delay(5000);
      return;
    }
  }

  debug("6. cleaning up");

#if defined(LCD_CONNECTED)
  if (wasCleared) {
    setLCD("Distance: ");
    wasCleared = false;
  }

  lcd.setCursor(0, 1);
  lcd.print(bestDistance);
  lcd.print("cm, ");
  lcd.print(angle[bestAngle]);
  lcd.print("deg");
#endif
}

/**
 * Scan for options at all available angles
 *
 * @return position of the best direction to go in (index in the distance[] and angle[] arrays) or SCAN_ANGLES if no viable option is found
 */
byte lookAround() {
#if defined(__DEBUG__)
  const unsigned long startTime = millis();
#endif

  const bool asc = scanningServo.getAngle() <= 90;

  int largestDistance = 0;
  byte bestAngle = SCAN_ANGLES;

  for (byte j = 0; j < SCAN_ANGLES; j++) {
    const byte i = asc ? j : 10 - j;

    const byte scanAngle = (i + 1) * SCAN_ANGLE_INCREMENT;

    steeringServo.setAngle(scanAngle);
    scanningServo.setAngle(scanAngle, true);

    const int distance = getRange();

    distanceAtAngle[i] = distance;
    angle[i] = scanAngle;

    if (distance > largestDistance) {
      largestDistance = distance;
      bestAngle = i;
    }
  }

#if defined(__DEBUG__)
  Serial.print("lookAround() took ");
  Serial.print(millis() - startTime);
  Serial.println("ms");

  Serial.print("Best distance: ");
  Serial.print(largestDistance);
  Serial.print(" at angle ");
  Serial.print(bestAngle);
  Serial.print(" - ");
  Serial.println(angle[bestAngle]);
#endif

  return bestAngle;
}

/**
 * Scan for options and try to get out of a stuck state by going either forward or backwards (function of the reverseIfStuck parameter)
 * and at a random angle.
 *
 * @return the index in the distanceAtAngle[] and angle[] arrays, or SCAN_ANGLES if there is no viable option at the moment
 */
byte scanForOptions(const bool reverseIfStuck) {
  const byte bestOption = lookAround();

#if defined(__DEBUG__)
  if (bestOption > 0) {
    Serial.print("Largest distance: ");
    Serial.print(distanceAtAngle[bestOption]);
    Serial.print("cm, found at angle: ");
    Serial.print(angle[bestOption]);
  }
  Serial.println();
#endif

  if (bestOption == SCAN_ANGLES || distanceAtAngle[bestOption] < 60) {
    // try going back (or forward) a bit, in a random angle
    steeringServo.setPercentage(random(0, 201) - 100, true);

    goForward(reverseIfStuck ? -50 : 30); // half speed reverse (reversing is limited anyway by the ESC) or 30% forward
    delay(500);     // for half a second

    fullStop();

    return lookAround();
  }

  return bestOption;
}
