#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define steeringServoPin 23
#define mastTiltServoPin 22
#define cabLights 32
#define auxLights 33

#define mastMotor0 25  // Used for controlling auxiliary attachment movement
#define mastMotor1 26  // Used for controlling auxiliary attachment movement
#define auxAttach0 18  // Used for controlling auxiliary attachment movement
#define auxAttach1 17  // Used for controlling auxiliary attachment movement

#define leftMotor0 21   // Used for controlling the left motor movement
#define leftMotor1 19   // Used for controlling the left motor movement
#define rightMotor0 33  // Used for controlling the right motor movementc:\Users\JohnC\Desktop\SOLIDWORKS Connected.lnk
#define rightMotor1 32  // Used for controlling the right motor movement

// #define cell_1 12
#define cell_1_2 34

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo steeringServo;  //min 50 max 120 (30 range)
Servo mastTiltServo;   //min 36 max 114 

int servoDelay = 0;
int lightSwitchTime = 0;
float adjustedSteeringValue = 86;
float steeringAdjustment = 1;
int steeringTrim = 0;
int mastTiltValue = 90;

bool lightsOn = false;
bool moveMastTiltServoDown = false;
bool moveMastTiltServoUp = false;
bool hardLeft;
bool hardRight;
bool enoughBattery = true;
double lowBattRequired = 50;
double lowBattCounts = 0;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //Throttle
  processThrottle(ctl->axisY());
  //Steering
  processSteering(ctl->axisRX());
  //Rasing and lowering of mast
  processMast(ctl->axisRY());
  //MastTilt
  processMastTilt(ctl->dpad());
  //Aux
  processAux(ctl->thumbR());

  processTrimRight(ctl->r1());
  processTrimLeft(ctl->l1());

  if (ctl->l2()) {
    hardLeft = true;
  } else {
    hardLeft = false;
  }
  if (ctl->r2()) {
    hardRight = true;
  } else {
    hardRight = false;
  }
}

void processThrottle(int axisYValue) {
  float adjustedThrottleValue = axisYValue / 2;
  if (adjustedThrottleValue > 15 || adjustedThrottleValue < -15) {
    if (hardRight) {
      moveMotor(rightMotor0, rightMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (hardLeft) {
      moveMotor(leftMotor0, leftMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (adjustedSteeringValue > 100) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue * steeringAdjustment);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    } else if (adjustedSteeringValue < 80) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue * steeringAdjustment);
    } else {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    }
  } else {
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
  }
}

void processMast(int axisRYValue) {
  int adjustedMastValue = axisRYValue / 2;
  if (adjustedMastValue > 100 || adjustedMastValue < -100) {
    moveMotor(mastMotor0, mastMotor1, adjustedMastValue);
  } else {
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}
void processTrimRight(int trimValue) {
  if (trimValue == 1 && trimValue < 20) {
    steeringTrim = steeringTrim + 2;
    delay(50);
  }
}

void processTrimLeft(int trimValue) {
  if (trimValue == 1 && trimValue > -20) {
    steeringTrim = steeringTrim - 2;
    delay(50);
  }
}

void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 9));

  float steeringRange = 30;
  float steeringCenter = 90 - steeringTrim;
  float maxSteering = steeringCenter + steeringRange;
  float minSteering = steeringCenter - steeringRange;
  float steeringOutput = adjustedSteeringValue - steeringTrim;

  steeringOutput = min(steeringOutput, maxSteering);
  steeringOutput = max(steeringOutput, minSteering);

  if ( enoughBattery){
    steeringServo.write(steeringOutput);
  }

  if (adjustedSteeringValue > 100) {
    steeringAdjustment = ((200 - adjustedSteeringValue) / 100);
  } else if (adjustedSteeringValue < 80) {
    steeringAdjustment = ((200 - (90 + (90 - adjustedSteeringValue))) / 100);
  }
}

void calcBattVoltage(void){
  // float inputRaw = analogRead(cell_1_2);
  float inputRaw  = 0;
  for  (byte n = 0; n < 5; n++)
  {
    inputRaw += analogRead(cell_1_2);
    delay (2);
  }
  inputRaw = inputRaw / 5;
  float scaleFactor = 2.837/3.3;
  float battFull = 8.4;
  float battEmpty = 6.1;
  // float battEmpty = 7.36;

  float adcMax = 4095;
  float adcScaled = adcMax * scaleFactor;
  float battFullMapped = adcScaled; 
  float battEmptyMapped = battEmpty/battFull * adcScaled;

  float battVoltage =  min(inputRaw/adcScaled, float(1.0)) * battFull;
  Serial.print("\nBatt voltage ");
  Serial.print(battVoltage);
  Serial.print("       RAW ");
  Serial.print(inputRaw);
  if (battVoltage <= battEmpty){
    lowBattCounts ++;
  }else{
    lowBattCounts =0;
  }

  if(lowBattCounts > lowBattRequired){
    enoughBattery = false;
  }
  Serial.print(" Enought batt  ");
  Serial.print(enoughBattery);
  Serial.print("   Low counts");
  Serial.print(lowBattCounts);

}

void processMastTilt(int dpadValue) {
  if (dpadValue == 1) {
    if (servoDelay == 4) {
      if (mastTiltValue >= 47 && mastTiltValue < 114) {
        //if using a ps3 controller that was flashed as an xbox360 controller change the value "1 " below to a "3" or "4" to make up for the slower movement.
        mastTiltValue = mastTiltValue + 1;
        if( enoughBattery){
          mastTiltServo.write(mastTiltValue);
        }
      }
      servoDelay = 0;
    }
    servoDelay++;
  } else if (dpadValue == 2) {
    if (servoDelay == 4) {
      if (mastTiltValue <= 114 && mastTiltValue > 47) {
        //if using a ps3 controller that was flashed as an xbox360 controller change the value "1" below to a "3" or "4" to make up for the slower movement.
        mastTiltValue = mastTiltValue - 1;
        if (enoughBattery){
          mastTiltServo.write(mastTiltValue);
        }
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}
void processAux(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(auxAttach0, HIGH);
      digitalWrite(auxAttach1, LOW);
      lightsOn = true;
    }
    
    lightSwitchTime = millis();
  }
}
void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (! enoughBattery){
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  } else if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  // analogReadResolution(12); //12 bits
  // analogSetAttenuation(ADC_0db);  //For all pins
  // analogSetPinAttenuation(A18, ADC_0db); //0db attenuation on pin A18
  // analogSetPinAttenuation(A19, ADC_2_5db); //2.5db attenuation on pin A19
  // analogSetPinAttenuation(A6, ADC_6db); //6db attenuation on pin A6
  // analogSetPinAttenuation(A7, ADC_11db); //11db attenuation on pin A7
 // Settings for ADC
  // analogSetPinAttenuation(cell_1_2 , ADC_0db);

  pinMode(mastMotor0, OUTPUT);
  pinMode(mastMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  // pinMode(cell_1, INPUT);
  pinMode(cell_1_2, INPUT);
  // analogSetWidth(11);               // 11Bit resolution
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


  steeringServo.attach(steeringServoPin);
  steeringServo.write(adjustedSteeringValue);
  mastTiltServo.attach(mastTiltServoPin);
  mastTiltServo.write(mastTiltValue);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
    else { vTaskDelay(1); }

  // Serial.println("\nMast ");
  // Serial.print(mastTiltValue);
  // Serial.print("         Steering ");
  // Serial.print(adjustedSteeringValue);

  // float batt_val = analogRead(cell_1_2);

  // Serial.print("         Batt ");
  // Serial.print(batt_val);

  calcBattVoltage();
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
}
