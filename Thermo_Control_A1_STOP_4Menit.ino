#include <Wire.h>
#include <max6675.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

// RelayPin 1 - ATAS
// RelayPin 2 - BAWAH
#define RelayPin1 2
#define RelayPin2 47

int curCount = 0;
int editStep = 0;
int buzzerPin = 5;

// KIPAS A1 - TETAP SEPERTI SEMULA
int fanRelayPin = A1;
unsigned long fanTimerStart = 0;
boolean fanCooling = false;

// KIPAS A2
int fanRelayPin2 = A2;
unsigned long fanTimerStart2 = 0;
boolean fanCooling2 = false;

// Tombol
int pb1 = 3;
int pb2 = 4;
int upSwitchPin = 6;
int downSwitchPin = 7;
int editSwitchPin = 8;
int cancelSwitchPin = 9;
int okSwitchPin = 10;

int upSwitchState = 0;
int downSwitchState = 0;
int leftSwitchState = 0;
int rightSwitchState = 0;
int editSwitchState = 0;
int cancelSwitchState = 0;
int okSwitchState = 0;

// Profile
byte currentProfile = 1;
int currentStep = 1;
byte profileSteps;

int rampRateStep[9];
int dwellTimerStep[9];

int kp1;
int ki1;
int kd1;
int kp2;
int ki2;
int kd2;

int setpointRamp;
double startTemp;

int temperatureStep[9];

int eepromAddress = 0;

long previousMillis;
double counter;

// STATE
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_MENU_STEPS,
  REFLOW_STATE_MENU_BOTTOM_HEAT,
  REFLOW_STATE_MENU_STEP_RAMP,
  REFLOW_STATE_MENU_STEP_TARGET,
  REFLOW_STATE_MENU_STEP_DWELL,
  REFLOW_STATE_MENU_BOTTOM_P,
  REFLOW_STATE_MENU_BOTTOM_I,
  REFLOW_STATE_MENU_BOTTOM_D,
  REFLOW_STATE_MENU_TOP_P,
  REFLOW_STATE_MENU_TOP_I,
  REFLOW_STATE_MENU_TOP_D,
  REFLOW_STATE_STEP_RAMP,
  REFLOW_STATE_STEP,
  REFLOW_STATE_STEP_DWELL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR
}
reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
}
reflowStatus_t;

#define SENSOR_SAMPLING_TIME 1000
#define GRAPHICS_SAMPLING_TIME 500

reflowStatus_t reflowStatus;
reflowState_t reflowState;

unsigned long nextCheck1;
unsigned long nextRead1;
unsigned long nextRead2;

double Setpoint1, Input1, Output1;
PID myPID1(&Input1, &Output1, &Setpoint1, kp1, ki1, kd1, DIRECT);

int WindowSize = 500;
unsigned long windowStartTime;

double Setpoint2, Input2, Output2;
PID myPID2(&Input2, &Output2, &Setpoint2, kp2, ki2, kd2, DIRECT);

boolean alarmOn = false;

unsigned long finalAlarmLastToggle = 0;
boolean finalAlarmBeepOn = false;

unsigned long lastStepAlarmStart = 0;
boolean lastStepAlarmPlayed = false;
boolean lastStepAlarmActive = false;

boolean updateScreen = true;

// Thermocouple atas
int thermoCLK = 13;
int thermoCS = 12;
int thermoDO = 11;

// Thermocouple bawah
int thermoCLK2 = 35;
int thermoCS2 = 34;
int thermoDO2 = 33;

int tc1;
int tc2;

MAX6675 thermocouple1(thermoCLK, thermoCS, thermoDO);
MAX6675 thermocouple2(thermoCLK2, thermoCS2, thermoDO2);

void loadProfile()
{
  profileSteps = EEPROM.read((currentProfile - 1) * 29);
  Setpoint2 = EEPROM.read((currentProfile - 1) * 29 + 1);

  for (int i = 0; i < 9; i + 1) {
    rampRateStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 2);
    i++;
  }

  for (int i = 0; i < 9; i + 1) {
    dwellTimerStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 11) * 5;
    i++;
  }

  for (int i = 0; i < 9; i + 1) {
    temperatureStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 20) * 5;
    i++;
  }

  kp1 = EEPROM.read((currentProfile - 1) * 6 + 122);
  ki1 = EEPROM.read((currentProfile - 1) * 6 + 123);
  kd1 = EEPROM.read((currentProfile - 1) * 6 + 124);
  kp2 = EEPROM.read((currentProfile - 1) * 6 + 125);
  ki2 = EEPROM.read((currentProfile - 1) * 6 + 126);
  kd2 = EEPROM.read((currentProfile - 1) * 6 + 127);

  return;
}

void setup()
{
  Serial.begin(9600);

  pinMode(pb1, INPUT);
  pinMode(pb2, INPUT);

  pinMode(upSwitchPin, INPUT);
  digitalWrite(upSwitchPin, HIGH);

  pinMode(downSwitchPin, INPUT);
  digitalWrite(downSwitchPin, HIGH);

  pinMode(editSwitchPin, INPUT);
  digitalWrite(editSwitchPin, HIGH);

  pinMode(cancelSwitchPin, INPUT);
  digitalWrite(cancelSwitchPin, HIGH);

  pinMode(okSwitchPin, INPUT);
  digitalWrite(okSwitchPin, HIGH);

  pinMode(fanRelayPin, OUTPUT);
  digitalWrite(fanRelayPin, LOW);

  pinMode(fanRelayPin2, OUTPUT);
  digitalWrite(fanRelayPin2, LOW);

  pinMode(buzzerPin, OUTPUT);

  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();

  lcd.print("OGE EVOLUTION");

  // Welcome melody
  tone(buzzerPin, 523);
  delay(200);
  tone(buzzerPin, 659);
  delay(200);
  tone(buzzerPin, 784);
  delay(200);
  tone(buzzerPin, 1046);
  delay(200);
  noTone(buzzerPin);

  delay(2000);
  lcd.clear();

  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);

  windowStartTime = millis();
  nextCheck1 = millis();
  nextRead1 = millis();

  myPID1.SetOutputLimits(0, WindowSize);
  myPID2.SetOutputLimits(0, WindowSize);

  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}

int i = 0;

void loop()
{
  upSwitchState = digitalRead(upSwitchPin);
  downSwitchState = digitalRead(downSwitchPin);
  editSwitchState = digitalRead(editSwitchPin);
  cancelSwitchState = digitalRead(cancelSwitchPin);
  okSwitchState = digitalRead(okSwitchPin);

  if (upSwitchState == HIGH) {
    upSwitchState = LOW;
  } else {
    upSwitchState = HIGH;
  }

  if (downSwitchState == HIGH) {
    downSwitchState = LOW;
  } else {
    downSwitchState = HIGH;
  }

  if (editSwitchState == HIGH) {
    editSwitchState = LOW;
  } else {
    editSwitchState = HIGH;
  }

  if (cancelSwitchState == HIGH) {
    cancelSwitchState = LOW;
  } else {
    cancelSwitchState = HIGH;
  }

  if (okSwitchState == HIGH) {
    okSwitchState = LOW;
  } else {
    okSwitchState = HIGH;
  }

  unsigned long currentMillis = millis();

  int SP1 = Setpoint1;
  int SP2 = Setpoint2;

  if (upSwitchState == HIGH || downSwitchState == HIGH ||
      editSwitchState == HIGH || cancelSwitchState == HIGH ||
      okSwitchState == HIGH) {
    tone(buzzerPin, 523);
    delay(100);
    noTone(buzzerPin);
  }

  if (reflowState == REFLOW_STATE_COMPLETE || alarmOn) {
    if (i < 15 && cancelSwitchState == LOW) {
      alarmOn = true;

      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);

      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);

      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);

      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(400);

      i++;
    }
    else {
      i = 0;
      alarmOn = false;
    }
  }

  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:

      if (millis() > nextRead1)
      {
        nextRead1 += SENSOR_SAMPLING_TIME;

        Input1 = thermocouple1.readCelsius();
        Input2 = thermocouple2.readCelsius();

        tc1 = Input1;
        tc2 = Input2;

        lcd.setCursor(16, 2);
        lcd.print("    ");
        lcd.setCursor(16, 2);

        if (isnan(Input1)) {
          lcd.print("Er");
        } else {
          lcd.print(tc1);
        }

        lcd.setCursor(16, 3);
        lcd.print("    ");
        lcd.setCursor(16, 3);

        if (isnan(Input2)) {
          lcd.print("Er");
        } else {
          lcd.print(tc2);
        }
      }

      if (updateScreen) {
        lcd.clear();

        lcd.setCursor(8, 0);
        lcd.print("OFF");

        lcd.setCursor(1, 1);
        lcd.print("PTN:");
        lcd.print("      STEP:1 ");

        lcd.setCursor(0, 2);
        lcd.print(" TH  SP:");

        lcd.setCursor(13, 2);
        lcd.print("PV:");

        lcd.setCursor(0, 3);
        lcd.print(" BH  SP:");

        lcd.setCursor(13, 3);
        lcd.print("PV:");

        updateScreen = false;
      }

      lcd.setCursor(5, 1);
      lcd.print(currentProfile);
      lcd.print(" ");

      lcd.setCursor(8, 2);
      lcd.print(temperatureStep[0]);
      lcd.print(" ");

      lcd.setCursor(8, 3);
      lcd.print(SP2);
      lcd.print(" ");

      windowStartTime = millis();

      if (upSwitchState == HIGH)
      {
        currentProfile = currentProfile + 1;
        delay(25);

        if (currentProfile >= 5) {
          currentProfile = 1;
        }
      }

      if (downSwitchState == HIGH)
      {
        currentProfile = currentProfile - 1;
        delay(25);

        if (currentProfile <= 0) {
          currentProfile = 4;
        }
      }

      loadProfile();

      if (editSwitchState == HIGH)
      {
        delay(25);
        reflowState = REFLOW_STATE_MENU_STEPS;
        updateScreen = true;
      }

      if (okSwitchState == HIGH)
      {
        delay(250);

        // Kipas A1 tetap seperti semula
        digitalWrite(fanRelayPin, HIGH);
        fanCooling = false;

        // Kipas A2 mati selama pemanasan
        digitalWrite(fanRelayPin2, LOW);
        fanCooling2 = false;

        updateScreen = true;
        currentStep = 1;
        counter = 0;
        setpointRamp = 0;

        previousMillis = millis();

        finalAlarmLastToggle = millis();
        finalAlarmBeepOn = false;

        lastStepAlarmPlayed = false;
        lastStepAlarmActive = false;

        noTone(buzzerPin);

        curCount = 0;
        nextRead2 = millis();

        reflowStatus = REFLOW_STATUS_ON;
        reflowState = REFLOW_STATE_STEP_RAMP;
      }

      break;

    case REFLOW_STATE_MENU_STEPS:

      if (updateScreen) {
        lcd.clear();

        lcd.setCursor(3, 0);
        lcd.print("Profile ");
        lcd.print(currentProfile);
        lcd.print(" Edit");

        lcd.setCursor(0, 1);
        lcd.print("                    ");

        lcd.setCursor(2, 2);
        lcd.print("Profile Steps: ");
        lcd.print(profileSteps);

        updateScreen = false;
      }

      lcd.setCursor(17, 2);
      lcd.print(profileSteps);

      if (upSwitchState == HIGH)
      {
        profileSteps = profileSteps + 1;
        delay(25);

        if (profileSteps >= 10) {
          profileSteps = 1;
        }
      }

      if (downSwitchState == HIGH)
      {
        profileSteps = profileSteps - 1;
        delay(25);

        if (profileSteps <= 0) {
          profileSteps = 9;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_HEAT;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_BOTTOM_HEAT:

      if (updateScreen) {
        lcd.setCursor(2, 2);
        lcd.print("Bottom Heat:      ");
        updateScreen = false;
      }

      lcd.setCursor(14, 2);
      lcd.print(SP2);

      if (upSwitchState == HIGH)
      {
        Setpoint2 = Setpoint2 + 10;
        delay(25);

        if (Setpoint2 >= 0) {
          Setpoint2 = 0;
        }
      }

      if (downSwitchState == HIGH)
      {
        Setpoint2 = Setpoint2 - 10;
        delay(25);

        if (Setpoint2 <= 0) {
          Setpoint2 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_STEP_RAMP;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_STEP_RAMP:

      if (updateScreen) {
        lcd.setCursor(2, 2);
        lcd.print("Step ");
        lcd.print(editStep + 1);
        lcd.print(" Ramp:      ");
        updateScreen = false;
      }

      lcd.setCursor(14, 2);
      lcd.print(rampRateStep[editStep]);

      if (upSwitchState == HIGH)
      {
        rampRateStep[editStep] = rampRateStep[editStep] + 1;
        delay(25);

        if (rampRateStep[editStep] >= 9) {
          rampRateStep[editStep] = 9;
        }
      }

      if (downSwitchState == HIGH)
      {
        rampRateStep[editStep] = rampRateStep[editStep] - 1;
        delay(25);

        if (rampRateStep[editStep] <= 1) {
          rampRateStep[editStep] = 1;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;

        if (editStep + 1 == profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_TARGET;
        }
        else {
          editStep++;
        }
      }

      if (cancelSwitchState == HIGH)
      {
        updateScreen = true;
        editStep = 0;
        delay(25);
        lcd.clear();
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_STEP_TARGET:

      if (updateScreen) {
        lcd.setCursor(2, 2);
        lcd.print("Step ");
        lcd.print(editStep + 1);
        lcd.print(" Target: ");
        lcd.print(temperatureStep[editStep]);
        lcd.print(" ");
        updateScreen = false;
      }

      lcd.print(" ");
      lcd.setCursor(16, 2);
      lcd.print(temperatureStep[editStep]);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        temperatureStep[editStep] = temperatureStep[editStep] + 5;
        delay(25);

        if (temperatureStep[editStep] >= 400) {
          temperatureStep[editStep] = 400;
        }
      }

      if (downSwitchState == HIGH)
      {
        temperatureStep[editStep] = temperatureStep[editStep] - 1;
        delay(25);

        if (temperatureStep[editStep] <= 0) {
          temperatureStep[editStep] = 0;
        }

        if (temperatureStep[editStep] <= 99) {
          lcd.setCursor(18, 2);
          lcd.print(" ");
        }

        if (temperatureStep[editStep] <= 9) {
          lcd.setCursor(17, 2);
          lcd.print(" ");
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;

        if (editStep + 1 == profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_DWELL;
        }
        else {
          editStep++;
        }
      }

      if (cancelSwitchState == HIGH)
      {
        updateScreen = true;
        delay(25);
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_STEP_DWELL:

      if (updateScreen) {
        lcd.setCursor(1, 2);
        lcd.print("Step ");
        lcd.print(editStep + 1);
        lcd.print(" Dwell: ");
        lcd.print(dwellTimerStep[editStep]);
        lcd.print(" ");
      }

      lcd.print(" ");
      lcd.setCursor(15, 2);
      lcd.print(dwellTimerStep[editStep]);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        dwellTimerStep[editStep] = dwellTimerStep[editStep] + 5;
        delay(25);

        if (dwellTimerStep[editStep] >= 1000) {
          dwellTimerStep[editStep] = 1000;
        }
      }

      if (downSwitchState == HIGH)
      {
        dwellTimerStep[editStep] = dwellTimerStep[editStep] - 5;
        delay(25);

        if (dwellTimerStep[editStep] <= 0) {
          dwellTimerStep[editStep] = 0;
        }

        if (dwellTimerStep[editStep] <= 99) {
          lcd.setCursor(18, 2);
          lcd.print(" ");
        }

        if (dwellTimerStep[editStep] <= 9) {
          lcd.setCursor(17, 2);
          lcd.print(" ");
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;

        if (editStep + 1 == profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_BOTTOM_P;
        }
        else {
          editStep++;
        }
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_BOTTOM_P:

      if (updateScreen) {
        lcd.setCursor(0, 2);
        lcd.print("                  ");

        lcd.setCursor(3, 2);
        lcd.print("Bottom Heater:");

        lcd.setCursor(8, 3);
        lcd.print("P=");
        lcd.print(kp2);
        lcd.print(" ");

        updateScreen = false;
      }

      lcd.setCursor(10, 3);
      lcd.print(kp2);
      lcd.print("  ");

      if (upSwitchState == HIGH)
      {
        kp2 = kp2 + 1;
        delay(25);

        if (kp2 >= 500) {
          kp2 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        kp2 = kp2 - 1;
        delay(25);

        if (kp2 <= 0) {
          kp2 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_I;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_BOTTOM_I:

      lcd.setCursor(8, 3);
      lcd.print("I=");
      lcd.print(ki2);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        ki2 = ki2 + 1;
        delay(25);

        if (ki2 >= 500) {
          ki2 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        ki2 = ki2 - 1;
        delay(25);

        if (ki2 <= 0) {
          ki2 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        reflowState = REFLOW_STATE_MENU_BOTTOM_D;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_BOTTOM_D:

      lcd.setCursor(8, 3);
      lcd.print("D=");
      lcd.print(kd2);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        kd2 = kd2 + 1;
        delay(25);

        if (kd2 >= 500) {
          kd2 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        kd2 = kd2 - 1;
        delay(25);

        if (kd2 <= 0) {
          kd2 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        reflowState = REFLOW_STATE_MENU_TOP_P;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_TOP_P:

      if (updateScreen) {
        lcd.setCursor(0, 2);
        lcd.print("                  ");

        lcd.setCursor(5, 2);
        lcd.print("Top Heater:");

        lcd.setCursor(8, 3);
        lcd.print("P=");
        lcd.print(kp1);
        lcd.print(" ");

        updateScreen = false;
      }

      lcd.setCursor(10, 3);
      lcd.print(kp1);
      lcd.print("  ");

      if (upSwitchState == HIGH)
      {
        kp1 = kp1 + 1;
        delay(25);

        if (kp1 >= 500) {
          kp1 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        kp1 = kp1 - 1;
        delay(25);

        if (kp1 <= 0) {
          kp1 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_I;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_TOP_I:

      lcd.setCursor(8, 3);
      lcd.print("I=");
      lcd.print(ki1);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        ki1 = ki1 + 1;
        delay(25);

        if (ki1 >= 500) {
          ki1 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        ki1 = ki1 - 1;
        delay(25);

        if (ki1 <= 0) {
          ki1 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);
        reflowState = REFLOW_STATE_MENU_TOP_D;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_MENU_TOP_D:

      lcd.setCursor(8, 3);
      lcd.print("D=");
      lcd.print(kd1);
      lcd.print(" ");

      if (upSwitchState == HIGH)
      {
        kd1 = kd1 + 1;
        delay(25);

        if (kd1 >= 500) {
          kd1 = 500;
        }
      }

      if (downSwitchState == HIGH)
      {
        kd1 = kd1 - 1;
        delay(25);

        if (kd1 <= 0) {
          kd1 = 0;
        }
      }

      if (okSwitchState == HIGH)
      {
        delay(250);

        // Simpan parameter profile
        EEPROM.write((currentProfile - 1) * 29, profileSteps);
        EEPROM.write((currentProfile - 1) * 29 + 1, Setpoint2);

        for (int i = 0; i < 9; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 2), rampRateStep[i]);
          i++;
        }

        for (int i = 0; i < 9; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 11), dwellTimerStep[i] / 5);
          i++;
        }

        for (int i = 0; i < 9; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 20), temperatureStep[i] / 5);
          i++;
        }

        EEPROM.write(((currentProfile - 1) * 6 + 122), kp1);
        EEPROM.write(((currentProfile - 1) * 6 + 123), ki1);
        EEPROM.write(((currentProfile - 1) * 6 + 124), kd1);
        EEPROM.write(((currentProfile - 1) * 6 + 125), kp2);
        EEPROM.write(((currentProfile - 1) * 6 + 126), ki2);
        EEPROM.write(((currentProfile - 1) * 6 + 127), kd2);

        lcd.clear();
        reflowState = REFLOW_STATE_IDLE;
      }

      if (cancelSwitchState == HIGH)
      {
        delay(25);
        lcd.clear();
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_STEP_RAMP:

      // Alarm saat masuk step terakhir
      if (currentStep == profileSteps && !lastStepAlarmPlayed) {
        lastStepAlarmPlayed = true;

        for (int beep = 0; beep < 5; beep++) {
          tone(buzzerPin, 1046);
          delay(500);
          noTone(buzzerPin);
          delay(500);
        }

        lastStepAlarmActive = false;
      }

      if (updateScreen) {
        lcd.setCursor(8, 0);
        lcd.print(" ON");
        updateScreen = false;
      }

      startTemp = tc1;

      lcd.setCursor(16, 1);
      lcd.print(currentStep);

      lcd.setCursor(8, 3);
      lcd.print(SP2);

      if ((currentMillis - previousMillis) > 1000 * rampRateStep[currentStep - 1]) {

        previousMillis = currentMillis;
        counter = counter + 1;
        setpointRamp = counter + startTemp;

        lcd.setCursor(8, 2);
        lcd.print("   ");

        lcd.setCursor(8, 2);
        lcd.print(setpointRamp);
        lcd.print("  ");

        Setpoint1 = setpointRamp;
      }

      if (setpointRamp >= temperatureStep[currentStep - 1]) {
        lcd.setCursor(8, 2);
        lcd.print(temperatureStep[currentStep - 1]);
        reflowState = REFLOW_STATE_STEP;
      }

      if (cancelSwitchState == HIGH)
      {
        // KIPAS A1 tetap menyala 4 menit setelah STOP
        digitalWrite(fanRelayPin, HIGH);
        fanTimerStart = millis();
        fanCooling = true;

        // TAMBAHAN: KIPAS A2 ON 4 MENIT SAAT STOP
        digitalWrite(fanRelayPin2, HIGH);
        fanTimerStart2 = millis();
        fanCooling2 = true;

        noTone(buzzerPin);
        lastStepAlarmActive = false;
        currentStep = 1;
        counter = 0;
        setpointRamp = 0;
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
        updateScreen = true;
      }

      break;

    case REFLOW_STATE_STEP:

      Setpoint1 = temperatureStep[currentStep - 1];

      if (Input1 >= temperatureStep[currentStep - 1])
      {
        counter = 0;
        reflowState = REFLOW_STATE_STEP_DWELL;
      }

      if (cancelSwitchState == HIGH)
      {
        // KIPAS A1 tetap menyala 4 menit setelah STOP
        digitalWrite(fanRelayPin, HIGH);
        fanTimerStart = millis();
        fanCooling = true;

        // TAMBAHAN: KIPAS A2 ON 4 MENIT SAAT STOP
        digitalWrite(fanRelayPin2, HIGH);
        fanTimerStart2 = millis();
        fanCooling2 = true;

        updateScreen = true;
        currentStep = 1;
        counter = 0;
        setpointRamp = 0;
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_STEP_DWELL:

      if (currentMillis - previousMillis > 1000) {
        previousMillis = currentMillis;
        counter = counter + 1;
      }

      // Alarm 10 detik terakhir pada step terakhir
      if (currentStep >= profileSteps) {

        int alarmStart = dwellTimerStep[currentStep - 1] - 10;

        if (alarmStart < 0) alarmStart = 0;

        if (counter >= alarmStart && counter < dwellTimerStep[currentStep - 1]) {

          if (millis() - finalAlarmLastToggle >= 500) {
            finalAlarmLastToggle = millis();
            finalAlarmBeepOn = !finalAlarmBeepOn;

            if (finalAlarmBeepOn) tone(buzzerPin, 1046);
            else noTone(buzzerPin);
          }
        }
        else {
          noTone(buzzerPin);
          finalAlarmBeepOn = false;
        }
      }
      else {
        noTone(buzzerPin);
        finalAlarmBeepOn = false;
      }

      // Selesaikan dwell step
      if (counter >= dwellTimerStep[currentStep - 1]) {

        noTone(buzzerPin);
        finalAlarmBeepOn = false;

        counter = 0;
        setpointRamp = 0;

        if (currentStep >= profileSteps) {
          reflowStatus = REFLOW_STATUS_OFF;
          reflowState = REFLOW_STATE_COMPLETE;
        }
        else {
          currentStep++;
          reflowState = REFLOW_STATE_STEP_RAMP;
        }
      }

      if (cancelSwitchState == HIGH)
      {
        // KIPAS A1 tetap menyala 4 menit setelah STOP
        digitalWrite(fanRelayPin, HIGH);
        fanTimerStart = millis();
        fanCooling = true;

        // TAMBAHAN: KIPAS A2 ON 4 MENIT SAAT STOP
        digitalWrite(fanRelayPin2, HIGH);
        fanTimerStart2 = millis();
        fanCooling2 = true;

        noTone(buzzerPin);
        finalAlarmBeepOn = false;
        updateScreen = true;
        currentStep = 1;
        counter = 0;
        setpointRamp = 0;
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    case REFLOW_STATE_COMPLETE:

      noTone(buzzerPin);
      finalAlarmBeepOn = false;

      reflowStatus = REFLOW_STATUS_OFF;

      digitalWrite(RelayPin1, LOW);
      digitalWrite(RelayPin2, LOW);

      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("COMPLETE");

      // Kipas A1 tetap seperti semula
      digitalWrite(fanRelayPin, HIGH);
      fanTimerStart = millis();
      fanCooling = true;

      // Kipas A2 ON 4 menit setelah COMPLETE
      digitalWrite(fanRelayPin2, HIGH);
      fanTimerStart2 = millis();
      fanCooling2 = true;

      delay(2000);

      currentStep = 1;
      counter = 0;
      setpointRamp = 0;

      reflowState = REFLOW_STATE_IDLE;
      updateScreen = true;

      break;
  }

  if (reflowStatus == REFLOW_STATUS_ON)
  {
    if (millis() > nextRead2) {

      Input1 = thermocouple1.readCelsius();
      Input2 = thermocouple2.readCelsius();

      tc1 = Input1;
      tc2 = Input2;

      nextRead2 += GRAPHICS_SAMPLING_TIME;

      if (curCount <= 6) {
        lcd.setCursor(curCount, 0);
        lcd.print(" *");

        lcd.setCursor(18 - curCount, 0);
        lcd.print("* ");
      }

      if (curCount > 6) {
        lcd.setCursor(13 - curCount, 0);
        lcd.print("* ");

        lcd.setCursor(curCount + 5, 0);
        lcd.print(" *");
      }

      curCount++;

      if (curCount > 13) curCount = 0;
    }

    if (millis() > nextRead1)
    {
      nextRead1 += SENSOR_SAMPLING_TIME;

      lcd.setCursor(16, 2);
      lcd.print("    ");

      lcd.setCursor(16, 2);

      if (isnan(Input1)) {
        lcd.print("Er");
      } else {
        lcd.print(tc1);
      }

      lcd.setCursor(16, 3);
      lcd.print("    ");

      lcd.setCursor(16, 3);

      if (isnan(Input2)) {
        lcd.print("Er");
      } else {
        lcd.print(tc2);
      }
    }

    myPID1.SetTunings(kp1, ki1, kd1);
    myPID2.SetTunings(kp2, ki2, kd2);

    myPID1.Compute();
    myPID2.Compute();

    if (millis() - windowStartTime > WindowSize) {
      windowStartTime += WindowSize;
    }

    if (Output1 > millis() - windowStartTime)
      digitalWrite(RelayPin1, HIGH);
    else
      digitalWrite(RelayPin1, LOW);

    if (Output2 > millis() - windowStartTime)
      digitalWrite(RelayPin2, HIGH);
    else
      digitalWrite(RelayPin2, LOW);
  }
  else
  {
    digitalWrite(RelayPin1, LOW);
    digitalWrite(RelayPin2, LOW);
  }

  // Matikan kipas A1 setelah 4 menit
  if (fanCooling && millis() - fanTimerStart >= 240000UL) {
    digitalWrite(fanRelayPin, LOW);
    fanCooling = false;
  }

  // Matikan kipas A2 setelah 4 menit
  if (fanCooling2 && millis() - fanTimerStart2 >= 240000UL) {
    digitalWrite(fanRelayPin2, LOW);
    fanCooling2 = false;
  }
}
