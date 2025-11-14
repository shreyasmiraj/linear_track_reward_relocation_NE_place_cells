#include "config.h"
#include "data.h"
#include "helper.h"


bool runTimeFlag;
bool ledBlink;

unsigned long tNow;
unsigned long tStart;
unsigned long tRuntimeStart;
unsigned long tLedBlinkOn;

IRState irDetectorA, irDetectorB;
TouchState touchSensorA, touchSensorB;
SolenoidState solenoidValveA, solenoidValveB;

void setup()
{
  Serial.begin(BAUD_RATE);
  delay(1001); // to allow serial to sync

  ledBlink = false;
  runTimeFlag = false;
  initIR(&irDetectorA, IR_A_PIN, SIDE_A);
  initIR(&irDetectorB, IR_B_PIN, SIDE_B);
  initTouch(&touchSensorA, TOUCH_A_PIN, SIDE_A);
  initTouch(&touchSensorB, TOUCH_B_PIN, SIDE_B);
  initSolenoid(&solenoidValveA, SOLENOID_A_PIN, SIDE_A);
  initSolenoid(&solenoidValveB, SOLENOID_B_PIN, SIDE_B);
  digitalWrite(LED_RUNTIME, LOW);

  // log
  Serial.print("Linear Track Behaviour in mode: ");
  OPERATION_MODE ? Serial.println("Mode_B") : Serial.println("Mode_A");

  tStart = currentTime();
}

void loop()
{
  tNow = currentTime();
  if (tNow - tRuntimeStart >= RUN_TIME)
  {
    digitalWrite(LED_RUNTIME, LOW);
    runTimeFlag = 0;

    // log
    Serial.print('E');
    Serial.println(tNow);

    while (1)
      ;
  }

  if (!runTimeFlag && tNow - tStart >= DELAY_START)
  {
    runTimeFlag = 1;
    digitalWrite(LED_RUNTIME, HIGH);
    tRuntimeStart = tNow;
    digitalWrite(LED_BLINK_PIN, HIGH);
    tLedBlinkOn = tNow;
    ledBlink = true;

    // log
    Serial.print('S');
    Serial.println(tRuntimeStart);
  }

  if (runTimeFlag)
  {

    detectIR(&irDetectorA, IR_A_PIN, SIDE_A);
    detectIR(&irDetectorB, IR_B_PIN, SIDE_B);
    detectTouch(&touchSensorA, TOUCH_A_PIN, SIDE_A);
    detectTouch(&touchSensorB, TOUCH_B_PIN, SIDE_B);
    if (ledBlink && tNow - tLedBlinkOn > LED_BLINK_A_INTERVAL)
    {
      digitalWrite(LED_BLINK_PIN, LOW);
      tLedBlinkOn = tNow;
      ledBlink = false;
    }
    if (!ledBlink && tNow - tLedBlinkOn > LED_BLINK_A_INTERVAL)
    {
      digitalWrite(LED_BLINK_PIN, HIGH);
      tLedBlinkOn = tNow;
      ledBlink = true;
    }

    if (irDetectorA.breakEventMutable)
    {
      irDetectorB.currentPersistant = false;
      irDetectorA.breakEventMutable = false;
    }
    if (irDetectorB.breakEventMutable)
    {
      irDetectorA.currentPersistant = false;
      irDetectorB.breakEventMutable = false;
    }

    if (OPERATION_MODE == MODE_A && irDetectorA.currentPersistant && irDetectorB.lastPersistant)
    {
      activateSolenoid(&solenoidValveA, SOL_DEFAULT, tNow, tRuntimeStart);
      // activateSolenoid(solB, SOL_B_PIN, SOL_DEFAULT_MS, SIDE_B);//ensure the reservoir inlet valve is closed
      updateSolenoid(&solenoidValveA, tNow, tRuntimeStart);
      updateSolenoid(&solenoidValveB, tNow, tRuntimeStart);
      irDetectorB.lastPersistant = false;
      return;
    }
    if (OPERATION_MODE == MODE_B && irDetectorB.currentPersistant && irDetectorA.lastPersistant)
    {

      activateSolenoid(&solenoidValveB, SOL_DEFAULT, tNow, tRuntimeStart);
      // activateSolenoid(solA, SOL_A_PIN, SOL_DEFAULT_MS, SIDE_A);//ensure the reservoir inlet valve is closed
      updateSolenoid(&solenoidValveA, tNow, tRuntimeStart);
      updateSolenoid(&solenoidValveB, tNow, tRuntimeStart);
      irDetectorA.lastPersistant = false;
      return;
    }

    updateSolenoid(&solenoidValveA, tNow, tRuntimeStart);
    updateSolenoid(&solenoidValveB, tNow, tRuntimeStart);
  }
}