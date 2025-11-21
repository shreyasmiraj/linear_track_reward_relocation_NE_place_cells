#include "config.h"
#include "data.h"
#include "helper.h"

RuntimeState runtime;
BlinkLEDState ledA;
IRState irDetectorA, irDetectorB;
TouchState touchSensorA, touchSensorB;
SolenoidState solenoidValveA, solenoidValveB;

void setup()
{
  Serial.begin(BAUD_RATE);
  delay(1001); // to allow serial conenction to be established
  
  initRuntime(runtime, LED_RUNTIME);
  initBlinkLED(ledA, LED_BLINK_PIN, SIDE_A);
  initIR(irDetectorA, IR_A_PIN, SIDE_A);
  initIR(irDetectorB, IR_B_PIN, SIDE_B);
  initTouch(touchSensorA, TOUCH_A_PIN, SIDE_A);
  initTouch(touchSensorB, TOUCH_B_PIN, SIDE_B);
  initSolenoid(solenoidValveA, SOLENOID_A_PIN, SIDE_A);
  initSolenoid(solenoidValveB, SOLENOID_B_PIN, SIDE_B);
  initRuntime(runtime, LED_RUNTIME);

  // log
  Serial.print("Linear Track Behaviour in mode: ");
  OPERATION_MODE ? Serial.println("Mode_B") : Serial.println("Mode_A");
}

void loop()
{
  updateRuntime(runtime);
  
  if (runtime.runTimeFlag)
  {
    detectIR(irDetectorA, runtime.tNow);
    detectIR(irDetectorB, runtime.tNow);
    detectTouch(touchSensorA, runtime.tNow);
    detectTouch(touchSensorB, runtime.tNow);

    updateBlinkLED(ledA, runtime.tNow);
    updateSolenoid(solenoidValveA, runtime.tNow);
    updateSolenoid(solenoidValveB, runtime.tNow);

    switch (OPERATION_MODE)
    {
      case MODE_A:
        if (irDetectorA.currentPersistant && irDetectorB.lastPersistant)
        {
          activateSolenoid(solenoidValveA, runtime.tNow);
          activateSolenoid(solenoidValveB, runtime.tNow);//ensure the reservoir inlet valve is closed
          irDetectorB.lastPersistant = false;
          irDetectorB.currentPersistant = false;
        }
        break;
      case MODE_B:
        if (irDetectorB.currentPersistant && irDetectorA.lastPersistant)
        {
          activateSolenoid(solenoidValveB, runtime.tNow);
          activateSolenoid(solenoidValveA, runtime.tNow);//ensure the reservoir inlet valve is closed
          irDetectorA.lastPersistant = false;
          irDetectorA.currentPersistant = false;
        }
      default:
        Serial.println("Operation Mode configuration incorrect/incomplete");
        break;
    }
  }
  runtime.tLast = runtime.tNow;
}