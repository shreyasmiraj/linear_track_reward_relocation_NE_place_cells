#include "config.h"
#include "data.h"
#include "helper.h"

int lastIR;
unsigned long lastTTL = 0;

TTLState inputTrigger, outputTrigger, outputIR, outputTouch, outputSolenoid;
RuntimeState runtime;
BlinkLEDState ledA;
IRState irDetectorA, irDetectorB;
TouchState touchSensorA, touchSensorB;
SolenoidState solenoidValveA, solenoidValveB;

void setup()
{
  lastIR = -1;
  Serial.begin(BAUD_RATE);
  delay(1001); // to allow serial conenction to be established
  initTTL(inputTrigger, INPUT_TRIGGER, INPUT);
  initTTL(outputTrigger, OUTPUT_TRIGGER, OUTPUT);
  initTTL(outputIR, OUTPUT_IR, OUTPUT);
  initTTL(outputTouch, OUTPUT_TOUCH, OUTPUT);
  initTTL(outputSolenoid, OUTPUT_SOLENOID, OUTPUT);
  initRuntime(runtime, LED_RUNTIME, &outputTrigger, &inputTrigger);
  initBlinkLED(ledA, LED_BLINK_PIN, SIDE_A);
  initIR(irDetectorA, IR_A_PIN, SIDE_A, IR_A_INDICATOR, &outputIR, TTL_PULSE_PERIOD);
  initIR(irDetectorB, IR_B_PIN, SIDE_B, IR_B_INDICATOR, &outputIR, TTL_PULSE_PERIOD / 2);
  initTouch(touchSensorA, TOUCH_A_PIN, SIDE_A, &outputTouch, TTL_PULSE_PERIOD);
  initTouch(touchSensorB, TOUCH_B_PIN, SIDE_B, &outputTouch, TTL_PULSE_PERIOD / 2);
  initSolenoid(solenoidValveA, SOLENOID_A_PIN, SIDE_A, &outputSolenoid, TTL_PULSE_PERIOD);
  initSolenoid(solenoidValveB, SOLENOID_B_PIN, SIDE_B, &outputSolenoid, TTL_PULSE_PERIOD / 2);
  // log
  Serial.print("Linear Track Behaviour in mode: ");
  OPERATION_MODE ? Serial.println("Mode_B") : Serial.println("Mode_A");
}

void loop()
{ 
  
  updateRuntime(runtime); //inputTrigger detectTTL is interlocked with updateRuntime due to its interdependency
                          //inputTrigger detect state is stored in inputTrigger.detect as boolean.
  updateTTL(outputTrigger, runtime.tNow);
  updateTTL(outputIR, runtime.tNow);
  updateTTL(outputTouch, runtime.tNow);
  updateTTL(outputSolenoid, runtime.tNow);
  if (runtime.runtimeFlag)
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
        if (irDetectorA.currentPersistant && (lastIR == SIDE_B))
        {
          activateSolenoid(solenoidValveA, runtime.tNow);
          // activateSolenoid(solenoidValveB, runtime.tNow);//ensure the reservoir inlet valve is closed
        }
        break;
      case MODE_B:
        if (irDetectorB.currentPersistant && (lastIR == SIDE_A))
        {
          activateSolenoid(solenoidValveB, runtime.tNow);
          activateSolenoid(solenoidValveA, runtime.tNow);//ensure the reservoir inlet valve is closed
        }
      default:
        Serial.println("Operation Mode configuration incorrect/incomplete");
        break;
    }
    if (irDetectorA.breakEventMutable) 
    {
      irDetectorA.breakEventMutable = false;
      lastIR = SIDE_A;
    }
    else if (irDetectorB.breakEventMutable)
    {
      irDetectorB.breakEventMutable = false;
      lastIR = SIDE_B;
    }
  }
}