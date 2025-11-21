/*
 * Helper function definitions
 */

#ifndef HELPER
#define HELPER

#include "data.h"
#include "config.h"

void eventLog(byte side, byte type, byte state, unsigned long t)
{
  /*
  Optimized serial print- encoded sensor/actuator identifier with event time
  <byte> side : side identifier
  <byte> type : sensor/actuator identifier
  <byte> state : sensor/actuator state identifier
  <unsigned long> t : event time
  */
  Serial.print(side);
  Serial.print(type);
  Serial.print(state);
  Serial.println(t);
}

unsigned long currentTime(unsigned long tLast = 0, unsigned long tolerance = CLOCK_TOLERANCE, bool time_in_microseconds = false)
{
  /*
  Get current time in millis() by default
  <bool> time_in_microseconds : set true to get time in micros()

  CAUTION: milli() can go up to ~49days before overflow where as
  micros() encounters overflow in ~70min
  */
  unsigned long tNow = time_in_microseconds ? micros() : millis();
  while (tNow - tLast > tolerance || tNow < tLast)
  {
    tNow = time_in_microseconds ? micros() : millis();
  }
  return tNow;
}

inline bool digitalReadCorrected(byte pin, bool sensorLogicLow = false)
{
  /*
  Get logic corrected digital read for IR input pin
  <byte> pin : digital pin ID
  <bool> sensorLogicLow : true if sensor return low for desired readout
  returns : <bool> corrected logic response based on sensor output, not of actual read if sensorLogicLow is true
  */
  bool v = digitalRead(pin);
  return sensorLogicLow ? !v : v;
}

void digitalWriteCorrected(byte pin, bool state, bool activeLogicLow = false)
{
  /*
  Write logic corrected value to digital pin
  <byte> pin : digital pin to write
  <bool> state : HIGH or LOW, or true or false, or 1 or 0
  */
  digital.Write(pin, activeLogicLow ? !state : state);
}

void initRuntime(RuntimeState &runtimeState, byte pin, unsigned long duration = RUN_TIME_DURATION, unsigned long delay = DELAY_START)
{
  /*
  Initialize default state variable for runtime
  <struct RuntimeState> runtimeState : runtime struct variable
  <byte> pin : led indicator pin for runtime - HIGH when on, LOW when off
  <unsigned long> duration : set total duration for runtime execution, defaults to RUN_TIME_DURATION
  */
  runtimeState.led_pin  = pin;
  runtimeState.runTimeFlag = false;
  runtimeState.duration = duration;
  runtimeState.delay = delay;
  digitalWrite(runtimeState.led_pin, OFF);
  runtime.tStart = currentTime();
}

void updateRuntime(RuntimeState &runtimeState)
{
  /*
  Poll for current time and check for start or exit conditions for runtime
  <struct RuntimeState> runtimeState : runtime struct variable
  */
  runtimeState.tNow = currentTime(runtimeState.tLast);
  
  //exit condition
  if (runtimeState.tNow - runtimeState.tRuntimeStart >= runtimeState.duration)
  {
    digitalWrite(runtimeState.led_pin, OFF);
    digitalWriteCorrected(SOLENOID_A_PIN, OFF, SOLENOID_ACTIVE_HIGH);
    digitalWriteCorrected(SOLENOID_B_PIN, OFF, SOLENOID_ACTIVE_HIGH);
    runtimeState.runTimeFlag = false;

    // log
    Serial.print('E');
    Serial.println(runtimeState.tNow);

    while (true);
  }
  
  //start condition
  if (!runtimeState.runTimeFlag && runtimeState.tNow - runtimeState.tStart >= DELAY_START)
  {
    runtimeState.runTimeFlag = 1;
    digitalWrite(runtimeState.led_pin, ON);
    runtimeState.tRuntimeStart = runtimeState.tNow;

    // log
    Serial.print('S');
    Serial.println(runtimeState.tRuntimeStart);
  }
  runtimeState.tLast = runtimeState.tNow;
}

void initBlinkLED(BlinkLEDState &ledState, byte pin, byte side, unsigned long blinkInterval = LED_BLINK_INTERVAL)
{
  /*
  Initialize default parameters for blinking LED
  <struct BlinkLEDState> ledState : struct for led state parameters
  <byte> side : led location/side identifier
  <byte> pin : led pin
  <unsigned long> blinkInterval : blink duration
  */
  ledState.pin = pin;
  ledState.side = side;
  ledState.ledBlinkState = false; 
  ledState.blinkInterval = blinkInterval;
}

void updateBlinkLED(BlinkLEDState &ledState, unsigned long tNow)
{
  /*
  update led blink between on and off
  <struct BlinkLEDState> ledState : struct for led state parameters
  <unsigned long> tNow : current time
  */
  if (ledState.ledBlinkState && tNow - ledState.tLEDon> ledState.blinkInterval)
  {
    digitalWrite(ledState.pin, OFF);
    ledState.tLEDoff = tNow;
    ledState.ledBlinkState = false;
  }
  if (!ledState.ledBlinkState && tNow - ledState.tLEDon > ledState.blinkInterval)
  {
    digitalWrite(ledState.pin, ON);
    ledState.tLEDon = tNow;
    ledState.ledBlinkState = true;
  }

}

void initIR(IRState &irDetector, byte pin, byte side)
{
  /*
  Init function to initialize irDetector with default parameters
  <IRState> irDetector : struct storing irDetector state parameters
  <byte> pin : input pin ID connected to ir sensor
  <byte> side : side identifier
  */
  pinMode(pin, INPUT_PULLUP);
  irDetector.pin = pin;
  irDetector.side = side;
  irDetector.currentRead = readCorrected(pin, IR_ACTIVE_LOW);
  irDetector.lastRead = false;
  irDetector.currentPersistant = false;
  irDetector.lastPersistant = false;
  irDetector.inBreak = false;
  irDetector.breakEvent = false;
  irDetector.breakEventMutable = false;
  irDetector.connectEvent = true;
}

void detectIR(IRState &irDetector, unsigned long tNow)
{
  /*
  Function to detect irDetector state changes and update state parameters accordingly
    IR state change event is recorded following persistance in signal
    Additionally since alternating high-low sig was detected when IR emitter and detector
    were placed inside a circular housing within the behavior setup, || login between current and last read is implemented

  <IRState> irDetector : struct storing irDetector state parameters
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  bool v = readCorrected(irDetector.pin, IR_ACTIVE_LOW);
  if ((irDetector.currentRead || irDetector.lastRead) && v && !irDetector.inBreak)
  {
    irDetector.tStart = tNow;
    irDetector.inBreak = true;
  }
  else if (!(irDetector.currentRead || irDetector.lastRead) && !v && irDetector.inBreak)
  {
    irDetector.tOff = tNow;
    irDetector.inBreak = false;
  }
  if (tNow - irDetector.tOff >= MIN_IR_BREAK && !irDetector.inBreak && irDetector.breakEvent)
  {
    irDetector.inBreak = false;
    irDetector.breakEvent = false;
    irDetector.breakEventMutable = false;
    irDetector.connectEvent = true;
    // log
    eventLog(irDetector.side, IR, OFF, tNow);
  }
  if (tNow - irDetector.tStart >= MIN_IR_BREAK && irDetector.inBreak && irDetector.connectEvent)
  {
    irDetector.breakEvent = true;
    irDetector.breakEventMutable = true;
    irDetector.connectEvent = false;
    // log
    eventLog(irDetector.side, IR, ON, tNow);
  }
  if (irDetector.breakEventMutable)
  {
    irDetector.lastPersistant = irDetector.currentPersistant;
    irDetector.currentPersistant = true;
    irDetector.breakEventMutable = false;
  }
  irDetector.lastRead = irDetector.currentRead;
  irDetector.currentRead = v;
}

void initTouch(TouchState &touchSensor, byte pin, byte side)
{
  /*
  Init function to initialize touchSensor with default parameters
  <TouchState> touchSensor : struct storing touchSensor state parameters
  <byte> pin : input pin ID connected to touch sensor
  <byte> side : side identifier
  */
  pinMode(pin, INPUT_PULLUP);
  touchSensor.pin = pin;
  touchSensor.side = side;
  touchSensor.current = digitalReadCorrected(touchSensor.pin, TOUCH_ACTIVE_LOW);
  touchSensor.last = false;
  touchSensor.inTouch = false;
  touchSensor.touchEvent = false;
  touchSensor.clearEvent = true;
}

void detectTouch(TouchState &touchSensor, unsigned long tNow)
{
  /*
  Function to detect touchSensor state changes and update state parameters accordingly
  <TouchState> touchSensor : struct storing irDetector state parameters
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  bool v = digitalReadCorrected(touchSensor.pin, TOUCH_ACTIVE_LOW);

  if (v && !touchSensor.last)
  {
    // add state change as and when needed for duration dependent reward release
    touchSensor.tStart = tNow;
    touchSensor.inTouch = true;
    touchSensor.touchEvent = true;
    touchSensor.clearEvent = false;
    // log
    eventLog(touchSensor.side, TOUCH, ON, tNow);
  }
  else if (!v && touchSensor.last)
  {
    touchSensor.inTouch = false;
    touchSensor.clearEvent = true;
    touchSensor.touchEvent = false;
    // log
    eventLog(touchSensor.side, TOUCH, OFF, tNow);
  }
  touchSensor.last = v;
}

void initSolenoid(SolenoidState &solenoidValve, byte pin, byte side)
{
  /*
  Init function to initialize solenoidState with default parameters
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <byte> pin : input pin ID connected to touch sensor
  <byte> side : side identifier
  */
  pinMode(pin, OUTPUT);
  digitalWriteCorrected(pin, OFF, SOLENOID_ACTIVE_HIGH);
  solenoidValve.open = false;
}

void activateSolenoid(SolenoidState &solenoidValve, unsigned long tNow, unsigned long duration = SOLENOID_DURATION)
{
  /*
  Function to activate solenoid valve and update state parameters accordingly
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <unsigned long> duration : duration to keep the solenoid valve open/close depending on type of solenoid
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  if (solenoidValve.open)
  {
    return;
  }
  if (!solenoidValve.open)
  {
    solenoidValve.open = true;
    solenoidValve.tOpen = tNow;
    solenoidValve.duration = duration;
    digitalWriteCorrected(solenoidValve.pin, ON, SOLENOID_ACTIVE_HIGH);

    // log
    eventLog(solenoidValve.side, SOLENOID, ON, tNow);
  }
}

void updateSolenoid(SolenoidState &solenoidValve, unsigned long tNow)
{
  /*
  Function to check for duration elapsed since solenoid valve activation and update state parameters accordingly
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  if (solenoidValve.open && tNow - solenoidValve.tOpen >= solenoidValve.duration)
  {
    solenoidValve.open = false;
    solenoidValve.tClose = tNow;
    digitalWriteCorrected(solenoidValve.pin, OFF, SOLENOID_ACTIVE_HIGH);

    // log
    eventLog(solenoidValve.side, SOLENOID, OFF, tNow);
  }
}

#endif