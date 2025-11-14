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

unsigned long currentTime(bool time_in_microseconds = false)
{
  /*
  Get current time in millis() by default
  <bool> time_in_microseconds : set true to get time in micros()

  CAUTION: milli() can go up to 50days before overflow where as
  micros() encounters overflow in ~70min
  */
  return TIME_IN_MICROSECONDS ? micros() : millis();
}

inline bool readIR(byte pin)
{
  /*
  Get logic corrected digital read for IR input pin
  */
  bool v = digitalRead(pin);
  return IR_ACTIVE_LOW ? !v : v;
}

void initIR(IRState *irDetector, byte pin, byte side)
{
  /*
  Init function to initialize irDetector with default parameters
  <IRState> irDetector : struct storing irDetector state parameters
  <byte> pin : input pin ID connected to ir sensor
  <byte> side : side identifier
  */
  pinMode(pin, INPUT_PULLUP);
  irDetector->pin = pin;
  irDetector->side = side;
  irDetector->currentRead = readIR(pin);
  irDetector->lastRead = false;
  irDetector->currentPersistant = false;
  irDetector->lastPersistant = false;
  irDetector->inBreak = false;
  irDetector->breakEvent = false;
  irDetector->breakEventMutable = false;
  irDetector->connectEvent = true;
}

void detectIR(IRState *irDetector, unsigned long tNow, unsigned long tRuntimeStart)
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
  bool v = readIR(irDetector->pin);
  if ((irDetector->currentRead || irDetector->lastRead) && v && !irDetector->inBreak)
  {
    irDetector->tStart = tNow;
    irDetector->inBreak = true;
  }
  else if (!(irDetector->currentRead || irDetector->lastRead) && !v && irDetector->inBreak)
  {
    irDetector->tOff = tNow;
    irDetector->inBreak = false;
  }
  if (tNow - irDetector->tOff >= MIN_IR_BREAK && !irDetector->inBreak && irDetector->breakEvent)
  {
    irDetector->inBreak = false;
    irDetector->breakEvent = false;
    irDetector->breakEventMutable = false;
    irDetector->connectEvent = true;
    // log
    eventLog(irDetector->side, IR, OFF, tNow - tRuntimeStart);
  }
  if (tNow - irDetector->tStart >= MIN_IR_BREAK && irDetector->inBreak && irDetector->connectEvent)
  {
    irDetector->breakEvent = true;
    irDetector->breakEventMutable = true;
    irDetector->connectEvent = false;
    // log
    eventLog(irDetector->side, IR, ON, tNow - tRuntimeStart);
  }
  if (irDetector->breakEventMutable)
  {
    irDetector->lastPersistant = irDetector->currentPersistant;
    irDetector->currentPersistant = true;
  }
  irDetector->lastRead = irDetector->currentRead;
  irDetector->currentRead = v;
}

inline bool readTouch(byte pin)
{
  /*
  Get logic corrected digital read for touch input pin
  */
  bool v = digitalRead(pin);
  return TOUCH_ACTIVE_LOW ? !v : v;
}

void initTouch(TouchState *touchSensor, byte pin, byte side)
{
  /*
  Init function to initialize touchSensor with default parameters
  <TouchState> touchSensor : struct storing touchSensor state parameters
  <byte> pin : input pin ID connected to touch sensor
  <byte> side : side identifier
  */
  pinMode(pin, INPUT_PULLUP);
  touchSensor->pin = pin;
  touchSensor->side = side;
  touchSensor->current = readTouch(touchSensor->pin);
  touchSensor->last = false;
  touchSensor->inTouch = false;
  touchSensor->touchEvent = false;
  touchSensor->clearEvent = true;
}

void detectTouch(TouchState *touchSensor, unsigned long tNow, unsigned long tRuntimeStart)
{
  /*
  Function to detect touchSensor state changes and update state parameters accordingly
  <TouchState> touchSensor : struct storing irDetector state parameters
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  bool v = readTouch(touchSensor->pin);

  if (v && !touchSensor->last)
  {
    // add state change as and when needed for duration dependent reward release
    touchSensor->tStart = tNow;
    touchSensor->inTouch = true;
    touchSensor->touchEvent = true;
    touchSensor->clearEvent = false;
    // log
    eventLog(touchSensor->side, TOUCH, ON, tNow - tRuntimeStart);
  }
  else if (!v && touchSensor->last)
  {
    touchSensor->inTouch = false;
    touchSensor->clearEvent = true;
    touchSensor->touchEvent = false;
    // log
    eventLog(touchSensor->side, TOUCH, OFF, tNow - tRuntimeStart);
  }
  touchSensor->last = v;
}

void initSolenoid(SolenoidState *solenoidValve, byte pin, byte side)
{
  /*
  Init function to initialize solenoidState with default parameters
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <byte> pin : input pin ID connected to touch sensor
  <byte> side : side identifier
  */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, SOLENOID_ACTIVE_HIGH ? LOW : HIGH);
  solenoidValve->open = false;
}

void activateSolenoid(SolenoidState *solenoidValve, unsigned long duration, unsigned long tNow, unsigned long tRuntimeStart)
{
  /*
  Function to activate solenoid valve and update state parameters accordingly
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <unsigned long> duration : duration to keep the solenoid valve open/close depending on type of solenoid
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  if (solenoidValve->open)
  {
    return;
  }
  if (!solenoidValve->open)
  {
    solenoidValve->open = true;
    solenoidValve->tOpen = tNow;
    solenoidValve->duration = duration;
    digitalWrite(solenoidValve->pin, SOLENOID_ACTIVE_HIGH ? HIGH : LOW);

    // log
    eventLog(solenoidValve->side, SOLENOID, ON, tNow - tRuntimeStart);
  }
}

void updateSolenoid(SolenoidState *solenoidValve, unsigned long tNow, unsigned long tRuntimeStart)
{
  /*
  Function to check for duration elapsed since solenoid valve activation and update state parameters accordingly
  <SolenoidState> solenoidValve : struct storing solenoid valve state parameters
  <unsigned long> tNow : current time of execution
  <unsigned long> tRuntimeStart : time of runtime start
  */
  if (solenoidValve->open && tNow - solenoidValve->tOpen >= solenoidValve->duration)
  {
    solenoidValve->open = false;
    solenoidValve->tClose = tNow;
    digitalWrite(solenoidValve->pin, SOLENOID_ACTIVE_HIGH ? LOW : HIGH);

    // log
    eventLog(solenoidValve->side, SOLENOID, OFF, tNow - tRuntimeStart);
  }
}

#endif