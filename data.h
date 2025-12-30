/*
 * Data structures for sensor and actuator state parameters;
 */

#ifndef DATA
#define DATA

// enumerated operation modes
enum Mode
{
	MODE_A,
	MODE_B,
};

struct TTLState
{
	byte pin;
	byte mode;
	bool state;
	bool detect;
	bool pulseState;
	unsigned long tTTLon;
	unsigned long tPulseon;
	unsigned long duration;
	unsigned long pulseWidth;
	unsigned long pulsePeriod;
};

struct RuntimeState
{
	byte led_pin;
	bool runtimeFlag;
	bool inputTriggerExists;
	unsigned long tNow;
	unsigned long tLast;
	unsigned long tStart;
	unsigned long tRuntimeStart;
	unsigned long duration;
	unsigned long delay;
	TTLState* inputTrigger;
	TTLState* outputTrigger;
};

struct BlinkLEDState
{
	byte pin;
	byte side;
	bool ledBlinkState;
	unsigned long blinkInterval;
	unsigned long tLEDon;
	unsigned long tLEDoff;
};

struct IRState
{
	byte pin;
	byte side;
	byte proxyLEDPin;
	bool currentRead;
	bool lastRead;
	bool inBreak;
	bool breakEvent;
	bool breakEventMutable;
	bool connectEvent;
	unsigned long tStart;
	unsigned long tOff;
	unsigned long ttlPulsePeriod;
	TTLState* outputTrigger;
};

struct TouchState
{
	byte pin;
	byte side;
	bool current;
	bool last;
	bool inTouch;
	bool touchEvent;
	bool clearEvent;
	unsigned long tStart;
	unsigned long tOff;
	unsigned long ttlPulsePeriod;
	TTLState* outputTrigger;
};

struct SolenoidState
{
	byte pin;
	byte side;
	bool open;
	unsigned long tOpen;
	unsigned long tClose;
	unsigned long duration;
	unsigned long ttlPulsePeriod;
	TTLState* outputTrigger;
};

struct LinearActuatorState
{
	byte pin;
	byte side;
	byte operationMode;
	bool distalLimitSwitch;
	bool proximalLimitSwitch;
	bool atCommandPosition;
	bool atHome;
	long distalLimitPosition;
	long proximalLimitPosition;
	long currentPosition;
	long commandPosition;
	long homePosition;
	long calibrationPositionOffset;
	unsigned long tStart;
	unsigned long tStop;
};

#endif