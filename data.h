/*
 * Data structures for sensor and actuator state parameters;
 */

#ifndef DATA
#define DATA

// enumerated operation modes
enum Mode
{
	MODE_A,
	MODE_B
};

struct IRState
{
	byte pin;
	byte side;
	bool currentRead;
	bool lastRead;
	bool currentPersistant;
	bool lastPersistant;
	bool inBreak;
	bool breakEvent;
	bool breakEventMutable;
	bool connectEvent;
	unsigned long tStart;
	unsigned long tOff;
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
};

struct SolenoidState
{
	byte pin;
	byte side;
	bool open;
	unsigned long tOpen;
	unsigned long tClose;
	unsigned long duration;
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