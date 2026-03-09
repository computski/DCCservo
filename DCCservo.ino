// 
// 
/*
    Name:       DCCservo.ino
    Created:	09/03/2026
    Author:     Julian Ossowski

	DEPENDENCIES:
	NRMA DCC decoder library NmraDcc.h
	Servo library Servo.h
	The board is set to ATmega328P (Old Bootloader) which is for the Nano with old bootloader.


	DESCRIPTION:	 
	Nano based servo driver for DCC model railroad.  Assigns pins 3-12 as servos (i.e. 10 servos). You program them via serial for DCC address,
	for swing range and invert.  You can command the pin directly from serial to closed|thrown|neutral|Toggle. Neutral will set
	the servo to 90 degress (its midpoint) which is useful in setting up the mechanics.  Max range is a full 180 degrees.

	Each servo will respond to its DCC address, which is held as the explicit address 1-2048 
	one or more servos can be assigned the same DCC address.  useful for crossovers along with invert feature.

	Remember that two or more servos can share the same control signal.  If you do this, you need to be mindful of the mechanical set up
	because they will all move in the same direction.

	Rate of movement is a new feature.  Useful rate values are +10 to -10, where negative values retard the rate of rotation.

	Signal aspects is a new feature.  A pin can drive high or low or be 'power off' as in tri-stated.

	Multiple Aspect Signals (MAS) is a new feature, it supports the DCC advanced accessory command for multiple aspect states.
	Note: when using this with Panel Pro, ensure you check the offset-address box.

	HARDWARE notes:
	The unit is powered from the track via a rectifier and 1000uF capacitor.
	pin 2 is the DCC signal.  Enable pullup, as I am using a rectifier to produce 12v power and a pulldown diode 
	in series with a 5k1k resistor as the DCC signal take-off.
	pins 3 to 12 are servo drivers (ten in all)
	pin 13 is the built-in LED, you could disable this and use as a servo output.  I use it as a heartbeat indicator
	the servo code can handle 12 servos on most boards (incl nano)
	neutral position is 90 degrees.
	Closed is defined as minimum position, Thrown as maximum (unless invert = true)
	The unit only actively drives the servo whilst executing a movement command, once the movement is complete, it disables
	the servo.  This stops servo chatter, but does assume the servo can hold its position unpowered.  You can optionally enable
	continuous powering of the servo.
	   
	On eBay there are low cost <Nano expansion board> for sale.  These have 12V DC power jack and lots of header pins which
	are ideal for servos as they support signal, 5v and ground, which is common servo pinout.  Its straightforward to plug the nano into
	one of these and then just jack in the servos.  The 12v rectifier goes to the power socket and the dcc signal to pin 2

	There is no need to use an opto isolator if you intend to power this unit from the track. There is one minor niggle
	with powering it off the track; if you run a loco into a turnout which is set against it, a short circuit results. The
	command station will cut track power and now you are unable to 'correct' the turnout position.  If the turnout board is driven
	from a separate DC supply (and opto isolator used) then you could set the turnout correctly and re-establish track power.
	BUT first this assumes you have a DCC command station with a control bus separate from the power bus, an second it overlooks
	the situation where many locos have keep alives fitted, so even when track power is cut, the loco carries on and rides over
	the point set against it and usually derails.   Ain't no fixing that except with your hand!  Keep it simple and just
	power this unit from the track.


	Serial COMMANDS.  All commands must be followed with newline:

	SETUP command for servos is
	s pin,addr,swing,invert,[continuous]
	pin is 4-12, addr is 1-2048 dcc address, swing is 5-180 and invert is 0|1 where 1 inverts the direction.  
	optional [continuous] 1|0 will ensure continuous servo drive (or not). Default is not.

	SETUP command for signal aspects is
	a pin,addr,invert,[ignorePower]
	Note that setting up [a] against a pin will override [p] and vice-versa
	pin is 4-12, addr is 1-2048 dcc address, invert is 0|1 where 1 inverts the pin logic level.  Normally thrown is a logic 1 on the pin.
	optional ignorePower defaults to 1=yes.  This prevents the unit from responding to the 'off' message that JRMI panel pro sends, which would 
	depower the pin output (makes it tri-state).

	EXECUTE command is
	p pin, c|t|n|T, [power]
	pin is 4-12, c|t|n|T correspond to closed, thrown, neutral, Toggle
	[power] is optional and can be 1|0. It only affects signal aspects, not servo turnouts.

	Once a pin is mapped to a DCC address, it will respond to that DCC turnout or aspect.

	DUMP command is x, this will dump current settings to serial.
	x
	pin 3  address 0  swing 25  invert 0  continuous 0 rate 0 thrown
	pin 4  address 0  swing 25  invert 0  continuous 0 rate 2 closed
	pin 5  address 0  aspect invert 0 power 1 closed


	DCC EMULATION command, a DCC controller itself can only send t|c (and power), whereas T|n are for our convenience when setting up
	d addr, t|c|T|n, [power]
	[power] is optional. 1|0 will set the power parameter to on or off for the given pin.  Only affects signal aspect pins.

	RATE command.
	r pin rate
	Will increase/retard the rate of rotation of a servo.  useful values are +10 faster through to -10 slower.  The currently 
	programmed value is displayed via the x command.

	VERBOSE command.
	v
	Will dump incoming DCC commands to serial. Useful to confirm hardware is receiving DCC and able to match to an address set up
	in the unit.  verbose=OFF on boot.  Dumping DCC to serial might cause delay in execution of commands.

	Further notes:
	Signal aspects:  A pin output can be logic 1|0 based on the throw position.  The pin can also be tristated if power is zero.
	i.e. you could have 1= red, 0= white and if power=0 both aspects are off.

	Routes are not supported.  Use JRMI Panel Pro or an advanced DCC controller.
	
	This unit does not support CV based set up over DCC.  There is no point.  It is easy to set-up the unit over
	serial, and once done you would rarely change it.  DCC is used only to command the turnouts to throw or aspects to change.
	
	Why not use arcomora software?  It won't fit into a nano with a standard bootloader.
	I also consider it too complex to set up.  Also its not clear if it can support the simple 
	solution used here where the nano is plugged direct into a low cost expansion board.


	https://www.arduino.cc/reference/en/libraries/servo/
	https://github.com/nzin/arduinodcc/tree/master/arduinoSource/dccduino
	https://rudysmodelrailway.wordpress.com/2015/01/25/new-arduino-dcc-servo-and-function-decoder-software/
	https://www.arcomora.com/mardec/

	The nrma library does not support POM CV writes in the sense that the address and cv and value are all presented in a callback.
	I think it only supports the idea of the board having a single dcc address, and then it might generate cv events against
	that address if it comes over POM.  My issue is that the project here can support 10 independent DCC addresses.
	One of the reasons I have not bothered to support CVs in this code.

*/

#include <DCC_Decoder.h>
#include <Servo.h>
#include <EEPROM.h>
#include <NmraDcc.h>
#include <stdlib.h>



#define TOTAL_PINS 10
#define BASE_PIN 3
#define ASPECT_PARAMETER_SIZE	8


/*DCC related*/
NmraDcc  Dcc;
DCC_MSG  Packet;

struct CVPair
{
	uint16_t  CV;
	uint8_t   Value;
};

CVPair FactoryDefaultCVs[] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
};

uint8_t FactoryDefaultCVIndex = 0;
bool verbose=false;

/*EEprom and version control*/
struct CONTROLLER
{
	long softwareVersion = 20260306;  //yyyymmdd captured as an integer
	bool isDirty = false;  //will be true if eeprom needs a write
	long long padding;
};

/*defaultController contains defaults defined in the CONTROLLER object and is used to override eeprom settings
on new compilations where data structures have changed.  The system runs off the m_bootController object normally
the padding variable seems to fix a bug on the nano that saw eeprom contents corrupt on readback*/
CONTROLLER bootController;
CONTROLLER m_defaultController;

//declare EEPROM functions
void getSettings(void);
void putSettings(void);


//serial communication related
void recvWithEndMarker();
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;

bool ledState;

/*servo control.  VIRTUALSERVO is each virtualised device with its params.  you command this over serial or DCC
it also points to an array of servo objects which actually drive the servos
2026-02-09 added signal aspects*/
enum servoState {
	SERVO_NEUTRAL,
	SERVO_TO_THROWN,
	SERVO_THROWN,
	SERVO_TO_CLOSED,
	SERVO_CLOSED,
	SERVO_BOOT,
	ASPECT_THROWN,
	ASPECT_CLOSED,
	ASPECT_MULTIPLE
};

struct VIRTUALSERVO {
	uint8_t pin;
	uint16_t address;
	uint8_t swing;
	bool invert;
	bool continuous;
	bool power;
	bool ignorePowerParameter;
	bool isServo;  //servo or aspect
	uint8_t state;
	uint8_t position;
	int8_t rate;  //+ve values speed up movement, -ve slow it down
	int8_t timeDelay;  //working register, loaded negative and counts up to zero
	int8_t aspectParameters[ASPECT_PARAMETER_SIZE * 4];
	Servo* thisDriver;
	uint8_t MASstate;  //multiple aspect signals commanded state
};


//define functions here
uint8_t parseBracketedParameters(char* token, int8_t* result);
bool isAdvanced(VIRTUALSERVO vs);



//statemachines and params
VIRTUALSERVO virtualservoCollection[TOTAL_PINS];

//servo drivers. This library creates a servo driver (pulse posn modulation) for each of the arduino pins
Servo servoDriver[TOTAL_PINS];



unsigned long currentMs;
unsigned long previousMs;
uint8_t bootTimer = 0;
uint8_t tick;

void setup() {
	Serial.begin(115200);
	delay(1000);

	//restore virtualservo array from EEPROM
	getSettings();
	pinMode(LED_BUILTIN, OUTPUT);

	//DCC, setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
	//for my circuit, dcc is active low through a pulldown diode and a series 5k1k res, hence need the pull-up
	//for the nano, int 0 is on pin 2
	Dcc.pin(0, 2, 1);

	// Call the main DCC Init function to enable the DCC Receiver
	Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);
	Serial.println("DCC initialised");
	Serial.println("Commands are s p x d a r v\n");

}

/* not needed. Can use this to dump every incoming dccPacket
void notifyDccMsg(DCC_MSG * Msg)
{
	Serial.print("notifyDccMsg: ");
	for (uint8_t i = 0; i < Msg->Size; i++)
	{
		Serial.print(Msg->Data[i], HEX);
		Serial.write(' ');
	}
	Serial.println();
}
*/


// MULTI ASPECT SIGNALS this function called whenever an Extended Accessory Packet is received
void notifyDccSigOutputState(uint16_t addr, uint8_t state) {
	if (verbose) {
		Serial.print(F("notifyDccSigOutputState: "));
		Serial.print(addr, DEC);
		Serial.print(',');
		Serial.println(state, DEC);
	}
	//once we find the VS slot from the addr, we need to confirm it isAdvanced() and only then process the action

	for (auto& vs : virtualservoCollection) {
		if (addr != vs.address) continue;
		if (vs.isServo) continue;
		if (!isAdvanced(vs)) continue;
		
		vs.MASstate = state;
		vs.state = ASPECT_MULTIPLE;
		
	}



}


// This function is called whenever a normal DCC Turnout Packet is received and we're in Board Addressing Mode
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower)
{
	if (!verbose) return;
	Serial.print("notifyDccAccTurnoutBoard: ");
	Serial.print(BoardAddr, DEC);
	Serial.print(',');
	Serial.print(OutputPair, DEC);
	Serial.print(',');
	Serial.print(Direction, DEC);
	Serial.print(',');
	Serial.println(OutputPower, HEX);
	//does nothing more, we are not using it
}


/// <summary>
/// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
/// </summary>
/// <param name="Addr">Per output address. There will be 4 Addr addresses per board for a standard accessory decoder with 4 output pairs.</param>
/// <param name="Direction">Turnout direction. It has a value of 0 or 1. Equivalent to bit 0 of the 3 DDD bits in the accessory packet. 1 means Thrown</param>
/// <param name="OutputPower">1 denotes power on, 0 power off.  Useful for signal aspects.  Solenoids and servos will ignore the off command.</param>
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
	if (verbose) {
		Serial.print("notifyDccAccTurnoutOutput: ");
		Serial.print(Addr, DEC);
		Serial.print(',');
		Serial.print(Direction, DEC);
		Serial.print(',');
		Serial.println(OutputPower, HEX);
	}

	//act on the data, finding all matching virtualservos with the given dcc address
	for (auto& vs : virtualservoCollection) {
		if (Addr != vs.address) continue;
		//execute for EVERY instance of this DCC address,not just the first.
		//For servos, ignore any power-off instruction
		vs.power = OutputPower == 0 ? false : true;

		//take action. Direction 0 is closed, 1 thrown
		if (vs.isServo) {
			if (!vs.power) continue;  //ignore 'off' commands to servos
			vs.state = Direction == 0 ? SERVO_TO_CLOSED : SERVO_TO_THROWN;
		}
		else {
			//2026-03-07 only respond to the command if the signal is not a multi-aspect device
			if (isAdvanced(vs)) return;
			vs.state = Direction == 0 ? ASPECT_CLOSED : ASPECT_THROWN;
		}

		if (verbose) {
			Serial.print("dcc command to pin: ");
			Serial.println(vs.pin, DEC);
		}
	}
	
}


void loop() {
	static VIRTUALSERVO* vsBoot = nullptr;

	Dcc.process();

	if (FactoryDefaultCVIndex && Dcc.isSetCVReady())
	{
		FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
		Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
	}



	//15ms timer
	currentMs = millis();
	if (currentMs - previousMs >= 15) {
		previousMs = currentMs;
		++tick;
		if (tick >= 33) {
			tick = 0;
			ledState = !ledState;
			if (ledState)	digitalWrite(LED_BUILTIN, HIGH);
			if (!ledState) digitalWrite(LED_BUILTIN, LOW);
		}

		//update all moving servos every 15mS
		//in normal non-invert mode, minPosition is turnout closed, and maxPosition is turnout thrown

		//for signal aspects, we move from ASPECT_CLOSED or ASPECT_THROWN straight to the antiphase, and we pay heed to the .power parameter

		for (auto& vs : virtualservoCollection) {
			uint8_t maxPosition = vs.swing + 90;
			uint8_t minPosition = 90 - vs.swing;

			//Servo rotation rates. +ve rate values will speed up movement by increasing the movement increment above 1
			uint8_t increment = vs.rate>0 ? vs.rate:1;
			//.timeDelay is used for -ve rate values
			vs.timeDelay += vs.timeDelay < 0 ? 1 : 0;


			switch (vs.state) {
			case SERVO_NEUTRAL:
				vs.position = 90;
				if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
				break;

			case SERVO_TO_CLOSED:
				//-ve rotation rate values will slow down movement
				if (vs.timeDelay != 0) break;
				vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

				//swing toward minPosition, unless invert is true
				if (vs.invert) {
					vs.position += vs.position < maxPosition ? increment : 0;
				}
				else {
					vs.position -= vs.position > minPosition ? increment : 0;
				}

				if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
					vs.state = SERVO_CLOSED;
				}

				if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
				break;

			case SERVO_TO_THROWN:
				//-ve roation rate values will slow down movement
				if (vs.timeDelay != 0) break;
				vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

				//swing toward maxPosition unless invert is true
				if (vs.invert) {
					vs.position -= vs.position > minPosition ? increment : 0;
				}
				else {
					vs.position += vs.position < maxPosition ? increment : 0;
				}

				if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
					vs.state = SERVO_THROWN;
				}

				if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
				break;

			case SERVO_THROWN:
				vs.position = vs.invert ? minPosition : maxPosition;
				if ((vs.thisDriver->attached()) && (!vs.continuous)) {
					vs.thisDriver->detach();
				}
				break;
			case SERVO_CLOSED:
				vs.position = vs.invert ? maxPosition : minPosition;
				if ((vs.thisDriver->attached()) && (!vs.continuous)) {
					vs.thisDriver->detach();
				}
				break;

			case ASPECT_CLOSED:
				if ((vs.power)||(vs.ignorePowerParameter)) {
					//actively drive the pin. Thrown is a high state unless invert is active
					pinMode(vs.pin, OUTPUT);
					digitalWrite(vs.pin, vs.invert? HIGH:LOW);
				}
				else {
					//pin to tri-state
					pinMode(vs.pin, INPUT);
				}
				break;

			case ASPECT_THROWN:
				if ((vs.power)||(vs.ignorePowerParameter)){
					//actively drive the pin. Thrown is a high state unless invert is active
					pinMode(vs.pin, OUTPUT);
					digitalWrite(vs.pin, vs.invert ? LOW : HIGH);
				}
				else {
					//pin to tri-state
					pinMode(vs.pin, INPUT);
				}
				break;


			case ASPECT_MULTIPLE:
				//2026-03-07 new code required to handle this, including gating on/off for flashing
				assertAspectState(vs);
				break;

			case SERVO_BOOT:
				if (vsBoot == nullptr) {
					//handle next-up servo to boot. servos are booted in the CLOSED position
					//and aspects are booted with POWER=off
					vsBoot = (VIRTUALSERVO*) &vs;
					bootTimer = 34;
					if (vs.isServo) {
						vs.position = vs.invert ? maxPosition : minPosition;
						if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
						vs.thisDriver->write(vs.position);
					}
					else {
						//aspect. Immediately go to closed state with power off
						vs.power = false;
						vs.state = isAdvanced(vs) ? ASPECT_MULTIPLE: ASPECT_CLOSED;
						vsBoot = nullptr;
						bootTimer = 0;
						vs.thisDriver->detach();
					}
										
				}
				else if (vsBoot == (VIRTUALSERVO*) &vs) {
					//if this is the current boot-servo, then decrement bootTimer
					bootTimer -= bootTimer > 0 ? 1 : 0;

					//timed out?
					if (bootTimer == 0) {
						vs.state = SERVO_CLOSED;
						Serial.print(F("pin booted"));
						Serial.println(vs.pin, DEC);
						//release for next vs to boot
						vsBoot = nullptr;
					}
				}
				break;
			}

			//update the servo position every 15mS
			vs.thisDriver->write(vs.position);
		}

	}//15ms timer


	//look for more incoming serial data
	recvWithEndMarker();

	//process serial command
	if (newData) {
		//need a temporary virtualservo object
		VIRTUALSERVO vsParse;
		//also need a pointer to a servo in virtualservoCollection
		VIRTUALSERVO* vsPointer = nullptr;

		//Serial.println(receivedChars);
		newData = false;

		//ASPECT set-up command. Usage: a pin,addr,invert,[ignorePower]
		//ignorePower is default true and will ignore dcc power off commands
		if (receivedChars[0] == 'a') {
			vsParse.isServo = false;
			vsParse.state = ASPECT_CLOSED;

			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");

			while (pch != NULL) {
				switch (i) {
				case 1:
					vsParse.pin = strtol(pch, NULL, 10);
					//valid pin range is BASE_PIN to BASE_PIN + TOTAL_PINS
					if ((vsParse.pin < BASE_PIN) || (vsParse.pin >= BASE_PIN + TOTAL_PINS)) {
						i = 10;  //error code
						Serial.println("bad pin");
					}
					break;
				case 2:
					vsParse.address = strtol(pch, NULL, 10);
					//if out of range 1-2048 then throw an error
					if ((vsParse.address > 2048) || (vsParse.address == 0)) {
						i = 10;
						Serial.println("bad address");
					}
					break;

				case 3:
					vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
					vsParse.ignorePowerParameter = true;  //default
					break;

				case 4:
					vsParse.ignorePowerParameter = strtol(pch, NULL, 10) == 0 ? false : true;
					break;
				}

				++i;
				pch = strtok(NULL, " ,");

			}
			if ((i == 4)||(i==5)) {
				Serial.println("OK");
				//match to a pin member of servoslot and copy it over  
				for (auto& vs : virtualservoCollection) {
					if (vs.pin == vsParse.pin) {
						//clear vsParse.aspectParameters to -1, as this array is only used by multi aspect signals
						memset(vsParse.aspectParameters, -1, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
						vsParse.thisDriver = vs.thisDriver;
						vs = vsParse;  //copy over from vsParse
						if (vs.thisDriver->attached()) vs.thisDriver->detach();
						//write to EEPROM
						bootController.isDirty = true;
						putSettings();
						break;
					}
				}

			}
			else {
				Serial.println(F("bad command. usage a pin,addr,invert,[ignorePower]"));
			}

		}


		//SERVO set-up command. Usage: s pin, addr, swing, invert, [continuous]
		if (receivedChars[0] == 's') {
			vsParse.isServo = true;
			vsParse.ignorePowerParameter = true;
			vsParse.continuous = false;  //default setting
			memset(vsParse.aspectParameters, -1, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));

			//detokenize
char* pch;
int i = 0;
pch = strtok(receivedChars, " ,");
while (pch != NULL) {
	switch (i) {
	case 1:
		vsParse.pin = strtol(pch, NULL, 10);
		//valid pin range is BASE_PIN to BASE_PIN + TOTAL_PINS
		if ((vsParse.pin < BASE_PIN) || (vsParse.pin >= BASE_PIN + TOTAL_PINS)) {
			i = 10;  //error code
			Serial.println("bad pin");
		}
		break;
	case 2:
		vsParse.address = strtol(pch, NULL, 10);
		//if out of range 1-2048 then throw an error
		if ((vsParse.address > 2048) || (vsParse.address == 0)) {
			i = 10;
			Serial.println("bad address");
		}
		break;
	case 3:
		vsParse.swing = strtol(pch, NULL, 10);
		if (vsParse.swing > 90) {
			i = 10;
			Serial.println("bad swing range");
		}
		break;
	case 4:
		vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
		break;
	case 5:
		//this param is optional
		vsParse.continuous = strtol(pch, NULL, 10) == 0 ? false : true;
		break;
	}
	//zero is the s char

	++i;
	pch = strtok(NULL, " ,");

}

if ((i == 6) || (i == 5)) {
	//[continuous] is optional, accept 5 || 6
	Serial.println("OK");
	//match to a pin member of virtualservoCollection and copy it over

	for (auto& vs : virtualservoCollection) {
		if (vs.pin != vsParse.pin) continue;
		//first copy servo-driver pointer to servoParse
		vsParse.thisDriver = vs.thisDriver;
		//then copy servoParse to vs
		vs = vsParse;
		vs.position = 90;
		vs.isServo = true;
		vs.state = SERVO_TO_CLOSED;
		//write to EEPROM
		bootController.isDirty = true;
		putSettings();
		break;
	}
}
else
{
	Serial.println("bad command. usage s pin,addr,swing,invert,[continuous]");
}


		}

		//PIN ACTION. Usage: p pin, c|t|T|n , [power]
		// where closed|thrown|TOGGLE|neutral
		//power is 1|0 and only affects aspects
		if (receivedChars[0] == 'p') {
			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			int p = -1;

			while (pch != NULL) {
				switch (i) {
				case 1:
					//pin
					p = strtol(pch, NULL, 10);
					if ((p < BASE_PIN) || (p >= BASE_PIN + TOTAL_PINS)) {
						i = 10;
						Serial.println("bad pin");
						p = -1;
						break;
					}
					//p is valid, use this to lookup the servoslot
					for (auto& vs : virtualservoCollection) {
						if (vs.pin != p) continue;
						//use a pointer because we subsequently want to modify the collection item, not copy data to it
						vsPointer = (VIRTUALSERVO*)&vs;

						//2026-03-09 if this is a MAS signal, this command will not work
						if (isAdvanced(vs)){
							Serial.println(F("Cannot use command on MAS aspect"));
							p = -1;
						}
					}
					


					break;

				case 2:
					if (vsPointer == nullptr) { i = 10;break; }

					if (vsPointer->isServo) {
						switch (pch[0]) {
						case 'c':
							vsPointer->state = SERVO_TO_CLOSED;
							break;
						case 't':
							vsPointer->state = SERVO_TO_THROWN;
							break;
						case 'n':
							vsPointer->state = SERVO_NEUTRAL;
							break;
						case 'T':
							vsPointer->state = vsPointer->state == SERVO_CLOSED ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
						}
					}
					else {
						//signal aspect. Only supports thrown or closed states
						switch (pch[0]) {
						case 't':
							vsPointer->state = ASPECT_THROWN;
							break;
						case 'T':
							vsPointer->state = (vsPointer->state == ASPECT_THROWN) ? ASPECT_CLOSED : ASPECT_THROWN;
							break;
						default:
							vsPointer->state = ASPECT_CLOSED;
							break;
						}
					}
					break;
				case 3:
					//optional [power] param for signal aspects
					vsPointer->power = pch[0] == '1' ? true : false;
				}

				++i;
				pch = strtok(NULL, " ,");
			}

			if ((i == 3) || (i == 4)) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage p pin,t|c|n|T,[power]");
			}

		}


		//EMULATE a dcc command.  This will affect all servos/aspects at a given dcc address
		//this code block can support T=toggle and n=neutral, which are not themselves a DCC command
		//usage: d addr,t|n|T|c,[power]
		if (receivedChars[0] == 'd') {
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			int p = -1;
			int address = -1;

			while (pch != NULL) {
				switch (i) {
				case 1:
					//resolve address
					address = strtol(pch, NULL, 10);
					if ((address < 1) || (address > 2048)) {
						i = 10;
						Serial.println("bad address");
						p = -1;
						break;
					}
					//address valid, use this to match to individual servos
					break;

				case 2:
					//command.  Iterate all servos and execute on all matching addresses
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;

						switch (pch[0]) {
						case 't':
							vs.state = vs.isServo ? SERVO_TO_THROWN : ASPECT_THROWN;
							break;
						case 'n':
							vs.state = vs.isServo ? SERVO_NEUTRAL : ASPECT_CLOSED;
							break;
						case 'T':
							if (vs.isServo) {
								vs.state = (vs.state == SERVO_CLOSED) ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
							}
							else {
								vs.state = (vs.state == ASPECT_THROWN) ? ASPECT_CLOSED : ASPECT_THROWN;
							}
							break;
						default:
							vs.state = vs.isServo ? SERVO_TO_CLOSED : ASPECT_CLOSED;
						}
					}
					break;
				case 3:
					//power.  Iterate all servos and execute on all matching addresses
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						vs.power = pch[0] == '1' ? true : false;
					}
					break;
				}
				++i;
				pch = strtok(NULL, " ,");

			}

			if ((i == 3) || (i == 4)) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage d address,t|c|T|n,[power]");
			}

		}

		//EMULATE a dcc command for Multi Aspect Signal.  This will affect all MAS at a given dcc address
		//usage: D addr,state
		if (receivedChars[0] == 'D') {
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			int address = -1;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					//resolve address
					address = strtol(pch, NULL, 10);
					if ((address < 1) || (address > 2048)) {
						i = 10;
						Serial.println("bad address");
						break;
					}
					//address valid, use this to match to individual servos
					break;
				case 2:
					//state command.  Iterate all MAS and execute on all matching addresses
					uint8_t state = strtol(pch, NULL, 10);

					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						if (!isAdvanced(vs)) continue;
						vs.MASstate = state;
						Serial.println(assertAspectState(vs),DEC);
					}
					break;
				}
				pch = strtok(NULL, " ,");
			}

			if (i<10) Serial.println("OK");
		}


		//DEBUG block. y will dump power states
		if (receivedChars[0] == 'y') {
			for (auto vs : virtualservoCollection) {
				Serial.print("pin ");
				Serial.print(vs.pin, DEC);
				Serial.print("  power ");
				Serial.println(vs.power, DEC);
			}
			Serial.print("\n");
		}

		//RATE command. sets a positive or negative rate on the servo swing.
		//usage r pin rate, where rate is + or -ve integer, useful values are -10 to +10
		if (receivedChars[0] == 'r') {
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			int p = -1;
			int rate = 0;

			while (pch != NULL) {
				switch (i) {
				case 1:
					p = strtol(pch, NULL, 10);
					if ((p < BASE_PIN) || (p >= BASE_PIN + TOTAL_PINS)) {
						i = 10;
						Serial.println("bad pin");
						p = -1;
						break;
					}
					//p is valid, use this to lookup the servoslot
					for (auto& vs : virtualservoCollection) {
						if (vs.pin != p) continue;
						//use a pointer because we subsequently want to modify the collection item, not copy data to it
						vsPointer = (VIRTUALSERVO*)&vs;
					}
					break;

				case 2:
					//resolve rate
					rate = strtol(pch, NULL, 10);
					if (rate > 10) rate = 10;
					if (rate < -10) rate = -10;
					//note improperly formed numbers such as -7.7 or 'three' will resolve to zero
					break;
				}

				++i;
				pch = strtok(NULL, " ,");
			}

			if (i == 3) {
				Serial.println("OK");
				vsPointer->rate = rate;
				bootController.isDirty = true;
				putSettings();
			}
			else
			{
				Serial.println("bad command. usage r pin rate");
				Serial.println(i, DEC);
			}
		}

		//Toggle VERBOSE serial output
		//if verbose = ON then incoming DCC commands are dumped to serial, which can slow down processing
		if (receivedChars[0] == 'v') {
			if (verbose) {
				Serial.println("verbose OFF\n");
				verbose = false;
			}
			else {
				Serial.println("verbose ON\n");
				verbose = true;
			}

		}


		//DUMP all servo/aspect information
		if (receivedChars[0] == 'x') {

			for (auto vs : virtualservoCollection) {
				//dump this pin
				if (vs.isServo) {
					Serial.print("servo  pin ");
					Serial.print(vs.pin, DEC);
					Serial.print("  address ");
					Serial.print(vs.address, DEC);
					Serial.print("  swing ");
					Serial.print(vs.swing, DEC);
					Serial.print("  invert ");
					Serial.print(vs.invert, DEC);
					Serial.print("  continuous ");
					Serial.print(vs.continuous, DEC);
					Serial.print("  rate ");
					Serial.print(vs.rate, DEC);

				}
				else {
					//dump signal aspects
					if (isAdvanced(vs)) {
						Serial.print(F("MAS pin "));
						Serial.print(vs.pin, DEC);
						Serial.print(F("  address "));
						Serial.print(vs.address, DEC);
						Serial.print(F("  invert "));
						Serial.print(vs.invert, DEC);

						Serial.print(F("  output "));
						switch (assertAspectState(vs)) {
						case 0:
							Serial.print("0 ");
							break;
						case 1:
							Serial.print("1 ");
							break;

						default:
							Serial.print("tristate ");
						
						}
											
						//now the param arrays
						bool noSpace = true;
						for (uint8_t a = 0;a < 32;a++) {
							if (a == 0) {
								Serial.print(" hi[");
								noSpace=true;
							}
							if (a == ASPECT_PARAMETER_SIZE - 1) {
								Serial.print("] lo[");
								noSpace = true;
							}
							if (a == (2 * ASPECT_PARAMETER_SIZE) - 1) {
								Serial.print("] hi-flash[");
								noSpace = true;
							}
							if (a == (3 * ASPECT_PARAMETER_SIZE) - 1) {
								Serial.print("] lo-flash[");
								noSpace = true;
							}

							if (vs.aspectParameters[a] == -1) continue;
							if (!noSpace) Serial.print(" ");
							Serial.print(vs.aspectParameters[a], DEC);
							noSpace = false;

						}
						Serial.print("]");
					}
					else
					{
					Serial.print(F("aspect pin "));
					Serial.print(vs.pin, DEC);
					Serial.print(F("  address "));
					Serial.print(vs.address, DEC);
					Serial.print(F("  invert "));
					Serial.print(vs.invert, DEC);
					Serial.print(F("  power "));
					if (vs.ignorePowerParameter) { Serial.print("x"); }
					else { Serial.print(vs.power, DEC); }
					}
				}

				if (vs.thisDriver == nullptr) {
					Serial.print(" pointer bad");
				}
				else
				{
					//dump output state
					switch (vs.state) {
					case ASPECT_MULTIPLE:
						Serial.print(F(" state "));
						Serial.print(vs.MASstate, DEC);
						break;
						
					case ASPECT_THROWN:
					case SERVO_THROWN:
					case SERVO_TO_THROWN:
						Serial.print(F(" thrown"));
						break;
					default:
						Serial.print(F(" closed"));
						break;
					}

				}
				Serial.print("\n");
			}
		}
	
		//EEPROM test.  verifies all locations work. But will destroy all user data and
		//fill EEPROM with junk.  You must reboot after this test to reinitialise EEPROM.
		if (receivedChars[0] == 'E') EEpromTest();


		//Advanced aspect signalling.  A pin addr invert [hi array] [low array] [hi flash array] [low flash array]
		if (receivedChars[0] == 'A') {
			//make use of vsParse to hold params as we verify them
			vsParse.pin = -1;
			vsParse.address = -1;
			char* pch;
			uint8_t i = 0;   //i is a state engine
			pch = strtok(receivedChars, " ,");
			bool resolved = true;
				uint8_t bufOffset=0;

			while (pch != NULL) {
				switch (i) {
				case 0:
					//ignore A character at start of command
					i++;
					bufOffset = 0;
					break;

				case 1:
					i++;
					vsParse.pin = strtol(pch, NULL, 10);
					if ((vsParse.pin < BASE_PIN) || (vsParse.pin >= BASE_PIN + TOTAL_PINS)) {
						Serial.println("bad pin");
						resolved = false;
					}
					break;

				case 2:
					//resolve address
					i++;
					vsParse.address = strtol(pch, NULL, 10);
					if ((vsParse.address < 1) || (vsParse.address > 2048)) {
						Serial.println("bad address");
						resolved = false;
					}
					//address valid, use this to match to individual servos
					break;

				case 3:
					vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
					i++;
					break;

				case 4:
				case 5:
				case 6:
				case 7:
					//expect 4 sets of [bracketed params]
					{//scope A
					//we keep looking at tokens until we find a [ start then we import to ] and declare end (2).
					switch (parseBracketedParameters(pch, vsParse.aspectParameters + bufOffset))
					{
					case 2:

						bufOffset += 8;
						i++;
						/*
						Serial.print("bufOffset=");
						Serial.println(bufOffset, DEC);
						Serial.println(vsParse.aspectParameters[0], DEC);
						Serial.println(vsParse.aspectParameters[1], DEC);
						Serial.println(vsParse.aspectParameters[2], DEC);
						*/
						break;

					case 4:
						//parser error in bracketed params
						resolved = false;
						break;
					default:
						break;
					}

					}//end scope A
					break;

				case 8://optional flash rate param
					i++;
					break;
				}//switch


				pch = strtok(NULL, " ,");
				if (resolved == false) 	break;
				
			}//while

			if (i < 8) resolved = false; 

			//next we expect [a b c] where a b c can be nothing through to dd digits
			if (resolved) {
				//all params were captured into vsParse
								
				for (auto& vs : virtualservoCollection) {
					if (vs.pin != vsParse.pin) continue;
				
				//copy received data to the item
					vs.address = vsParse.address;
					vs.isServo = false;
					vs.power = false;  //default is all output drivers are tri-state, await first dcc command
					vs.ignorePowerParameter = true;
					vs.invert = vsParse.invert;

					memcpy(vs.aspectParameters, vsParse.aspectParameters, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
					vs.state = ASPECT_MULTIPLE;
					break;
				}
			

				//DEBUG BLOCK
				/*
				Serial.println(vsParse.pin, DEC);
				Serial.println(vsParse.address, DEC);

				for (auto vs : virtualservoCollection) {
					if (vs.pin != vsParse.pin) continue;
					//dump the buffer
					for (uint8_t a = 0;a < 32;a++) {
						Serial.print(" ");
						Serial.print(vs.aspectParameters[a], DEC);
					
					}
					Serial.print(" A=");
					Serial.println(isAdvanced(vs), DEC);
				}
				*/

				putSettings();
				Serial.println("OK");
			}
			else {
				Serial.println(F("Error parsing command. Usage A pin addr invert [hi] [lo] [hi-flash] [low-flash]"));
			}

			



		}



	}
}//main loop






//https://forum.arduino.cc/index.php?topic=288234.0
void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	// if (Serial.available() > 0) {
	while ((Serial.available() > 0) && (newData == false)) {
		rc = Serial.read();

		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= numChars) {
				ndx = numChars - 1;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}


//getSettings reads from EEPROM. nano has 1024 bytes of eeprom
//https://raw.githubusercontent.com/RuiSantosdotme/Random-Nerd-Tutorials/master/Projects/Arduino_EEPROM.ino

void getSettings(void) {
	int eeAddr = 0;
	bool factory = false;
	EEPROM.get(eeAddr, bootController);
	if (m_defaultController.softwareVersion != bootController.softwareVersion) {
		/*If software version has changed, we need to re-initiatise eeprom with factory defaults*/
		Serial.println("restore factory defaults");
		factory = true;
		EEPROM.put(0, m_defaultController);
		//eeAddr += sizeof(m_defaultController);
		eeAddr = 32; //bug fix

		/*use pin 4 onward. set defaults*/
		int p = BASE_PIN;
		for (auto& vs : virtualservoCollection) {
			vs.pin = p;
			vs.invert = 0;
			vs.position = 90;
			vs.swing = 25;
			vs.continuous = 0;
			vs.state = SERVO_BOOT;
			vs.power = false;
			vs.ignorePowerParameter = true;
			vs.isServo = true;
			vs.rate = 0;
			memset(vs.aspectParameters, -1, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
			++p;
		}
		/*write back default values*/
		EEPROM.put(eeAddr, virtualservoCollection);
	}

	/*either way, now populate our structs with EEprom values*/
	eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	//eeAddr += sizeof(bootController);
	eeAddr = 32; //bug fix
	EEPROM.get(eeAddr, virtualservoCollection);
	eeAddr += sizeof(virtualservoCollection);
	Serial.print(F("EEPROM="));
	Serial.println(eeAddr, DEC);

	//initialise the pin assignments 4 onwards move all servos and aspects to closed position
	int p = BASE_PIN;
	int i = 0;

	for (auto& vs : virtualservoCollection) {
		vs.pin = p;
		vs.state = SERVO_BOOT;
		vs.MASstate = 127;
		//s.position = 90;  //neutral
		/*minimum useful swing is 5 degrees*/
		if ((vs.swing < 5) || (vs.swing > 90)) vs.swing = 5;

		//calculate closed posn (we may be inverted) then back off 5 degrees and set that as posn
		if (vs.invert) {
			//max position
			vs.position = 90 + vs.swing - 5;
		}
		else {
			//min position, normal for closed
			vs.position = 90 - vs.swing + 5;
		}

		//2026-02-13 for some reason, address data is corrupt when reading back pin 3 parameters.
		//try masking out irrelevant bits.  FFF allows 2048d, whereas 7FF allows upto 2047d
		vs.address &= 0xFFF;


		//initialise the servo driver
			//servoDriver[i].attach(p);
			//servoDriver[i].write(s.position);
			//2020-11-09 we don't want to attach at this time as it will asert an unhelpful position
		servoDriver[i].detach();
		vs.thisDriver = &servoDriver[i];
		++p;
		++i;
	}
	Serial.print(F("\nsofware version "));
	Serial.println(bootController.softwareVersion, DEC);
	if (factory) Serial.println(F("factory reset"));
	
}


//putSettings writes to EEPROM. to reduce wear only call if a change has been made
//2026-01-15 bug fix.  Even though eeAddr += sizeof(bootController); should work, it seems there's a bug which causes pin 3 or pin 4 settings to scramble on power cycle.
//possibly because sizeof(bootController) and sizeof(m_defaultController) are not always the same 13 bytes.  Fix is to just hardcode virtualservoCollection to start at
//location 32.
void putSettings(void) {
	int eeAddr = 0;
	if (bootController.isDirty == false) { return; }
	EEPROM.put(eeAddr, bootController);

	//eeAddr += sizeof(bootController);
	eeAddr = 32;  //bug fix
	EEPROM.put(eeAddr, virtualservoCollection);
	Serial.println("putSettings");
	bootController.isDirty = false;
}

/// <summary>
/// write and verify every EEPROM location.  This wipes all prior user data
/// use this routine via the E command if your nano seems to have corrupt
/// readback of the user settings.  Any bad locations will be listed.
/// </summary>
void EEpromTest(void) {
	int addr = 0;
	int eepromSize = EEPROM.length();

	unsigned char write_data=0b10101010;
	unsigned char read_data;
	Serial.print("EEPROM test ");
	Serial.println(eepromSize, DEC);
	Serial.println("Writing 0xAA");
	for (addr=0;addr<eepromSize;addr++) EEPROM.write(addr, write_data);

	//verify
	for (addr = 0; addr < eepromSize; addr++)
	{
		read_data = EEPROM.read(addr);
		if (read_data != write_data) {
			Serial.print("bad ");
			Serial.println(addr, DEC);
		}

	}
	Serial.println("Writing 0x55");
	write_data = 0x55;
	for (addr = 0;addr < eepromSize;addr++) EEPROM.write(addr, write_data);

	//verify
	for (addr = 0; addr < eepromSize; addr++)
	{
		read_data = EEPROM.read(addr);
		if (read_data != write_data) {
			Serial.print("bad ");
			Serial.println(addr, DEC);
		}

	}
	Serial.println(F("Finished. You must reboot as EEPROM is now full of junk data"));

}

/// <summary>
/// call from a strtok loop, repeatedly passing in tokens. It will parse looking for [aa bb cc]
/// max of 8 parameters, min no params.  params must be enclosed in square brackets.
/// </summary>
/// <param name="token">null terminated char string</param>
/// <param name="result">array to receive the numeric parameters recovered</param>
/// <returns>2 when the entire set of max 8 tokens is parsed.  non-values are represented as -1 </returns>
uint8_t parseBracketedParameters(char* token, int8_t* result) {
	static int8_t arr[ASPECT_PARAMETER_SIZE];
	static uint8_t c = 0;
	static uint8_t parserState = 0;
	char* endptr;
	/*0 not in block
	1 in block
	2 block ended, valid result
	4 format error
	if you call again for state other than 1, it will revert to case 0
	*/

	//self reset
	if (parserState != 1) parserState = 0;
	bool foundStart = token[0] == '[' ? true : false;
	bool foundEnd = token[strlen(token) - 1] == ']' ? true : false;
	

	switch (parserState) {
	case 0:  //not in a [] block
		if (!foundStart) break;
		parserState = 1;
		//fill arr[] with -1
		for (c = 0;c < 8;c++) {
			arr[c] = -1;
		}
		c = 0;

		//[] is a special case
		if ((strlen(token) == 2) && foundEnd) {
			parserState = 2;
			break;
		}

		arr[c++] = strtol(++token, &endptr, 10);

		//if the conversion worked and used entire string, *endptr==NULL
	//more useful if endptr==p then nothing was converted
		if (endptr == token) {
			parserState = 4;  //declare fail
			break;
		}

		if ((*endptr != '\0') && (*endptr != ']')) {
			parserState = 4;  //declare fail
			break;
		}

		//its possible this is [x]
		if (foundEnd) {
			parserState = 2;
		}
		//note that ]] results in no start found and [[ results in 1
		break;



	case 1:  //in a [] block
		if (foundStart) {
			//re start is an error
			parserState = 4;
			break;
		}

		arr[c++] = strtol(token, &endptr, 10);
		if (endptr == token) {
			parserState = 4;  //FAIL
			break;
		}

		if ((*endptr != '\0') && (*endptr != ']')) {
			parserState = 4;  //FAIL
			break;
		}


		if (c > ASPECT_PARAMETER_SIZE) {
			//too many params
			parserState = 4;
			break;
		}

		if (foundEnd) {
			parserState = 2;
		}
		break;

	}

	if (parserState == 2) {
		//copy static array to result.  If state=fail, result is left untouched
		//WARNING: Array Decay: When an array is passed as an argument to a function, it decays into a pointer to its first element.
		//this means sizeof tests can yeild unpredctable behaviour.  The safer approach is to use ASPECT_PARAMETER_SIZE

		for (int a = 0;a < ASPECT_PARAMETER_SIZE; a++) {
			result[a] = arr[a];
		}
	}
	return parserState;

};



bool isAdvanced(VIRTUALSERVO vs) {
//may be able to make this a member function of VIRTUALSERVO - it will increase struct size by a pointer var
	for (int a = 0;a < ASPECT_PARAMETER_SIZE * 4; a++) {
		if (vs.aspectParameters[a] != -1) return true;
	}
	return false;
}


/// <summary>
/// Default state, including boot (which sets MASstate=127) will be Tristate.  If MASstate appears in one of 
/// the parameter arrays, it will drive the pin active hi|low or flash variants thereof.
/// If MASstate is not a recognised code (on this pin) then the pin goes tristate.
/// </summary>
/// <param name="vs">target virtual servo</param>
/// <returns>the output state: 0 active low, 1 active hi, 2 tristate</returns>
uint8_t assertAspectState(VIRTUALSERVO vs) {
	/*All MAS pins will boot as power=off, i.e. tristate
	* The first MAS resolved code will set power=on and assert a hi|lo or flash variant thereof
	* Non resolved codes.  Will result in a tristate output.  If you don't want this, you need to 
	* map all MAScodes to the pin to describe a hi|lo behaviour.
	*/

	/*
	bool hasCommonCode = false;
	//first scan for at least one common code between [hi] and [lo]
	for (int a = 0;a< ASPECT_PARAMETER_SIZE - 1;a++) {  
		if (vs.aspectParameters[a] == -1) continue;
		if (hasCommonCode) continue;
		for (int c = ASPECT_PARAMETER_SIZE;c < (ASPECT_PARAMETER_SIZE * 2) - 1;c++) {
			if (vs.aspectParameters[c] == -1) continue;
			if (vs.aspectParameters[a] == vs.aspectParameters[c]) {
				hasCommonCode = true;
				break;
			}
		} 
	}

	//then scan for at least one common code between [hi-f] and [low-f]
	for (int a = 0;a < (ASPECT_PARAMETER_SIZE *3)- 1;a++) {  
		if (vs.aspectParameters[a] == -1) continue;
		if (hasCommonCode) continue;
		for (int c = ASPECT_PARAMETER_SIZE*3;c < (ASPECT_PARAMETER_SIZE *4) - 1;c++) {
			if (vs.aspectParameters[c] == -1) continue;
			if (vs.aspectParameters[a] == vs.aspectParameters[c]) {
				hasCommonCode = true;
				break;
			}
		}
	}
	*/

	
//order of precedence is flash low, flash high, assert low, asssert hi
/*use cases pin 2 addr 7 [hi 5] [low 5] [] []   will drive pin 2 hi/low but because both [hi] and [lo] contain same aspect code, its clear
* there is no tristate option for this pin, i.e. its always active
* pin 2 addr 7 [hi 3] [lo 4] means that there is a tristate case where some other code might be used on a separate in and in which case this
* pin will go tristate.
*/


	enum output {
		LO,
		HI,
		TRISTATE,
	};

	//note, arduino.h defines constants of LOW=0, HIGH=1 but no tristate
	

	uint8_t outputState = TRISTATE;
		

	for (int a = (ASPECT_PARAMETER_SIZE * 4) - 1;a > -1;a--) {
		switch (int(a / 8)) {
		case 3:
			//flash low
			if (vs.aspectParameters[a] == vs.MASstate) {
				//assert low gated with flash flag
				if (ledState) outputState = LO;
			}
			break;


		case 2:
			//flash high
			if (vs.aspectParameters[a] == vs.MASstate) {
				//assert low gated with flash flag
				if (ledState) outputState = HI;
			}
			break;

		case 1:
			//solid low
			if (vs.aspectParameters[a] == vs.MASstate) {
				outputState = LO;
			}
			break;

		case 0: //solid hi
			if (vs.aspectParameters[a] == vs.MASstate) {
				outputState = HI;
			}
			break;
		}

	}


	//assert the output, also process invert
	switch (outputState) {
	case TRISTATE:
		pinMode(vs.pin, INPUT);
		break;

	case LO:
		outputState = vs.invert ? HI : LO;
		pinMode(vs.pin, OUTPUT);
		digitalWrite(vs.pin, outputState==LO ? LOW : HIGH);
		break;

	case HI:
		outputState = vs.invert ?LO : HI;
		pinMode(vs.pin, OUTPUT);
		digitalWrite(vs.pin, outputState == LO ? LOW : HIGH);
		break;

	}

		return outputState;

}









/* aspect logic
* for 'a' we respond to addr and drive the pin hi/low there is no tri state
* for 'A'
* if [hi=7] and [lo=7] then we will actively drive pin based on Thrown =hi (unless invert)
* so it will be active hi|low, no tri state
* if [hi=7] and [lo=-1] then pin is active hi and tristate otherwise

So i can meld both a and A into the same struct, however if user is using a they don't want complexity of A
and also system needs to know whether to respond to turnout command or advanced-aspect

also for 'a' we don't use an aspect code.  so easiest way to know if something is A or a is to check if all elements of aspectPArams are -1
if they are, we are using 'a'







*/