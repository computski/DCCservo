// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       DCCservo.ino
    Created:	24/01/2026 12:46:37 PM
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

	HARDWARE notes:
	The unit is powered from the track via a rectifier and 1000uF capacitor.
	pin 2 is the DCC signal.  Enable pullup, as I am using a rectifier to produce 12v power and a pulldown diode 
	in series with a 1k resistor as the DCC signal take-off.
	pins 3 to 12 are servo drivers (10)
	pin 13 is the built-in LED, you could disable this and use as a servo output.  I use it as a heartbeat indicator
	the servo code can handle 12 servos on most boards (incl nano)
	neutral position is 90 degrees.
	Closed is defined as minimum position, Thrown as maximum (unless invert = true)
	The unit only actively drives the servo whilst executing a movement command, once the movement is complete, it disables
	the servo.  This stops servo chatter, but does assume the servo can hold its position unpowered.
	   
	On eBay there are low cost <Nano expansion board> for sale.  These have 12V DC power jack and lots of header pins which
	are ideal for servos as they support signal, 5v and gnd.  Its straightforward to plug the nano into one of these and then
	just jack in the servos.  the 12v rectifier goes to the power socket and the dcc signal to pin 2

	Note there is no need to use an opto isolator if you intend to power this unit from the track. There is one minor niggle
	with powering it off the track; if you run a loco into a turnout which is set against it, a short circuit results. The
	command station will cut track power and now you are unable to 'correct' the turnout position.  If the turnout board is driven
	from a separate DC supply (and opto isolator used) then you could set the turnout correctly and re-establish track power.
	BUT first this assumes you have a DCC command station with a control bus separate from the power bus, an second it overlooks
	the situation where many locos have keep alives fitted, so even when track power is cut, the loco carries on and rides over
	the point set against it and usually derails.   Ain't no fixing that except with your hand!  So why not keep it simple and just
	power this unit from the track.


	COMMANDS.  All commands must be followed with newline:
	SETUP command for servos is
	s pin,addr,swing,invert,[continuous]
	pin is 4-12, addr is 1-2048, swing is 5-180 and invert is 0|1 where 1 inverts the direction.  
	optional [continuous] 1|0 will ensure continuous servo drive (or not). Default is not.

	SETUP command for signal aspects is
	a pin,addr,invert
	Note that setting up [a] against a pin will override [p] and vice-versa

	EXECUTE command is
	p pin, c|t|n|T, [power]
	pin is 4-12, c|t|n|T correspond to closed, thrown, neutral, Toggle
	[power] is optional and can be 1|0. It only affects signal aspects, not servo turnouts.

	Once a pin is mapped to a DCC address, it will respond to that DCC turnout or aspect.

	DUMP command is x, this will dump current settings to serial.
	x
	pin 3  address 0  swing 25  invert 0  continuous 0 pointer good
	pin 4  address 0  swing 25  invert 0  continuous 0 pointer good
	pin 5  address 0  aspect invert 0



	DCC emulation command, a dcc controller itself can only send t|c, whereas T|n are for our convenience when setting up
	d addr, t|c|T|n, [power]
	[power] is optional. 1|0 will set the power parameter to on or off for the given pin.
	

	routes are not supported 

	Note on signal aspects:  A pin output can be logic 1|0 based on the throw position.  The pin can also be tristated if power is zero.
	i.e. you could have 1= red, 0= white and if power=0 both aspects are off.



	Note: this unit does not support CV based set up over DCC.  There is no point.  It is easy to set the unit up over
	serial, and once done you would rarely change it.  DCC is used only to command the turnouts to throw.
	
	possible FUTURES:
	Signal aspect control. a pin,addr,invert\n  i.e. a for aspect.  it wil respond to the power on/off commands from DCC, throw/clear will be its digital state
	i.e. throw sets 1 (unless inverted), clear sets 0.    power on/off is kinda redundant in the spec because it assumes two outputs one for 'thrown' one for 'clear'
	and then you can activate/deactivate each.  I wonder how Panel Pro deals with it?  It kinda means that a single address actually can support two outputs or aspects, and then
	each aspect can be driven on/off.

	Add direct pushbutton control.
	build lighting effects, say on/off control for night lighting/ fires/ welding

	Why not use arcomora software?  It won't fit into a nano with a standard bootloader.
	I also considered it too complex to set up.  Also its not clear if it can support the simple 
	solution used here where the nano is plugged direct into a low cost expansion board.


	https://www.arduino.cc/reference/en/libraries/servo/
	https://github.com/nzin/arduinodcc/tree/master/arduinoSource/dccduino
	https://rudysmodelrailway.wordpress.com/2015/01/25/new-arduino-dcc-servo-and-function-decoder-software/
	https://www.arcomora.com/mardec/



	2026-02-09 CHECK.  we are using aspects now also.  therefore I don't want to use a serverDriver, but what does this mean for converting the pin to a 
	regular servo, we need to reattach the driver, right? 

	
*/

#include <DCC_Decoder.h>
#include <Servo.h>
#include <EEPROM.h>
#include <NmraDcc.h>


#define TOTAL_PINS 10
#define BASE_PIN 3

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


/*EEprom and version control*/
struct CONTROLLER
{
	long softwareVersion = 20260209;  //yyyymmdd captured as an integer
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
	ASPECT_CLOSED
};

struct VIRTUALSERVO {
	uint8_t pin;
	uint16_t address;
	uint8_t swing;
	bool invert;
	bool continuous;
	bool power;
	bool isServo;  //servo or aspect
	uint8_t state;
	uint8_t position;
	Servo* thisDriver;
};


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
	Serial.println("boot\n");
	pinMode(LED_BUILTIN, OUTPUT);


	//DCC, setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
	//for my circuit, dcc is active low through a pulldown diode and a series 5k1k res, hence need the pull-up
	//for the nano, int 0 is on pin 2
	Dcc.pin(0, 2, 1);

	// Call the main DCC Init function to enable the DCC Receiver
	Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);
	Serial.println("DCC Init Done");
	Serial.println("Commands are s p x d a");

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

// This function is called whenever a normal DCC Turnout Packet is received and we're in Board Addressing Mode
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower)
{
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
	Serial.print("notifyDccAccTurnoutOutput: ");
	Serial.print(Addr, DEC);
	Serial.print(',');
	Serial.print(Direction, DEC);
	Serial.print(',');
	Serial.println(OutputPower, HEX);

	
	//strictly, since we are controlling servos, we should ignore any command with OutputPower==0


	//act on the data, finding all matching virtualservos with the given dcc address
	for (auto& vs : virtualservoCollection) {
		if (Addr != vs.address) continue;
			//2020-08-31 execute for EVERY instance of this DCC address,not just the first

			//take action. 0 is closed, 1 thrown
			if (Direction == 0) {
				vs.state = vs.isServo ? SERVO_TO_CLOSED : ASPECT_CLOSED;
			}
			else {
				vs.state = vs.isServo ? SERVO_TO_THROWN : ASPECT_THROWN;
			}
			vs.power = OutputPower;

			Serial.print("dcc command to pin: ");
			Serial.println(vs.pin, DEC);
						
	}

	//2020-11-05 process virtual routes.  for these we need to match on pins and issue commands to those
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

		//for signal aspects, we move from SERVO_CLOSED or SERVO_THROWN straight to the antiphase, and we pay heed to the .power variable

		for (auto& vs : virtualservoCollection) {
			uint8_t maxPosition = vs.swing + 90;
			uint8_t minPosition = 90 - vs.swing;

			switch (vs.state) {
			case SERVO_NEUTRAL:
				vs.position = 90;
				if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
				break;
			case SERVO_TO_CLOSED:
				//swing toward minPosition, unless invert is true
				if (vs.invert) {
					vs.position += vs.position < maxPosition ? 1 : 0;
				}
				else {
					vs.position -= vs.position > minPosition ? 1 : 0;
				}

				if ((vs.position == maxPosition) || (vs.position == minPosition)) {
					vs.state = SERVO_CLOSED;
				}

				if (!vs.thisDriver->attached()) vs.thisDriver->attach(vs.pin);
				break;

			case SERVO_TO_THROWN:
				//swing toward maxPosition unless invert is true
				if (vs.invert) {
					vs.position -= vs.position > minPosition ? 1 : 0;
				}
				else {
					vs.position += vs.position < maxPosition ? 1 : 0;
				}

				if ((vs.position == maxPosition) || (vs.position == minPosition)) {
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
				if (vs.power) {
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
				if (vs.power) {
					//actively drive the pin. Thrown is a high state unless invert is active
					pinMode(vs.pin, OUTPUT);
					digitalWrite(vs.pin, vs.invert ? LOW : HIGH);
				}
				else {
					//pin to tri-state
					pinMode(vs.pin, INPUT);
				}
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
						//aspect. Immediately go to closed state with power on
						vs.power = true;
						vs.state = ASPECT_CLOSED;
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
						Serial.print("pin booted");
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

		//ASPECT set-up command. Usage: a pin,addr,invert
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
					vsParse.pin = atoi(pch);
					//valid pin range is BASE_PIN to BASE_PIN + TOTAL_PINS
					if ((vsParse.pin < BASE_PIN) || (vsParse.pin >= BASE_PIN + TOTAL_PINS)) {
						i = 10;  //error code
						Serial.println("bad pin");
					}
					break;
				case 2:
					vsParse.address = atoi(pch);
					//if out of range 1-2048 then throw an error
					if ((vsParse.address > 2048) || (vsParse.address == 0)) {
						i = 10;
						Serial.println("bad address");
					}
					break;

				case 3:
					vsParse.invert = atoi(pch) == 0 ? false : true;
					break;
				}

				++i;
				pch = strtok(NULL, " ,");

			}
			if (i==4) {
				Serial.println("OK");
				//match to a pin member of servoslot and copy it over  
				for (auto &vs : virtualservoCollection) {
					if (vs.pin == vsParse.pin) {
						vsParse.thisDriver = vs.thisDriver;
						vs = vsParse;  //copy over from vsParse
						if (vs.thisDriver->attached()) vs.thisDriver->detach();
						//write to EEPROM
						bootController.isDirty = true;
						putSettings();
						exit;
					}
				}

			}else{
				Serial.println("bad command. usage a pin,addr,invert");
			}

		}


		//SERVO set-up command. Usage: s pin, addr, swing, invert, [continuous]
		if (receivedChars[0] == 's') {
			vsParse.isServo = true;
			vsParse.continuous = false;  //default setting

			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			while (pch != NULL) {
				switch (i) {
				case 1:
					vsParse.pin = atoi(pch);
					//valid pin range is BASE_PIN to BASE_PIN + TOTAL_PINS
					if ((vsParse.pin < BASE_PIN) || (vsParse.pin >= BASE_PIN + TOTAL_PINS)) {
						i = 10;  //error code
						Serial.println("bad pin");
					}
					break;
				case 2:
					vsParse.address = atoi(pch);
					//if out of range 1-2048 then throw an error
					if ((vsParse.address > 2048) || (vsParse.address == 0)) {
						i = 10;
						Serial.println("bad address");
					}
					break;
				case 3:
					vsParse.swing = atoi(pch);
					if (vsParse.swing > 90) {
						i = 10;
						Serial.println("bad swing range");
					}
					break;
				case 4:
					vsParse.invert = atoi(pch) == 0 ? false : true;
					break;
				case 5:
					//this param is optional
					vsParse.continuous = atoi(pch) == 0 ? false : true;
					break;
				}
				//zero is the s char

				++i;
				pch = strtok(NULL, " ,");

			}

			if ((i == 6) || (i==5)) {
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
						exit;
					
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
					p = atoi(pch);
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
						vsPointer = (VIRTUALSERVO*) &vs;
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

			if ((i == 3)||(i==4)) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage p pin,t|c|n|T,[power]");
			}

		}

		/*
		//EMULATE a dcc command.  This will affect multiple servos/aspects for a given dcc address
		//This version of the code calls notifyDccAccTurnoutOutput, so can only support thrown|closed
		//usage: d addr,t|c,[power]
		if (receivedChars[0] == 'd') {

			char* pch;
			int i = 0;
			pch = strtok(receivedChars, " ,");
			int p = -1;
			int address = -1;
			uint8_t isThrown = 0;
			uint8_t power = 0;

			while (pch != NULL) {

				switch (i) {
				case 1:
					//resolve address
					address = atoi(pch);
					if ((address < 1) || (address > 2048)) {
						i = 10;
						Serial.println("bad address");
						p = -1;
						break;
					}
					//address valid, use this to match to individual servos
					break;

				case 2:
					//we can only support thrown|closed in the call to notifyDccAccTurnoutOutput
					isThrown = pch[0] == 't' ? 1 : 0;
					break;

				case 3:
					power = pch[0] == '1' ? true : false;
					break;

				}

				++i;
				pch = strtok(NULL, " ,");
			}//while
				if ((i == 3) || (i == 4)) {
					notifyDccAccTurnoutOutput(address, isThrown, power);
					Serial.println("OK");
				}
				else
				{
					Serial.println("bad command. usage d address,t|c,[power]");
				}
			
		}
		*/


		//EMULATE a dcc command.  This will affect all servos/aspects on a given dcc address
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
					address = atoi(pch);
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

			if ((i == 3)||(i==4)) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage d address,t|c|T|n,[power]");
			}

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


		//DUMP all servo/aspect information
		if (receivedChars[0] == 'x') {

			for (auto vs : virtualservoCollection) {
				//dump this pin
				if (vs.isServo){
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
			
				}
				else {
					//dump signal aspects
					Serial.print("aspect pin ");
					Serial.print(vs.pin, DEC);
					Serial.print("  address ");
					Serial.print(vs.address, DEC);
					Serial.print("  invert ");
					Serial.print(vs.invert, DEC);
					Serial.print("  power ");
					Serial.print(vs.power, DEC);
				
				}
				
				if (vs.thisDriver == nullptr) {
					Serial.print(" pointer bad");
				}
				else
				{
					//dump output state
					switch (vs.state) {
					case ASPECT_THROWN:
					case SERVO_THROWN:
					case SERVO_TO_THROWN:
						Serial.print(" thrown");
						break;
					default:
						Serial.print(" closed");
						break;
					}
				
				}
				Serial.print("\n");
			}
		}
	}//newData

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
	EEPROM.get(eeAddr, bootController);
	if (m_defaultController.softwareVersion != bootController.softwareVersion) {
		/*If software version has changed, we need to re-initiatise eeprom with factory defaults*/
		Serial.println("restore factory defaults");
		EEPROM.put(0, m_defaultController);
		eeAddr += sizeof(m_defaultController);

		/*use pin 4 onward. set defaults*/
		int p = BASE_PIN;
		for (auto& s : virtualservoCollection) {
			s.pin = p;
			s.invert = 0;
			s.position = 90;
			s.swing = 25;
			s.continuous = 0;
			s.state = SERVO_BOOT;
			s.power = false;
			s.isServo = true;
			++p;
		}
		/*write back default values*/
		EEPROM.put(eeAddr, virtualservoCollection);
	}

	/*either way, now populate our structs with EEprom values*/
	eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.get(eeAddr, virtualservoCollection);

	/*initialise the pin assignments 4 onwards move all servos to closed position*/

	/*2020-10-29 seems little point moving all servos to mid point then moving all to closed.  will just boot
	as closed*/

	int p = BASE_PIN;
	int i = 0;

	for (auto& s : virtualservoCollection) {
		s.pin = p;
		s.state = SERVO_BOOT;
		//s.position = 90;  //neutral
		/*minimum useful swing is 5 degrees*/
		if ((s.swing < 5) || (s.swing > 90)) s.swing = 5;

		//calculate closed posn (we may be inverted) then back off 5 degrees and set that as posn
		if (s.invert) {
			//max position
			s.position = 90 + s.swing - 5;
		}
		else {
			//min position, normal for closed
			s.position = 90 - s.swing + 5;
		}


		//initialise the servo driver
			//servoDriver[i].attach(p);
			//servoDriver[i].write(s.position);
			//2020-11-09 we don't want to attach at this time as it will asert an unhelpful position
		servoDriver[i].detach();
		s.thisDriver = &servoDriver[i];
		++p;
		++i;
	}
	Serial.print("\nsofware version ");
	Serial.print(bootController.softwareVersion, DEC);

	Serial.println("\n.............\n");

}


//putSettings writes to EEPROM. to reduce wear only call if a change has been made
void putSettings(void) {
	int eeAddr = 0;
	if (bootController.isDirty == false) { return; }
	EEPROM.put(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.put(eeAddr, virtualservoCollection);
	Serial.println("putSettings");
	bootController.isDirty = false;
}

