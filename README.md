Nano based servo driver for DCC model railroad.  Assigns pins 3-12 as servos (i.e. 10 servos). You program them via serial for DCC address,
for swing range and invert.  You can command the pin directly from serial to closed|thrown|neutral|Toggle. Neutral will set
the servo to 90 degress (its midpoint) which is useful in setting up the mechanics.  Max range is a full 180 degrees.

Each servo will respond to its DCC address, which is held as the explicit address 1-2048 
one or more servos can be assigned the same DCC address.  useful for crossovers along with invert feature.

Remember that two or more servos can share the same control signal.  If you do this, you need to be mindful of the mechanical set up
because they will all move in the same direction.

Software now supports signal aspects, giving a solid logic 1|0 for thrown|closed or the inverted version of this.

HARDWARE notes:
The unit is powered from the track via a rectifier and 1000uF capacitor.
pin 2 is the DCC signal.  Enable pullup, as I am using a rectifier to produce 12v power and a pulldown diode 
in series with a 10k resistor as the DCC signal take-off.
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
the point set against it and usually derails.   Ain't no fixing that except with your hand!  Why not keep it simple and just
power this unit from the track.


COMMANDS:
SERVO SET-UP command is: s pin,addr,swing,invert,[continuous]
pin is 4-12, addr is 1-2048, swing is 5-180 and invert is 0|1 where 1 inverts the direction. Optional continuous (default 0) will keep drive
active after movement has ceased.

ASPECT SET-UP command is: a pin,addr,invert,[ignorePower]
pin is 4-12, addr is 1-204, invert is 0|1 and optional ignorePower (default 1) will ignore DCC power parameter and instead is always powered on.

PIN command is: p pin, c|t|n|T
pin is 4-12, c|t|n|T correspond to closed, thrown, neutral, Toggle.x
Once a pin is mapped to a DCC address, it will respond to that DCC turnout.

RATE command is: r pin,rate
pin is 4-12, rate is -10 to +10 where negative values retard rate of servo rotation.  Command has no effect on aspects.

DCC EMULATION command is:d addr t|c|T|n, [power]
Simulates a DCC command from the controller. A dcc controller itself can only send t|c, whereas T|n are for our convenience when setting up
providing Toggle and neutral.  Optional power will power/depower a signal aspect

DUMP command is: x, this will dump current settings to serial

VERBOSE command is: v, this will dump incoming DCC messages to serial.  reverts to verbose OFF at boot.

routes are not supported 


