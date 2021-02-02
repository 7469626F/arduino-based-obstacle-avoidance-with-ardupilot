# arduino-based-obstacle-avoidance-with-ardupilot

This project eplains how to set up an arduino with ardupilot so that it can send ranging data from any sensor that the arduino can read over mavlink.

# basics

For the basic obstacle avoidance to work ardupilot requires a minimum of one distance and a maximum of ten distances. Each distance is attached to one of the ten available directions. These include eight directions in the horizontal plane. Direction 0 is straight ahead then with each increment the heading changes 45° clockwise up to number 7. The 9th and 10th  direction are upwards(designated with 24) and downwards(designated with 25) respectively.
An arduino can be used to supply these distances over mavlink. The origin of these values is not important, any sensor that can be used with an arduino or even a virtual sensor is sufficient.

# ardupilot setup

Ardupilot needs to be setup in a way that it will use mavlink as a source fro the ranging data.

AVOID_ENABLE = 7     (all)

PRX_TYPE = 2     (mavlink)

I my case the arduino is hooked up to serial 2. Ay free serial port that has no will do the trick but avoid sbus specific serial ports since those may have a hardware inverter.
The baud rate needs to match the baud rate that is defined in the arduino script.

SERIAL2_PROTOCOL = 2     (mavlink2) 

SERIAL2_BAUD = 115     (115200)

# wiring

The arduino only needs two data wires in order to communicate with the flightcontroller.

Arduino TX goes to RX and the arduino RX obviously goes to the matching TX.

The used mavlink libraries don't suport softserial, this means that the flight controller has to be conected to a hardware serial port. Take this in consideration when you want to use a sensor that also requires a hardware serial port.

Al that's left now is a stable power supply for the arduino and a ground wire in between the arduino and the flight controller.
