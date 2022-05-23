# uwr-class-rov-control

This is an Arduino program I wrote for my underwater robotics class in high school. This code is given out to students that take the course, so it is heavily commented.

This program reads the position of two joysticks and uses that data to move two horizontal thrusters and a vertical thruster. The thrusters this program moves are meant to be controlled using Bipolar MOSFET Amplifiers (BMAs).

## The circuit
- X axis of first analog joystick connected to analog input 1
- Y axis of first analog joystick connected to analog input 0
- Y axis of second analog joystick connected to analog input 2
- Positive signal wire of left BMA connected to digital pin 9
- Negative signal wire of left BMA connected to digital pin 10
- Positive signal wire of right BMA connected to digital pin 5
- Negative signal wire of right BMA connected to digital pin 6
- Positive signal wire of vertical BMA connected to digital pin 11
- Negative signal wire of vertical BMA connected to digital pin 3
