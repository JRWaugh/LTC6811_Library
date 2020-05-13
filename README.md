# LTC6811 Library
This is a reimagining of the library found here: https://github.com/analogdevicesinc/Linduino/tree/master/LTSketchbook/libraries/LTC681x

The purpose of this library is to:
- Remove all unncessary copying between arrays
- Remove "out parameters", and most parameters in general. Users of this library are not responsible for allocating the correctly sized arrays and passing them into functions.
- Remove as many side-effects in functions as possible. Functions should do what they say they're doing. 
- Remove the burden of having to go through all the correct steps to get readings from the LTC6811 sensor.
