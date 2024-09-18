# Sea-Turtle-Nest-Monitor
The Arduino-based software component of a system to monitor sea turtle nests and notify interested parties 
# Hardware
* Accelerometer
* Time-of-Flight sensor
* Microphone x3
* Temperature and Humidity sensor
* Water Level sensor x3
* Photoresistor
* Arduino Uno R3

# What it does
After the Arduino wakes up, it runs the required setup for each attached sensor. Then, it initializes both software and hardware serial. The software serial is for communicating with a Raspberry Pi while the hardware serial is used for the sensors.

In the loop section of the Arduino code, the Arduino constructs a Json document containing the latest data from each sensor. At the end of each iteration in the loop, it'll send this data over software serial to a Raspberry Pi for processing and uploading.
