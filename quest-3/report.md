# Quest 3: Hurricane Box with Remote Access
Authors: Mario Han, Vivek Cherian, Hussain Valiuddin

Date: 2020-10-26
-----

## Summary

 This project uses previous skills and quests to create a Hurricane Box. In this quest, we take the Accelerometer and thermistor sensory data and send it from the ESP32 to a raspberry pi. The data transfer takes place over a UDP socket between the ESP32 and the Pi. When the raspberry pi sends a response to the ESP about receiving its data packet, we sneak in the state of the on-board LED in the response. Using this state, the ESP turns the onboard LED on or off. This sensory data is shown by the raspberry pi on a local server. The raspberry pi also hosts another server to stream video feed from the camera connected to the pi. By implementing DDNS to our router and enabling port forwarding we are able to view this webpage from anywhere over the internet, and are no longer limited ot viewing it through the local host.

The end result is a web page which displays sensory information in 2 line charts, has a button to access the webcam and a button to toggle the on-board LED on the ESP.

How the Node Js server works
The NodeJS server hasnt changed much from Quest2. Button to control the LED and Button to access the webcam have been added


## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1/0 |  1     | //depending if scheduled time accounts for time extension
| Objective Seven | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 4 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 2 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design

In the code, we scheduled individual modules as tasks. At first, all I2C, UDP, ADC and wifi initializations are made. After the initializations, 5 tasks are started. These tasks functions are:

1. Setting up the udp client on the ESP to communicate with the raspberry pi's internal port 1234.
2. Thermistor task calculates temperature using input from ADC pin A3 on the ESP32
3. LED task toggles the onboard LED at pin 13 based on the message received by the ESP from the Pi
4. Adxl343 task reads the data using I2C communication from the accelorometer and converts it into acceleration
5. Print task just prints all of this data to the console on ESP for easier debugging.
   The Webcam is run on a seperate Node server which can be accessed by pressing the button link on the main webpage.

The raspberry pi recieves ESP32 data at its port 1234. The Nodejs application reads the incoming data and displays it on a webpage and sends back the LED status to the ESP. The

## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>

Board Pic

Console Pic (NodeJs receiving / ESP monitor)

Webpage

## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s



## Modules, Tools, Source Used Including Attribution

[UDP_client](https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client)

[Quest 2 code](https://github.com/BU-EC444/Team2-Cherian-Han-Valiuddin/tree/master/quest-2)

[Adafruit ADXL343 Accelerometer](https://github.com/adafruit/Adafruit_ADXL343)

[Wifi Station](https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station)

## References

[UDP client code Whizzer](http://whizzer.bu.edu/briefs/design-patterns/dp-sockets)

[Run NodeJs server on Raspberry Pi](https://desertbot.io/blog/nodejs-git-and-pm2-headless-raspberry-pi-install)

[Portable Video Streaming using Pi Camera](https://www.hackster.io/narender-singh/portable-video-streaming-camera-with-raspberry-pi-zero-w-dc22fd)

-----

