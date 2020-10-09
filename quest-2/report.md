# Quest 2: Tactile Internet
Authors: Hussain Valiuddin, Mario Han, Vivek Cherian

Date: 2020-10-08
-----

## Summary

This project uses all of the previous skills and adds them into one board. The code from Quest 1 (with three tasks) is used as a template for Quest 2's code. Instead of the three tasks being I2C, Servo, and Clock , we used it to calculate the necessary values of the thermistor, IR rangefinder, and ultrasonic sensors along with an additional task for sending the data to stdout. In trying to find a way to turn voltage into the sensors' respective values, each teammember was assigned to one sensor and its readings.

In addition to this, we use a python file (read_console.py) to take the output to stdout which we can get from demo.js (skill16) and write to/create a file called "sensors.csv" inside a data folder. Demo.js is taken from skill 16 and is used to get the ouput from the Serial Port and print on console instead of using flash monitor like we usually do using node. Index.js is taken from skill 17 where we start our own localhost:8080 to show the readings in a dynamic multi series chart. For this index.js and index.html specifically we used the example code taken from the BU-EC444 repo.

The dynamic array should take the lastest 20 values from the csv file every 1 second and rerender the chart.
Instructions on running this quest.

  1: navigate to ./code/ and run 'idf.py -p [port] flash' to build and flash but NOT monitor the project.
  
  2: from there, navigate to ../modules/ and run 'npm install' to install packages from package-lock.json
  
  3. now in the modules/ directory, run 'python read_console.py' to start outputting sensor values to console (which it does by executing demo.js) and writing to ./code/data/sensors.csv
  
  4: run 'node index.js' 
  
  5: navigate to localhost:8080 to view plot, localhost:8080/data to view all parsed csv elements, and localhost:8080/data/last to view the latest csv element.
  
The circuit itself is all the sensors added together. 
Thermistor: we used a 1k Ohm value in order to make sure the voltages are in range to read up to -10 C to 105 C and not exceed 3V. The reading is plugged into the ADC Pin A3 and uses 5V as source.
IR range finder: we use ADC Pin A2 to get readings and this uses 5V as source.
Ultrasonic: we used ADC Pin A4 to get readings and this uses 3V as source.

Problems:
By the time of the live demo, we were experienced issues where the server would crash after about 40 seconds, and lines were plotted twice for about the first 20. We have sinced fixed these problems. We changed the way that we plotted points by adding a /data/last path where we now pull the latest data point from. This significantly improved the issue with crashing as it was related to memory. We also found that the crashing was due to an issue with Safari. Currently, it runs for about 7 minutes and 30 seconds on Safari without crashing, so it is better than it was during the live demo, but we recommend using Chrome to avoid crashing altogether (Firefox should be fine as well since it doesn't have the same issue as Safari). 

Since the live demo, we also made our plot much easier to read by adding proper axes and labels to everything.

For the investigative question, we chose to use the IR rangefinder for thee robot car. It is more accurate, works for a larger range of values with less noise, where as the ultrasonic sensor seems to take in more noise and can take time to update to the accurate calue. In addition, the minimum range to take values for the IR rangefinder is about 20cm while the ultrasonic sensor has a minimum range to take values of about 40cm.

## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 4 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 2 |  3     | 


## Solution Design

For the code itself, we used the same format as Quest 1 in which where we use tasks. We use 3 tasks for caluclating sensor values and 1 task for printing. To display the data onto localhose:8080 we first made a python file that reads the console called console.py in order to make a csv file and read the console. This python script calls demo.js in order to capture the flash that is coming from the serial port. From that, index.js needs to be called using node to start localhost:8080.

## Sketches and Photos

Picture of Board:
![circuit](https://user-images.githubusercontent.com/45515930/95498616-14834100-0972-11eb-9490-fc985d68eaba.JPG)

Console:

<img width="458" alt="console-output" src="https://user-images.githubusercontent.com/45515930/95498702-37155a00-0972-11eb-9918-49a1778e4671.png">

Testing:
![testing](https://user-images.githubusercontent.com/45515930/95498770-50b6a180-0972-11eb-89cc-e43ff89fd266.JPG)

Graph:

<img width="1320" alt="plot" src="https://user-images.githubusercontent.com/45515930/95498831-63c97180-0972-11eb-8ce9-8ef47f791f4f.png">

## Supporting Artifacts
- [Link to video demo](https://drive.google.com/file/d/1_FcqQhSHdW0oKBtw2r8R6hQpk2EmkIxH/view?usp=sharing). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

[Quest 1 Code](https://github.com/BU-EC444/Team2-Cherian-Han-Valiuddin/tree/master/quest-1/code/main)

[CSV example code](https://github.com/BU-EC444/code-examples/tree/master/DataLoadingTest)

## References

[Thermistor Specs](https://eaa.net.au/PDF/Hitech/MF52type.pdf)

[IR Range Finder Specs](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf)

[Ultrasonic Specs](https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf)

-----

