# Quest 2: Tactile Internet
Authors: Hussain Valiuddin, Mario Han, Vivek Cherian

Date: 2020-10-08
-----

## Summary

This project uses all of the previous skills and adds them into one board. The code from Quest 1 (with three tasks) is used as a template for Quest 2's code. Instead of the three tasks being I2C, Servo, and Clock , we used it to calculate the necessary values of the thermistor, IR rangefinder, and ultrasonic sensors along with an additional task for sending the data to stdout. In trying to find a way to turn voltage into the sensors' respective values, each teammember was assigned to one sensor and its readings.

In addition to this, we use a python file (read_console.py) to take the output to stdout which we can get from demo.js (skill16) and write to/create a file called "sensors.csv" inside a data folder. Demo.js is taken from skill 16 and is used to get the ouput from the Serial Port and print on console instead of using flash monitor like we usually do using node. Index.js is taken from skill 17 where we start our own localhost:8080 to show the readings in a dynamic multi series chart. For this index.js and index.html specifically we used the example code taken from the BU-EC444 repo.

The dynamic array should take the lastest 20 values from the csv file every 1 second and rerender the chart.
This would be how the quest 2 would be run: 

  1: flash quest 2 but not monitor
  
  2: call python read_console.py to make a .csv file
  
  3: call node index.js (demo.js would've already been called from read_console)
  
  4: observe chart (open localhost:8080 in a browser)
  
The circuit itself is all the sensors added together. 
Thermistor: we used a 1k Ohm value in order to make sure the voltages are in range to read up to -10 C to 105 C and not exceed 3V. The reading is plugged into the ADC Pin A3 and uses 5V as source.
IR range finder: we use ADC Pin A2 to get readings and this uses 5V as source.
Ultrasonic: we used ADC Pin A4 to get readings and this uses 3V as source.

Problems:
For some reason,the graph crashes sometimes. Sometimes on safari it runs for 7 minutes and sometimes in crashes 30 seconds and we don't know the reason. Chrome seems to work fine but we haven't tested running the graph past 8 mins.

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
| Quality of solution | 5 |  5     | 
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
- [Link to video demo](https://drive.google.com/file/d/1YaCqCBOu767zFlsxw2To4wG5mpJmq6m9/view?usp=sharing). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

[Quest 1 Code](https://github.com/BU-EC444/Team2-Cherian-Han-Valiuddin/tree/master/quest-1/code/main)

[CSV example code](https://github.com/BU-EC444/code-examples/tree/master/DataLoadingTest)

## References

[Thermistor Specs](https://eaa.net.au/PDF/Hitech/MF52type.pdf)

[IR Range Finder Specs](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf)

[Ultrasonic Specs](https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf)

-----

