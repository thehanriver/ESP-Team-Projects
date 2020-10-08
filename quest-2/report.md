# Quest 2: Tactile Internet
Authors: Hussain Valiuddin, Mario Han, Vivek Cherian

Date: 2020-10-08
-----

## Summary

This project uses all of the previous skills and adds them into one board. The code from Quest 1 (with three tasks) is used as a template for Quest 2's code. Instead of the three tasks being I2C, Servo, and Clock , we used it to calculate the necessary values of the thermistor, IR rangefinder, and ultrasonic sensors along with an additional task for printing. In trying to find a way to turn voltage into the sensors' respective values, each teammember was assigned to one sensor and its readings.

In addition to this, we use a python file (read_console.py) to rewrite and make a file called "sensors.csv" inside a data folder that listens to Demo.js. Demo.js is taken from skill 16 and is used to get the ouput from the Serial Port and print on console instead of using flash monitor like we usually do using node. Index.js is taken from skill 17 where we start our own localhost:8080 to show the readings in a dynamic multi series chart. For this index.js and index.html specifically we used the example code taken from the BU-EC444 repo.

The dynamic array should take the lastest 20 values from the csv file every 1 second and rerender the chart.
This would be how the quest 2 would be run: 
  1: flash quest 2 but not monitor
  2: call python read_console.py to make a csv file
  3: call node index.js (demo.js would've already been called from read_console)
  4: observe chart
  
The circuit itself is all the sensors added together. 
Thermistor: we used a 1k Ohm value in order to make sure the voltages are in range to read up to -20 C to 105 C and not exceed 3V. The reading is plugged into the ADC Pin A3 and uses 5V as source.
IR range finder: we use ADC Pin A2 to get readings and this uses 5V as source.
Ultrasonic: we used ADC Pin A4 to get readings and this uses 3V as source.
  
## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One |  |  1     | 
| Objective Two |  |  1     | 
| Objective Three |  |  1     | 
| Objective Four |  |  1     | 
| Objective Five |  |  1     | 
| Objective Six |  |  1     | 
| Objective Seven |  |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution |  |  5     | 
| Quality of report.md including use of graphics |  |  3     | 
| Quality of code reporting |  |  3     | 
| Quality of video presentation |  |  3     | 


## Solution Design



## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References

-----

