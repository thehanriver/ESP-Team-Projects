# Quest 5: Cruise Control
Authors: Vivek Cherian, Hussain Valiuddin, Mario Han

Date: 2020-11-30
-----

## Summary

For designing our cruise control rover, we implemented all the skills from this cluster into one rover.
We have a LIDAR sensor, speed sensor, and two IR sensors for left and right distance detection.

The LIDAR sensor is used to detect any thing infront of it. If there is anything closer than 50 cm the car will start slowing down and will stop the rover is less than 20 cm from the object.
The two IR sensors are used to detect anything that is on its sides and to steer accordingly. The IR sensors allow the car to remain in a steady straight course that should always be 25 cm away from the wall.
The speed sensor is used to detect wheel speed in m/s and is displayed on the alphanumeric display. The values from the speed sensor are used to keep the car at a speed of 0.5 m/s. The speed is calculated by seeing how many times the speed sensor detects the black on the 12 pattern encoder and measures the speed by multiplying rotations per second and the circumference of the wheel in meters to get meters per second.

The node server was copied from our last quest with many things taken out so it only has an two buttons(on and off) which is nothing special. It sends a response of 
1 or 0 to the esp to determine if the rover should start or stop.

PROBLEMS:
1) We weren't able to adjust PID settings to make the car stop before hitting the obstacle.
2) It detects the wall but doesn't slow down fast enough and ends up crashing.
3) Steering issues: steers left or right too hard.
4) Takes too long to stay on track.

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

We put all the sensors onto one bread board ontop of a cardboard square that is thoroughly taped and ziptied onto the rover. 
The LIDAR is pointed to the front and the IR sensors are pointed to the sides. The speed sensors are put close to the wheels and a 12 cycle encoder is taped to the inside of the wheel. The power bank is directly under the board as well.

INVESTIGATIVE QUESTION:
To maintain "adaptive" cruise control, we would need to track the speed of any object in front of us and adjust speed accordingly.

We tried to do this by maintaining a certain speed and slowing down once there was an obstacle in front of us. If we enter too close to the obstacle, we would wait for it to move out of the critical distance for our crawler to move at a certain speed. In doing so, if the obstacle was moving, we would ideally be moving at a constant speed and distance away from the obstacle.

## Sketches and Photos

IR sensors
![IMG_9449](https://user-images.githubusercontent.com/45515930/100826102-9d28d580-3427-11eb-991f-9bfe68825704.JPG)

Alphanumeric display for speed sensor
![IMG_9450](https://user-images.githubusercontent.com/45515930/100826104-9dc16c00-3427-11eb-831a-78d14effd66d.JPG)

Speed Sensor
![IMG_9451](https://user-images.githubusercontent.com/45515930/100826105-9dc16c00-3427-11eb-999b-eb0f4cdebe41.JPG)

LIDAR
![IMG_9448](https://user-images.githubusercontent.com/45515930/100826106-9e5a0280-3427-11eb-90c7-b8d153c97701.JPG)


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

[Optical Encoder](https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide#example-circuit)

[Encoder Template](http://whizzer.bu.edu/images/encoder.gif)

[Servo Control Ex](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control)


## References

[LIDAR data sheet](https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf)

[IR Range Finder Specs](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf)

[Ultrasonic Specs](https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf)

-----

