# Day 1

## Ex1 3.5.1
Robot moves approx. 1.5m forward (errors of 0.002m's) 

## Ex1 3.5.2
Robot moves approx. in a rectangle 

## Ex1 3.5.3 
Robot moves in a fluid motion. 

## Ex1 3.5.4
Robot follows straight black line as expected 

## Ex1 3.5.5
Robot follows right branch and passes the fork without a hitch. 
Robot is slightly offset to the left, since it uses its right sensors to follow the line 

## Ex1 3.5.6
Robot moves along the left branch, ignores the perpendicular black line. 
Robot is slightly offset to the right, since it uses its left sensors to follow the line 

## Ex1 3.5.7
Same as above, except stopping at the perpendicular black line 

## Ex1 3.5.8
Robot follows straight line until it comes within 0.4m's of obstacle and stops. 

## Ex2 3.1
Formulas are implemented in MatLab and give expected results 

## Ex2 3.2
Simulation was run, and large errors observed. 
These were used in our MatLab script, and Ed & Eb values were found. (Eb = ; Ed = ) 
Odometry was updated with zeroed coordinates, and got the following values: (0.26; 1, 0.0002~ ) 
We used these values and our Eb to recalibrate the real width between the wheels. 
After second execution of the simulation the square was much neater. 
Recalibration with the odometry-update function gave the same calibration data. 

# Day 2

## Ex4 2
Square program was run, turning around one wheel observed 
Square program modified, robot now turns around center point

## Ex4 3.1
Square program was modified to include logging 
This was done by having a staticly allocated array of logentry-structs that is added to once per loop-iteration.
The array is output to "output.dat" after the ms\_end is reached 

Plot made: (plot_3.1.png) / Matlab file: (ex_3_1.m)

## Ex4 3.2
Odometry implemeted by finding x, y and theta every time update\_odo is called. 
Formulas from "Where Am I?" p.20. 
Logging now prints the new values as well as the values from 3.1. 

Plot made: (plot_3.2.png) / Matlab file: (ex_3_2.m)

## Ex4 3.3
Speed limit of about 0.4~0.5 observed. 

Plot made: (plot_3.3) / Matlab file: (ex_3_3.m). 

## Ex4 3.4 
Acceleration limit implemented, tested for 0.2, 0.4, 0.6 m/s.
Anomaly observed, slope never ends up as steep as the original when slowly stepping up the speed. TA asked. 

Plot made: (plot_3.4) / Matlab file: (ex_3_4.m)

## Ex4 3.5
Decceleration limit implemented tested for 0.2, 0.4, 0.6 m/s.
Nice slowdown observed at the end of the robot movement. 

Plot made: (plot_3.5) / Matlab file: (ex_3_5.m)

## Ex4 3.6
Both limits are now applied to turning

## Ex3 2.1
Test scripts executed as expected

## Ex3 2.2
Odometry calibration succeeded after changing robots twice.
The first two were uncalibrateable in the selected area because they got so off course they left the area.

## Ex3 2.3
Worked fine both ways

# Day 3

## Ex5 Squareprog
Worked as expected

## Ex5 Zoneobst
9 zone distance values were received as expected

## Ex5 Zoneobst with MRC
A script was made, logging $l0-8
Without a "wait" command it logs a handful of times before stopping
With "wait 1" it logs approx 100 times

## Ex5 Zoneobst with square
Square program was changed to save laser-server data to the output file.
Inspecting the output file, many datapoints are logged, in a format similar to the zoneobst command.

# Day 4

## Ex6 2.1
SMRCL script was made to make 100 measurements, move forward 30cm and then make 100 more measurements.
Matlab script that is able to generate the linear calibration transformation was written.
First tested with simulator, then later with the actual SMR robot.

SMRCL script: (ex_6_2_1) / Matlab script: (linesensor_converter.m)

## Ex6 2.2
SMRCL script was made to make 100 measurements, and step forward in 10cm increments and taking 100 more measurements.
Matlab script that is able to approximate Ka & Kb from measured values and fit a function.
Thus far only run on measurements from the simulator. 

SMRCL script: (ex_6_2_2) / Matlab script: (irdist.m)

# Day 5

## Ex7 2
An angular controller was implemented, first with a K value of 10, leading to a far too fast acceleration.
The K was subsequently changed to 0.1, leading to a much smoother speed-up.
Plotting the x and y, the x and y displacement from 0 fluctuated to a maximum of 0.1 mm over the duration.
Plotting theta we saw a smooth increase in the angle, with curves at the beginning at the end, due to the implementation of acceleration and deceleration.

## Ex7 3.1
Very simple normalising function was implemened: val\*factor\+offset
This normalises the sensor value based upon the calibration values

## Ex7 3.2
Simply looping through the normalised sensor values and finding the lowest value, and returning the found values index.
This was expanded to also be able to find the highest value index, for finding a white line.
A function to determine if there was a line at all was also implemented.

## Ex7 3.3
Implementing the simple line following algorithm using the DeltaV = K * (DeltaLs). 
K was, through imperical testing, decided to be 0.005.
This line following was very rough, as the lowest sensor value will regularly change between the two middle sensors when driving straight.

## Ex7 3.4
The center of gravity algorithm was implemented directly.
x = {-7, -5 ... 5, 7}     positions of sensors on a line segment, offset from the center of the robot
I = normalised sensor measurements

## Ex7 3.5
The output of the above function was used instead of the lookup value from the above x-table.
K = 0.01 since the center of gravity algorithm gives a finer granularity, meaning we need to steer more aggressively.
Robot followed line in all test cases. 