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

Plot made: (plot_3.1.png) / Matlab file: (matlab.m)

## Ex4 3.2
Odometry implemeted by finding x, y and theta every time update\_odo is called. 
Formulas from "Where Am I?" p.20. 
Logging now prints the new values as well as the values from 3.1. 

Plot made: (plot_3.2.png) / Matlab file: (matlab.m)

## Ex4 3.3 
Speed limit of about 0.4~0.5 observed. 

Plot made: (plot_3.3) / Matlab file: (matlab.m)

## Ex4 3.4 
Acceleration limit implemented, tested for 0.2, 0.4, 0.6 m/s.
Anomaly observed, slope never ends up as steep as the original when slowly stepping up the speed. TA asked. 

Plot made: (plot_3.4) / Matlab file: (matlab.m)

## Ex4 3.5
Decceleration limit implemented tested for 0.2, 0.4, 0.6 m/s.
Nice slowdown observed at the end of the robot movement. 

Plot made: (plot_3.5) / Matlab file: (matlab.m)

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
9 zone distance values received as expected

## Ex5 Zoneobst with MRC
A script was made, logging $l0-8
Without a "wait" command it logs a handful of times before stopping
With "wait 1" it logs approx 100 times

## Ex5 Zoneobst with square
Square program was changed to save laser-server data to the output file.