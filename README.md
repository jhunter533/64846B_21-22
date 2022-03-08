# 64846B_21-22
Code for the tipping point season

Code includes turn PID system developed the year prior.
This turn PID allows for the robot to slow down as it approaches its target angle, allowing it to not overshoot. In addition it allows the robot to correct not only in the case of undershooting but also overshooting. This is thanks to the absolute heading used with help of the inertial sensor.

Includes a second drive PID system. Allows robot to slow down approaching target distance inputed in feet while checking in ticks to ensure better accuracy. Functions much the same as the turning PID. This PID was developed rather quickly in a handful of weeks so it will require more testing to hammer out bugs and efficiency issues for the next season.

Includes coding for autonomous buttons allowing selection of 8 or more programs for 15 second and 1 minute autonomous.

To fix:
Fully comment through code for the future. Ensure formatting is consistent and easy to follow.

FOR NEXT YEAR: 
I would like to not only ensure both PIDs allow custom parameter and are more flexible but add odometrey through use of tracking wheels. Allow autonomous selection menu to include more than 8 buttons through selection of more than one button at a time. This could allow more flexible programming and pre-load placements.

USE: This code is not to be reproduced or used by anyone else.
