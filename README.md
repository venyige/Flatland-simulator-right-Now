# Flatland simulator right "Now" 
## Links: 
[ Assignment ](https://venyige.github.io/Flatland-simulator-right-Now/assignment.html) 

[ Further notes and considerations ](https://venyige.github.io/Flatland-simulator-right-Now/misc/notes.pdf) 

[ Doxygen generated ](https://venyige.github.io/Flatland-simulator-right-Now/html/index.html) 
## Scene file
The file is to be read line by line, one line in file corresponds a line of the 80X25 screen. 
A minimal scene file is a text file containing a sole "x" or "X" character. This minimal scene file is being considered as empty 80X25 scene, with car position at the upper-left corner. 
The empty grid points can be denoted by spaces, or for better readability by dots "." or plus signs "+". 
Any character other than "x", "X", "<", "^", ">",  "v",  "#" or "~" being evaluated as empty space. 
Empty line being considered as line full of spaces. Lines shorter than 80, being padded with spaces. Missing lines of the full set of 25 being considered as empty lines. 
Lines longer than 80 being truncated. 
In the 80X25 range the file must contain one and only one "x" or "X" character denoting the car position. 
If car position is missing, the program quits with error message: "The car position is missing from scene file". 
In the 80X25 range of the scene file only one "x" or "X" allowed, if more than one found, the program quits with error message: "Only one car position is allowed". 
## Control
To make it really physical, the human control should be implemented in a quasi real-time manner as e.g. a joystick unit. In order to gain a WASD control in "joystick" mode, I had to utilize the X11 library features. The other alternative was to get access to the low-level keyboard files of Linux, but it requires "sudo" access that nobody wants to grant to a third-party executable. 
 
## Rotation implementation
However the linear velocity can be adjusted in a continuous manner, the rotation speed is not similarly simple case. Why? Because even adjusting the direction angle at any arbitrary value (case B), the visualization takes place through a low resolution grid, so the controller would have no visual feedback about the accurate direction of the vehicle, so accidental collisions would occur, or the guardian agent’s warnings would cause confusion having conflict with the visual sense.
 
In the other case the rotation manifests only in predefined grades namely 0, 45, 90, 135 etc… (case A) so the visual feedback is fitted to the visualization of the underlying physical phenomenon (in this case not quite physical).
So I decided to consider both version with their advantages and drawbacks.
The continuous rotation happens in both cases, but takes effect in different ways.
 
Case A: As the car can proceed only in the predefined directions, the rotation snaps at them by the rules of floating point rounding of the angle divided by ```pi/4```. At the last snapping, if the rotation speed is not enough to bring it to the next named direction, the difference get lost from the last snapped direction as soon as the decelerating rotation eventually stops, so the car can proceed at "clear" directions. But if the rotating keys (a or d) having pressed during the not yet stopped previous rotation, the rotation gets accelerated/decelerated from its value at that point in time. 
Case B: The car can proceed in any arbitrary directions, that is the grid get hit in an "aliased" manner, so proceeding in some low angle with a main axis, there would be several unit long way at the actual main axis until transfers one unit sideways. 
 
Note: "Case B" is deliberately omitted, because of 1. lack of time, 2. from the description "It can rotate in place by 45 degrees (= 1 turn)." it is not exactly as prescribed. 

## Prerequisites

```sudo apt-get install cmake libx11-dev libncurses5-dev libncursesw5-dev doxygen``` 

## Build and run 
 
```$mkdir build&&cd $_``` \
```$cmake ..``` \
```$make$./flsrn /path/to/scene/file``` 
 
To display Driver Assistant warnings and logs in a second terminal: 

```$tail -f /tmp/car_log.txt``` 


