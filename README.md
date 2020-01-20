# Flatland simulator right "Now"

Create a multi-threaded interactive simulation of a vehicle in a simplified world
with text-mode and keyboard interfaces. Please provide gcc makefile and doxygen style
code documentation in english.
## World:
- The whole universe is flat and has 80x25 characters.
- The map of the world is loaded as plain text from file.
- ‘#’ objects are considered as immovable rigid walls.
- ‘>’ ‘<’ ‘v’ ‘^’ objects represent force vectors of “nature”.
- ‘~’ is a movable object that absorbs kinetic energy.
- ‘x’ is the origin where the vehicle shall be placed.
- Newtonian laws of physics apply.
## Vehicle:
- It is a multi character construct consisting of a head and a tail. (e.g.“o-”) The tail has a heading visualization purpose only so collisions shall be calculated on the head part.
- It has two differentially driven “wheels”.
- It can rotate in place by 45 degrees (= 1 turn).
- It’s linear and angular acceleration / deceleration is fixed. (1u/s)
- It can achieve a maximum linear speed of 3 grids per second.
- It’s maximum angular velocity is 1 turn per second.
- Reversing is allowed.
## Interactions:
- The vehicle is controlled by “wasd” buttons, where ‘w’ is forward
acceleration, ‘a’ is rotating left.
- When a button is pressed the vehicle accelerates, on it’s release it
decelerates.
- Hitting the '~' movable object is allowed - its mass equals the
vehicles. On collision the ‘~’ absorbs the vehicle's kinetic energy and
moves until decelerated.
- '~' pushed against a wall becomes immovable.
- Hitting a wall the vehicle is trashed.
- If the head part of the vehicle collides with an arrow ‘>’ symbol the
vehicle gains its maximum speed in the direction the symbol is pointing.
Forces acting sideways or diagonal to course apply as angular
velocities. (e.g. perpendicular to course, the vehicle is forced to turn
into the opposite direction )\
## “Drive Assistant”
In the above framework, create an agent that disregards user commands if there would
be a fatal collision on course and warns the user to slow down if there is no user
interaction but the event would occur due to current vehicle velocities.
