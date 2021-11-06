# ROS2 BNO055 Driver
Driver written in C++

## Params
### Units
It is highly adviced to use the default values to adher to [REP 103](https://www.ros.org/reps/rep-0103.html).

| Param | Values | Description |
|:----- |:------ |:----------- |
|`unit.angular_velocity`|`"rps"`|radians per second|
||`"rps"`|radians per second|
|`unit.linear_acceleration`|`"mps2"`|meter per second squared|
||`"mg"`|milli force|
||`"g"`|force|
|`unit.magnetic_field`|`"T"`|tesla|
||`"uT"`|micro tesla|
|`unit.temperature`|`"C"`|celsius|
||`"F"`|fahrenheit|

## Data sheet
- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf