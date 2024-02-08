# LSDB CAN INTERFACE

## Input / Output

### Input Topics

From lsdb_interface

| Name            | Type                                   | Description    |
| :-------------- | :------------------------------------- | :------------- |
| ~/input/command | lsdb_msgs/msg/LsdbCommandStamped       | input commands |

### Output Topics

To lsdb_interface

| Name            | Type                                  | Description   |
| :-------------- | :------------------------------------ | :------------ |
| ~/output/status | lsdb_msgs/msg/LsdbStatusStamped | output status |

## ROS Parameter

| Name            | Type   | Description      | Default      |
| :-------------- | :----- | :--------------- | :----------- |
| input_command       | string | input topic name from lsdb_interface command     | command |
| output_status       | string | output status topic name to lsdb_interface       | status |
| input_can           | string | input topic name from can bus                    | /from_can_bus |
| output_can          | string | output topic name to can bus                     | /to_can_bus |
| can_device          | string | can device name                                  | can0 |
| node_id             | int    | node id of motor driver                          | 1 |
| trapezoidal_accel   | float  | trapezoidal accel parameter of lsdb motor driver | 2.0 |
| trapezoidal_decel   | float  | trapezoidal decel parameter of lsdb motor driver | 2.0 |
| status_loop_rate_hz | float  | period to get motor driver status                | 10.0 |

## CAN Communication

Transmitted data

- Motor Speed [rpm]

Received data

- Motor Speed status [rpm]
- Fault Flag status
