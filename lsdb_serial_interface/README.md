# LSDB SERIAL INTERFACE

## Input / Output

### Input Topics

From lsdb_interface

| Name            | Type                                   | Description    |
| :-------------- | :------------------------------------- | :------------- |
| ~/input/command | lsdb_msgs/msg/LsdbCommandStamped | input commands |

### Output Topics

To lsdb_interface

| Name            | Type                                  | Description   |
| :-------------- | :------------------------------------ | :------------ |
| ~/output/status | lsdb_msgs/msg/LsdbStatusStamped | output status |

## ROS Parameter

| Name        | Type   | Description      | Default      |
| :---------- | :----- | :--------------- | :----------- |
| serial_port | string | serial port name | /dev/ttyUSB0 |

## Serial Communication

Transmitted data

- Motor Speed [rpm]
- Digital Output Control (connected to turn signal)

Received data

- Motor Speed status [rpm]
- Fault Flag status
- Battery Voltage status
- Digital Input status (connected to emergency stop button)
- Digital Output status (connected to turn signal)
