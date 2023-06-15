# speed and acc, decel
float32 speed             # input = speed [RPM]
int32 acceleration      # input = acceleration * 0.1 [RPM/s]
int32 deceleration      # input = deceleration * 0.1 [RPM/s]

# Digital Output Command
lsdb_msgs/DigitalOutCommand digital_output
