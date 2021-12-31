# ROS_EPOS2_CAN_TEST
Profile Velocity Mode

Msg info: std_msgs/Int32

Topic to subscribe: /TargetVel

Topic to publish: /ActualVel

CAN Data histogram result

Subscribe topic info : TargetVel 100 Hz

1. Target Velocity Discrete Time histogram result
<img src="epos2_test/picture/histogram_TargetVel.jpg">

2. Actual Velocity Discrete Time histogram result
<img src="epos2_test/picture/histogram_ActualVel.jpg">

3. t - RPM graph
<img src="epos2_test/picture/t_RPM.jpg">

4. Code Modified

When a function is destroyed, declared variable in the function, also, is perished, which means that if you declare a variable in a function, the memory assigned for the variable would be destroyed frequently.

This makes memory efficiency decrease.

Therefore, you should declare the data to read or write in private to prevent the memory from destroying constantly.
