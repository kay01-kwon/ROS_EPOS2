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



Information on COB.
|Object	|COB-ID	    |PDO Mapping
|:---:|:---:|:---:|
|RxPDO1	|0x00000201	|Controlword (dlc = 2)
|RxPDO2	|0x00000301	|Controlword + Modes of Operation (dlc = 3)
|RxPDO3	|0x00000401	|Controlword + TargetVelocity (dlc = 6)
|RxPDO4	|0x00000501	|Controlword + TargetPosition (dlc = 6)
|TxPDO1	|0x00000181	|Statusword (dlc = 2)
|TxPDO2	|0x00000281	|ActualPosition (dlc = 4)
|TxPDO3	|0x00000381	|ActualVelocity (dlc = 6)
|TxPDO4	|0x00000481	|None


|Process Data Object|Data type|dlc|
|:---:|:---:|:---:|
|Controlword|Unsigned16| 4 times 4 bits = 2 times 8 bits (dlc = 2)
|Modes of Operation|Unsigned8| 2 times 4 bits = 1 times 8 bits (dlc = 1)
|TargetVelocity|Integer32|8 times 4 bits = 4 times 8 bits (dlc = 4)
|TargetPosition|Integer32|8 times 4 bits = 4 times 8 bits (dlc = 4)
|statusword|Unsigned16| 4 times 4 bits = 2 times 8 bits (dlc = 2)
|Modes of Operation Display|Unsigned8|2 times 4 bits = 1 times 8 bits (dlc = 1)
|ActualVelocity|Integer32|8 times 4 bits = 4 times 8 bits (dlc = 4)
|ActualPosition|Integer32|8 times 4 bits = 4 times 8 bits (dlc = 4)
