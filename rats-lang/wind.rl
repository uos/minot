# bag file settings
_bag.{
	imu.topic = /ouster/imu
	lidar.topic = /ouster/points
}

# relative to executing binary
reset! ./dlg_cut

# start doing things with the bagfile
pf! il 1 var4
pf! l 1 var4


