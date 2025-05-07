_bag.{
	lidar.{
		short = l
		topic = /ouster/points
	}
	imu.{
		short = i
		topic = /ouster/imu
	}
	# imu.topic = /ouster/imu
	# lidar.topic = /ouster/points
}
# bag file settings
_bag.{
	imu.topic = /ouster/imu
	lidar.topic = /ouster/points
}

# relative to executing binary
reset! ./../../bags/dlg_cut

pf! m ..1s 3s

# start doing things with the bagfile
# pf! il 1 var4
# pf! l 1 var4



