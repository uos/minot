_bag.{
	lidar.{
		_short = l
		_topics = /ouster/points
		_type = Pointcloud2
	}
	imu.{
		_short = i
		_topics = /ouster/imu
		_type = Imu
	}
	odom.{
		_short = o
		_topics = /odom
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

pf! [l, i], i 2

# start doing things with the bagfile
# pf! il 1 var4
# pf! l 1 var4



