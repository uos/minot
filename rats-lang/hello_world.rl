_log_level = "info"

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

_wind_file = ./wind.rl

# make wind known
# _wind = [ turbine_ros2 ]

# manually setting rats when starting without rules
# _rats = [ testRat1, testRat2 ]

# _start_locked = true

# define rules for rats
# rule! (var1
# 	testRat1 -> testRat2
# )

# rule! (var3
# 	testRat1, LOG <- testRat2
# )

# rule! (var4
# 	testRat1 == testRat2
# )
