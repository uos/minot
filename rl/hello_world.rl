_log_level = "info"

# _rules = ./rules.rl

_bag.{
	lidar.{
		_short = l
		_topic = /ouster/points
		# _type = cloud
	}
	imu.{
		_short = i
		_topic = /ouster/imu
		# _type = Imu
	}
	odom.{
		_short = o
		_topic = /odom
	}
	any.{
		_short = a
		_type = any
	}
}

# relative to executing binary
reset! ./../../bags/dlg_cut

# pf! i, l 1
pf! a 1
