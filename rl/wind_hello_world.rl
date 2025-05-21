_log_level = "debug"

# _rules = ./rules.rl
# _winds = ["turbine_ratpub", "pelorus"]

_bag.{
	lidar.{
		_short = l
		_topic = /ouster/points
		_type = cloud
	}
	imu.{
		_short = i
		_topic = /ouster/imu
		_type = Imu
	}
	# odom.{
	# 	_short = o
	# 	_topic = /odom
	# }
	# any.{
	# 	_short = a
	# 	_type = any
	# }
}

# relative to executing binary
reset! ./dlg_cut

pf! i 1
# pf! i, l 1
# pf! l 1
