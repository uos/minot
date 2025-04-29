# Wind settings (where they publish)
<- ./wind_ros2.rl

reset! ./dlg_cut

_bag.{
	imu.topic = /ouster/imu
	lidar.topic = /ouster/points
}

_wind = [ turbine_ros2 ]

pf! l 1 var1
