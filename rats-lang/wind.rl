# Wind settings (where they publish)
<- ./wind_ros2.rl

_bag.{
	imu.topic = /ouster/imu
	lidar.topic = /ouster/points
}

reset! ./dlg_cut

# pf! il 2 10s # check if there are really only 9 msgs before the first 2 lidar frames

pf! il 2 10s
