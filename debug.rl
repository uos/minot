#--- bag_init
_bag.{
  cloud.{
    _topic = "/ouster/points"
    _short = "l"
    _type = Cloud
  }
  imu.{
    _topic = "/ouster/imu"
    _short = "i"
    _type = Imu
  }
}
_start_locked = false
reset! ./../../bags/crash_bag

# rules = ./rules.rl
# _winds = [ "turbine_ros1" ]
#---

#--- tick
pf! [i, l], l 1 tick_end
pf! l 1 tick_end
pf! i 1 tick_end
#---

pf! i ..1s 1. # my imu init

# pf! [i, l] 9800ms..12s 2.
pf! [i, l] 9800ms..9850ms 2.
pf! [i, l], l 1
pf! l 1
pf! i 1
pf! [i, l] ..1100ms 4s

#--- init
pf! [i, l] 156s..157s 1.0
pf! [i, l], l 1
pf! l 1
pf! i 1
