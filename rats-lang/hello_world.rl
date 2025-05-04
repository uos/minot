# make wind known
_wind = [ turbine_ros2 ]

# manually setting rats when starting without rules
_rats = [ testRat1, testRat2 ]

_start_locked = true

# define rules for rats
rule! (var1
	testRat1 -> testRat2
)

rule! (var3
	testRat1, LOG <- testRat2
)

rule! (var4
	testRat1 == testRat2
)
