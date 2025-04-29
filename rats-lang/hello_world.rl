rule! (var1
	testRat1 -> testRat2
)

rule! (var3
	testRat1, LOG <- testRat2
)

rule! (var4
	testRat1 == testRat2
)
