#alike simple wall following alorithm

heading=0
if min_dist <2:
	turn left 45
	heading+=45
	if min_dist <2:
		turn right 90
		heading-=45
		if min_dist <2:
			turn right 45
			heading-=90
			if min_dist <2:
				turn left 180
				heading+=90
			else:
				move forward
		else:
			move forward
	else:
		move forward
else:
	move forward
