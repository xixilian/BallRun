import optimize as op

if __name__ == "__main__":

	import argparse
	opt = op.Optimizer()


	# the number has to be at least 3, that the goal object is consists of 3 cubes
	opt.get_layout(4)
	
	
