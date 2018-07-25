# BallRun
This is my master thesis on figuring how AI playing marble run.

It is highly related to the assigment from the 
CMU RI 16-745: Dynamic Optimization
http://www.cs.cmu.edu/~cga/dynopt/ass2/

and inspired by Leonid and Alex : https://github.com/leonidk/Box2D/ (thousand thanks !)

The simulation is based on ODE : http://www.ode.org/
To run the simulation and the optimiter, you have to install ODE on your computer first.

In the folder opt : there is the optimitzer file, change the number in main.py to change the number of obstacles to play
                    The dynamics.exe file is an executable file without visualization for the dynamic environment.
                    
In the folder sim : there's the simulator, sim.exe, needs ODE compiled on the computer first, and put the setup file with name "dyn_setup.txt" in the subfolder params, then can call ./sim in the terminal. You can also play around with the parameters in the environment by changing them in the sim.cpp file, then use the MAKEFILE to compile it.

In the folder dynamics : there's the environment simulator without visualization, which is used by the optimitzer. You can play around the paramters in dynamics.cpp to change the physical environment.

*** How to use ****

After compiled ODE on the PC, we can start to install the python dependencies 
  numpy, scipy, multimethod, glob
  
After all these dependencies stuffs, move to the opt folder, simply type : python main.py

**** Fun with parameters ****

In the ./opt/main.py, the parameter n passed to the method optimize.get_layout(n), is for given obstacles added to the game board. 
The number n has to be at least 3, that the goal object is consists of 3 cubes... (so far it is tested with 3 or 4 )

In the file ./opt/optimize.py : 
line 629 - line 634 : choose your favourite optimization algortihm
Each algorithm except DE (differential evolution) has some parameters to trim.

line 414, 416 : two different setups, choose one to use.

line 114 - 116 : positions of the additional fixed obstacle on board.


