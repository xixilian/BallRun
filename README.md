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
