OVERSHOOT CONTROLLER README:

This file includes the type of controller, the gains I used, and the starting configuration as well as a copy of the final code for this simulation.

The best controller I used was a PI-feed forward controller with the following gains:
K_p = 1*np.eye(6)
K_i = 3*np.eye(6)

The starting configuration I used is listed below:
starting_q = np.array([0,.05,np.pi/3,0,.1,.1,.1,0,0,0,0,0])
