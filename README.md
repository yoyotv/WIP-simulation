# WIP-simulation
use CMAC, BELC and sliding control to balance wheeled inverted pendulum

Study base on [1],[2],[3] and [4]   and revise the structure to accomplish the simulation.

## GET started

Success in :

Matlab2014a

### Try

Model: PID
* Run " pid_balance.m "

Model: CMAC
* Run " cmac_balance.m "

Model: BELC
* Run " BELC_balance "

Model: RBELC
* Run " RBELC_balance.m "

### Scheme 

Feed sliding function as the controller input to balance the WIP.  

### Paramater

Learning rate of weight and variance, sliding function parameter

### References

[1] Y.  Kanayama,  Y.  Kimura,  F.  Miyazaki,  and  T.  Noguchi,  “A  stabletracking control method for an autonomous mobile robot,” inProc.IEEE International Conference on Robotics and Automation,May1990, pp. 384 – 389

[2] B. d’Andr ́ea-Novel, G. Bastin, and G. Campion, “Modeling andcontrol of non holonomic wheeled mobile robots,” inthe 1991 IEEEInternational Conference on Robotics and Automation, 1991, pp.1130–1135

[3] Dianwei Qian · Jianqiang Yi, "Hierarchical Sliding Mode Control for Underactuated Cranes"

[4] Xia, D.; Yao, Y.; Cheng, L. Indoor autonomous control of a two-wheeled inverted pendulum vehicle using ultra wide band technology. Sensors 2017, 17, 1401


