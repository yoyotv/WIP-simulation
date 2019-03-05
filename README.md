# WIP-simulation

There are two subtitles in this repository.

First one is balance.

The other one is path following.

Use CMAC, BELC and sliding control, three kinds of control method to control wheeled inverted pendulum to balance and path following.

Study base on [1], [2], [3] and [4] ,then modify the structure to accomplish the simulation.

## GET started

Success in :

Matlab2014a

### Try balance

* Model: PID --------------------
  ` Run " pid_balance.m "`

* Model: CMAC-----------------
 ` Run " cmac_balance.m "`

* Model: BELC-------------------
 `Run " BELC_balance "`

* Model: RBELC-----------------
 `Run " RBELC_balance.m "`
 
 If everything is fine, it should look like these.
 ![control_signal](https://raw.githubusercontent.com/yoyotv/WIP-simulation/master/balance/figures/control.jpg)
 ![angle](https://raw.githubusercontent.com/yoyotv/WIP-simulation/master/balance/figures/tilt_angle.jpg)
 
### Try path following

* Model: RCMAC-----------------
 ` Run " indoor_decoupled_ieee_good_track_2_hybird_rcmac_2_zc.m "`

* Model: RBELC-------------------
 `Run " indoor_decoupled_ieee_good_track_2_hybird_rbrain_2_zc_w.m "`
 
  If everything is fine, it should look like these.
  ![rbrain](https://raw.githubusercontent.com/yoyotv/WIP-simulation/master/path%20following/figure/rbrain_trajectory.jpg)
  ![rcmac](https://raw.githubusercontent.com/yoyotv/WIP-simulation/master/path%20following/figure/rcmac_trajectory.jpg)

### Scheme 

Feed sliding function as the controller input to balance the WIP. 

Use hybrid control (CMAC/BELC+Robust) to follow the route.

### Paramater

Learning rate of weight and variance, sliding function parameter

### References

[1] Y.  Kanayama,  Y.  Kimura,  F.  Miyazaki,  and  T.  Noguchi,  “A  stabletracking control method for an autonomous mobile robot,” inProc.IEEE International Conference on Robotics and Automation,May1990, pp. 384 – 389

[2] B. d’Andr ́ea-Novel, G. Bastin, and G. Campion, “Modeling andcontrol of non holonomic wheeled mobile robots,” inthe 1991 IEEEInternational Conference on Robotics and Automation, 1991, pp.1130–1135

[3] Dianwei Qian · Jianqiang Yi, "Hierarchical Sliding Mode Control for Underactuated Cranes"

[4] Xia, D.; Yao, Y.; Cheng, L. Indoor autonomous control of a two-wheeled inverted pendulum vehicle using ultra wide band technology. Sensors 2017, 17, 1401


