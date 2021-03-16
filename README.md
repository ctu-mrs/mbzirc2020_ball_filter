# Filtration and estimation for the ball catching challenge of MBZIRC2020

Implementation filtration and estimation algorithms for the LiDAR-based detection of an MAV with a suspended ball for the MBZIRC2020 competition, as described in the paper [1].
Other related packages:
 * the detection algorithm: [`uav_detect`](https://github.com/ctu-mrs/uav_detect) [1]
 * the planning algorithm: [`ball_planner`](https://github.com/ctu-mrs/mbzirc2020_ball_planner) [1]
 * the MRS UAV control and estimation pipeline: [`mrs_uav_system`](https://github.com/ctu-mrs/mrs_uav_system) [2]

### To launch simulation, detection, localization and visualization:
See the [`ball_planner`](https://github.com/ctu-mrs/mbzirc2020_ball_planner) package to launch the simulation.

----
References:
 * [1]: M. Vrba, Y. Stasinchuk, T. Báča, V. Spurný, M. Petrlík, D. Heřt, D. Žaitlík and M. Saska, "Autonomous Capturing of Agile Flying Objects using MAVs: The *MBZIRC 2020* Challenge", submitted to the IEEE Transactions on Systems, Man and Cybernetics - Systems 2021.
 * [2]: T. Báča, M. Petrlík, M. Vrba, V. Spurný, R. Pěnička, D. Heřt and M. Saska, "The MRS UAV System: Pushing the Frontiers of Reproducible Research, Real-world Deployment, and Education with Autonomous Unmanned Aerial Vehicles", eprint arXiv: 2008.08050, August 2020 (https://arxiv.org/abs/2008.08050).
