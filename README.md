# Lane Keeping Assist System
Model Predictive Control (MPC) based Lane-Keeping Assist (LKA) system using a Kinematic Bicycle Model, validated against ISO 11270 standards.
## Key Features

* **Kinematic Vehicle Modeling:** Formulated state-space equations using a two-wheel Kinematic Bicycle Model tracking longitudinal velocity, lateral position error, and heading angle deviation.
* **Model Predictive Control (MPC):** Formulated an online optimization controller minimizing lateral deviation and control effort (steering angle input) across a defined prediction horizon.
* **Perception Integration:** Simulates Velodyne VLP-16 LiDAR point-cloud processing to identify lane boundaries and extract curvature parameters.
* **ISO 11270 Compliance:** Evaluated lateral acceleration limits, time-to-line crossing (TLC), and smooth override transitions under standard test maneuvers.