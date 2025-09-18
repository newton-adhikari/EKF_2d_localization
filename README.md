# EKF Localization IN 2D

A Python project demonstrating robot localization using the **Extended Kalman Filter (EKF)**. This project models a differential drive robot navigating in a 2D plane, estimating its position and orientation based on noisy motion and sensor measurements relative to a fixed landmark.

---

## 📌 Features

- Differential drive robot motion model with control inputs for velocity and angular rate
- Range and bearing measurement model to a known landmark
- Analytical Jacobians for motion and measurement models
- Full EKF cycle: prediction (motion update) and correction (measurement update)
- Uncertainty propagation via covariance matrices
- Limited 2D visualization of Robot trajectory and Pose:

---

## 🧠 Core Concepts

- **State Vector**: \(\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}\) — robot’s 2D position and heading angle  
- **Control Inputs**: \(\mathbf{u} = \begin{bmatrix} v \\ \omega \end{bmatrix}\) — linear and angular velocities  
- **Measurement Model**: noisy range and bearing to a static landmark  
- **Jacobians**: matrices \(F\) and \(H\) for linearizing nonlinear motion and measurement models  
- **Covariance Matrix**: uncertainty representation updated at each step  
- **Kalman Gain**: weights prediction and measurement to produce optimal estimate  
- **Uncertainty Ellipse**: graphical depiction of pose covariance  

---

## 📊 Visualization

The simulation uses `matplotlib` to display:

- Current robot pose as an arrow indicating orientation  
- Covariance ellipse around the robot’s position to visualize uncertainty  
- Landmark position as a fixed point  
- Lines from robot to landmark representing measurements  
- Full trajectory history of the robot’s estimated positions  

---

## 🧪 How It Works

1. **Initialization**  
   Set initial robot state, covariance matrix, landmark coordinates, and noise parameters.

2. **Prediction (Motion Update)**  
   Use the control inputs and the motion model to predict the new robot state and propagate uncertainty.

3. **Measurement Simulation**  
   Generate noisy range and bearing measurements to the landmark based on the predicted state.

4. **Correction (Measurement Update)**  
   Compute the Kalman Gain and update the robot’s state and covariance with measurement information.

5. **Visualization Update**  
   Plot the updated state, covariance ellipse, and trajectory in real time.

---

### Prerequisites

Need to have Python 3.6+ installed. Then install the required packages:

```bash
pip install numpy matplotlib
```
##  Running the Project

```bash
python ekf_localization.py
```

---

##  Project Structure
/
├── ekf_localization.py      # Main simulation script
└── README.md                # This file


---

## 📚 References

Probabilistic Robotics by Sebastian Thrun, Wolfram Burgard, and Dieter Fox
Wikipedia: Extended Kalman Filter
Robotics tutorials and documentation on EKF localization


Feel free to copy-paste! Let me know if you want me to add or tweak anything else.

