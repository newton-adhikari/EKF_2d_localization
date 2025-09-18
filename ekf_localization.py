import numpy as np
import matplotlib.pyplot as plt

def motion_model(x, u, dt):
    theta = x[2]
    v, omega = u
    x_new = x[0] + v * dt * np.cos(theta)
    y_new = x[1] + v * dt * np.sin(theta)
    theta_new = theta + omega * dt
    return np.array([x_new, y_new, theta_new])

def jacobian_F(x, u, dt):
    theta = x[2]
    v = u[0]
    return np.array([
        [1, 0, -v * dt * np.sin(theta)],
        [0, 1,  v * dt * np.cos(theta)],
        [0, 0, 1]
    ])

def measurement_model(x, landmark):
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]
    r = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx) - x[2]
    return np.array([r, phi])

def jacobian_H(x, landmark):
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]
    q = dx**2 + dy**2
    r = np.sqrt(q)
    return np.array([
        [-dx/r, -dy/r, 0],
        [dy/q, -dx/q, -1]
    ])

def ekf_update(x_pred, P_pred, z, landmark, R):
    # Expected measurement
    z_hat = measurement_model(x_pred, landmark)

    # Innovation
    y = z - z_hat
    y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # normalize angle

    # Measurement Jacobian
    H = jacobian_H(x_pred, landmark)

    # Innovation covariance
    S = H @ P_pred @ H.T + R

    # Kalman Gain
    K = P_pred @ H.T @ np.linalg.inv(S)

    # Updated state
    x_new = x_pred + K @ y

    # Updated covariance
    P_new = (np.eye(len(x_pred)) - K @ H) @ P_pred

    return x_new, P_new

def plot_state(x, P, landmarks, z=None):
    plt.clf()
    ax = plt.gca()

    # Fixed grid view
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    # Plot robot
    plt.plot(x[0], x[1], 'bo', label='Robot')

    # Uncertainty ellipse
    cov = P[0:2, 0:2]
    eigvals, eigvecs = np.linalg.eig(cov)
    angle = np.arctan2(eigvecs[1,0], eigvecs[0,0])
    ellipse = plt.Circle((x[0], x[1]), np.sqrt(eigvals[0]), fill=False, edgecolor='blue', linestyle='--')
    ax.add_patch(ellipse)

    # Landmarks
    for i, lm in enumerate(landmarks):
        plt.plot(lm[0], lm[1], 'rx')
        plt.text(lm[0]+0.1, lm[1]+0.1, f"LM{i}", color='red')
        if z is not None:
            plt.plot([x[0], lm[0]], [x[1], lm[1]], 'g--', alpha=0.5)

    # Annotate robot
    plt.text(x[0]+0.2, x[1]+0.2, f"θ={np.round(x[2],2)}", color='blue')

    plt.legend()
    plt.grid(True)
    plt.title("EKF Localization")
    plt.pause(0.01)


# Initial state
x = np.array([2.0, 3.0, np.pi / 4])
P = np.eye(3) * 0.5
landmark = [5.0, 5.0]
R = np.diag([0.5, 0.1])  # measurement noise
Q = np.eye(3) * 0.1      # process noise
u = np.array([1.0, 0.1])
dt = 0.1
trajectory = []

plt.figure()

for t in range(50):
    # Predict
    x_pred = motion_model(x, u, dt)
    F = jacobian_F(x, u, dt)
    P_pred = F @ P @ F.T + Q

    # Simulate noisy measurement
    z = measurement_model(x_pred, landmark) + np.random.multivariate_normal([0, 0], R)

    # Update
    x, P = ekf_update(x_pred, P_pred, z, landmark, R)

    # Plot with measurement line
    plot_state(x, P, [landmark], z=z)

    trajectory.append(x[:2].copy())
    plot_state(x, P, [landmark], z=z)
    plt.plot(*zip(*trajectory), 'k-', alpha=0.3)  # draw path
    
    print(f"Step {t}:")
    print(f"Predicted State: {np.round(x_pred, 2)}")
    print(f"Updated State:   {np.round(x, 2)}")
    print(f"Uncertainty:\n{np.round(P, 2)}\n")

plt.show()