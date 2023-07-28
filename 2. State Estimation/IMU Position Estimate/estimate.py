import pickle
import numpy as np
import matplotlib.pyplot as plt

#### 1. Data ###################################################################################
with open('data/onlineData-imu-gps.pkl', 'rb') as file:
    data = pickle.load(file)

acc = np.array(data.a, dtype=float);
gps = np.array(data.gps, dtype=float);
omaga = data.w;
time = data.t;
g = [0, 0, 9.81];
direction = 2;


################################################################################################
# Calibrate Data 
# Data set is not valued due to movement of sensor 
################################################################################################
acc_cali = acc[100:400, direction];
H = np.zeros([acc_cali.shape[0], 1]) + 1;
acc_estimate = np.linalg.inv(H.T @ H) @ H.T @ acc_cali;
bias = acc_estimate - g[direction];
print(bias);
acc[:, direction] -= acc_estimate;



#### 2. Constants ##############################################################################
var_imu_a = 0.4;
var_gps = 0.9;
w = 0.8; # process no🇮🇲 

Q = np.eye(3)*w;
R_imu = np.eye(1)*var_imu_a;
R_gps = np.eye(1)*var_gps;

################################################################################################
# We can also set up some constants that won't change for any iteration of our solver.
################################################################################################
H_jac_imu = np.array([0, 0, 1]).reshape([1, 3]); 
H_jac_gps = np.array([1, 0, 0]).reshape([1, 3]);
L_jac = np.eye(3); # Additive Noise

### 3. Initial Values #########################################################################
################################################################################################
# Let's set up some initial values for our EKF solver.
################################################################################################
x_est = np.zeros([acc.shape[0], 3]) *100; # x, v_x, a_x cm
p_cov = np.zeros([acc.shape[0], 3, 3]) *100; # cm

x_est[0] = np.array([0, 0, 0]);
p_cov[0] = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 1]]);

def measurement_update(R, H_jac, p_cov_check, y_k, x_check):

    # Compute Kalman Gain 🍭 
    K = p_cov_check @ H_jac.T @ np.linalg.inv( H_jac @ p_cov_check @ H_jac.T + R);

    # Compute predicted states
    x_hat = x_check + K @ (y_k - H_jac @ x_check);

    # Compute Corrected Covariance 
    p_cov_hat = (np.eye(3) - K @ H_jac) @p_cov_check;

    return x_hat, p_cov_hat;

#### 5. Main Filter Loop #######################################################################
maxTime = len(time) - 0;
for k in range(1, maxTime):  # start at 1 b/c we have initial prediction from gt
    delta_t = time[k] - time[k - 1]
    
    F = np.array([
        [1, delta_t, delta_t**2/2],
        [0, 1, delta_t],
        [0, 0, 1]
    ]).reshape([3,3]);

    B = np.array([delta_t**2/2, delta_t, 1]).reshape([3, 1]);

    x_est_km = x_est[k-1].reshape([3,1]);

    # Update State (Prediction)
    Q = Q + np.eye(3)*w*delta_t;
    R_imu = R_imu - np.eye(1)*var_imu_a*delta_t**2;
    
    x_est_k = F @ x_est_km + B * x_est_km[direction];

    # Propagate uncertainty
    p_cov_k = F @ p_cov[k-1] @ F.T + L_jac @ Q @ L_jac.T;

    x_est_k, p_cov_k = measurement_update(R_imu, H_jac_imu, p_cov_k, acc[k, direction], x_est_k);
    x_est_k, p_cov_k = measurement_update(R_gps, H_jac_gps, p_cov_k, gps[k, direction], x_est_k);
    x_est[k] = x_est_k.T;
    p_cov[k] = p_cov_k;


p_cov_std = np.sqrt(np.diagonal(p_cov, axis1=1, axis2=2));

directionLabel = ['X', 'Y', 'Z'];

################################################################################################
# Let's plot the Data 
################################################################################################
gt_fig = plt.figure()
ax1 = gt_fig.add_subplot(311)
ax1.plot(data.t[:maxTime], acc[:maxTime,direction]);
ax1.plot(data.t[:maxTime], x_est[:maxTime,2]);
# Covariance
ax1.plot(data.t[:maxTime], 3*p_cov_std[:, 2], 'g--')
ax1.plot(data.t[:maxTime], -3*p_cov_std[:, 2], 'g--')
ax1.set_ylabel('Acce cm/s^2')
ax1.set_title(directionLabel[direction]+' Direction')
plt.legend(['measure', 'estimation'])


ax = gt_fig.add_subplot(312)
ax.plot(data.t[:maxTime], x_est[:maxTime,1]);
# Covariance
ax.plot(data.t[:maxTime], 3*p_cov_std[:, 1], 'g--')
ax.plot(data.t[:maxTime], -3*p_cov_std[:, 1], 'g--')
ax.set_ylabel('vel (cm)')
plt.legend(['estimation'])


ax2 = gt_fig.add_subplot(313)
ax2.plot(data.t[:maxTime], gps[:maxTime,direction]);
ax2.plot(data.t[:maxTime], x_est[:maxTime,0]);
# Covariance
ax2.plot(data.t[:maxTime], 3*p_cov_std[:, 0], 'g--')
ax2.plot(data.t[:maxTime], -3*p_cov_std[:, 0], 'g--')
ax2.set_xlabel('[time]')
ax2.set_ylabel('Position (cm)')
plt.legend(['measure','estimation'])
plt.show()


