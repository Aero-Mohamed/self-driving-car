"""
    Kinematic Bicycle Model
    Inputs: [Forward Velocity (v), Steering Rate (omega=delta_dot)]
    State: [x, y, theta(heading angle), delta(steering angle)]
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22
        
        self.sample_time = 0.01
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

    def step(self, v, w):
        # ==================================
        #  Implement kinematic model here
        # ==================================
        if w > 0:
            w = min(w, self.w_max)
        else:
            w = max(w, -self.w_max)
        
        self.beta = np.arctan(self.lr * np.tan(self.delta)/self.L)
        
        self.xc += (v*np.cos(self.theta + self.beta)*self.sample_time)
        self.yc += (v*np.sin(self.theta + self.beta)*self.sample_time)
        self.theta += ((v/self.L) * (np.cos(self.beta) * np.tan(self.delta))) * self.sample_time 
        self.delta += w * self.sample_time


model = Bicycle()
model.sample_time = 0.01
time_end = 30


t_data = np.arange(0,time_end,model.sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

# ==================================
#  Square Path: set w at corners only
# ==================================
#w_data[670:670+100] = 0.753
#w_data[670+100:670+100*2] = -0.753
#w_data[2210:2210+100] = 0.753
#w_data[2210+100:2210+100*2] = -0.753
#w_data[3670:3670+100] = 0.753
#w_data[3670+100:3670+100*2] = -0.753
#w_data[5220:5220+100] = 0.753
#w_data[5220+100:5220+100*2] = -0.753

# ==================================
#  Spiral Path: high positive w, then small negative w
# ==================================
#w_data[:] = -1/100
#w_data[0:100] = 1

# ==================================
#  Wave Path: square wave w input
# ==================================
#w_data[:] = 0
#w_data[0:100] = 1
#w_data[100:300] = -1
#w_data[300:500] = 1
#w_data[500:5700] = np.tile(w_data[100:500], 13)
#w_data[5700:] = -1


radius = 8;
v_data[:] = 2*(2*np.pi*8)/30
delta = np.arctan(model.L/radius);

for i in range(t_data.shape[0]):   
    if i <= t_data.shape[0]/8:
        if model.delta < delta:
            w_data[i] = 0.9*model.w_max
        else:
            w_data[i] = 0;
            
    elif i <= 5*t_data.shape[0]/8:
        if model.delta > -delta:
            w_data[i] = -model.w_max;
        else:
            w_data[i] = 0;
    else:
        if model.delta < delta:
            w_data[i] = 0.9*model.w_max
        else:
            w_data[i] = 0;
    

    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(v_data[i], w_data[i])
    
plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.legend()
plt.show()