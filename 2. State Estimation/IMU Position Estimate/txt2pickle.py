import pickle
import numpy as np
from types import SimpleNamespace

data = {
    'a': [],
    'w': [],
    'gps': [],
    't': [],
};
t = 0;
dt = 0.01;
# 000000.txt
# 000999.txt

for fn in range(0,999):
    with open("data/imu/000"+f'{fn:03}'+".txt", "r") as filestreamtwo:
        for line in filestreamtwo:
            currentline = (line.split(" "));
            currentline[-1] = currentline[-1].rstrip("\n");
            if(len(currentline)==7):
                data['a'].append(currentline[:3]);
                data['w'].append(currentline[3:6]);
                data['t'].append(t);
                t += dt;
t = 0;
for fn in range(0,999):
    with open("data/gps/000"+f'{fn:03}'+".txt", "r") as filestreamtwo:
        for line in filestreamtwo:
            currentline = (line.split(" "));
            currentline[-1] = currentline[-1].rstrip("\n");
            if(len(currentline)==3):
                data['gps'].append(currentline);
                t += dt;

data['a'] = np.array(data['a'])
data['w'] = np.array(data['w'])
data['t'] = np.array(data['t'])
data['gps'] = np.array(data['gps'])

n = SimpleNamespace(**data);

with open('onlineData-imu-gps.pkl', 'wb') as handle:
    pickle.dump(n, handle, protocol=pickle.HIGHEST_PROTOCOL)