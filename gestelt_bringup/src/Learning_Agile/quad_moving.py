## this file helps quadrotor traverse through a moving narrow window
## including kalman filter, coodinate transformation, get the solution
import sys
sys.path.append('../')
import numpy as np
from solid_geometry import *
from quad_model import*

class kalman:
    def __init__(self,velo_devia):
        A = np.array([[1,0.1],[0,1]])
        E = np.array([0,0.1])
        C = np.array([1,0])
        sw = velo_devia
        R1 = np.matmul(sw*E.T,sw*E)

        Kf = np.zeros(2,60)
        K = np.zeros(2,60)
        P = np.zeros(2,2,60)
        Pm = np.zeros(2,2,60)

        Pm[:,:,0] = np.array([[1e5,0],[0,1e5]])
        for i in range(60):
            Kf[:,i] = np.matmul(np.matmul(Pm[:,:,i],C.T),1)
        self. X = velo_devia

    def v_es(self, position):
        return position

def input_cal(quad_state, final_point, gate_x):

    inputs = np.zeros(23)
    inputs[13] = magni(gate_x.gate_point[0,:]-gate_x.gate_point[1,:])
    gate_pitch = atan((gate_x.gate_point[0,2]-gate_x.gate_point[1,2])/(gate_x.gate_point[0,0]-gate_x.gate_point[1,0]))
    ##==calculate the gate RM
   
    rot=R.from_euler('zyx',[0,gate_pitch,0])
    inputs[14:23]=rot.as_matrix().flatten()
    
    inputs[0:10] = gate_x.transform(quad_state)
    inputs[10:13] = gate_x.t_final(final_point)

    return inputs, -gate_pitch

def binary_search_solver(model,device, quad_state, final_point, gate1, velo, w ):
    velo = np.array(velo)

    t_guess = magni(gate1.centroid-quad_state[0:3])/1
    
    t1 = t_guess
    gate_x = Gate(gate1.translate_out(velo*t1))
    gate_x.rotate_y(w*t1)

    inputs,_ = input_cal(quad_state, final_point, gate_x)
    
    outputs = model(torch.tensor(inputs, dtype=torch.float).to(device)).to('cpu')
    outputs = outputs.data.numpy()
    t2 = outputs[-1]

    while abs(t2-t1)>0.1:
        t1 += (t2-t1)/2
        gate_x = Gate(gate1.translate_out(velo*t1)) # prediction of the gap future position
        gate_x.rotate_y(w*t1)

        inputs,_ = input_cal(quad_state, final_point, gate_x)
        
        outputs = model(torch.tensor(inputs, dtype=torch.float).to(device)).to('cpu')
        outputs = outputs.data.numpy()
        t2 = outputs[-1]
    
    return t1
