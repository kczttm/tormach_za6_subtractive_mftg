import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import sys
from scipy.optimize import fminbound
from scipy import signal
sys.path.append('toolbox')
from lambda_calc import *
from utils import *

# R = 25.4 * 2
# H = 25.4 * 1
# W = 30
#38x89mm
data_path = 'data/wave/Curve_dense.csv'

spindle_diameter = 2.794  # mm


ylen = 50  # mm
xlen = 50  # mm

x_margin = spindle_diameter 

# one period will advance 1.5*d_spindle mm in y-dir
wave_freq = np.ceil(ylen / (1.2 * spindle_diameter))  # cycle on the workspace rounded up
print('number of spindle cycle:', wave_freq)

W = xlen/2  # amplitude of the wave
    

###generate curve for 1.5x3.5 parabola
def find_point(t):
    fr = wave_freq * 2* np.pi

    #x = (W+x_margin) * -np.cos(np.multiply(fr,t/xlen)) + W
    x = (W+x_margin) * -signal.square(np.multiply(fr,t/xlen)) + W
    y = t
    z = np.zeros(len(np.atleast_1d(t)))

    return np.vstack((x,y,z)).T

def find_normal(p):
    curve_normal = np.zeros((len(np.atleast_1d(p)),3))
    curve_normal[:,-1]=1
    return curve_normal


def distance_calc(t,p,step_size):
    p_next=find_point(t)
    return np.abs(step_size-np.linalg.norm(p-p_next))

def find_next_point(t,p,step_size):
    t_next=fminbound(distance_calc,t,t+step_size,args=(p,step_size))
    p_next=find_point(t_next)
    normal_next=find_normal(p_next)
    return t_next, p_next, normal_next


def main():
    
    t = np.linspace(0,int(ylen),1000)
    curve=find_point(t)
    curve_normal=find_normal(curve)

    ##############3D plots####################
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    ax.plot3D(curve[:,0],curve[:,1],curve[:,2],'r.-')
   
    plt.title('Curve 1')
    plt.show()

    ###################check curve normal##############################
    # curve_tan=np.gradient(curve,axis=0)
    # print(curve_tan)
    # for i in range(len(curve_tan)):
    #     print(curve_tan[i]@curve_normal[i])
    # plt.plot(np.diag(np.inner(curve_tan,curve_normal)))
    # plt.show()
    # lam=calc_lam_cs(curve)
    # diff=np.linalg.norm(np.diff(curve,axis=0),axis=1)
    # plt.plot(diff)
    # plt.show()

    ####################generate equally spaced points##########################
    num_points=2000
    lam=calc_lam_cs(curve)
    lam=np.linspace(0,lam[-1],num_points)
    curve_act=[curve[0]]
    curve_normal_act=[curve_normal[0]]
    t_act=[0]
    lam_act=np.linspace(0,lam[-1],num_points)
    for i in range(1,num_points):
        t_next, p_next, normal_next=find_next_point(t_act[-1],curve_act[-1],lam[i]-lam[i-1])
        curve_act.append(p_next.flatten())
        curve_normal_act.append(normal_next.flatten())
        t_act.append(t_next)

    curve_act=np.array(curve_act)
    curve_normal_act=np.array(curve_normal_act)

    DataFrame(np.hstack((curve_act,curve_normal_act))).to_csv(data_path,header=False,index=False)

    # diff=np.linalg.norm(np.diff(curve_act,axis=0),axis=1)
    # plt.figure()
    # plt.plot(diff)
    # plt.show()

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot3D(curve_act[:,0],curve_act[:,1],curve_act[:,2],'r.-')
    ax.quiver(curve_act[:,0],curve_act[:,1],curve_act[:,2],curve_normal_act[:,0],curve_normal_act[:,1],curve_normal_act[:,2],length=1, normalize=True)
    plt.show()

    visualize_curve_w_normal(curve_act,curve_normal_act,stepsize=10)


if __name__ == '__main__':
    main()