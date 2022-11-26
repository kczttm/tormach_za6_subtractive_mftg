import numpy as np
np.set_printoptions(suppress=True)
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../toolbox/')
from robot_def import *
from utils import *
from lambda_calc import *
sys.path.append('../path_gen/')
from constraint_solver import *

def pose_opt(robot,curve,curve_normal):

    ###get curve center and best place to center in robot frame
    C=np.average(curve,axis=0)
    p=robot.fwd(np.zeros(len(robot.joint_vel_limit))).p
    #arbitrary best center point for curve
    p[0]=3*p[0]/4
    p[-1]=p[-1]/2
    ###determine Vy by eig(cov)
    curve_cov=np.cov(curve.T)
    eigenValues, eigenVectors = np.linalg.eig(curve_cov)
    idx = eigenValues.argsort()[::-1]   
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]
    Vy=eigenVectors[0]
    ###get average curve normal
    N=np.sum(curve_normal,axis=0)
    N=N/np.linalg.norm(N)
    ###project N on to plane of Vy
    N=VectorPlaneProjection(N,Vy)

    ###form transformation
    R=np.vstack((np.cross(Vy,-N),Vy,-N))
    T=p-R@C

    return H_from_RT(R,T)

def curve_frame_conversion(curve,curve_normal,H):
    curve_base=np.zeros(curve.shape)
    curve_normal_base=np.zeros(curve_normal.shape)

    for i in range(len(curve_base)):
        curve_base[i]=np.dot(H,np.hstack((curve[i],[1])).T)[:-1]

    #convert curve direction to base frame
    curve_normal_base=np.dot(H[:3,:3],curve_normal.T).T

    return curve_base,curve_normal_base

def find_js_qp(robot,curve,curve_normal):
    ###find all possible inv solution for given curve
    opt=lambda_opt(curve[:,:3],curve_normal,robot1=robot,steps=len(curve))

    # ###get R first 
    # R_init=direction2R(curve_normal[0],-curve[1]+curve[0])


    # ###get all possible initial config
    # try:
    # 	print(curve[0],R_init)
    # 	q_inits=np.array(robot.inv(curve[0],R_init))
    # except:
    # 	print('no solution available')
    # 	return

    q_inits=[np.zeros(6)]
    curve_js_all=[]
    for q_init in q_inits:
        q_out=opt.single_arm_stepwise_optimize(q_init)

        if len(q_out)==len(curve):
            #if solution found
            break

    return q_out

def base_correction(H,N):
    ### account for the slight tilting of the working surface
    p1 = H[:3,2]
    p2 = N
    k1 = H[:3,0]
    k2 = H[:3,1]
    q_solve = subproblem2(p1,p2,k1,k2)
    q1, q2 = q_solve[0]
    
    R1 = rot(k1, q1)
    R2 = rot(k2, q2)
    R_corrOpt_opt = R2.T@R1
    H[:3,:3] = R_corrOpt_opt@H[:3,:3]
    # print(R_corrOpt_opt)
    return H

def pose_opt_wRef(robot,curve,curve_normal,N,p_ref):
    ###get curve center and best place to center in robot frame
    C=np.average(curve,axis=0)
    p=robot.fwd(np.zeros(len(robot.joint_vel_limit))).p
    #arbitrary best center point for curve
    p_x = (p_ref[1,:]+p_ref[0,:])/2
    p_y = (p_ref[2,:]+p_ref[0,:])/2
    d = p_ref[1,:].dot(N) # solving ax1+bx2+cx3 = d where N = [x1,x2,x3]
    # p[0]=3*p[0]/4
    # p[-1]=p[-1]/2
    p[0] = p_x[0]
    p[1] = p_y[1]
    p[2] = (d - p[:2].dot(N[:2]))/N[-1]
    # print(p)
    ###determine Vy by eig(cov)
    curve_cov=np.cov(curve.T)
    eigenValues, eigenVectors = np.linalg.eig(curve_cov)
    idx = eigenValues.argsort()[::-1]   
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]
    Vy=eigenVectors[0]
    ###get average curve normal
    N=np.sum(curve_normal,axis=0)
    N=N/np.linalg.norm(N)
    ###project N on to plane of Vy
    N=VectorPlaneProjection(N,Vy)

    ###form transformation
    R=np.vstack((np.cross(Vy,-N),Vy,-N))
    T=p-R@C
    return H_from_RT(R,T)


def main():
    #select dataset
    data_dir='wave/'
    #select TCP
    with open('../config/tcp.yaml') as file:
        H_tcp = np.array(yaml.safe_load(file)['H'],dtype=np.float64)

    with open('../surface_points.npy', 'rb') as f:
        p1 = np.load(f)
        p2 = np.load(f)
        p3 = np.load(f)
    print('Found and loaded calibrated surface points:\n', p1, '\n', p2, '\n', p3)

    x_p = (p2-p1)/np.linalg.norm(p2-p1)
    z_p = np.cross(x_p,p3-p1)/np.linalg.norm(np.cross(x_p,p3-p1))
    y_p = np.cross(z_p, x_p)
    N = z_p
    d = p2.dot(N)
    print("Calibrated TCP normal:\n", N)
    
    ###read in curves
    curve = read_csv(data_dir+"Curve_dense.csv",header=None).values
    lam=calc_lam_cs(curve)
    robot=tormach(R_tool=H_tcp[:3,:3],p_tool=H_tcp[:-1,-1])

    print("OPTIMIZING ON CURVE POSE")
    p_ref = np.vstack(([p1,p2,p3]))
    H=pose_opt_wRef(robot,curve[:,:3],curve[:,3:],N,p_ref)
    H = base_correction(H,N)
    print('curve pose:\n',H)


    curve_base,curve_normal_base=curve_frame_conversion(curve[:,:3],curve[:,3:],H)

    ###visualization
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot3D(curve_base[:,0], curve_base[:,1],curve_base[:,2], 'gray')
    # ax.quiver(curve_base[:,0],curve_base[:,1],curve_base[:,2],5*curve_normal_base[:,0],5*curve_normal_base[:,1],5*curve_normal_base[:,2])
    # ax.set_xlabel('x (mm)')
    # ax.set_ylabel('y (mm)')
    # ax.set_zlabel('z (mm)')
    # plt.show()

    curve_js=find_js_qp(robot,curve_base,curve_normal_base)


    ###save file
    df=DataFrame({'x':curve_base[:,0],'y':curve_base[:,1], 'z':curve_base[:,2],'x_dir':curve_normal_base[:,0],'y_dir':curve_normal_base[:,1], 'z_dir':curve_normal_base[:,2]})
    df.to_csv(data_dir+'Curve_in_base_frame.csv',header=False,index=False)
    DataFrame(curve_js).to_csv(data_dir+'Curve_js.csv',header=False,index=False)
    with open(data_dir+'curve_pose.yaml', 'w') as file:
        documents = yaml.dump({'H':H.tolist()}, file)

if __name__ == "__main__":
    main()