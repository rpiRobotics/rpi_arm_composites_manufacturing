# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv
from scipy.linalg import logm, norm, sqrtm

from ControlParams import *
#import time
#import timeit
from pyquaternion import Quaternion
import quadprog

def robotParams():
    I3 = np.eye(3)
    ex = I3[:,0]
    ey = I3[:,1]
    ez = I3[:,2]
    
    h1 = ez
    h2 = ey
    h3 = ey
    h4 = ex
    h5 = ey
    h6 = ex
    P = np.array([[0,0,0], [0.32, 0, 0.78], [0, 0, 1.075], [0, 0, 0.2], [1.142, 0, 0], [0.2, 0, 0], [0,0,0]]).T
    q = np.zeros((6, 1))
    H = np.array([h1, h2, h3, h4, h5, h6]).T
    ttype = np.zeros((1, 6))

    n = 6
    
    dq_bounds = np.array([[100,110], [90,90], [90,90], [170,190], [120,140], [190,235]]).T
    dq_bounds = dq_bounds*np.pi/180
    
    return ex,ey,ez,n,P,q,H,ttype,dq_bounds


    
# find closest rotation matrix 
# A=A*inv(sqrt(A'*A))   
def Closest_Rotation(R):
    R_n = np.dot(R, inv(sqrtm(np.dot(R.T, R))))
    
    return R_n

# ROT Rotate along an axis h by q in radius
def rot(h, q):
    h=h/norm(h)
    R = np.eye(3) + np.sin(q)*hat(h) + (1 - np.cos(q))*np.dot(hat(h), hat(h))
    
    return R

def hat(h):
    h_hat = np.array([[0, -h[2], h[1]], [h[2], 0, -h[0]], [-h[1], h[0], 0]])
    
    return h_hat
    
def fwdkin_alljoints(q, ttype, H, P, n):
    R=np.eye(3)
    p=np.zeros((3,1))
    RR = np.zeros((3,3,n+1))
    pp = np.zeros((3,n+1))
    
    for i in range(n):
        h_i = H[0:3,i]
       
        if ttype[0][i] == 0:
        #rev
            pi = P[0:3,i].reshape(3, 1)
            p = p+np.dot(R,pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R,Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
        #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)
        else: 
	    # default pris
	        pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
	        p = p+np.dot(R,pi)
        
        pp[:,[i]] = p
        RR[:,:,i] = R
    
    # end effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    pp[:,[n]] = p
    RR[:,:,n] = R
    
    return pp, RR

def getJacobian(q,ttype,H,P,n):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    
    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))
    
    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    """ """
    
    return J


def getqp_H(J, vr, vp, er, ep):
    n = 6

    H1 = np.dot(np.hstack((J,np.zeros((n,2)))).T,np.hstack((J,np.zeros((n,2)))))
    
    tmp = np.vstack((np.hstack((np.hstack((np.zeros((3,n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,n)),np.zeros((3,1)))),vp)))) 
    H2 = np.dot(tmp.T,tmp)

    H3 = -2*np.dot(np.hstack((J,np.zeros((n,2)))).T, tmp)
    H3 = (H3+H3.T)/2;
    
    tmp2 = np.vstack((np.array([0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,np.sqrt(ep)])))
    H4 = np.dot(tmp2.T, tmp2)

    H = 2*(H1+H2+H3+H4)

    return H

def getqp_f(er, ep):
    f = -2*np.array([0,0,0,0,0,0,er,ep]).reshape(8, 1)
    
    return f

def inequality_bound(h,c,eta,epsilon,e):
    sigma = np.zeros((h.shape))
    h2 = h - eta
    sigma[np.array(h2 >= epsilon)] = -np.tan(c*np.pi/2)
    sigma[np.array(h2 >= 0) & np.array(h2 < epsilon)] = -np.tan(c*np.pi/2/epsilon*h2[np.array(h2 >= 0) & np.array(h2 < epsilon)])
    sigma[np.array(h >= 0) & np.array(h2 < 0)] = -e*h2[np.array(h >= 0) & np.array(h2 < 0)]/eta
    sigma[np.array(h < 0)] = e
    
    return sigma


def QP_abbirb6640(q,v):
    #Init the joystick

    
    # Initialize Robot Parameters    
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()
    # joint limits
    lower_limit = np.transpose(np.array([-170*np.pi/180, -65*np.pi/180, -np.pi, -300*np.pi/180, -120*np.pi/180, -2*np.pi]))
    upper_limit = np.transpose(np.array([170*np.pi/180, 85*np.pi/180, 70*np.pi/180, 300*np.pi/180, 120*np.pi/180, 2*np.pi]))
    
    # Initialize Control Parameters
    # initial joint angles
    

    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))

    # inequality constraints
    h = np.zeros((15, 1))
    sigma = np.zeros((13, 1))
    dhdq = np.vstack((np.hstack((np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.hstack((-np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.zeros((1, 8))))

    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))
    
    # keyboard controls
    # define position and angle step
    inc_pos_v = 0.01 # m/s
    inc_ang_v = 0.5*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters
    
    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.15
    E = 0.005

    pp,RR = fwdkin_alljoints(q,ttype,H,P,n)
    orien_tmp = Quaternion(matrix=RR[:, :, -1])
    orien_tmp = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]]).reshape(1, 4)
    
    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pp[:, -1],orien_tmp,pos_v,ang_v.reshape(1, 4),w_t,v_t,epsilon,inc_pos_v,inc_ang_v,0,er,ep,0)

    
    J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
    
    
    # desired rotational velocity
    vr = v[0:3]
    
    # desired linear velocity
    vp = v[3:6]
                
    Q = getqp_H(J_eef, vr.reshape(3, 1),  vp.reshape(3, 1), obj.params['opt']['er'], obj.params['opt']['ep']) 
    
    # make sure Q is symmetric
    Q = 0.5*(Q + Q.T)
    
    f = getqp_f(obj.params['opt']['er'], obj.params['opt']['ep'])
    f = f.reshape((8, ))

    
    # bounds for qp
    if obj.params['opt']['upper_dq_bounds']:
        bound = obj.params['defi']['dq_bounds'][1, :]
    else:
        bound = obj.params['defi']['dq_bounds'][0, :]

    LB = np.vstack((-0.1*bound.reshape(6, 1),0,0))
    UB = np.vstack((0.1*bound.reshape(6, 1),1,1))
            
    # inequality constrains A and b
    h[0:6] = obj.params['controls']['q'] - lower_limit.reshape(6, 1)
    h[6:12] = upper_limit.reshape(6, 1) - obj.params['controls']['q']

    sigma[0:12] = inequality_bound(h[0:12], c, eta, epsilon_in, E)
    

    A = np.vstack((dhdq,np.eye(8), -np.eye(8)))
    b = np.vstack((sigma, LB, -UB))

    # solve the quadprog problem
    dq_sln = quadprog.solve_qp(Q, -f, A.T, b.reshape((29, )))[0]

        
    if len(dq_sln) < obj.params['defi']['n']:
        obj.params['controls']['dq'] = np.zeros((6,1))
        V_scaled = 0
        print 'No Solution'
        dq_sln = np.zeros((int(n),1))
    else:
        obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
        obj.params['controls']['dq'] = obj.params['controls']['dq'].reshape((6, 1))
        #print dq_sln
#        V_scaled = dq_sln[-1]*V_desired
#        vr_scaled = dq_sln[-2]*vr.reshape(3,1)
        
        #print np.dot(np.linalg.pinv(J_eef),v)
     
    return dq_sln[0:6]
                           
        
#if __name__ == '__main__':
#    q = np.array([0,0,0,0,np.pi/2,0]).reshape(6, 1)
#    v = np.array([0,0,0,0,0,0.1])
#    a=QP_abbirb6640(q,v)
#    print a
    
