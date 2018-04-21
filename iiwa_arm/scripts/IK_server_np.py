#!/usr/bin/env python

# import modules
import rospy
import tf
from iiwa_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import pickle, os, time
import numpy as np

from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, lambdify
from sympy.matrices import Matrix


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        def DH_transform(q, d, alpha, a): 
            '''The modified DH convention transform matrix
            alpha: twist angle, a: link length, 
            d: link offset, q: joint angle'''
            T = Matrix([[             cos(q),             -sin(q),           0,               a],
                        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                        [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                        [                  0,                   0,           0,               1]])
            return T 

        
        def rot_x(q):
            ''' Rotation matrix along x axis'''
            R_x = Matrix([[      1,      0,      0],
                          [      0, cos(q), -sin(q)],
                          [      0, sin(q),  cos(q)]])
            
            return R_x
    
        def rot_y(q):
            ''' Rotation matrix along y axis'''
            R_y = Matrix([[ cos(q),     0, sin(q)],
                          [      0,     1,      0],
                          [-sin(q),     0, cos(q)]])
            
            return R_y

        def rot_z(q):
            ''' Rotation matrix along z axis'''
            R_z = Matrix([[ cos(q),-sin(q), 0],
                          [ sin(q), cos(q), 0],
                          [      0,      0, 1]])
            
            return R_z
        

        def Euler_angles_from_matrix_URDF(R, angles_pre):
            ''' Calculate q4-6 from R3_6 rotation matrix
            Input R is 3x3 rotation matrix, output are Euler angles :q4, q5, q6'''
            r13 = R[0,2]
            r21, r22, r23 = R[1,0], R[1,1], R[1,2] 
            r32, r33 = R[2,1], R[2,2]
            # # Euler angles from rotation matrix
            # q5 = atan2(sqrt(r13**2 + r33**2), r23)
            # q4 = atan2(r33, -r13)
            # q6 = atan2(-r22, r21)
            if r23 is not 0:
                q6 = atan2(sqrt(r13**2 + r33**2), -r23)

                if sin(q6) < 0:
                    q5 = atan2(-r33, -r13)
                    q7 = atan2(r22, -r21)
                else:
                    q5 = atan2(r33, r13)
                    q7 = atan2(-r22, r21)
            else:
                q5 = angles_pre[4]
                q6 = angles_pre[5]
                q7 = angles_pre[6]
                
            return np.float64(q5), np.float64(q6), np.float64(q7)
            

        # Define variables
        q1, q2, q3, q4, q5, q6, q7, q8 = symbols('q1:9')
        d1, d2, d3, d4, d5, d6, d7, d8 = symbols('d1:9')
        a0, a1, a2, a3, a4, a5, a6, a7 = symbols('a0:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha0:8')
        r, p, y = symbols("r p y") # end-effector orientation
        px_s, py_s, pz_s = symbols("px_s py_s pz_s") # end-effector position

        theta1,theta2,theta3,theta4,theta5,theta6,theta7 = 0,0,0,0,0,0,0
        angles_pre = (0,0,0,0,0,0)
        r2d = 180./np.pi
        d2r = np.pi/180

        Joint_limits = [170.0*d2r, 120.0*d2r,
                        170.0*d2r, 120.0*d2r,
                        170.0*d2r, 120.0*d2r,
                        175.0*d2r]
                    
        # The Modified DH params
        # fix theta 3
        s = {alpha0:      0, a0:      0, d1:  0.36, q1: q1,
            alpha1: -pi / 2, a1:      0, d2:     0, q2:  q2-pi/2,
        #      alpha2:  pi / 2, a2:      0, d3:  0.42, q3: q3,
            alpha3:       0, a3:   0.42, d4:     0, q4: -q4+pi/2,
            alpha4:  pi / 2, a4:      0, d5:   0.4, q5: q5,
            alpha5: -pi / 2, a5:      0, d6:     0, q6: q6,
            alpha6:  pi / 2, a6:      0, d7: 0.081, q7: q7,
            alpha7:       0, a7:      0, d8:  0.08, q8: 0}

        # Define Modified DH Transformation matrix
        if not os.path.exists("R0_4_inv.p"):
            print("-----------------------------------------")
            print("No DH matrices, create and save it.")
            # base_link to link1
            T0_1 = DH_transform(q1, d1, alpha0, a0).subs(s)
            # linke1 to link 2
            T1_2 = DH_transform(q2, d2, alpha1, a1).subs(s)
            # link2 to link4
            T2_4 = DH_transform(q4, d4, alpha3, a3).subs(s)
            # link4 to link5
            T4_5 = DH_transform(q5, d5, alpha4, a4).subs(s)
            # link5 to link6
            T5_6 = DH_transform(q6, d6, alpha5, a5).subs(s)
            # link6 to link7
            T6_7 = DH_transform(q7, d7, alpha6, a6).subs(s)
            # link7 to ee
            T7_8 = DH_transform(q8, d8, alpha7, a7).subs(s)
   
            # Create individual transformation matrices
            T0_2 = simplify(T0_1 * T1_2)
            p2_0_sym = T0_2 * Matrix([0,0,0,1])
            # R0_4 inv matrix would be used in R4_7
            T0_4 = simplify(T0_2 * T2_4)
            R0_4 = T0_4[0:3, 0:3]
            R0_4_inv = simplify(R0_4 ** -1)

            R0_g_sym = simplify(rot_z(y) * rot_y(p) * rot_x(r))

            pickle.dump(p2_0_sym, open("p2_0_sym.p", "wb"))
            pickle.dump(R0_4_inv, open("R0_4_inv.p", "wb"))
            pickle.dump(R0_g_sym, open("R0_g_sym.p", "wb"))
            
            print("Transformation matrices have been saved!!")
            print("-----------------------------------------")
        else:
            p2_0_sym = pickle.load(open("p2_0_sym.p", "rb"))
            R0_4_inv = pickle.load(open("R0_4_inv.p", "rb"))
            R0_g_sym = pickle.load(open("R0_g_sym.p", "rb"))
            
            print("-----------------------------------------")
            print("Transformation matrices have been loaded!")         

        # Joint angle calculation
        pg_0 = Matrix([[px_s],[py_s],[pz_s]])
        R0_g = R0_g_sym[0:3,0:3]
        pwc_0 = pg_0 - 0.161 * R0_g * Matrix([[0],[0],[1]])
        theta1_sym = atan2(pwc_0[1], pwc_0[0]).subs(s)
        
        pwc_2 = pwc_0 - p2_0_sym[0:3,:]

        l24 = a3
        l46 = d5
        l26 = sqrt(pwc_2[0]**2 + pwc_2[1]**2 + pwc_2[2]**2)

        theta21 = atan2(pwc_2[2], sqrt(pwc_2[0]**2 + pwc_2[1]**2))
        c624 = (-l46**2 + l24**2 + l26**2) / (2*l24 * l26)
        theta22 = atan2(sqrt(1 - c624**2), c624)
        theta2_sym = (pi/2 - (theta21 + theta22)).subs(s)

        c246 = (-l26**2 + l24**2 + l46**2) / (2*l24*l46)
        theta4_aux = atan2(sqrt(1 - c246**2), c246)
        theta4_sym = -(pi - theta4_aux).subs(s)
        
        # Transfer symbol into numpy function to evaluate an expression more efficient
        theta1_f = lambdify((px_s,py_s,pz_s,r,p,y), theta1_sym, "numpy")
        theta2_f = lambdify((q1,px_s,py_s,pz_s,r,p,y), theta2_sym, "numpy")
        theta4_f = lambdify((q1,px_s,py_s,pz_s,r,p,y), theta4_sym, "numpy")

        # Calculate R3_6 for theta4-6 
        R4_7_sym = R0_4_inv * R0_g
        R4_7_f = lambdify((q1,q2,q4,px_s,py_s,pz_s,r,p,y), R4_7_sym, "numpy")

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            # There is no rotation discrepancy between DH tables and urdf here
            # R_corr = 1 # Compensation matix from URDF to world frame

            # Calculate joint angles using Geometric IK method           
            theta1 = theta1_f(px, py, pz, roll, pitch, yaw)          
            theta2 = theta2_f(theta1, px, py, pz, roll, pitch, yaw)
            theta4 = theta4_f(theta1, px, py, pz, roll, pitch, yaw)

            R4_7 = R4_7_f(theta1, theta2, theta4, px, py, pz, roll, pitch, yaw)

            theta5, theta6, theta7 = Euler_angles_from_matrix_URDF(R4_7, angles_pre)
            angles_pre = (theta1, theta2, theta4, theta5, theta6, theta7)

            print "theta1: ", theta1 * r2d
            print "theta2: ", theta2 * r2d
            print "theta4: ", theta4 * r2d
            print "theta5: ", theta5 * r2d
            print "theta6: ", theta6 * r2d
            print "theta7: ", theta7 * r2d

            thetas = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]
            for i in range(0,7):
                if np.abs(thetas[i]) > Joint_limits[i]:
                    thetas[i] = Joint_limits[i] * np.sign(thetas[i])
                    print("Warning: IK gives value out of bounds for joint {:>d}".format(i+1))

            joint_trajectory_point.positions = thetas
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
