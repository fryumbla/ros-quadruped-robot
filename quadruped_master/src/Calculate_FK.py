from sympy import *
from time import time
from mpmath import radians

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-0.99,-0.12,0.94,4.06,1.29,-4.12]]}


def test_code(test_case):
    ### Your FK code here
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # Create Modified DH parameters
    s = {alpha0:-pi/2.,  a0:   0, d1: 0.205, q1: q1,
         alpha1:    pi,  a1:0.35, d2:     0, q2: q2,
         alpha2:-pi/2.,  a2:   0, d3:     0, q3: q3,
         alpha3: pi/2.,  a3:   0, d4: 0.305, q4: q4,
         alpha4:-pi/2.,  a4:   0, d5:     0, q5: q5,
         alpha5:     0,  a5:   0, d6: 0.075, q6: q6}
         

    # Define Modified DH Transformation matrix
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[        cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha) ,  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
        return TF
    # Create individual transformation matrices
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
    T5_EE = TF_Matrix(alpha5, a5, d6, q6).subs(s)

     
    # Extract rotation matrices from the transformation matrices
    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_EE
    #theta1 = test_case[2][0]
    #theta2 = test_case[2][1]
    #theta3 = test_case[2][2]
    #theta4 = test_case[2][3]
    #theta5 = test_case[2][4]
    #theta6 = test_case[2][5]
    theta1 = 2.2
    theta2 = -1.8
    theta3 = 2.3
    theta4 = -1.2
    theta5 = 0.5
    theta6 = 1.4

    T0_EE = T0_EE.evalf(subs = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    your_ee = T0_EE[0:3, 3] # <--- Load your calculated end effector value from your forward kinematics

    ee_x_e = abs(your_ee[0])
    ee_y_e = abs(your_ee[1])
    ee_z_e = abs(your_ee[2])
    print ("\nEnd effector for x position is: %04.8f" % ee_x_e)
    print ("End effector for y position is: %04.8f" % ee_y_e)
    print ("End effector or z position is: %04.8f" % ee_z_e)

    # Find FK EE error
    #for i=0, i

    #if not(sum(your_ee)==3):
    #    ee_x_e = abs(your_ee[0]-test_case[0][0][0])
    #    ee_y_e = abs(your_ee[1]-test_case[0][0][1])
    #    ee_z_e = abs(your_ee[2]-test_case[0][0][2])
    #    ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
    #    print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
    #    print ("End effector error for y position is: %04.8f" % ee_y_e)
    #    print ("End effector error for z position is: %04.8f" % ee_z_e)
    #    print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # Change test case number for different scenarios
    for i in range(1, 4):
              test_code(test_cases[i])
