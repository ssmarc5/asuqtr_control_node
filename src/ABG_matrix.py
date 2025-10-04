#!/usr/bin/env python3
import math
import numpy as np
import rospy

# State cost Matrix
# [X Y Z ROLL PITCH YAW U V W P Q R]
#Q = np.diag(np.array(rospy.get_param('/control_node/state_cost_matrix'), dtype=np.float32))

#Matrice du cout des moteurs du sous-marin
#M1 M2 M3 M4 M5 M6 M7 M8
#R = np.diag(np.array(rospy.get_param('/control_node/motor_cost_matrix'), dtype=np.float32))
#inv_R = np.linalg.inv(R)

THRUST_ALLOC_MAT = np.array([[-1, 1, 0, 0, 0, 1],
							 [-1, -1, 0, 0, 0, -1],
							 [0, 0, -1, -1, -1, 0],
							 [0, 0, -1, 1, -1, 0],
							 [0, 0, -1, -1, 1, 0],
							 [0, 0, -1, 1, 1, 0],
							 [1, 1, 0, 0, 0, -1],
							 [1, -1, 0, 0, 0, 1]], dtype=np.int8)


Bm = np.zeros((12, 8), dtype=np.float32)
Bm[0][0] = 0.0
Bm[0][1] = 0.0
Bm[0][2] = 0.0
Bm[0][3] = 0.0
Bm[0][4] = 0.0
Bm[0][5] = 0.0
Bm[0][6] = 0.0
Bm[0][7] = 0.0
Bm[1][0] = 0.0
Bm[1][1] = 0.0
Bm[1][2] = 0.0
Bm[1][3] = 0.0
Bm[1][4] = 0.0
Bm[1][5] = 0.0
Bm[1][6] = 0.0
Bm[1][7] = 0.0
Bm[2][0] = 0.0
Bm[2][1] = 0.0
Bm[2][2] = 0.0
Bm[2][3] = 0.0
Bm[2][4] = 0.0
Bm[2][5] = 0.0
Bm[2][6] = 0.0
Bm[2][7] = 0.0
Bm[3][0] = 0.0
Bm[3][1] = 0.0
Bm[3][2] = 0.0
Bm[3][3] = 0.0
Bm[3][4] = 0.0
Bm[3][5] = 0.0
Bm[3][6] = 0.0
Bm[3][7] = 0.0
Bm[4][0] = 0.0
Bm[4][1] = 0.0
Bm[4][2] = 0.0
Bm[4][3] = 0.0
Bm[4][4] = 0.0
Bm[4][5] = 0.0
Bm[4][6] = 0.0
Bm[4][7] = 0.0
Bm[5][0] = 0.0
Bm[5][1] = 0.0
Bm[5][2] = 0.0
Bm[5][3] = 0.0
Bm[5][4] = 0.0
Bm[5][5] = 0.0
Bm[5][6] = 0.0
Bm[5][7] = 0.0
Bm[6][0] = 0.037210526315789473684210526315789
Bm[6][1] = 0.037210526315789473684210526315789
Bm[6][2] = 0.0
Bm[6][3] = 0.0
Bm[6][4] = 0.0
Bm[6][5] = 0.0
Bm[6][6] = -0.037210526315789473684210526315789
Bm[6][7] = -0.037210526315789473684210526315789
Bm[7][0] = -0.037210526315789473684210526315789
Bm[7][1] = 0.037210526315789473684210526315789
Bm[7][2] = 0.0
Bm[7][3] = 0.0
Bm[7][4] = 0.0
Bm[7][5] = 0.0
Bm[7][6] = -0.037210526315789473684210526315789
Bm[7][7] = 0.037210526315789473684210526315789
Bm[8][0] = 0.0
Bm[8][1] = 0.0
Bm[8][2] = 0.052631578947368421052631578947368
Bm[8][3] = 0.052631578947368421052631578947368
Bm[8][4] = 0.052631578947368421052631578947368
Bm[8][5] = 0.052631578947368421052631578947368
Bm[8][6] = 0.0
Bm[8][7] = 0.0
Bm[9][0] = 0.0
Bm[9][1] = 0.0
Bm[9][2] = 1.1473684210526315789473684210526
Bm[9][3] = -1.1473684210526315789473684210526
Bm[9][4] = 1.1473684210526315789473684210526
Bm[9][5] = -1.1473684210526315789473684210526
Bm[9][6] = 0.0
Bm[9][7] = 0.0
Bm[10][0] = 0.0
Bm[10][1] = 0.0
Bm[10][2] = 0.43923976608187134502923976608187
Bm[10][3] = 0.43923976608187134502923976608187
Bm[10][4] = -0.43923976608187134502923976608187
Bm[10][5] = -0.43923976608187134502923976608187
Bm[10][6] = 0.0
Bm[10][7] = 0.0
Bm[11][0] = -0.85989925297113747273926919923383
Bm[11][1] = 0.85989925297113747273926919923383
Bm[11][2] = 0.0
Bm[11][3] = 0.0
Bm[11][4] = 0.0
Bm[11][5] = 0.0
Bm[11][6] = 0.85989925297113747273926919923383
Bm[11][7] = -0.85989925297113747273926919923383

def get_AG_matrices(state):

    #Extracting state's data
    x = 0
    y = 0
    z = state[2]
    roll_ = state[3]
    pitch_ = state[4]
    yaw_ = state[5]
    u = 0
    v = 0
    w = 0
    p = state[9]
    q = state[10]
    r = state[11]

    # Precomputing of sin, cos and tan to avoid recomputing it 100x
    sin_roll = math.sin(roll_)
    sin_yaw = math.sin(yaw_)
    sin_pitch = math.sin(pitch_)
    cos_roll = math.cos(roll_)
    cos_yaw = math.cos(yaw_)
    cos_pitch = math.cos(pitch_)
    tan_pitch = math.tan(pitch_)

    # AUV params
    vehicule_radius = 0.26

    #Si le sous-marin sort de l'eau (surface de l'eau) la flotabilit� diminue
    #radius = max(vehicule_radius - abs(z), 0) if z < 0.8 else vehicule_radius
    radius = vehicule_radius

    #Fill Matrices with zeros
    Am = np.zeros((12, 12), dtype=np.float32)
    Gm = np.zeros((6, 1), dtype=np.float32)

    Am[0][0] = 0.0
    Am[0][1] = 0.0
    Am[0][2] = 0.0
    Am[0][3] = v * (sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch) + w * (
                cos_roll * sin_yaw - 1.0 * cos_yaw * sin_pitch * sin_roll)
    Am[0][4] = w * cos_pitch * cos_roll * cos_yaw - 1.0 * u * cos_yaw * sin_pitch + v * cos_pitch * cos_yaw * sin_roll
    Am[0][5] = w * (cos_yaw * sin_roll - 1.0 * cos_roll * sin_pitch * sin_yaw) - 1.0 * v * (
                cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw) - 1.0 * u * cos_pitch * sin_yaw
    Am[0][6] = cos_pitch * cos_yaw
    Am[0][7] = cos_yaw * sin_pitch * sin_roll - 1.0 * cos_roll * sin_yaw
    Am[0][8] = sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch
    Am[0][9] = 0.0
    Am[0][10] = 0.0
    Am[0][11] = 0.0
    Am[1][0] = 0.0
    Am[1][1] = 0.0
    Am[1][2] = 0.0
    Am[1][3] = - 1.0 * w * (cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw) - 1.0 * v * (
                cos_yaw * sin_roll - 1.0 * cos_roll * sin_pitch * sin_yaw)
    Am[1][4] = w * cos_pitch * cos_roll * sin_yaw - 1.0 * u * sin_pitch * sin_yaw + v * cos_pitch * sin_roll * sin_yaw
    Am[1][5] = w * (sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch) - 1.0 * v * (
                cos_roll * sin_yaw - 1.0 * cos_yaw * sin_pitch * sin_roll) + u * cos_pitch * cos_yaw
    Am[1][6] = cos_pitch * sin_yaw
    Am[1][7] = cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw
    Am[1][8] = cos_roll * sin_pitch * sin_yaw - 1.0 * cos_yaw * sin_roll
    Am[1][9] = 0.0
    Am[1][10] = 0.0
    Am[1][11] = 0.0
    Am[2][0] = 0.0
    Am[2][1] = 0.0
    Am[2][2] = 0.0
    Am[2][3] = v * cos_pitch * cos_roll - 1.0 * w * cos_pitch * sin_roll
    Am[2][4] = - 1.0 * u * cos_pitch - 1.0 * w * cos_roll * sin_pitch - 1.0 * v * sin_pitch * sin_roll
    Am[2][5] = 0.0
    Am[2][6] = -1.0 * sin_pitch
    Am[2][7] = cos_pitch * sin_roll
    Am[2][8] = cos_pitch * cos_roll
    Am[2][9] = 0.0
    Am[2][10] = 0.0
    Am[2][11] = 0.0
    Am[3][0] = 0.0
    Am[3][1] = 0.0
    Am[3][2] = 0.0
    Am[3][3] = q * cos_roll * tan_pitch - 1.0 * r * tan_pitch * sin_roll
    Am[3][4] = r * cos_roll * (tan_pitch ** 2 + 1.0) + q * sin_roll * (tan_pitch ** 2 + 1.0)
    Am[3][5] = 0.0
    Am[3][6] = 0.0
    Am[3][7] = 0.0
    Am[3][8] = 0.0
    Am[3][9] = 1.0
    Am[3][10] = tan_pitch * sin_roll
    Am[3][11] = cos_roll * tan_pitch
    Am[4][0] = 0.0
    Am[4][1] = 0.0
    Am[4][2] = 0.0
    Am[4][3] = - 1.0 * r * cos_roll - 1.0 * q * sin_roll
    Am[4][4] = 0.0
    Am[4][5] = 0.0
    Am[4][6] = 0.0
    Am[4][7] = 0.0
    Am[4][8] = 0.0
    Am[4][9] = 0.0
    Am[4][10] = cos_roll
    Am[4][11] = -1.0 * sin_roll
    Am[5][0] = 0.0
    Am[5][1] = 0.0
    Am[5][2] = 0.0
    Am[5][3] = (q * cos_roll) / cos_pitch - (1.0 * r * sin_roll) / cos_pitch
    Am[5][4] = (r * cos_roll * sin_pitch) / cos_pitch ** 2 + (q * sin_pitch * sin_roll) / cos_pitch ** 2
    Am[5][5] = 0.0
    Am[5][6] = 0.0
    Am[5][7] = 0.0
    Am[5][8] = 0.0
    Am[5][9] = 0.0
    Am[5][10] = sin_roll / cos_pitch
    Am[5][11] = cos_roll / cos_pitch
    Am[6][0] = 0.0
    Am[6][1] = 0.0
    Am[6][2] = 0.0
    Am[6][3] = 0.0
    Am[6][4] = 0.026315789473684210526315789473684 * cos_pitch * (
                41092.031908954495559091375453296 * radius ** 3 - 274.68)
    Am[6][5] = 0.0
    Am[6][6] = 0.58447368421052631578947368421053
    Am[6][7] = r
    Am[6][8] = -1.0 * q
    Am[6][9] = 0.0
    Am[6][10] = -1.0 * w
    Am[6][11] = v
    Am[7][0] = 0.0
    Am[7][1] = 0.0
    Am[7][2] = 0.0
    Am[7][3] = -0.026315789473684210526315789473684 * cos_pitch * cos_roll * (
                41092.031908954495559091375453296 * radius ** 3 - 274.68)
    Am[7][4] = 0.026315789473684210526315789473684 * sin_pitch * sin_roll * (
                41092.031908954495559091375453296 * radius ** 3 - 274.68)
    Am[7][5] = 0.0
    Am[7][6] = -1.0 * r
    Am[7][7] = 0.73368421052631578947368421052632
    Am[7][8] = p
    Am[7][9] = w
    Am[7][10] = 0.0
    Am[7][11] = -1.0 * u
    Am[8][0] = 0.0
    Am[8][1] = 0.0
    Am[8][2] = 0.0
    Am[8][3] = 0.026315789473684210526315789473684 * cos_pitch * sin_roll * (
                41092.031908954495559091375453296 * radius ** 3 - 274.68)
    Am[8][4] = 0.026315789473684210526315789473684 * cos_roll * sin_pitch * (
                41092.031908954495559091375453296 * radius ** 3 - 274.68)
    Am[8][5] = 0.0
    Am[8][6] = q
    Am[8][7] = -1.0 * p
    Am[8][8] = 1.1089473684210526315789473684211
    Am[8][9] = -1.0 * v
    Am[8][10] = u
    Am[8][11] = 0.0
    Am[9][0] = 0.0
    Am[9][1] = 0.0
    Am[9][2] = 0.0
    Am[9][3] = 0.0
    Am[9][4] = 0.0
    Am[9][5] = 0.0
    Am[9][6] = 0.0
    Am[9][7] = 0.0
    Am[9][8] = 0.0
    Am[9][9] = 3.4105263157894736842105263157895
    Am[9][10] = -0.74285714285714285714285714285714 * r
    Am[9][11] = -0.74285714285714285714285714285714 * q
    Am[10][0] = 0.0
    Am[10][1] = 0.0
    Am[10][2] = 0.0
    Am[10][3] = 0.0
    Am[10][4] = 0.0
    Am[10][5] = 0.0
    Am[10][6] = 0.0
    Am[10][7] = 0.0
    Am[10][8] = 0.0
    Am[10][9] = 0.75 * r
    Am[10][10] = 3.3157894736842105263157894736842
    Am[10][11] = 0.75 * p
    Am[11][0] = 0.0
    Am[11][1] = 0.0
    Am[11][2] = 0.0
    Am[11][3] = 0.0
    Am[11][4] = 0.0
    Am[11][5] = 0.0
    Am[11][6] = 0.0
    Am[11][7] = 0.0
    Am[11][8] = 0.0
    Am[11][9] = -0.016129032258064516129032258064516 * q
    Am[11][10] = -0.016129032258064516129032258064516 * p
    Am[11][11] = 1.9252971137521222410865874363328

    return Am
