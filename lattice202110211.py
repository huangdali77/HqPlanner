#!/usr/bin/env python



import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
# import matplotlib.patches as mpatch
import random
# import bisect

# from can_msgs.msg import delphi_msges
from test_msgs.msg import Test
from lattice_planning.msg import Traj
from rs_perception.msg import PerceptionListMsg
# from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from fsd_common_msgs.msg import Gnss
#from visualization_msgs import  MarkerArray
from fsd_common_msgs.msg import CarState
from fsd_common_msgs.msg import CarStateDt
from fsd_common_msgs.msg import Trajectory
from fsd_common_msgs.msg import TrajectoryPoint

from nav_msgs.msg import Path

from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D

import time
from nav_msgs.msg import Path
import copy
import cubic_spline_planner
import collision_checker_obb
import reeds_shepp_planner

ego_s = 1
ego_l = 2
ob_s = 1
ob_l = 2
expand_a = 0.15
max_speed = 10.0
long_max_acc = 2.0
lat_max_acc = 2.0
long_sample_N = 6
lat_sample_N = 5
preview_T = 4.0
d_T = 2.0
res = 1.

MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 8.0  # maximum acceleration [m/ss]
MAX_DEC = -10.0  # maximum  deceleration [m/ss]
MAX_CURVATURE = 1 / 4.5  # maximum curvature [1/m]  TOO LARGE  better to set it 0.2
MAX_ROAD_WIDTH = 4.0  # maximum road width [m]
D_ROAD_W = 1  # road width sampling length [m]
DT = 0.06  # time tick [s]
MAXT = 2.6  # max prediction time [s]
MINT = 2.5  # min prediction time [s]
TOTAL_TIME = 6.
SEGMENT_TIME = 2.5
EMERGENCY_STOP_TIME = 3.2



TARGET_SPEED = 12.0 / 3.6  # target speed [m/s]
D_T_S = 1.8 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
SAFEDISTANCE = 20  # basic safe distance between two vehicles [m]
veh_width, veh_len = 2.0, 4.0  # the length and width of ego car [m]
tau = 2.0  # 车头时距时间常数 [s]

TIME_SERIES = []
Vx_SERIES = []
Vy_SERIES = []
Yaw_SERIES = []
dYaw_SERIES = []

old_path = None
old_trajectory = None

D0 = 20
# cost weights
KJ = 0.5
KT = 1.0
KD = 1.0
KLAT = 1.0
KLON = 1.0
v_ref = 20 / 3.6

"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""


# Parameter

class quintic_polynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt


class quartic_polynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, T):
        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf =0.0

        # global path params
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

def Frenet_path_conjunction(path1, path2):
    conjted_path = Frenet_path()
    conjted_path.t = path1.t + path2.t
    conjted_path.d = path1.d + path2.d
    conjted_path.d_d = path1.d_d + path2.d_d
    conjted_path.d_dd = path1.d_dd + path2.d_dd
    conjted_path.d_ddd = path1.d_ddd + path2.d_ddd
    conjted_path.s = path1.s + path2.s
    conjted_path.s_d = path1.s_d + path2.s_d
    conjted_path.s_dd = path1.s_dd + path2.s_dd
    conjted_path.s_ddd = path1.s_ddd + path2.s_ddd
    conjted_path.cd = max(path1.cd,path2.cd)
    conjted_path.cv = max(path1.cv,path2.cv)
    conjted_path.cf = max(path1.cf,path2.cf)
    # global path params
    conjted_path.x = path1.x + path2.x
    conjted_path.y = path1.y + path2.y
    conjted_path.yaw = path1.yaw + path2.yaw
    conjted_path.ds = path1.ds + path2.ds
    conjted_path.c = path1.c + path2.c
    return conjted_path


class ReferencePath:
    def __init__(self, x, y, yaw, curvature, csp):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.csp = csp
        self.d_curvature = []
        for i in np.arange(len(self.curvature) - 1):
            self.d_curvature.append((self.curvature[i + 1] - self.curvature[i + 1]) / 0.1)
        self.d_curvature.append(self.d_curvature[-1])


class Obstacle:
    def __init__(self, x, y, width, length, orientation, ref_paths, lane_number):
        self.x = x
        self.y = y
        self.width = width
        self.length = length
        self.orientation = orientation
        self.lane_number = lane_number
        # self.s, self.d, _, _ = cartesian_to_frenet(self.x, self.y, 0, self.orientation, ref_paths, lane_number)


# max errors of s and d are around 0.1 and 0.05 m
# reference_path_x, reference_path_y, reference_path_yaw can be gained from fcn generate_target_course
# the step like 0.1 to generate list s must equal to the step to generate list s in fcn generate_target_course
def cartesian_to_frenet(xx, yx, vx, yaw_x, ref_paths, lane_number):
    r_p = ref_paths[lane_number].csp
    r_p_x = ref_paths[lane_number].x
    r_p_y = ref_paths[lane_number].y
    r_p_yaw = ref_paths[lane_number].yaw
    r_p_curvature = ref_paths[lane_number].curvature

    #index = len(r_p_x) - 1
    index = 0
    s = np.arange(0, r_p.s[-1], 0.1)
    for i in range(len(r_p_x) - 2):
        dot_pro1 = (xx - r_p_x[i]) * (r_p_x[i + 1] - r_p_x[i]) + \
                   (yx - r_p_y[i]) * (r_p_y[i + 1] - r_p_y[i])
        dot_pro2 = (xx - r_p_x[i + 1]) * (r_p_x[i + 2] - r_p_x[i + 1]) + \
                   (yx - r_p_y[i + 1]) * (r_p_y[i + 2] - r_p_y[i + 1])
        if dot_pro1 * dot_pro2 <= 0:
            index = i + 1
            break
    sr = s[index]
    absolute_d = math.sqrt((xx - r_p_x[index]) ** 2 + (yx - r_p_y[index]) ** 2)
    d = np.sign((yx - r_p_y[index]) * math.cos(r_p_yaw[index])
                - (xx - r_p_x[index]) * math.sin(r_p_yaw[index])) * absolute_d
    s_d = vx * math.cos(yaw_x - r_p_yaw[index]) / (1 - r_p_curvature[index] * d)
    d_d = (1 - r_p_curvature[index] * d) * math.tan(yaw_x - r_p_yaw[index])
    return sr, d, s_d, d_d


def get_global_path_nearest_yaw(xx, yx, ref_paths, lane_number):
    r_p = ref_paths[lane_number].csp
    r_p_x = ref_paths[lane_number].x
    r_p_y = ref_paths[lane_number].y
    r_p_yaw = ref_paths[lane_number].yaw
    index = len(r_p_x) - 1
    s = np.arange(0, r_p.s[-1], 0.1)
    for i in range(len(r_p_x) - 2):
        dot_pro1 = (xx - r_p_x[i]) * (r_p_x[i + 1] - r_p_x[i]) + \
                   (yx - r_p_y[i]) * (r_p_y[i + 1] - r_p_y[i])
        dot_pro2 = (xx - r_p_x[i + 1]) * (r_p_x[i + 2] - r_p_x[i + 1]) + \
                   (yx - r_p_y[i + 1]) * (r_p_y[i + 2] - r_p_y[i + 1])
        if dot_pro1 * dot_pro2 <= 0:
            index = i + 1
            break
    # print('index',index)
    nearst_yaw = r_p_yaw[index]
    return nearst_yaw


def frenet_to_cartesian(sr, d, d_d, ref_path):
    s = np.arange(0, ref_path.csp.s[-1], 0.1)
    xr, yr, theta_r, kr = 0., 0., 0., 0.
    for i in range(len(s) - 1):
        if (s[i] - sr) * (s[i + 1] - sr) <= 0.:
            xr = ref_path.x[i]
            yr = ref_path.y[i]
            theta_r = ref_path.yaw[i]
            kr = ref_path.curvature[i]
            break
    xx = xr - d * math.sin(theta_r)
    yx = yr + d * math.cos(theta_r)
    theta_x = math.atan2(d_d, 1 - kr) + theta_r
    return xx, yx, theta_x


class OtherVehicle:
    def __init__(self, width, length, center_x, center_y, orientation, vx, vy, ref_paths,
                 cur_lane_number):
        self.width = width
        self.length = length
        self.center_x = center_x
        self.center_y = center_y
        self.orientation = orientation
        self.vx = vx
        self.vy = vy
        self.shape = collision_checker_obb.Rectangle(self.center_x, self.center_y,
                                                     self.width, self.length, self.orientation)
        self.cur_lane_number = cur_lane_number
        # self.sr, self.d = cartesian_to_frenet(self.center_x, self.center_y, ref_paths, cur_lane_number)

    def update_cartesian_parameters(self, vx, vy, sr, d, d_d, ref_paths, delta_t, cur_lane_number):
        sr = sr + self.vx * delta_t
        d = d + self.vy * delta_t
        ref_path = ref_paths[cur_lane_number]
        xx, yx, theta_x = frenet_to_cartesian(sr, d, d_d, ref_path)
        self.vx = vx
        self.vy = vy
        self.center_x = xx
        self.center_y = yx
        self.orientation = theta_x
        self.cur_lane_number = cur_lane_number


class Behaviour:
    #  cur_s, cur_d, cur_speed, cur_lane_num, ref_paths，
    #  分别代表在当前参考路径下的s, d, 速度，当前车道（最左侧车道为0, 即第1条参考路径），所有的参考路径
    def __init__(self, cur_s, cur_d, cur_speed, cur_lane_num, ref_paths):
        self.cur_s = cur_s
        self.cur_d = cur_d
        self.cur_speed = cur_speed
        self.cur_lane_num = cur_lane_num
        self.ref_path = ref_paths[cur_lane_num]

    # lane_num, cur_lane_closest_car_v, left_lane_fcar_s,
    # e_fcar_v, left_lane_rcar_s, left_lane_rcar_v
    # 分别是所有车道的编号，数据类型为list[]；当前车道最近前车的速度；左侧车道前后最近车辆的纵向位置和速度

    def left_lane_change_check(self, lane_num, cur_lane_num, left_lane_fcar_s, left_lane_fcar_v, left_lane_rcar_s,
                               left_lane_rcar_v, cur_lane_closest_car_v):
        print(lane_num, cur_lane_num, left_lane_fcar_s, left_lane_fcar_v, left_lane_rcar_s,
              left_lane_rcar_v, cur_lane_closest_car_v)
        left_lane_change_ok = False
        left_check_action = 2
        if cur_lane_closest_car_v:
            left_target_speed = cur_lane_closest_car_v[0]
        else:
            left_target_speed = TARGET_SPEED
        left_following_index = 0
        if cur_lane_num + 1 in lane_num:
            # print('left cur_lane_num', cur_lane_num)
            if left_lane_fcar_s:
                if left_lane_fcar_s[0] <= 15:
                    if len(left_lane_fcar_s) == 1:
                        left_lane_change_ok = True
                        left_check_action = 31
                        left_target_speed = TARGET_SPEED
                    else:
                        if left_lane_fcar_s[1] - self.cur_s >= 50 and self.cur_speed >= left_lane_fcar_v[0] \
                                and 1.1 * cur_lane_closest_car_v[0] < left_lane_fcar_v[1] < TARGET_SPEED:
                            left_lane_change_ok = True
                            left_check_action = 32
                            left_following_index = 1
                            left_target_speed = min(left_lane_fcar_v[1], TARGET_SPEED)
                        elif left_lane_fcar_s[1] - self.cur_s >= 50 and self.cur_speed >= left_lane_fcar_v[0] \
                                and 1.1 * cur_lane_closest_car_v[0] < left_lane_fcar_v[1] >= TARGET_SPEED:
                            left_lane_change_ok = True
                            left_check_action = 31
                            left_target_speed = TARGET_SPEED
                        else:
                            left_lane_change_ok = False
                elif left_lane_fcar_s[0] >= 50:
                    if not left_lane_rcar_v and \
                                                    1.1 * cur_lane_closest_car_v[0] < left_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 32
                        left_target_speed = min(left_lane_fcar_v[1], TARGET_SPEED)
                    elif not left_lane_rcar_v and \
                                                    1.1 * cur_lane_closest_car_v[0] < left_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 31
                        left_target_speed = TARGET_SPEED
                    elif left_lane_rcar_v and self.cur_s - left_lane_rcar_s[0] >= 25 and \
                                                    1.1 * cur_lane_closest_car_v[0] <= left_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 32
                        left_target_speed = min(left_lane_fcar_v[1], TARGET_SPEED)
                    elif left_lane_rcar_v and self.cur_s - left_lane_rcar_s[0] >= 25 and \
                                                    1.1 * cur_lane_closest_car_v[0] <= left_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 31
                        left_target_speed = TARGET_SPEED
                    elif left_lane_rcar_v and self.cur_s - left_lane_rcar_s[0] < 25 and \
                                    self.cur_speed >= left_lane_rcar_v[0] and \
                                                    1.1 * cur_lane_closest_car_v[0] <= left_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 32
                        left_target_speed = min(left_lane_fcar_v[1], TARGET_SPEED)
                    elif left_lane_rcar_v and self.cur_s - left_lane_rcar_s[0] < 25 and \
                                    self.cur_speed >= left_lane_rcar_v[0] and \
                                                    1.1 * cur_lane_closest_car_v[0] <= left_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        left_lane_change_ok = True
                        left_check_action = 31
                        left_target_speed = TARGET_SPEED
                    else:
                        left_lane_change_ok = False
                else:
                    left_lane_change_ok = False
            else:
                if left_lane_rcar_s and self.cur_s - left_lane_rcar_s[0] <= 20 \
                        and self.cur_speed <= left_lane_rcar_v[0]:
                    left_lane_change_ok = False
                else:
                    left_lane_change_ok = True
                    left_check_action = 31
                    left_target_speed = TARGET_SPEED
        return left_lane_change_ok, left_check_action, left_target_speed, left_following_index

    def right_lane_change_check(self, lane_num, cur_lane_num, right_lane_fcar_s, right_lane_fcar_v, right_lane_rcar_s,
                                right_lane_rcar_v, cur_lane_closest_car_v):
        right_lane_change_ok = False
        right_check_action = 2
        if cur_lane_closest_car_v:
            right_target_speed = cur_lane_closest_car_v[0]
        else:
            right_target_speed = TARGET_SPEED
        right_following_index = 0
        if cur_lane_num - 1 in lane_num:
            if right_lane_fcar_s:
                if right_lane_fcar_s[0] <= 15:
                    if len(right_lane_fcar_s) == 1:
                        right_lane_change_ok = True
                        right_check_action = 41
                        right_target_speed = TARGET_SPEED
                    else:
                        if right_lane_fcar_s[1] - self.cur_s >= 50 and self.cur_speed >= right_lane_fcar_v[0] \
                                and 1.1 * cur_lane_closest_car_v[0] < right_lane_fcar_v[1] < TARGET_SPEED:
                            right_lane_change_ok = True
                            right_check_action = 42
                            right_following_index = 1
                            right_target_speed = min(right_lane_fcar_v[1], TARGET_SPEED)
                        elif right_lane_fcar_s[1] - self.cur_s >= 50 and self.cur_speed >= right_lane_fcar_v[0] \
                                and 1.1 * cur_lane_closest_car_v[0] < right_lane_fcar_v[1] >= TARGET_SPEED:
                            right_lane_change_ok = True
                            right_check_action = 41
                            right_target_speed = TARGET_SPEED
                        else:
                            right_lane_change_ok = False
                elif right_lane_fcar_s[0] >= 50:
                    if not right_lane_rcar_v and \
                                                    1.1 * cur_lane_closest_car_v[0] < right_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 42
                        right_target_speed = min(right_lane_fcar_v[1], TARGET_SPEED)
                    elif not right_lane_rcar_v and \
                                                    1.1 * cur_lane_closest_car_v[0] < right_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 41
                        right_target_speed = TARGET_SPEED
                    elif right_lane_rcar_v and self.cur_s - right_lane_rcar_s[0] >= 25 and \
                                                    1.1 * cur_lane_closest_car_v[0] <= right_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 42
                        right_target_speed = min(right_lane_fcar_v[1], TARGET_SPEED)
                    elif right_lane_rcar_v and self.cur_s - right_lane_rcar_s[0] >= 25 and \
                                                    1.1 * cur_lane_closest_car_v[0] <= right_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 41
                        right_target_speed = TARGET_SPEED
                    elif right_lane_rcar_v and self.cur_s - right_lane_rcar_s[0] < 25 and \
                                    self.cur_speed >= right_lane_rcar_v[0] and \
                                                    1.1 * cur_lane_closest_car_v[0] <= right_lane_fcar_v[
                                        0] < TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 42
                        right_target_speed = min(right_lane_fcar_v[1], TARGET_SPEED)
                    elif right_lane_rcar_v and self.cur_s - right_lane_rcar_s[0] < 25 and \
                                    self.cur_speed >= right_lane_rcar_v[0] and \
                                                    1.1 * cur_lane_closest_car_v[0] <= right_lane_fcar_v[
                                        0] >= TARGET_SPEED:
                        right_lane_change_ok = True
                        right_check_action = 41
                        right_target_speed = TARGET_SPEED
                    else:
                        right_lane_change_ok = False
                else:
                    right_lane_change_ok = False
            else:
                if right_lane_rcar_s and self.cur_s - right_lane_rcar_s[0] <= 20 \
                        and self.cur_speed <= right_lane_rcar_v[0]:
                    right_lane_change_ok = False
                else:
                    right_lane_change_ok = True
                    right_check_action = 41
                    right_target_speed = TARGET_SPEED
        return right_lane_change_ok, right_check_action, right_target_speed, right_following_index

    # point stop, action0; cruising action1; following action2;
    # left_lane_change  action3; right_lane_change action4.
    #  behaviour_planning需要的参数target_point_s, cur_lane_closest_car_s, cur_lane_closest_car_v,
    #  (以上3项参数数据类型为list)
    #  cruise_speed,left_lane_change_ok, right_lane_change_ok
    #  分别代表定点停车点在参考路径上的纵向位置s,若无定点停车点，应该输入空的list[]；
    #  当前车道最近前车在参考路径上的纵向位置和车速；
    #  巡航车速（cruise_speed, or TARGET_SPEED）；
    #  向左换道（TRUE，or FALSE）；
    #  向右换道（TRUE，or FALSE）。
    def behaviour_planning(self, target_point_s, cur_lane_closest_car_s, cur_lane_closest_car_v,
                           left_lane_change_ok, right_lane_change_ok, left_check_action, right_check_action):
        # print('left_lane_change_ok', left_lane_change_ok)
        # print('right_lane_change_ok', right_lane_change_ok)
        if target_point_s and target_point_s[0] - self.cur_s <= 40:
            action = 0
        else:
            if cur_lane_closest_car_s:
                if cur_lane_closest_car_s[0] - self.cur_s >= 40:
                    action = 1
                elif cur_lane_closest_car_s[0] - self.cur_s < 40 and \
                                cur_lane_closest_car_v[0] >= TARGET_SPEED:
                    action = 1
                else:
                    if cur_lane_closest_car_v[0] >= 0.9 * TARGET_SPEED and \
                                            cur_lane_closest_car_s[0] - self.cur_s <= 40:
                        action = 2
                        # print('behaviour planning action =2')
                    elif cur_lane_closest_car_v[0] >= 0.9 * TARGET_SPEED and \
                                            cur_lane_closest_car_s - self.cur_s > 40:
                        action = 1
                    elif cur_lane_closest_car_v[0] < 0.9 * TARGET_SPEED and left_lane_change_ok:
                        action = left_check_action
                    elif cur_lane_closest_car_v[0] < 0.9 * TARGET_SPEED and right_lane_change_ok:
                        action = right_check_action
                    else:
                        action = 2
                        # print('behaviour planning else action =2')
            else:
                action = 1
        return action

    # point stop, action0
    def point_stop(self, target_point_s, target_point_d):
        target_s = target_point_s[0]
        target_d = target_point_d[0]
        target_speed = 0.
        target_acc = 0.
        target_lane_num = self.cur_lane_num
        return target_s, target_d, target_speed, target_acc, target_lane_num

    # cruising action1
    # cur_lane_closest_car_s - cur_s > 50 or no car in current lane
    def cruising(self, cruising_speed):
        target_s = 10  # target_s only use for return
        target_d = 0.
        target_speed = cruising_speed
        target_acc = 0.
        target_lane_num = self.cur_lane_num
        return target_s, target_d, target_speed, target_acc, target_lane_num

    # following action2
    # cur_lane_closest_car_s <= 50 and cur_lane_closest_car_v >= 0.9 * cru_speed:
    def following(self, target_car_s, target_car_speed, target_car_acc):
        target_s = target_car_s - tau * target_car_speed
        target_d = 0.
        target_speed = target_car_speed
        target_acc = target_car_acc
        target_lane_num = self.cur_lane_num
        return target_s, target_d, target_speed, target_acc, target_lane_num

    # left_lane_change  action3
    # cur_lane_closest_car_s <= 50 and cur_lane_closest_car_v < 0.9 * cru_speed
    # left_lane_fcar_s, left_lane_fcar_v 左侧车道前车在参考路径下的纵向位置和速度
    def left_lane_change(self, left_check_action, left_target_speed, left_lane_fcar_v,
                         left_lane_fcar_s, left_following_index):
        target_lane_num = self.cur_lane_num + 1
        # target lane exits other car
        if left_check_action == 32:
            target_s = left_lane_fcar_s[left_following_index] - tau * left_lane_fcar_v[left_following_index]
            target_d = 0.
            target_speed = left_target_speed
            target_acc = 0.
            action = 32
        # no car in target lane
        else:
            target_s = 10  # target_s only use for return
            target_d = 0.
            target_speed = TARGET_SPEED
            target_acc = 0
            action = 31
        return target_s, target_d, target_speed, target_acc, target_lane_num, action

    # right_lane_change action4
    # left_lane_change done and go back to original lane
    # right_lane_fcar_s, right_lane_fcar_v 右侧车道前车在参考路径下的纵向位置和速度
    def right_lane_change(self, right_check_action, right_target_speed, right_lane_fcar_v,
                          right_lane_fcar_s, right_following_index):
        target_lane_num = self.cur_lane_num + 1
        # target lane exits other car
        if right_check_action == 42:
            target_s = right_lane_fcar_s[right_following_index] - tau * right_lane_fcar_v[right_following_index]
            target_d = 0.
            target_speed = right_target_speed
            target_acc = 0.
            action = 42
        # no car in target lane
        else:
            target_s = 10  # target_s only use for return
            target_d = 0.
            target_speed = TARGET_SPEED
            target_acc = 0
            action = 41
        return target_s, target_d, target_speed, target_acc, target_lane_num, action


# motion planning (cruising)
# c_speed, c_d, c_d_d, c_d_dd, s0, cur_lane_num, lane_num
# 分别是本车当前车速，距离参考路径横向距离，横向速度，横向加速度，本车在参考路径下的纵向位置，当前车道编号，所有车道编号组
def calc_frenet_paths_cruising(c_speed, c_d, c_d_d, c_d_dd, s0, cur_lane_num, lane_num, action, target_speed, sample_di,
                               mint, maxt):
    frenet_paths = []
    # generate path to each offset goal
    '''if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[0]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    else:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    for di in di_list:'''

        # Lateral motion planning
    for Ti in np.arange(mint, maxt, DT):
        fp = Frenet_path()

        lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, sample_di, 0.0, 0.0, Ti)

        fp.t = [t for t in np.arange(0.0, Ti, DT)]
        fp.d = [lat_qp.calc_point(t) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        # Longitudinal motion planning (Velocity keeping)
        for tv in np.arange(target_speed - D_T_S * N_S_SAMPLE, target_speed + D_T_S * N_S_SAMPLE, D_T_S):
            tfp = copy.deepcopy(fp)
            lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (target_speed - 0.5 * (tfp.s_d[math.floor(0.5 * len(tfp.s_d))] + tfp.s_d[-1])) ** 2
            if action == 31:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] - MAX_ROAD_WIDTH) ** 2
            elif action == 41:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] + MAX_ROAD_WIDTH) ** 2
            else:
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
            tfp.cv = KJ * Js + KT * Ti + KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv
            frenet_paths.append(tfp)
    return frenet_paths


# motion planning (Following)
# 本车在参考路径中的纵向位置，速度，加速度以及前车在参考路径中的纵向位置，速度，加速度
def calc_frenet_paths_following(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                cur_lane_num, lane_num, action):
    frenet_paths = []
    # generate path to each offset goal
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[0]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    else:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    for di in di_list:

        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT + DT, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Following)
            # Assuming that the acceleration of front vehicle is a const

            s_fv1 = s_fv0 + s_fv_d0 * Ti + 1 / 2 * s_fv_dd0 * Ti ** 2
            s_fv_d1 = s_fv_d0 + s_fv_dd0 * Ti
            s_fv_dd1 = s_fv_dd0
            s_target = s_fv1 - (D0 + tau * s_fv_d1)
            s_target_d = s_fv_d1 - tau * s_fv_dd1
            s_target_dd = 0.5 * s_fv_dd0 + 0.5 * s_fv_dd1

            if s_target > s0 + s0_d * Ti + 1 / 2 * MAX_ACCEL ** 2 * Ti:
                s_target = s0 + 1 / 2 * MAX_ACCEL ** 2 * Ti
            if s_target_d > MAX_SPEED:
                s_target_d = 0.98 * MAX_SPEED
            if s_target_dd > MAX_ACCEL:
                s_target_dd = 0.98 * MAX_ACCEL
            tfp = copy.deepcopy(fp)
            lon_qp = quintic_polynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (0.5 * (tfp.s_d[math.floor(0.5 * len(tfp.s_d))] + tfp.s_d[-1]) - s_fv_d0) ** 2

            if action == 32:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] - MAX_ROAD_WIDTH) ** 2
            elif action == 42:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] + MAX_ROAD_WIDTH) ** 2
            else:
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
            tfp.cv = 0.2 * KJ * Js + 0.3 * KT * Ti + 0.3 * KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

            frenet_paths.append(tfp)
    return frenet_paths


# motion planning (point stopping)
# s_stop 停车点在参考路径中的位置
def calc_frenet_paths_stopping(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_stop, cur_lane_num, lane_num):
    frenet_paths = []

    # generate path to each offset goal
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[0]:
        di_list = np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    else:
        di_list = np.arange(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                            1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, D_ROAD_W)
    for di in di_list:

        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT + DT, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Following)
            # Assuming that the acceleration of front vehicle is a const
            s_target = s_stop
            s_target_d = 0
            s_target_dd = 0

            if s_target > s0 + s0_d * Ti + 1 / 2 * MAX_DEC ** 2 * Ti:
                s_target = s0 + 1 / 2 * MAX_DEC ** 2 * Ti

            tfp = copy.deepcopy(fp)
            lon_qp = quintic_polynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (tfp.s_d[-1] - 0) ** 2

            tfp.cd = 0.2 * KJ * Jp + 0.3 * KT * Ti + 0.3 * KD * tfp.d[-1] ** 2
            tfp.cv = 0.2 * KJ * Js + 0.3 * KT * Ti + 0.3 * KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx ** 2 + dy ** 2))

        fp.yaw.append(fp.yaw[len(fp.yaw) - 1])
        fp.ds.append(fp.ds[len(fp.ds) - 1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def calc_global_paths_for_best_path(best_path, csp):
    # calc global positions
    best_path.x = []
    best_path.y = []
    best_path.yaw = []
    best_path.ds= []
    best_path.c = []
    for i in range(len(best_path.s)):
        ix, iy = csp.calc_position(best_path.s[i])
        if ix is None:
            break
        iyaw = csp.calc_yaw(best_path.s[i])
        di = best_path.d[i]
        fx = ix + di * math.cos(iyaw + math.pi / 2.0)
        fy = iy + di * math.sin(iyaw + math.pi / 2.0)
        best_path.x.append(fx)
        best_path.y.append(fy)

    # calc yaw and ds
    for i in range(len(best_path.x) - 1):
        dx = best_path.x[i + 1] - best_path.x[i]
        dy = best_path.y[i + 1] - best_path.y[i]
        best_path.yaw.append(math.atan2(dy, dx))
        best_path.ds.append(math.sqrt(dx ** 2 + dy ** 2))

    best_path.yaw.append(best_path.yaw[len(best_path.yaw) - 1])
    best_path.ds.append(best_path.ds[len(best_path.ds) - 1])

    # calc curvature
    for i in range(len(best_path.yaw) - 1):
        best_path.c.append((best_path.yaw[i + 1] - best_path.yaw[i]) / best_path.ds[i])
    best_path.c.append(best_path.c[-1])


    return best_path



def check_collision(path, veh_width, veh_len, obstacles, other_vehicles):
    collision1 = False
    collision2 = False
    obs_rectangles = []
    other_veh_rectangles = []
    if len(obstacles) > 0:
        for obs in obstacles:
            obs_rectangles.append(collision_checker_obb.Rectangle(obs.x, obs.y, obs.width,
                                                                  obs.length, obs.orientation))
    if len(other_vehicles) > 0:
        for other_vehicle in other_vehicles:
            other_veh_rectangles.append(collision_checker_obb.Rectangle(other_vehicle.x, other_vehicle.y,
                                                                        other_vehicle.width, other_vehicle.length,
                                                                        other_vehicle.orientation))
    path_length = len(path.x)
    for i in range(len(path.x)):
        if not obs_rectangles and not other_veh_rectangles:
            collision1 = False
            collision2 = False
            break
        if i%5==0:
            j = path_length - 1 - i
            # j = i
            ego_vehicle_rectangle = collision_checker_obb.Rectangle(path.x[j], path.y[j], veh_width,
                                                                    veh_len, path.yaw[j])
            collision1 = collision_checker_obb.collision_check_multi(ego_vehicle_rectangle, obs_rectangles)
            if collision1:
                break
            collision2 = collision_checker_obb.collision_check_multi(ego_vehicle_rectangle, other_veh_rectangles)
            if collision2:
                break
    collision = collision1 or collision2
    return collision

def check_global_path_collision(gx, gy,gyaw, veh_width, veh_len, obstacles, other_vehicles):
    collision1 = False
    collision2 = False
    obs_rectangles = []
    other_veh_rectangles = []
    if len(obstacles) > 0:
        for obs in obstacles:
            obs_rectangles.append(collision_checker_obb.Rectangle(obs.x, obs.y, obs.width,
                                                                  obs.length, obs.orientation))
    if len(other_vehicles) > 0:
        for other_vehicle in other_vehicles:
            other_veh_rectangles.append(collision_checker_obb.Rectangle(other_vehicle.x, other_vehicle.y,
                                                                        other_vehicle.width, other_vehicle.length,
                                                                        other_vehicle.orientation))
    path_length = len(gx)
    for i in range(len(gx)):
        if not obs_rectangles and not other_veh_rectangles:
            collision1 = False
            collision2 = False
            break
        if i % 2 == 0:
            j = path_length - 1 - i
            #j = i
            ego_vehicle_rectangle = collision_checker_obb.Rectangle(gx[j], gy[j], veh_width,
                                                                    veh_len, gyaw[j])
            collision1 = collision_checker_obb.collision_check_multi(ego_vehicle_rectangle, obs_rectangles)
            if collision1:
                break
            collision2 = collision_checker_obb.collision_check_multi(ego_vehicle_rectangle, other_veh_rectangles)
            if collision2:
                break
    collision = collision1 or collision2
    return collision

def generate_target_course(x, y):   #根据路点生成三次插值曲线
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    ryaw = []
    for i in range(len(rx) - 1):
        ryaw.append(math.atan2(ry[i + 1] - ry[i], rx[i + 1] - rx[i]))
    ryaw.append(ryaw[-1])
    return rx, ry, ryaw, rk, csp

def generate_target_course_for_pub(x, y, ss):
    csp = cubic_spline_planner.Spline2D(x, y)
    # s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in ss:
        #print('i_s',i_s)
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    ryaw=[]
    for i in range(len(rx)-1):
        try:
            ryaw.append(math.atan2(ry[i + 1] - ry[i], rx[i + 1] - rx[i]))
        except:
            if i == 0:
                ryaw.append(0)
            else:
                ryaw.append(ryaw[i-1])
    ryaw.append(ryaw[-1])

    return rx, ry, ryaw, rk, csp

# check velocity, acceleration, curvature
def check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles,left_road_width,right_road_width,c_d=0.):
    okind = []

    print("left_road_width",left_road_width)
    print ("right_road_width",right_road_width)
    for i in range(len(fplist)):
        if 0<= c_d < left_road_width-0.5*veh_width and any([dl > left_road_width-0.5*veh_width for dl in fplist[i].d]):  # left road edge check
            print("left max over?, yes")
            print(max(fplist[i].d))
            continue
        if -right_road_width + 0.5*veh_width<c_d<0 and any([dr < -right_road_width + 0.5*veh_width for dr in fplist[i].d]):  # right road edge  check
            print("right max over?, yes")
            print(min(fplist[i].d))
            continue
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            print("v max over?, yes")
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            print("a max over?, yes")
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            print("curvature max over?,yes")
            continue
        elif check_collision(fplist[i], veh_width, veh_len, obstacles, other_vehicles):
            print("collision?,yes")
            continue
        okind.append(i)
    return [fplist[i] for i in okind]


def check_global_path_curvature(global_path_x, global_path_y,max_curvature):
    tx, ty, tyaw, tc, csp = generate_target_course(global_path_x, global_path_y)
    '''with open('global_path_max_curvature25.txt','a') as f:
        f.write(str(max(tc))+'\n')'''
    print('global path max curvature', max(tc))
    max_cur = np.mean(tc)
    return any(cur > max_curvature for cur in tc),max_cur


#  与当前车道对应的参考路径
# veh_width, veh_len,ob_x, ob_y, ob_len, ob_width, ob_ori
# 车辆宽度，车辆长度，障碍物的x坐标，y坐标，长度，宽度和方向
def frenet_optimal_planning_cruising(csp, s0, c_speed, c_d, c_d_d, c_d_dd, veh_width, veh_len,
                                     obstacles, other_vehicles, cur_lane_num, lane_num, action,
                                     target_speed, sample_di, old_path1,
                                     left_road_width, right_road_width, implement_time=0.):


    if old_path1:
        start = 0
    if not old_path1:
        start = 1
    if start==1:
        maxt = TOTAL_TIME + 5.1*DT  # max prediction time [s]
        mint = TOTAL_TIME  # min prediction time [s]
        fplist = calc_frenet_paths_cruising(c_speed, c_d, c_d_d, c_d_dd, s0, cur_lane_num, lane_num, action,
                                            target_speed, sample_di, mint, maxt)

        if fplist and action == 31:
            for fp in fplist:
                fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
        if fplist and action == 41:
            for fp in fplist:
                fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]

        fplist = calc_global_paths(fplist, csp)


        fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles,left_road_width,right_road_width,c_d)


        min_cost = float("inf")
        best_path1 = None
        best_path1_id = 0
        for i in np.arange(len(fplist)):
            if min_cost >= fplist[i].cf:
                min_cost = fplist[i].cf
                best_path1 = fplist[i]
                best_path1_id = i
        return best_path1, best_path1_id

    if start==0:
        maxt = implement_time + 0.01 # max prediction time [s]
        mint = implement_time  # min prediction time [s]
        print("len(old_path1.s)",len(old_path1.s))
        print("len(old_path1.d)", len(old_path1.d))
        c_speed = math.sqrt((old_path1.s[-1]-old_path1.s[-2])**2+(old_path1.d[-1]-old_path1.d[-2])**2)/DT
        c_d, c_d_d, c_d_dd, s0 = old_path1.d[-1], old_path1.d_d[-1], old_path1.d_dd[-1],old_path1.s[-1]
        fplist = calc_frenet_paths_cruising(c_speed, c_d, c_d_d, c_d_dd, s0, cur_lane_num, lane_num, action,
                                            target_speed, sample_di, mint, maxt)
        if fplist and action == 31:
            for fp in fplist:
                fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
        if fplist and action == 41:
            for fp in fplist:
                fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]

        fplist = calc_global_paths(fplist, csp)


        fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles,left_road_width,right_road_width,c_d)


        min_cost = float("inf")
        best_path1 = None
        best_path1_id = 0
        for i in np.arange(len(fplist)):
            if min_cost >= fplist[i].cf:
                min_cost = fplist[i].cf
                best_path1 = fplist[i]
                best_path1_id = i
        if best_path1:
            print("aaaaaaaaaaaaaaaa")
            print("old_path.s[-1]",old_path1.s[-1])
            print("best_path1.s[0]",best_path1.s[0])
            old_path1 = path_pop_last_point(old_path1)
            best_path1 = Frenet_path_conjunction(old_path1, best_path1)
        return best_path1, best_path1_id


def frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                      veh_width, veh_len, obstacles, other_vehicles, cur_lane_num,
                                      lane_num, action):
    fplist = calc_frenet_paths_following(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                         cur_lane_num, lane_num, action)

    if fplist and action == 32:
        for fp in fplist:
            fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
    if fplist and action == 42:
        for fp in fplist:
            fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles)
    min_cost = float("inf")
    best_path = None
    best_path_id = 0
    for i in np.arange(len(fplist)):
        if min_cost >= fplist[i].cf:
            min_cost = fplist[i].cf
            best_path = fplist[i]
            best_path_id = i
    return best_path, best_path_id


'''def frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_stop, veh_width, veh_len,
                                     obstacles, other_vehicles, cur_lane_num, lane_num):
    fplist = calc_frenet_paths_stopping(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_stop,
                                        cur_lane_num, lane_num)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles)
    min_cost = float("inf")
    best_path = None
    best_path_id = 0
    for i in np.arange(len(fplist)):
        if min_cost >= fplist[i].cf:
            min_cost = fplist[i].cf
            best_path = fplist[i]
            best_path_id = i
    return best_path, best_path_id'''

def frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, cur_lane_num, lane_num):
    s_stop = s0+8.
    fplist = calc_frenet_paths_stopping(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_stop,
                                        cur_lane_num, lane_num)
    fplist = calc_global_paths(fplist, csp)
    #fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles)
    min_cost = float("inf")
    best_path = None
    best_path_id = 0
    for i in np.arange(len(fplist)):
        if min_cost >= fplist[i].cf:
            min_cost = fplist[i].cf
            best_path = fplist[i]
            best_path_id = i
    return best_path, best_path_id


def prediction(x, y, vx, vy, ref_paths, lane_number, pre_time):
    sr0, d0 = cartesian_to_frenet(x, y, ref_paths, lane_number)
    TIME_SERIES = np.arange(0.1, pre_time + 0.2, 0.2)
    sr = [sr0 + vx * ti for ti in TIME_SERIES]
    d = [d0 + vy * ti for ti in TIME_SERIES]
    d_d = [0 * ti for ti in TIME_SERIES]
    x, y, theta = [], [], []
    for i in range(len(sr)):
        xx, yx, theta_x = frenet_to_cartesian(sr[i], d[i], d_d[i], ref_paths[lane_number])
        x.append(xx)
        y.append(yx)
        theta.append(theta_x)
    return x, y, theta


def path_pop_first_point(path):
    path.t.pop(0)
    path.d.pop(0)
    path.d_d.pop(0)
    path.d_dd.pop(0)
    path.d_ddd.pop(0)
    path.s.pop(0)
    path.s_d.pop(0)
    path.s_dd.pop(0)
    path.s_ddd.pop(0)
    path.x.pop(0)
    path.y.pop(0)
    path.yaw.pop(0)
    path.ds.pop(0)
    path.c.pop(0)
    return path

def path_pop_last_point(path):
    path.t.pop(-1)
    path.d.pop(-1)
    path.d_d.pop(-1)
    path.d_dd.pop(-1)
    path.d_ddd.pop(-1)
    path.s.pop(-1)
    path.s_d.pop(-1)
    path.s_dd.pop(-1)
    path.s_ddd.pop(-1)
    path.x.pop(-1)
    path.y.pop(-1)
    path.yaw.pop(-1)
    path.ds.pop(-1)
    path.c.pop(-1)
    return path

def connect_two_path(path1,path2):
    new_path =Frenet_path()
    new_path.t=path1.t+path2.t
    new_path.d = path1.d + path2.d
    new_path.d_d = path1.d_d+ path2.d_d
    new_path.d_dd = path1.d_dd + path2.d_dd
    new_path.d_ddd = path1.d_ddd + path2.d_ddd
    new_path.s = path1.s + path2.s
    new_path.s_d = path1.s_d + path2.s_d
    new_path.s_dd = path1.s_dd + path2.s_dd
    new_path.s_ddd = path1.s_ddd + path2.s_ddd
    new_path.x = path1.x+path2.x
    new_path.y = path1.y+path2.y
    new_path.yaw = path1.yaw + path2.yaw
    new_path.ds = path1.ds + path2.ds
    new_path.c = path1.c+path2.c
    return new_path

def merge_obstacles(obstacles1, obstacles2):
    obstacles = []
    i = j = 0
    while i < len(obstacles1) and j < len(obstacles2):
        if obstacles1[i].s <= obstacles2[j].s:
            obstacles.append(obstacles1[i])
            i = i + 1
        else:
            obstacles.append(obstacles2[j])
            j = j + 1
    if i < len(obstacles1):
        for obstacle in obstacles1[i:]:
            obstacles.append(obstacle)
    if j < len(obstacles2):
        for obstacle in obstacles2[j:]:
            obstacles.append(obstacle)
    return obstacles


def sort_obstacles(obstacles):
    if len(obstacles) <= 1:
        return obstacles
    else:
        middle_index = math.floor(len(obstacles) / 2)
        left_obstacles = sort_obstacles(obstacles[:middle_index])
        right_obstacles = sort_obstacles(obstacles[middle_index:])
    return merge_obstacles(left_obstacles, right_obstacles)


# select the obstacles 40m in front of ego_vehicle and 20m behind ego_vehicle
def select_obstacles(obstacles, ego_vehicle_s, front_distance, behind_distance):
    new_obstacles = obstacles
    obstacles_num = len(new_obstacles)
    for i in range(len(new_obstacles)):
        if new_obstacles[obstacles_num - 1 - i].s <= ego_vehicle_s - behind_distance or \
                        new_obstacles[obstacles_num - 1 - i].s >= ego_vehicle_s + front_distance:
            new_obstacles.pop(obstacles_num - 1 - i)
    return new_obstacles


def merge_vehicles(vehicles1, vehicles2):
    vehicles = []
    i = j = 0
    while i < len(vehicles1) and j < len(vehicles2):
        if vehicles1[i].sr <= vehicles2[j].sr:
            vehicles.append(vehicles1[i])
            i = i + 1
        else:
            vehicles.append(vehicles2[j])
            j = j + 1
    if i < len(vehicles1):
        for vehicle in vehicles1[i:]:
            vehicles.append(vehicle)
    if j < len(vehicles2):
        for vehicle in vehicles2[j:]:
            vehicles.append(vehicle)
    return vehicles


def sort_vehicles(vehicles):
    if len(vehicles) <= 1:
        return vehicles
    else:
        middle_index = math.floor(len(vehicles) / 2)
        left_vehicles = sort_vehicles(vehicles[:middle_index])
        right_vehicles = sort_vehicles(vehicles[middle_index:])
    return merge_vehicles(left_vehicles, right_vehicles)


def select_vehicles(vehicles, ego_vehicle_s, front_distance, behind_distance):
    new_vehicles = vehicles
    vehicles_num = len(new_vehicles)
    for i in range(len(new_vehicles)):
        if new_vehicles[vehicles_num - 1 - i].sr <= ego_vehicle_s - behind_distance or \
                        new_vehicles[vehicles_num - 1 - i].sr >= ego_vehicle_s + front_distance:
            new_vehicles.pop(vehicles_num - 1 - i)
    return new_vehicles


def pop_obstacle(obstacles):
    obstacles.xs.pop(0)
    obstacles.ys.pop(0)
    obstacles.widths.pop(0)
    obstacles.lengths.pop(0)
    obstacles.orientations.pop(0)
    return obstacles


def show_vehicle(vehicle_path, color='blue', line_width=0.5):
    vehicles = []
    for i in range(len(vehicle_path.x)):
        if i % 3 == 0:
            vehicle = collision_checker_obb.Rectangle(vehicle_path.x[i], vehicle_path.y[i],
                                                      veh_width, veh_len, vehicle_path.yaw[i])
            vehicles.append(vehicle)
    collision_checker_obb.show_rectangles([vehicles[0]], color="black", line_width=1.5)
    collision_checker_obb.show_rectangles(vehicles, color=color, line_width=line_width)


def show_obstacles(obstacles):
    rec_obstacles = []
    for i in range(len(obstacles)):
        rec_obstacle = collision_checker_obb.Rectangle(obstacles[i].x, obstacles[i].y, obstacles[i].width,
                                                       obstacles[i].length, obstacles[i].orientation)
        rec_obstacles.append(rec_obstacle)
    collision_checker_obb.show_rectangles(rec_obstacles, color='red', line_width=0.5)


def get_other_vehicle_paras(behaviour, lane0_veh, lane1_veh):
    cur_lane_closest_car_s = []
    cur_lane_closest_car_v = []
    left_lane_fcar_s = []
    left_lane_fcar_v = []
    left_lane_rcar_s = []
    left_lane_rcar_v = []
    right_lane_fcar_s = []
    right_lane_fcar_v = []
    right_lane_rcar_s = []
    right_lane_rcar_v = []

    s0 = behaviour.cur_s
    # get left lane and right lane vehicles paras
    if behaviour.cur_lane_num == 0:
        right_lane_fcar_s = []
        right_lane_fcar_v = []
        right_lane_rcar_s = []
        right_lane_rcar_v = []
        if len(lane1_veh) == 0:
            left_lane_fcar_s = []
            left_lane_fcar_v = []
            left_lane_rcar_s = []
            left_lane_rcar_v = []
        elif len(lane1_veh) == 1:
            if lane1_veh[0].sr >= s0:
                left_lane_fcar_s.append(lane1_veh[0].sr)
                left_lane_fcar_v.append(lane1_veh[0].vx)
            else:
                left_lane_rcar_s.append(lane1_veh[0].sr)
                left_lane_rcar_v.append(lane1_veh[0].vx)
        else:
            for i in range(len(lane1_veh)):
                if lane1_veh[i].sr <= s0:
                    left_lane_rcar_s.append(lane1_veh[i].sr)
                    left_lane_rcar_v.append(lane1_veh[i].vx)
                else:
                    left_lane_fcar_s.append(lane1_veh[i].sr)
                    left_lane_fcar_v.append(lane1_veh[i].vx)

    if behaviour.cur_lane_num == 1:
        left_lane_fcar_s = []
        left_lane_fcar_v = []
        left_lane_rcar_s = []
        left_lane_rcar_v = []
        if len(lane0_veh) == 0:
            right_lane_fcar_s = []
            right_lane_fcar_v = []
            right_lane_rcar_s = []
            right_lane_rcar_v = []
        elif len(lane0_veh) == 1:
            if lane0_veh[0].sr >= s0:
                right_lane_fcar_s.append(lane0_veh[0].sr)
                right_lane_fcar_v.append(lane0_veh[0].vx)
            else:
                right_lane_rcar_s.append(lane0_veh[0].sr)
                right_lane_rcar_v.append(lane0_veh[0].vx)
        else:
            for i in range(len(lane0_veh)):
                if lane0_veh[i].sr <= s0:
                    right_lane_rcar_s.append(lane0_veh[i].sr)
                    right_lane_rcar_v.append(lane0_veh[i].vx)
                else:
                    right_lane_fcar_s.append(lane0_veh[i].sr)
                    right_lane_fcar_v.append(lane0_veh[i].vx)
    if behaviour.cur_lane_num == 0:
        if lane0_veh:
            for i in range(len(lane0_veh)):
                if lane0_veh[i].sr > s0:
                    cur_lane_closest_car_s.append(lane0_veh[i].sr)
                    cur_lane_closest_car_v.append(lane0_veh[i].vx)
                    break
    if behaviour.cur_lane_num == 1:
        if lane1_veh:
            for i in range(len(lane1_veh)):
                if lane1_veh[i].sr > s0:
                    cur_lane_closest_car_s.append(lane1_veh[i].sr)
                    cur_lane_closest_car_v.append(lane1_veh[i].vx)
                    break
    return cur_lane_closest_car_s, cur_lane_closest_car_v, left_lane_fcar_s, left_lane_fcar_v, \
           left_lane_rcar_s, left_lane_rcar_v, right_lane_fcar_s, right_lane_fcar_v, \
           right_lane_rcar_s, right_lane_rcar_v


def get_best_path(action, behaviour, cur_lane_num, last_best_path, lane_num, ref_paths, obstacles,
                  other_vehicles, target_point_s, target_point_d, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                  c_speed, cur_lane_closest_car_s, cur_lane_closest_car_v):
    best_path = []
    new_behaviour = behaviour
    if action == 0:  # point stop
        target_s, target_d, target_speed, target_acc, target_lane_num \
            = behaviour.point_stop(target_point_s, target_point_d)
        csp = ref_paths[cur_lane_num].csp
        best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                                                        target_s, veh_width, veh_len, obstacles, other_vehicles,
                                                        cur_lane_num, lane_num)
        if not best_path:
            print('no best for point stop!')
            if len(last_best_path.s) > 1 and \
                    not check_collision(last_best_path, veh_width, veh_len, obstacles, other_vehicles):
                best_path = path_pop_first_point(last_best_path)

    elif action == 1:  # cruising
        csp = ref_paths[cur_lane_num].csp
        target_speed = TARGET_SPEED
        best_path, _ = frenet_optimal_planning_cruising(csp, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                        veh_width, veh_len, obstacles, other_vehicles,
                                                        cur_lane_num, lane_num, action, target_speed)
        if not best_path:
            print('no best path for cruising!')
            for target_speed in np.arange(0.9 * TARGET_SPEED, 0.4 * TARGET_SPEED, -0.2 * TARGET_SPEED):
                best_path, _ = frenet_optimal_planning_cruising(csp, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                veh_width, veh_len, obstacles,
                                                                other_vehicles, cur_lane_num, lane_num,
                                                                action, target_speed)
                if best_path:
                    break
            if not best_path:
                best_path = []
                for s_i in np.arange(10, 40, 3):
                    target_s = s0 + s_i
                    best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                                                                    target_s, veh_width, veh_len, obstacles,
                                                                    other_vehicles, cur_lane_num, lane_num)
                    if best_path:
                        break

    elif action == 2:  # following
        csp = ref_paths[cur_lane_num].csp
        s_fv0 = cur_lane_closest_car_s[0]
        s_fv_d0 = min(cur_lane_closest_car_v[0], TARGET_SPEED)
        s_fv_dd0 = 0
        # print('action == 2 s_fv_0', s_fv0)
        # print('action == 2 s_fv_d0', s_fv_d0)
        for target_s in np.arange(s_fv0, s_fv0 - 15, -4):
            s_fv0 = target_s
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                                                             s_fv0, s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, action)
            if best_path:
                break
        if not best_path:
            print('no best path for following!')
            for s_i in np.arange(35, 50, 4):
                for target_d in np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                                          0.5 * MAX_ROAD_WIDTH - 0.5 * veh_width + 0.2, 0.2):
                    target_s = s0 + s_i
                    target_point_s.append(target_s)
                    target_point_d.append(target_d)
                    best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, target_d, c_d_d, c_d_dd,
                                                                    target_s, veh_width, veh_len, obstacles,
                                                                    other_vehicles, cur_lane_num, lane_num)
    elif action == 32:  # left lane change
        cur_lane_num = cur_lane_num + 1
        csp = ref_paths[cur_lane_num].csp
        s_fv0 = cur_lane_closest_car_s[0]
        s_fv_d0 = min(cur_lane_closest_car_v[0], TARGET_SPEED)
        s_fv_dd0 = 0
        for target_s in np.arange(s_fv0, s_fv0 - 15, -4):
            s_fv0 = target_s
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                                                             s_fv0, s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, action)
            if best_path:
                break
        if best_path:
            new_behaviour.cur_lane_num = behaviour.cur_lane_num + 1
            cur_lane_num = new_behaviour.cur_lane_num
        if not best_path:
            print('no best path for left_lane_change and following!')
            cur_lane_num = cur_lane_num - 1
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0,
                                                             s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, 2)
    elif action == 31:  # cruising in left lane
        cur_lane_num = cur_lane_num + 1
        print('get path cur_lane_num', cur_lane_num)
        csp = ref_paths[cur_lane_num].csp
        target_speed = TARGET_SPEED
        best_path, _ = frenet_optimal_planning_cruising(csp, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                        veh_width, veh_len, obstacles, other_vehicles,
                                                        cur_lane_num, lane_num, action, target_speed)
        if best_path:
            new_behaviour.cur_lane_num = behaviour.cur_lane_num + 1
            cur_lane_num = new_behaviour.cur_lane_num
            print('if best path cur_lane_num', new_behaviour.cur_lane_num)
        if not best_path:
            print('no best path for left_lane_change and cruising!')
            cur_lane_num = cur_lane_num - 1
            s_fv0 = cur_lane_closest_car_s[0]
            s_fv_d0 = cur_lane_closest_car_v[0]
            s_fv_dd0 = 0
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0,
                                                             s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, 2)

    elif action == 42:  # right lane change
        cur_lane_num = cur_lane_num - 1
        csp = ref_paths[cur_lane_num].csp
        s_fv0 = cur_lane_closest_car_s[0]
        s_fv_d0 = min(cur_lane_closest_car_v[0], TARGET_SPEED)
        s_fv_dd0 = 0
        for target_s in np.arange(s_fv0, s_fv0 - 15, -4):
            s_fv0 = target_s
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                                                             s_fv0, s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, action)
            if best_path:
                break
        if best_path:
            new_behaviour.cur_lane_num = behaviour.cur_lane_num - 1
            cur_lane_num = new_behaviour.cur_lane_num
        if not best_path:
            print('no best path for right_lane_change and following!')
            cur_lane_num = cur_lane_num + 1
            s_fv0 = cur_lane_closest_car_s[0]
            s_fv_d0 = cur_lane_closest_car_v[0]
            s_fv_dd0 = 0
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0,
                                                             s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, 2)
    elif action == 41:  # cruising in right lane
        cur_lane_num = cur_lane_num - 1
        csp = ref_paths[cur_lane_num].csp
        target_speed = TARGET_SPEED
        best_path, _ = frenet_optimal_planning_cruising(csp, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                        veh_width, veh_len, obstacles, other_vehicles,
                                                        cur_lane_num, lane_num, action, target_speed)
        if best_path:
            new_behaviour.cur_lane_num = behaviour.cur_lane_num - 1
            cur_lane_num = new_behaviour.cur_lane_num
        if not best_pglobal_pathath:
            cur_lane_num = cur_lane_num + 1
            s_fv0 = cur_lane_closest_car_s[0]
            s_fv_d0 = cur_lane_closest_car_v[0]
            s_fv_dd0 = 0
            best_path, _ = frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0,
                                                             s_fv_d0, s_fv_dd0, veh_width, veh_len, obstacles,
                                                             other_vehicles, cur_lane_num, lane_num, 2)
    if not best_path:
        print('no best path for lane_change and cruising!')
        csp = ref_paths[cur_lane_num].csp
        if obstacles:
            ob_s = obstacles[0].s
            if (ob_s - s0) > 50:
                for s_i in np.arange(25, 45, 5):
                    for target_d in np.arange(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                                              0.5 * MAX_ROAD_WIDTH - 0.5 * veh_width + 0.2, 0.2):
                        target_s = s0 + s_i
                        target_point_s.append(target_s)
                        target_point_d.append(target_d)
                        best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, target_d, c_d_d, c_d_dd,
                                                                        target_s, veh_width, veh_len, obstacles,
                                                                        other_vehicles, cur_lane_num, lane_num)
                        if best_path:
                            break
                    else:
                        continue
                    break

            else:
                for s_i in np.arange(5, 20, 3):
                    target_s = ob_s - s_i
                    target_ds = np.arange(-0.5 * MAX_ROAD_WIDTH, 0.5 * MAX_ROAD_WIDTH, 0.5)
                    for target_d in target_ds:
                        target_point_s = [target_s]
                        target_point_d = [target_d]
                        best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, target_d, c_d_d, c_d_dd,
                                                                        target_s, veh_width, veh_len, obstacles,
                                                                        other_vehicles, cur_lane_num, lane_num)
                        if best_path:
                            break
                    else:
                        continue
                    break
        else:
            for s_i in np.arange(25, 40, 5):
                target_s = s0 + s_i
                target_d = 0
                target_point_s.append(target_s)
                target_point_d.append(target_d)
                print("s0:", s0, "target_s:", target_s)
                best_path, _ = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, target_d, c_d_d, c_d_dd,
                                                                target_s, veh_width, veh_len, obstacles,
                                                                other_vehicles, cur_lane_num, lane_num)
                if best_path:
                    break
                print("no best path no ob", len([best_path]))
    return best_path, new_behaviour


def turn_collision_check(ego_car, obstacles):
    collision = False
    # print('num of obs:', len(obstacles))
    if not obstacles:
        return collision
    elif len(obstacles) == 1:
        return collision_checker_obb.collision_check_single(ego_car, obstacles)
    else:
        num_of_obs = len(obstacles)
        for i in range(len(obstacles)):
            # j = num_of_obs - 1 - i
            j = i
            collision = collision_checker_obb.collision_check_single(ego_car, obstacles[j])
            if collision:
                break
    return collision


def turn_path_planning(sx, sy, syaw, gx, gy, gyaw, egoCar_width, egoCar_length, obstacles, cur_speed, minr):
    if minr < 5:
        minr = 5
    maxr = 10
    rStep = 2
    step_size = 0.8
    found_bestpath = False
    best_path = []
    path_indexs = []
    px, py, pyaw, pt, pspeed = [], [], [], [], []
    new_obstacles = []
    if not obstacles:
        new_obstacles = []
    if len(obstacles) >= 1:
        for i in range(len(obstacles)):
            new_obstacles.append(collision_checker_obb.Rectangle(obstacles[i].x, obstacles[i].y, obstacles[i].width,
                                                                 obstacles[i].length, obstacles[i].orientation))
    while not found_bestpath and minr <= maxr:
        print(found_bestpath)
        paths = reeds_shepp_planner.get_paths(sx, sy, syaw, gx, gy, gyaw, minr, step_size)
        # (len(paths))
        if len(paths) == 0:
            minr = float("Inf")
            break
        for i in range(len(paths)):
            if any(length < 0 for length in paths[i].lengths):
                path_indexs.append(i)
                continue
            for j in range(len(paths[i].x)):
                if j % 8 == 0:
                    ego_car = collision_checker_obb.Rectangle(paths[i].x[j], paths[i].y[j], egoCar_width, egoCar_length,
                                                              paths[i].yaw[j])
                    if turn_collision_check(ego_car, new_obstacles):
                        path_indexs.append(i)
                        break
        # print('path_indexs', path_indexs)
        for i in range(len(path_indexs)):
            pathIndex = path_indexs.pop()
            # print('pathIndex', pathIndex)
            # print('len(paths)', len(paths))
            paths.pop(pathIndex)
        minL = float("Inf")
        if paths:
            best_path_index = -1
            for i in range(len(paths)):
                if paths[i].L <= minL:
                    minL = paths[i].L
                    best_path_index = i
            best_path = paths[best_path_index]
        if not paths:
            best_path = []
        if not best_path:
            found_bestpath = False
            minr = minr + rStep

        if best_path:
            found_bestpath = True

    if found_bestpath:
        # print(len(best_path.x))
        # print(best_path.lengths)
        # print(minr)

        distances = []
        px, py, pyaw = best_path.x, best_path.y, best_path.yaw
        pt = [0]
        # print('len(best_path.x)', len(best_path.x))
        for i in range(len(px) - 1):
            distance = math.sqrt((px[i + 1] - px[i]) ** 2 + (py[i + 1] - py[i]) ** 2)
            distances.append(distance)
            pt.append(pt[-1] + distance / cur_speed)
        # print(distance)nav_msgs
        # print(pt)
        pspeed = np.array([cur_speed] * len(px))
    return px, py, pspeed, pt, pyaw


# start_time = time1
def interplating_and_transforming_curover(pub_x, pub_y, veh_ve_v, start_time,inter_nums):
    big_interval_time = DT
    small_interval_time = big_interval_time/(inter_nums+1)
    tx21, ty21, tyaw21, tc21, csp21 = generate_target_course(pub_x, pub_y)
    max_s = csp21.s[-1]
    pub_best_path_s = np.arange(0., max_s, veh_ve_v * small_interval_time)
    tx211, ty211, tyaw211, tc211, csp211 = generate_target_course_for_pub(pub_x, pub_y, pub_best_path_s)
    pub_veh_x = tx211
    pub_veh_y = ty211
    pub_veh_yaw = tyaw211
    pub_curvature = tc211
    pub_velocity = []
    pub_yaw_rate = []
    pub_acc = []
    for i in range(len(tx211) - 1):
        pub_velocity.append(
            math.sqrt((tx211[i + 1] - tx211[i]) ** 2 + (ty211[i + 1] - ty211[i]) ** 2) / small_interval_time)
    pub_velocity.append(pub_velocity[-1])
    for i in range(len(pub_veh_yaw) - 1):
        pub_yaw_rate.append((pub_veh_yaw[i + 1] - pub_veh_yaw[i]) / small_interval_time)
    pub_yaw_rate.append(pub_yaw_rate[-1])
    for i in range(len(pub_velocity) - 1):
        pub_acc.append((pub_velocity[i + 1] - pub_velocity[i]) / small_interval_time)
    pub_acc.append(pub_acc[-1])
    pub_sum_dist = pub_best_path_s
    #pub_time_diff = list(small_interval_time* np.array(range(len(pub_best_path_s))))
    header = Header(0, rospy.Time.now(), 'world')
    trajectory_points = []
    for i in range(len(pub_velocity)):
        trajectory_points.append(
            TrajectoryPoint(header, Pose2D(pub_veh_x[i], pub_veh_y[i], pub_veh_yaw[i]),
                            pub_curvature[i],
                            pub_velocity[i], pub_yaw_rate[i], pub_acc[i], pub_sum_dist[i],
                            0.05))
    trajectory = Trajectory(header, trajectory_points)

    end_time = time.time()
    ## start_time = time1
    drop_frame = math.ceil((end_time - start_time) / DT)
    rviz_pub_x = pub_x[drop_frame:-1]
    rviz_pub_y = pub_y[drop_frame:-1]
    rviz_pub_speed = pub_speeds[drop_frame:-1]
    rviz_pub_t = pub_t[drop_frame:-1]
    rviz_pub_th = pub_th[drop_frame:-1]

    return rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory

def interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time,inter_nums,s0):
    big_interval_time = DT
    small_interval_time = big_interval_time / (inter_nums + 1)
    s1 = best_path.s[0]
    best_path.s = list(np.array(best_path.s) -s1  + s0 + 1e-5)
    tx22, ty22,tc22 = best_path.x, best_path.y,best_path.c
    min_len = min(len(tx22),len(ty22),len(tc22))
    tyaw22=[]
    for i in range(min_len-1):
        tyaw22.append(math.atan2(ty22[i + 1] - ty22[i], tx22[i + 1] - tx22[i]))
    tyaw22.append(tyaw22[-1])

    pub_veh_x = tx22[0:min_len]
    pub_veh_y = ty22[0:min_len]
    pub_veh_yaw = tyaw22[0:min_len]
    pub_curvature = tc22[0:min_len]
    pub_velocity = []
    pub_yaw_rate = []
    pub_acc = []

    for i in range(min_len - 1):
        pub_velocity.append(math.sqrt((tx22[i + 1] - tx22[i]) ** 2 + (ty22[i + 1] - ty22[i]) ** 2) / DT)
    pub_velocity.append(pub_velocity[-1])
    for i in range(min_len - 1):
        pub_yaw_rate.append((pub_veh_yaw[i + 1] - pub_veh_yaw[i]) / DT)
    pub_yaw_rate.append(pub_yaw_rate[-1])
    for i in range(min_len - 1):
        pub_acc.append((pub_velocity[i + 1] - pub_velocity[i]) / DT)
    pub_acc.append(pub_acc[-1])
    pub_best_path_s = best_path.s[0:min_len]
    pub_sum_dist = pub_best_path_s
    #pub_time_diff = list(small_interval_time* np.array(range(len(pub_best_path_s))))
    header = Header(0, rospy.Time.now(), 'world')
    trajectory_points = []
    for i in range(len(pub_velocity)):
        trajectory_points.append(
            TrajectoryPoint(header, Pose2D(pub_veh_x[i], pub_veh_y[i], pub_veh_yaw[i]), pub_curvature[i],
                            pub_velocity[i], pub_yaw_rate[i], pub_acc[i], pub_sum_dist[i], DT))
    trajectory = Trajectory(header, trajectory_points)

    pub_x = best_path.x
    pub_y = best_path.y
    pub_th = best_path.yaw
    pub_t = list(DT * np.arange(len(best_path.x)))
    distances = [0]
    pub_speeds = [veh_ve_vx]
    for i in range(len(best_path.x) - 1):
        distances.append(
            math.sqrt((best_path.x[i + 1] - best_path.x[i]) ** 2 + (best_path.y[i + 1] - best_path.y[i]) ** 2))
    for i in range(len(distances) - 2):
        pub_speeds.append((distances[i + 1] + distances[i + 2]) / (2 * DT))
    pub_speeds.append(pub_speeds[-1])

    end_time = time.time()
    drop_frame = math.ceil((end_time - start_time) / DT)
    rviz_pub_x = pub_x[drop_frame:-1]
    rviz_pub_y = pub_y[drop_frame:-1]
    rviz_pub_speed = pub_speeds[drop_frame:-1]
    rviz_pub_t = pub_t[drop_frame:-1]
    rviz_pub_th = pub_th[drop_frame:-1]

    return rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory

def imterplating_no_obstacle_pub(path,ref_path,veh_ve_vx):
    best_path = copy.deepcopy(path)

    #best_path.s = [si + s0 - s1 for si in best_path.s]
    '''print ("best_path.s", best_path.s)

    print("********************************************************")
    print(len(best_path.s))
    print("best_path.s", best_path.s)
    print("********************************************************")'''
    best_path = calc_global_paths_for_best_path(best_path, ref_path)
    time3 = time.time()
    tx22, ty22, tyaw22, tc22 = best_path.x, best_path.y, best_path.yaw, best_path.c
    min_len = min(len(tx22), len(ty22), len(tc22),len(tyaw22))
    print("tx22",tx22)
    print("ty22",ty22)
    time4 = time.time()
    '''print("********************************************************")
    print("cost time generate_target_course_for_pu", time4 - time3)
    print("********************************************************")'''
    pub_path_s = best_path.s
    pub_veh_x = tx22[0:min_len]
    pub_veh_y = ty22[0:min_len]
    pub_veh_yaw = tyaw22[0:min_len]
    pub_curvature = tc22[0:min_len]
    pub_velocity = []
    pub_yaw_rate = []
    pub_acc = []


    for i in range(min_len - 1):
        pub_velocity.append(
            math.sqrt((pub_veh_x[i + 1] - pub_veh_x[i]) ** 2 + (
                pub_veh_y[i + 1] - pub_veh_y[i]) ** 2) / DT)
    pub_velocity.append(pub_velocity[-1])
    for i in range(len(pub_veh_yaw) - 1):
        pub_yaw_rate.append((pub_veh_yaw[i + 1] - pub_veh_yaw[i]) / DT)
    pub_yaw_rate.append(pub_yaw_rate[-1])
    for i in range(len(pub_velocity) - 1):
        pub_acc.append((pub_velocity[i + 1] - pub_velocity[i]) / DT)
    pub_acc.append(pub_acc[-1])

    pub_sum_dist = pub_path_s
    # pub_time_diff=list(small_interval_time* np.array(range(path.s)))

    pub_veh_yaw = []
    for i in range(min_len - 1):
        pub_veh_yaw.append(math.atan2(pub_veh_y[i + 1] - pub_veh_y[i], pub_veh_x[i + 1] - pub_veh_x[i]))
    pub_veh_yaw.append(pub_veh_yaw[-1])
    print("pub_veh_x",pub_veh_x)
    print("pub_veh_y",pub_veh_y)
    header = Header(0, rospy.Time.now(), 'world')
    trajectory_points = []
    for i in range(len(pub_veh_x)):
        trajectory_points.append(
            TrajectoryPoint(header, Pose2D(pub_veh_x[i], pub_veh_y[i], pub_veh_yaw[i]), pub_curvature[i],
                            pub_velocity[i], pub_yaw_rate[i], pub_acc[i], pub_sum_dist[i], 0.05))

    trajectory = Trajectory(header, trajectory_points)
    pub_x = tx22
    pub_y = ty22
    pub_th = tyaw22
    pub_t = list(DT * np.arange(len(tx22)))
    distances = [0]
    pub_speeds = [veh_ve_vx]
    for i in range(len(tx22) - 1):
        distances.append(
            math.sqrt((tx22[i + 1] - tx22[i]) ** 2 + (ty22[i + 1] - ty22[i]) ** 2))
    for i in range(len(distances) - 2):
        pub_speeds.append((distances[i + 1] + distances[i + 2]) / (2 * DT))
    pub_speeds.append(pub_speeds[-1])

    end_time = time.time()
    #start_time = time1
    #drop_frame = math.ceil((end_time - start_time) / DT)
    rviz_pub_x = pub_x
    rviz_pub_y = pub_y
    rviz_pub_speed = pub_speeds
    rviz_pub_t = pub_t
    rviz_pub_th = pub_th

    return  best_path,trajectory,rviz_pub_x,rviz_pub_y,rviz_pub_speed,rviz_pub_t,rviz_pub_th

def get_keeping_speed_path(current_speed, current_s,current_d,ref_path_csp,total_time=TOTAL_TIME):
    dt = 0.25
    path = Frenet_path()
    path.t = list(np.arange(0,total_time,dt))
    path_points_num = len(path.t)
    for i in np.arange(path_points_num):
        path.d_d.append(0.)
        path.d_dd.append(0.)
        path.d_ddd.append(0.)
        path.s.append(current_s+current_speed*dt*i)
        path.s_d.append(0.)
        path.s_dd.append(0.)
        path.s_ddd.append(0.)
        path.d.append(current_d)


    path.cd = 1e6
    path.cv = 1e6
    path.cf = 1e6
    # global path params
    path = calc_global_paths([path], ref_path_csp)
    return path[0]

# todo if once stopping generated, it should be executed totally
def get_stopping_path(current_speed, current_d,ref_path_csp):
    path = Frenet_path()
    path.t = list(np.arange(0,EMERGENCY_STOP_TIME,DT))
    for i in np.arange(len(path.t)):
        path.d.append(current_d)
        path.d_d.append(0)
        path.d_dd.append(0)
        path.d_ddd.append(0)
        path.s_d.append(0)
        path.s_dd.append(0)
        path.s_ddd.append(0)
        if i==0:
            path.s.append(0)
        if i!=0:
            path.s.append((len(path.t) - i) / len(path.t) * current_speed * DT + path.s[i - 1])

    path.cd = 1e6
    path.cv = 1e6
    path.cf = 1e6
    # global path params
    path = calc_global_paths([path], ref_path_csp)
    return path[0]

class sub_and_pub():
    def __init__(self):

        self.global_x = []
        self.global_y = []
        self.cur_x = 0.
        self.cur_y = 0.
        self.yaw = 0.
        self.yawrate = 0.
        # self.last_x = 0
        # self.last_y = 0
        self.ve_vx = 0.
        self.ve_vy = 0.
        self.ve_v = 0.
        self.ve_th = 0.
        self.Ax = 0.
        self.Ay = 0.
        self.obs = []
        self.s0 = 0.
        self.c_d = 0.
        self.s0_d =0.
        self.c_d_d = 0.
        self.s0_dd = 0.
        self.c_d_dd = 0.



        self.path_subscriber = rospy.Subscriber('/path_points_1', Test, self.global_callback, queue_size=1)
        self.loca_subscriber = rospy.Subscriber('/gnss_odometry', Odometry, self.gps_callback, queue_size=1)
        self.yaw_subscriber = rospy.Subscriber('/gnss_odom', Gnss, self.odom_callback, queue_size=1)
        self.acc_subscriber = rospy.Subscriber('/gnss_imu', Imu, self.acc_callback, queue_size=1)
        self.obs_subscriber = rospy.Subscriber('/rs_percept_result', PerceptionListMsg, self.ob_callback, queue_size=1)
        #self.obs_subscriber = rospy.Subscriber('/delphi_0/delphi_esr', delphi_msges, self.ob_callback, queue_size=1)
        self.road_width_subscriber = rospy.Subscriber('road_width', Test, self.road_width_callback, queue_size=1)

        # FOR RVIZ
        self.traj_publisher = rospy.Publisher('/planning/traj_pts', Traj, queue_size=1)
        # FOR CONTROL
        self.carstate_publisher = rospy.Publisher('/estimation/slam/state', CarState, queue_size=1)
        # FOR CONTROL
        self.trajectory_publisher = rospy.Publisher('/planning/TrajectoryPoint', Trajectory, queue_size=1)

        self.time = time.time()
        self.tt0 = 0
        self.pub_x = []
        self.pub_y = []
        self.pub_speed = []
        self.pub_t = []
        self.pub_th = []

    def global_callback(self, data_gl):
        assert isinstance(data_gl, Test)
        self.global_x = data_gl.data_x[0:40] #为什么是40而不是全部，全局路径实时发布？
        self.global_y = data_gl.data_y[0:40]


    def odom_callback(self, odom_data):
        assert isinstance(odom_data, Gnss)
        # print('odom_data', odom_data)
        self.yaw = odom_data.yaw + 0.5 * math.pi
        #print('self.yaw', self.yaw)

    def acc_callback(self, imu_data):
        assert isinstance(imu_data, Imu)
        self.Ax = imu_data.linear_acceleration.x
        self.Ay = imu_data.linear_acceleration.y
        self.yawrate = imu_data.angular_velocity.z

    def gps_callback(self, data_gps):
        assert isinstance(data_gps, Odometry)
        print("gps odometry")
        x_pos = data_gps.pose.pose.position.x
        y_pos = data_gps.pose.pose.position.y
        t = time.time()
        self.ve_vx = data_gps.twist.twist.linear.x  # (x_pos-self.cur_x)/(t-self.time)
        self.ve_vy = data_gps.twist.twist.linear.y  # data_gps.(y_pos-self.cur_y)/(t-self.time)
        self.ve_v = math.sqrt(self.ve_vx ** 2 + self.ve_vy ** 2)
        self.cur_x = x_pos
        self.cur_y = y_pos
        self.ve_th = math.atan2(self.ve_vy, self.ve_vx)
        self.time = t


        # print('len of time_series', len(TIME_SERIES))

        carstate_header = Header(0., rospy.Time.now(), 'world')
        carstate0 = Pose2D(self.cur_x, self.cur_y, self.yaw)
        carstatedt = CarStateDt(carstate_header, Pose2D(self.ve_vx, self.ve_vy, self.yawrate), Pose2D(self.Ax, self.Ay, 0.))
        self.carstate_publisher.publish(carstate_header, carstate0, carstatedt)

    def road_width_callback(self, data_road_width):
        assert isinstance(data_road_width, Test)
        self.left_width = data_road_width.data_x[0]
        self.right_width = data_road_width.data_y[0]


    def ob_callback(self, data_ob):

        assert isinstance(data_ob, PerceptionListMsg)

        # params initialization
        time1 = time.time()
        # get vehicle params under global frame
        veh_yaw = self.yaw
        veh_cur_x = self.cur_x
        veh_cur_y = self.cur_y
        veh_ve_vx = self.ve_vx
        veh_ve_vy = self.ve_vy
        veh_ve_v = self.ve_v
        c_speed = self.ve_v
        veh_ve_th = self.ve_th
        veh_ax = self.Ax
        veh_ay = self.Ay
        # get ref path
        global_x = self.global_x
        global_y = self.global_y
        tx1, ty1, tyaw1, tc1, csp1 = generate_target_course(global_x, global_y)
        #print("********************************************************")
        #print("ref path tyaw", tyaw1)
        #print("********************************************************")
        # max_curvature is use for speed down when turning
        max_curvature = max(tc1)
        # sudden sheer change happends in some situation,
        # so linear interpolation yaw is introduced
        tyaw1=[]
        for i in range(len(tx1)-1):
            tyaw1.append(math.atan2(ty1[i + 1] -ty1[i], tx1[i + 1] - tx1[i]))
        tyaw1.append(tyaw1[-1])
        global_path_max_curvature = max(tc1)
        if global_path_max_curvature<0.20:
            sample_speed = [TARGET_SPEED,0.8*TARGET_SPEED,1.1*TARGET_SPEED]
        if global_path_max_curvature>=0.20:
            print("turning needed")
            sample_speed = [0.8 * TARGET_SPEED, 0.7 * TARGET_SPEED, 0.6*TARGET_SPEED]
        ref_path = ReferencePath(tx1, ty1, tyaw1, tc1, csp1)
        ref_paths = [ref_path]
        best_path = None
        lane_num = np.arange(len(ref_paths))
        cur_lane_num = 0

        # get vehicle params under frenet frame
        self.s0, self.c_d, _, _ = cartesian_to_frenet(veh_cur_x, veh_cur_y, veh_ve_v, veh_ve_th, ref_paths,
                                                      cur_lane_num)
        global_path_yaw = get_global_path_nearest_yaw(veh_cur_x, veh_cur_y, ref_paths, cur_lane_num)
        self.s0_d = veh_ve_vx * math.cos(global_path_yaw) + veh_ve_vy * math.sin(global_path_yaw)
        self.c_d_d = veh_ve_vy * math.cos(global_path_yaw) - veh_ve_vx * math.sin(global_path_yaw)
        self.s0_dd = veh_ax* math.cos(global_path_yaw) + veh_ay * math.sin(global_path_yaw)
        self.c_d_dd = veh_ay * math.cos(global_path_yaw) - veh_ax * math.sin(global_path_yaw)

        s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd = self.s0, self.c_d, self.s0_d, self.c_d_d, self.s0_dd, self.c_d_dd

        if s0_d<2.5 :
            s0_d =2.5
        # In general the lateral acc of vehicle is small,
        # so to avoid impact of pitching and rolling, we assume it to be 0
        c_d_dd = 0.
        print('s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd:', s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd)

        # get road width
        left_road_width = self.left_width
        right_road_width = self.right_width
        print("left_road_width", left_road_width)
        print ("right_road_width", right_road_width)
        # lateral offset in left side is positive, in right side is negative
        left_sample_di = list(np.arange(0, self.left_width, 0.4))

        total_sample_di = sorted( left_sample_di)
        # get obstacles
        self.obs = []


        print("veh_cur_x,veh_cur_y,veh_yaw",veh_cur_x,veh_cur_y,veh_yaw)

        for per_msg in data_ob.perceptions:
            x = per_msg.location.x
            y = per_msg.location.y
            if y > left_road_width - 0.5 or y < -right_road_width + 0.3:
                continue
            length = per_msg.length
            width = per_msg.width
            if length >4. or width > 4.0:
                continue
            if length <0.15 and width <0.15:
                continue
            th = per_msg.yaw
            # v = math.sqrt(per_msg.velocity.x**2 + per_msg.velocity.y**2)
            obi = Obstacle(x * math.cos(veh_yaw) - y * math.sin(veh_yaw) + veh_cur_x + 0.5,
                           x * math.sin(veh_yaw) + y * math.cos(veh_yaw) + veh_cur_y,
                           width, length, veh_yaw + th, ref_paths, cur_lane_num)
            self.obs.append(obi)
        print("num of obstacles", len(self.obs))
        obstacles = self.obs
        # obstacles = []

        # params for target stop
        target_point_s = []
        target_point_d = []

        # params for other vehicles
        lane0_veh, lane1_veh = [], []
        other_vehicles = []
        # global  old_path is for reusing of best_path
        global  old_path
        global old_trajectory
        time2 = time.time()
        #print("********************************************************")
        print("cost time before planning", time2-time1)
        #print("********************************************************")
        if not obstacles:
            print("!!!!!no obstacles!!!!")
            print("!!!!!no obstacles!!!!")
            if not old_path:
                old_path=[]
                time4 = time.time()
                action=1
                for speed in sample_speed:
                    for sam_di in total_sample_di:
                        # for sam_di in [0.,  1., 2., 3., 4.,]:
                        sample_di = sam_di
                        target_speed = speed
                        if c_speed < 2.5:
                            c_speed = 2.5
                        best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                        veh_width, veh_len, obstacles, other_vehicles,
                                                                        cur_lane_num, lane_num, action,
                                                                        target_speed, sample_di, old_path,
                                                                        left_road_width, right_road_width)
                        if best_path:break
                    if best_path:break
                start_time = time1
                print("best_path.s",best_path.s)
                rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                    interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                self.trajectory_publisher.publish(trajectory)
                #print(trajectory.points[0:30])
                self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                time5 = time.time()
                #print("********************************************************")
                #print("cost time interplate and pub", time5 - time4)
                #print("********************************************************")
                old_path = best_path
            if old_path:
                if global_path_max_curvature < 0.25:
                    c_speed = TARGET_SPEED
                if global_path_max_curvature >= 0.25:
                    c_speed = 0.8 * TARGET_SPEED
                index = 0
                min_dists = float("Inf")
                print("*****")
                time3 = time.time()
                print("*****")
                for i in np.arange(len(old_path.x)):
                    interpoint_distances = math.sqrt(
                        (self.cur_x - old_path.x[i]) ** 2 + (self.cur_y - old_path.y[i]) ** 2)
                    if interpoint_distances < min_dists:
                        min_dists = interpoint_distances
                        index = i
                index = index - 1



                print("!!!!!")
                print("index", index)
                print("!!!!!")

                segment_points_nums = SEGMENT_TIME / DT
                if index <1:
                    print("index <1")
                    time4 = time.time()
                    path = copy.deepcopy(old_path)
                    s1 = path.s[0]
                    path.s = [si + s0 - s1 for si in path.s]
                    best_path, trajectory, rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th=\
                        imterplating_no_obstacle_pub(path,csp1,c_speed)
                    print("best_path.s", best_path.s)
                    self.trajectory_publisher.publish(trajectory)
                    #print(trajectory.points[0:30])
                    self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                    time5 = time.time()
                    #print("********************************************************")
                    #print("cost time interplate and pub", time5 - time4)
                    #print("********************************************************")
                if 1<=index<segment_points_nums:
                    print("segment_points_nums")
                    time4 = time.time()
                    new_path = copy.deepcopy(old_path)
                    for i in range(index):
                        path_pop_first_point(new_path)
                    path=new_path
                    s1 = path.s[0]

                    path.s = [si + s0 - s1 for si in path.s]
                    best_path, trajectory, rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th=\
                        imterplating_no_obstacle_pub(path,csp1,c_speed)
                    print("best_path.s", best_path.s)
                    self.trajectory_publisher.publish(trajectory)
                    #print(trajectory.points[0:30])
                    self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                    time5 = time.time()
                    #print("********************************************************")
                    #print("cost time interplate and pub", time5 - time4)
                    #print("********************************************************")
                if index > segment_points_nums:
                    print("index > segment_points_nums + 5")
                    time4 = time.time()
                    new_path = copy.deepcopy(old_path)
                    if old_path.s and len(old_path.s) > 30:
                        for i in np.arange(index):
                            path_pop_first_point(new_path)
                    s1 = new_path.s[0]
                    for i in range(len(new_path.s)):
                        new_path.s[i]=new_path.s[i]-s1+s0

                    action, target_speed, sample_di = 1, TARGET_SPEED, 0.
                    implement_time = (TOTAL_TIME / DT - len(new_path.s)) * DT + 0.05
                    best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                    veh_width, veh_len, obstacles, other_vehicles,
                                                                    cur_lane_num, lane_num, action,
                                                                    target_speed, sample_di, new_path,
                                                                    left_road_width, right_road_width,implement_time)

                    if best_path:
                        start_time = time1
                        rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                            interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                        print("best_path.s", best_path.s)
                        self.trajectory_publisher.publish(trajectory)
                        # print(trajectory.points[0:30])
                        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                        time5 = time.time()
                        # print("********************************************************")
                        # print("cost time interplate and pub", time5 - time4)
                        # print("********************************************************")
                        old_path = best_path
        if obstacles:
            print("there is/are obstacle(s)")
            if old_path:
                index = 0
                min_dists = float("Inf")
                print("*****")
                time3 = time.time()
                print("*****")
                for i in np.arange(len(old_path.x)):
                    interpoint_distances = math.sqrt(
                        (self.cur_x - old_path.x[i]) ** 2 + (self.cur_y - old_path.y[i]) ** 2)
                    if interpoint_distances < min_dists:
                        min_dists = interpoint_distances
                        index = i
                # noinspection PyBroadException
                try:
                    dist1 = math.sqrt((old_path.x[index + 1] - old_path.x[index]) ** 2 + (old_path.y[index + 1] - old_path.y[index]) ** 2)
                    dist2 = math.sqrt((self.cur_x - old_path.x[index + 1]) ** 2 + (self.cur_y - old_path.y[index + 1]) ** 2)
                    if dist1 < dist2:
                        index = index - 1
                    if dist1 >= dist2:
                        index = index
                except:
                    index=len(old_path.s)-1


                print("!!!!!")
                print("index", index)
                print("!!!!!")
                segment_points_nums = SEGMENT_TIME / DT
                if index<1:
                    start_time = time1
                    print('index <1 len(new_path.s)',len(old_path.s))
                    print('index <1 len(new_path.d)')
                    new_path = copy.deepcopy(old_path)
                    collision = check_collision(new_path, veh_width, veh_len, obstacles, other_vehicles)
                    if not collision:
                        best_path = old_path
                        s1 = best_path.s[0]
                        compensate_distance = 0.3
                        best_path.s = [best_path_si - s1 + self.s0 - compensate_distance for best_path_si in
                                       best_path.s]
                        rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                            interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                        print("best_path.s", best_path.s)
                        self.trajectory_publisher.publish(trajectory)
                        # print(trajectory.points[0:30])
                        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                    if collision:
                        old_path = []
                        action = 1
                        for speed in sample_speed:
                            for sam_di in total_sample_di:
                                # for sam_di in [0.,  1., 2., 3., 4.,]:
                                sample_di = sam_di
                                target_speed = speed
                                if c_speed < 2.5:
                                    c_speed = 2.5
                                best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d,
                                                                                c_d_dd,
                                                                                veh_width, veh_len,
                                                                                obstacles, other_vehicles,
                                                                                cur_lane_num,
                                                                                lane_num, action,
                                                                                target_speed, sample_di, old_path,
                                                                                left_road_width, right_road_width)
                                if best_path:
                                    break
                            if best_path:
                                break
                        if best_path:
                            start_time = time1
                            rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                                interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                            print("best_path.s", best_path.s)
                            self.trajectory_publisher.publish(trajectory)
                            # print(trajectory.points[0:30])
                            self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                            old_path = best_path
                            print("index <= segment_points_nums len(old_path.s)", len(old_path.s))
                if 1<=index < segment_points_nums:
                    new_path = copy.deepcopy(old_path)
                    for i in np.arange(index):
                        path_pop_first_point(new_path)
                    print('index <= segment_points_nums len(new_path.s)', len(new_path.s))
                    print('index <= segment_points_nums len(new_path.d)', len(new_path.d))
                    collision = check_collision(new_path, veh_width, veh_len, obstacles, other_vehicles)
                    if not collision:
                        print ("old_path no collision")
                        print(new_path.x)
                        print(new_path.y)
                        for obstacle in obstacles:
                            print("obstacle",i)
                            print(obstacle.x,obstacle.y,obstacle.width,obstacle.length)
                        best_path = new_path
                        # TODO  s change
                        best_path.s = [best_path_si - best_path.s[0] + self.s0 for best_path_si in best_path.s]
                        best_path = calc_global_paths_for_best_path(best_path, csp1)
                        start_time = time1
                        s1 = best_path.s[0]
                        compensate_distance = 0.3
                        best_path.s = [best_path_si - s1 + self.s0 - compensate_distance for best_path_si in
                                       best_path.s]
                        rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                            interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                        print("best_path.s", best_path.s)
                        self.trajectory_publisher.publish(trajectory)
                        #print(trajectory.points[0:30])
                        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                    if collision:
                        print ("old_path collision")
                        old_path = []
                        action = 1
                        for speed in sample_speed:
                            for sam_di in total_sample_di:
                                # for sam_di in [0.,  1., 2., 3., 4.,]:
                                sample_di = sam_di
                                target_speed = speed
                                if c_speed < 2.5:
                                    c_speed = 2.5
                                best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d,
                                                                                c_d_dd,
                                                                                veh_width, veh_len,
                                                                                obstacles, other_vehicles,
                                                                                cur_lane_num,
                                                                                lane_num, action,
                                                                                target_speed, sample_di, old_path,
                                                                                left_road_width, right_road_width)
                                if best_path:
                                    break
                            if best_path:
                                break
                        if best_path:
                            start_time = time1
                            rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                                interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                            print("best_path.s", best_path.s)
                            self.trajectory_publisher.publish(trajectory)
                            #print(trajectory.points[0:30])
                            self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                            old_path = best_path
                            print("index <= segment_points_nums len(old_path.s)", len(old_path.s))


                if index>=segment_points_nums:
                    '''print("index > segment_points_nums + 5")

                    old_path=[]
                    action, target_speed, sample_di = 1, TARGET_SPEED, 0.

                    best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                    veh_width, veh_len, obstacles, other_vehicles,
                                                                    cur_lane_num, lane_num, action,
                                                                    target_speed, sample_di, old_path,
                                                                    left_road_width, right_road_width)

                    if best_path:
                        start_time = time1
                        rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                            interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                        print("best_path.s", best_path.s)
                        self.trajectory_publisher.publish(trajectory)
                        # print(trajectory.points[0:30])
                        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                        time5 = time.time()
                        # print("********************************************************")
                        # print("cost time interplate and pub", time5 - time4)
                        # print("********************************************************")
                        old_path = best_path'''
                    print('index <= segment_points_nums ')
                    print('index <= segment_points_nums ')

                    new_path = copy.deepcopy(old_path)
                    if old_path.s and len(new_path.s) > index+70:
                        for i in np.arange(index):
                            path_pop_first_point(new_path)

                    collision = check_collision(new_path,veh_width,veh_len,obstacles,other_vehicles)
                    if len(old_path.s) <= index + 70:
                        collision=True
                    if collision:
                        action = 1
                        old_path=[]
                        for speed in sample_speed:
                            for sam_di in total_sample_di:
                                # for sam_di in [0.,  1., 2., 3., 4.,]:
                                sample_di = sam_di
                                target_speed = speed
                                if c_speed < 2.5:
                                    c_speed = 2.5
                                best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                                veh_width, veh_len,
                                                                                obstacles, other_vehicles, cur_lane_num,
                                                                                lane_num, action,
                                                                                target_speed, sample_di, old_path,
                                                                                left_road_width, right_road_width)
                                if best_path:
                                    break
                            if best_path:
                                start_time = time1
                                rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                                    interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                                print("best_path.s", best_path.s)
                                self.trajectory_publisher.publish(trajectory)
                                # print(trajectory.points[0:30])
                                self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t,
                                                            rviz_pub_th)
                                old_path = best_path
                                break
                    if not collision:
                        s1 = new_path.s[0]
                        for i in range(len(new_path.s)):
                            new_path.s[i] = new_path.s[i] - s1 + s0
                        action, target_speed, sample_di = 1, TARGET_SPEED, 0.
                        implement_time = (TOTAL_TIME / DT - len(new_path.s)) * DT + 0.05
                        best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                        veh_width, veh_len, obstacles, other_vehicles,
                                                                        cur_lane_num, lane_num, action,
                                                                        target_speed, sample_di, new_path,
                                                                        left_road_width, right_road_width,
                                                                        implement_time)

                        if best_path:
                            start_time = time1
                            rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                                interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                            print("best_path.s", best_path.s)
                            self.trajectory_publisher.publish(trajectory)
                            # print(trajectory.points[0:30])
                            self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                            time5 = time.time()
                            # print("********************************************************")
                            # print("cost time interplate and pub", time5 - time4)
                            # print("********************************************************")
                            old_path = best_path
                time4 = time.time()
                #print("********************************************************")
                #print("cost time planning obstacles and old path", time4 - time3)
                #print("********************************************************")'''
            if not old_path:
                print("!!!!!!no old path!!!!!!!!!!!!!!!!!!!!!!!")
                print("!!!!!!no old path!!!!!!!!!!!!!!!!!!!!!!!")
                time3 = time.time()
                action = 1
                print("sample_speed",sample_speed)
                print("total_sample_di,total",total_sample_di)
                print(s0, c_speed, c_d, c_d_d, c_d_dd)
                print(veh_cur_x,veh_cur_y)
                print(self.obs[0].x,self.obs[0].y)
                for speed in sample_speed:
                    for sam_di in total_sample_di:
                        # for sam_di in [0.,  1., 2., 3., 4.,]:
                        sample_di = sam_di
                        target_speed = speed
                        if c_speed < 2.5:
                            c_speed = 2.5
                        best_path, _ = frenet_optimal_planning_cruising(csp1, s0, c_speed, c_d, c_d_d, c_d_dd,
                                                                        veh_width, veh_len,
                                                                        obstacles, other_vehicles, cur_lane_num,
                                                                        lane_num, action,
                                                                        target_speed, sample_di, old_path,
                                                                        left_road_width, right_road_width)
                        if best_path:
                            old_path = best_path
                            break
                    if best_path:
                        start_time = time1
                        rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                            interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, 4, s0)
                        print("best_path.s", best_path.s)
                        self.trajectory_publisher.publish(trajectory)
                        # print(trajectory.points[0:30])
                        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                        old_path = best_path
                        break
                time4 = time.time()
                #print("********************************************************")
                #print("cost time planning obstacles and not old path", time4 - time3)
                #print("********************************************************")
            if not best_path:
                # TODO stopping trajectory planning
                print('!!1 no best path!!')
                print('!!2 no best path!!')
                print('!!3 no best path!!')
                start_time = time1
                stopping_path = frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, cur_lane_num, lane_num)
                print(stopping_path.s)
                rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory = \
                    interplating_and_transforming_curnormal(stopping_path, veh_ve_vx, start_time, 4,s0)
                self.trajectory_publisher.publish(trajectory)
                #print(trajectory.points[0:30])
                self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
                print("*********************************")
                print('!!1 no best path but got stopping path!!')
                print('!!2 no best path but got stopping path!!')
                print("*********************************")
                '''header = Header(0, rospy.Time.now(), 'world')
                trajectory_points = []
                trajectory = Trajectory(header, trajectory_points)

                self.trajectory_publisher.publish(trajectory)
                self.traj_publisher.publish([], [], [], [], [])'''





        ######################
        # rate = rospy.Rate(1)
        # print("pos_pub",pub_x[0],pub_y[0])
        ######## traj_publisher 发布轨迹点 traj2 发布多项式拟合的参考路径点

        # self.traj_publisher.publish(self.pub_x, self.pub_y, self.pub_th)
        # self.traj2_publisher.publish(pub_xx, pub_yy, pub_tt, pub_thh)
        # print('published!')
        self.tt0 = self.tt0 + 0.2
        # rate.sleep()


        '''plt.clf()
        vehicle = collision_checker_obb.Rectangle(veh_cur_x, veh_cur_y, veh_width, veh_len, veh_yaw)
        vehicle.show_rectangle()
        show_obstacles(obstacles)
        plt.plot(global_x, global_y)
        if self.pub_x:
            plt.plot(self.pub_x, self.pub_y)
        plt.axis('equal')

        plt.pause(0.001)
        rate = rospy.Rate(50)
        rate.sleep()'''
        time9 = time.time()
        print('cost time:', time9 - time1)


def main():
    rospy.init_node('lattice_node_mbs', anonymous=True)
    sub_and_pub()
    rospy.spin()


if __name__ == '__main__':
    main()

