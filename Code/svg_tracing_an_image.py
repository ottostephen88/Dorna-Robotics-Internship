from __future__ import print_function
from sympy import symbols, Eq, solve
from dorna2 import dorna
from random import random
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import json
import sys
import re
import math

sys.path.append('..')


class svg(object):
    """svg for ClassName"""
    def __init__(self, t=10):
        super(svg, self).__init__()
        self.t = t

    def svg_to_robotpath(self, paths, T, B):
        """this function is to transform the xy coordinates of the
        svg path to the 3D coordinates of the robot path"""
        robot_path = []
        for path_pair in paths:
            x_path = []
            y_path = []
            z_path = []
            for i in range(len(path_pair[0])):
                old_path = np.array([[path_pair[0][i]], [path_pair[1][i]]], dtype='f')
                new_path = np.add(np.dot(T, old_path), B)
                x_path.append(new_path[0][0])
                y_path.append(new_path[1][0])
                z_path.append(new_path[2][0])
            robot_path.append([x_path, y_path, z_path])
        return robot_path

    def parse_text(self, s):
        res = re.split('([-+]?\d+\.\d+)|([-+]?\d+)', s.strip())
        res_f = [r.strip() for r in res if r is not None and r.strip() != '']
        rtn = []
        for s in res_f:
            if s != ",":
                try:
                    rtn.append(float(s))
                except:
                    rtn += [x for x in s]
        return rtn

    # this function generates the paths for a filename and plots the results
    def gen(self, file_name, x_max, y_max, x_min, y_min, a, b, cp, scale, velocity, acceleration, jerk, corner):
        paths = []
        path_x = []
        path_y = []

        # first we read the svg file
        tree = ET.parse(file_name)
        root = tree.getroot()
        for path in root.iter('{http://www.w3.org/2000/svg}path'):

            d = self.parse_text(path.attrib['d'])

            cur_x = 0
            cur_y = 0

            start_x = 0
            start_y = 0

            current_cmd = None
            last_cmd = None

            i = 0
            while i < len(d):
                element = d[i]
                if type(element) == str:
                    current_cmd = element
                    i += 1
                """when there is an m, a new path has started"""
                if current_cmd == 'm':
                    """start of new path command""""
                    if path_x:
                        paths.append([path_x, path_y])
                        path_x = []
                        path_y = []
                        """we should start a new path here
                        we should read x and y as the first points"""
                    cur_x += d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y += d[i]
                    path_y.append(cur_y)
                    i += 1

                    start_x = cur_x
                    start_y = cur_y

                elif current_cmd == 'M':
                    # start of new path command
                    if path_x:
                        paths.append([path_x, path_y])
                        path_x = []
                        path_y = []
                        # we should start a new path here
                        # we should read x and y as the first points
                    cur_x = d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y = d[i]
                    path_y.append(cur_y)
                    i += 1

                    start_x = cur_x
                    start_y = cur_y

                elif current_cmd == 'v':
                    # vertical line command
                    cur_y += d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'V':
                    # vertical line command
                    cur_y = d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'h':
                    # horizontal line command

                    cur_x += d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'H':
                    # horizontal line command
                    cur_x = d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'l':
                    # line command
                    cur_x += d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y += d[i]
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'L':
                    # line command
                    cur_x = d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y = d[i]
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'z' or current_cmd == 'Z':
                    # end of path command
                    path_x.append(start_x)
                    path_y.append(start_y)
                    cur_x = start_x
                    cur_y = start_y

                elif current_cmd == 'q':
                    # quadratic curve command
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 2 * (p0_x + cur_x) + 2 * t * (1 - t) * (p1_x + cur_x) + t ** 2 * (p2_x + cur_x)
                        p_y = (1 - t) ** 2 * (p0_y+cur_y) + 2 * t * (1 - t) * (p1_y+cur_y) + t ** 2 * (p2_y+cur_y)
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x += p2_x
                    cur_y += p2_y

                elif current_cmd == 'Q':
                    # quadratic curve command
                    p0_x = cur_x
                    p0_y = cur_y

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 2 * p0_x + 2 * t * (1 - t) * p1_x + t ** 2 * p2_x
                        p_y = (1 - t) ** 2 * p0_y + 2 * t * (1 - t) * p1_y + t ** 2 * p2_y
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x = p2_x
                    cur_y = p2_y

                elif current_cmd == 's':
                    # connecting several cubic bezier curve commands
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    p2_x = p1_x
                    i += 1

                    p1_y = d[i]
                    p2_y = p1_y
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t

                        p_x = (1 - t) ** 3 * p0_x + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * p0_y + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(cur_x + p_x)
                        path_y.append(cur_y + p_y)

                    cur_x += p3_x
                    cur_y += p3_y

                elif current_cmd == 'S':
                    # connecting several cubic bezier curve commands
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    p2_x = p1_x
                    i += 1

                    p1_y = d[i]
                    p2_y = p1_y
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t

                        p_x = (1 - t) ** 3 * p0_x + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * p0_y + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(cur_x + p_x)
                        path_y.append(cur_y + p_y)

                    cur_x = p3_x
                    cur_y = p3_y

                elif current_cmd == 'c':
                    # Bezier Curve Command
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 3 * (p0_x + cur_x) + 3 * t * (1 - t) ** 2 * (p1_x + cur_x) + 3 * (t**2) * (1 - t) * (p2_x + cur_x) + t ** 3 * (p3_x + cur_x)
                        p_y = (1 - t) ** 3 * (p0_y + cur_y) + 3 * t * (1 - t) ** 2 * (p1_y + cur_y) + 3 * (t**2) * (1 - t) * (p2_y + cur_y) + t ** 3 * (p3_y + cur_y)

                        path_x.append(p_x)
                        path_y.append(p_y)

                        # path_x.append(cur_x + p_x)
                        # path_y.append(cur_y + p_y)

                    cur_x += p3_x
                    cur_y += p3_y

                elif current_cmd == 'C':
                    # Bezier Curve Command
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 3 * (p0_x + cur_x) + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * (p0_y + cur_y) + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x = p3_x
                    cur_y = p3_y
                elif current_cmd == 't' and (last_cmd == 'Q' or last_cmd == 'T' or last_cmd == 'q' or last_cmd == 't'):
                    # connecting quadratic bezier curve commands
                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 2 * (p0_x + cur_x) + 2 * t * (1 - t) * (p1_x + cur_x) + t ** 2 * (p2_x + cur_x)
                        p_y = (1 - t) ** 2 * (p0_y + cur_y) + 2 * t * (1 - t) * (p1_y + cur_y) + t ** 2 * (p2_y + cur_y)
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x += p2_x
                    cur_y += p2_y
                elif current_cmd == 'T' and (last_cmd == 'Q' or last_cmd == 'T' or last_cmd == 'q' or last_cmd == 't'):
                    # connecting quadratic bezier curve commands
                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t
                        p_x = (1 - t) ** 2 * p0_x + 2 * t * (1 - t) * p1_x + t ** 2 * p2_x
                        p_y = (1 - t) ** 2 * p0_y + 2 * t * (1 - t) * p1_y + t ** 2 * p2_y
                        path_x.append(p_x)
                        path_y.append(p_y)
                    cur_x += p2_x
                    cur_y += p2_y
                # incomplete
                elif current_cmd == 'a':
                    # arc command
                    rx = d[i]
                    i += 1

                    ry = d[i]
                    i += 1

                    x_theta = d[i]
                    i += 1

                    l_arc = d[i]
                    i += 1

                    s_arc = d[i]
                    i += 1

                    p3_x = d[i]
                    p3_x = p3_x+cur_x
                    i += 1

                    p3_y = d[i]
                    p3_y = p3_y+cur_y
                    i += 1

                    x_theta = x_theta * math.pi / 180
                    angle1 = 0

                    # ellipse produces 4 equations and 2 coordinates,this solves them
                    x, y = symbols('x y')
                    eqn1 = x - p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y) ** 2)) / (ry ** 2))) ** .5
                    eqn2 = x - cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y) ** 2)) / (ry ** 2))) ** .5
                    sol = solve((eqn1, eqn2), (x, y))

                    x2, y2 = symbols('x2 y2')
                    eqn3 = x2 - p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y2) ** 2)) / (ry ** 2))) ** .5
                    eqn4 = -x2 + cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y2) ** 2)) / (ry ** 2))) ** .5
                    sol2 = solve((eqn3, eqn4), (x2, y2))

                    x3, y3 = symbols('x3 y3')
                    eqn5 = -x3 + p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y3) ** 2)) / (ry ** 2))) ** .5
                    eqn6 = x3 - cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y3) ** 2)) / (ry ** 2))) ** .5
                    sol3 = solve((eqn5, eqn6), (x3, y3))

                    x4, y4 = symbols('x4 y4')
                    eqn7 = -x4 + p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y4) ** 2)) / (ry ** 2))) ** .5
                    eqn8 = -x4 + cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y4) ** 2)) / (ry ** 2))) ** .5
                    sol4 = solve((eqn7, eqn8), (x4, y4))

                    # chooses solution based on order, needs to be changed
                    if not sol:
                        if not sol2:
                            if not sol3:
                                (sol_x, sol_y) = sol4[0]
                            else:
                                (sol_x, sol_y) = sol3[0]
                        else:
                            (sol_x, sol_y) = sol2[0]
                    else:
                        (sol_x, sol_y) = sol[0]

                    # this is for all the special conditions in arc tan, when top is 0 or bottom is 0
                    # everything is respect to +x axis
                    if (p3_y-sol_y) == 0:
                        if(p3_x < sol_x):
                            angle1 = 180
                        else:
                            angle1 = 0

                    elif (p3_x - sol_x) == 0:
                        if (p3_y < sol_y):
                            angle1 = 270
                        else:
                            angle1 = 90
                    else:
                        angle = math.atan((cur_y - sol_y)/(cur_x - sol_x))
                    if (cur_y - sol_y) == 0:
                        if(cur_x < sol_x):
                            angle2 = 180
                        else:
                            angle2 = 0

                    elif(cur_x - sol_x) == 0:
                        if (cur_y < sol_y):
                            angle2 = 90
                        else:
                            angle2 = 270

                    else:
                        angle2 = math.atan((cur_y - sol_y)/(cur_x - sol_x))
                    angle_diff = abs(angle2 - angle1)

                    # chooses small or large angle,also startingangle,and direction
                    if l_arc == 1:
                        if angle_diff <= 180:
                            angle_diff = 360 - angle_diff
                        if angle1 > angle2:
                            direction = -1
                            startangle = angle2
                        else:
                            direction = 1
                            startangle = angle1

                    else:
                        if angle_diff >= 180:
                            angle_diff = 360 - angle_diff
                        if angle1 > angle2:
                            direction = -1
                            startangle = angle2
                            if angle1 == 0:
                                angle1 = 360
                        else:
                            direction = 1
                            startangle = angle1
                    # change in angle
                    delta_angle = abs(angle_diff) / self.t
                    for j in range(self.t + 1):
                        t = (j) / self.t

                        # current angle
                        current_angle = math.radians(startangle) + math.radians(j * direction * delta_angle)

                        # main formula for path
                        p_x = sol_x + (rx * math.cos(current_angle) * math.cos(x_theta)) - (rx * math.sin(current_angle) * math.sin(x_theta))
                        p_y = sol_y + (ry * math.cos(current_angle) * math.sin(x_theta)) + (ry * math.sin(current_angle) * math.cos(x_theta))

                        # add to path
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x += p3_x
                    cur_y += p3_y
                # incomplete
                elif current_cmd == 'A':
                    # arc command
                    # radius of x
                    rx = d[i]
                    i += 1
                    # radius of y
                    ry = d[i]
                    i += 1
                    # x rotation
                    x_theta = d[i]
                    i += 1
                    # large arc
                    l_arc = d[i]
                    i += 1
                    # sweep arc
                    s_arc = d[i]
                    i += 1
                    # final x
                    p3_x = d[i]
                    i += 1
                    # final y
                    p3_y = d[i]
                    i += 1

                    x_theta = math.radians(x_theta)
                    angle1 = 0

                    # ellipse produces 4 equations and 2 coordinates,this solves them
                    x, y = symbols('x y')
                    eqn1 = x - p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y) ** 2)) / (ry ** 2))) ** .5
                    eqn2 = x - cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y) ** 2)) / (ry ** 2))) ** .5
                    sol = solve((eqn1, eqn2), (x, y))

                    x2, y2 = symbols('x2 y2')
                    eqn3 = x2 - p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y2) ** 2)) / (ry ** 2))) ** .5
                    eqn4 = -x2 + cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y2) ** 2)) / (ry ** 2))) ** .5
                    sol2 = solve((eqn3, eqn4), (x2, y2))

                    x3, y3 = symbols('x3 y3')
                    eqn5 = -x3 + p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y3) ** 2)) / (ry ** 2))) ** .5
                    eqn6 = x3 - cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y3) ** 2)) / (ry ** 2))) ** .5
                    sol3 = solve((eqn5, eqn6), (x3, y3))

                    x4, y4 = symbols('x4 y4')
                    eqn7 = -x4 + p3_x + (((rx ** 2) - ((rx ** 2) * ((p3_y - y4) ** 2)) / (ry ** 2))) ** .5
                    eqn8 = -x4 + cur_x + (((rx ** 2) - ((rx ** 2) * ((cur_y - y4) ** 2)) / (ry ** 2))) ** .5
                    sol4 = solve((eqn7, eqn8), (x4, y4))

                    # currently just goes in order and picks first solution
                    if not sol:
                        if not sol2:
                            if not sol3:
                                (sol_x, sol_y) = sol4[0]
                            else:
                                (sol_x, sol_y) = sol3[0]
                        else:
                            (sol_x, sol_y) = sol2[0]
                    else:
                        (sol_x, sol_y) = sol[0]

                    # this is for all the special conditions in arc tan, when top is 0 or bottom is 0
                    # everything is respect to +x axis
                    if (p3_y - sol_y) == 0:
                        if(p3_x < sol_x):
                            angle1 = 180
                        else:
                            angle1 = 0

                    elif (p3_x - sol_x) == 0:
                        if (p3_y < sol_y):
                            angle1 = 270
                        else:
                            angle1 = 90
                    else:
                        angle = math.atan((cur_y - sol_y) / (cur_x - sol_x))
                    if (cur_y - sol_y) == 0:
                        if(cur_x < sol_x):
                            angle2 = 180
                        else:
                            angle2 = 0

                    elif(cur_x - sol_x) == 0:
                        if (cur_y < sol_y):
                            angle2 = 90
                        else:
                            angle2 = 270

                    else:
                        angle2 = math.atan((cur_y - sol_y) / (cur_x - sol_x))

                    angle_diff = abs(angle2 - angle1)

                    # chooses small or large angle,also startingangle,and direction
                    if l_arc == 1:
                        if angle_diff <= 180:
                            angle_diff = 360 - angle_diff
                        if angle1 > angle2:
                            direction = -1
                            startangle = angle2
                        else:
                            direction = 1
                            startangle = angle1

                    else:
                        if angle_diff >= 180:
                            angle_diff = 360 - angle_diff
                        if angle1 > angle2:
                            direction = -1
                            startangle = angle2
                            if angle1 == 0:
                                angle1 = 360
                        else:
                            direction = 1
                            startangle = angle1

                    # change in angle
                    delta_angle = abs(angle_diff) / self.t
                    for j in range(self.t + 1):
                        t = (j)/self.t

                        # current angle
                        current_angle = math.radians(startangle) + math.radians(j * direction * delta_angle)

                        # main formula for path
                        p_x = sol_x + (rx * math.cos(current_angle) * math.cos(x_theta)) - (rx * math.sin(current_angle) * math.sin(x_theta))
                        p_y = sol_y + (ry * math.cos(current_angle) * math.sin(x_theta)) + (ry * math.sin(current_angle) * math.cos(x_theta))
                        print(current_angle*180/math.pi)
                        # add to path
                        path_x.append(p_x)
                        path_y.append(p_y)
                    cur_x = p3_x
                    cur_y = p3_y

                last_cmd = current_cmd

        paths.append([path_x, path_y])

        # now we should scale the paths
        # we first find the min and max of the paths
        x_path_min = min([min(pairs[0]) for pairs in paths])
        x_path_max = max([max(pairs[0]) for pairs in paths])

        y_path_min = min([min(pairs[1]) for pairs in paths])
        y_path_max = max([max(pairs[1]) for pairs in paths])
        """ for fitting the image
        width_paper = x_max
        width_path = x_path_max - x_path_min
        length_paper = y_max
        length_path = y_path_max - y_path_min
        ratio_paper = width_paper / length_paper
        ratio_path = width_path / length_path
        """
        if scale == 1:
            # next we scale the pairs
            paths = [[[(x_max - x_min) * (x - x_path_min) / (x_path_max - x_path_min) + x_min for x in pairs[0]], [(y_max - y_min) * (y - y_path_min) / (y_path_max - y_path_min) + y_min for y in pairs[1]]] for pairs in paths]
        else:
            # next we fit
            # incomplete
            pass

        for path_pair in paths:
            plt.plot(path_pair[0], path_pair[1])
        # uncomment below if you want to see image drawn in svg format
        # plt.show()

        robot_path = self.svg_to_robotpath(paths, T, B)
        # final_path = [[[x for x in pairs[0]], [y for y in pairs[1]],[z for z in pairs[2]]] for pairs in paths]
        # print("path: ", final_path)
        return self.gen_cmd(robot_path, cp, velocity, acceleration, jerk, corner)

    def gen_cmd(self, robot_path, cp, velocity, acceleration, jerk, corner):
        cmds = []
        cmds_length = []
        j = 100
        # go 6mm orthogonal to plane
        cmds.append([{"cmd": "lmove", "rel": 1, "x": 3 * cp[0], "y": 3 * cp[1], "z": 3 * cp[2], "a": 0, "b": 0, "vel": velocity, "accel": acceleration, "jerk": jerk, "id": j}])
        j += 1
        cmds_length.append(len(cmds))
        for path_pairs in robot_path:
            cmd = []

            # go 6 mm from starting point
            last_point = list([path_pairs[0][0], path_pairs[1][0], path_pairs[2][0]])
            cmd.append({"cmd": "lmove", "rel": 0, "x": path_pairs[0][0] + 6 * cp[0], "y": path_pairs[1][0] + 6 * cp[1], "z": path_pairs[2][0] + 6 * cp[2], "a": 0, "b": 0, "id": j})
            j += 1
            # go 6mm closer to paper
            cmd.append({"cmd": "lmove", "rel": 1, "x": -6*cp[0], "y": -6*cp[1], "z": -6*cp[2], "a": 0, "b": 0, "id": j})
            j += 1
            # go to each point in the path
            for i in range(len(path_pairs[0])):
                if (path_pairs[0][i] - last_point[0])**2 + (path_pairs[1][i] - last_point[1])**2 + path_pairs[2][i] - last_point[2] > 0.01:
                    cmd.append({"cmd": "lmove", "rel": 0, "x": path_pairs[0][i], "y": path_pairs[1][i], "z": path_pairs[2][i], "a": 0, "b": 0, "cont": 1, "corner": corner, "id": j})
                    j += 1
                    last_point = [path_pairs[0][i], path_pairs[1][i], path_pairs[2][i]]
            last_cmd = cmd.pop()

            # last point should not be continous because it will miss last point because the pen goes up
            if "cont" in last_cmd:
                last_cmd["cont"] = 0
            cmd.append(last_cmd)

            # go 6mm away from paper
            cmd.append({"cmd": "lmove", "rel": 1, "x": 6 * cp[0], "y": 6 * cp[1], "z": 6 * cp[2], "a": 0, "b": 0, "id": j})
            j += 1
            cmds.append(cmd)
            cmds_length.append(len(cmd))

        return cmds, cmds_length


def transformation(TL, TR, BR):
    # finds Transformation matrix and b
    b = TL
    x_max = ((TL[0] - TR[0]) ** 2 + (TL[1]-TR[1]) ** 2 + (TL[2]-TR[2]) ** 2) ** .5
    y_max = ((BR[0] - TR[0]) ** 2 + (BR[1]-TR[1]) ** 2 + (BR[2]-TR[2]) ** 2) ** .5
    t00 = (TR[0] - b[0]) / x_max
    t10 = (TR[1] - b[1]) / x_max
    t20 = (TR[2] - b[2]) / x_max
    t01 = (BR[0] - t00 * x_max - b[0]) / y_max
    t11 = (BR[1] - t10 * x_max - b[1]) / y_max
    t21 = (BR[2] - t20 * x_max - b[2]) / y_max

    # converts to correct matrix dimensions
    b = np.array([[TL[0]], [TL[1]], [TL[2]]])
    a = np.array([[t00, t01], [t10, t11], [t20, t21]])
    return a, b, x_max, y_max


def findcorner(TL, M, BR):
    # each vector from the 3 points
    vector1 = BR-TL
    vector2 = M-TL

    # finds magnitude of TL with dot product
    TR = np.dot(vector1, vector2, out=None) / np.linalg.norm(vector2)

    # magnitude times the unit vector added to the Left Bottom
    TR = (TR * vector2 / np.linalg.norm(vector2)) + TL
    return TR


def perpendicularvector(TL, M, BR):
    # each vector from the 3 points
    vector1 = BR-TL
    vector2 = M-TL
    # cross product
    cp = np.cross(vector1, vector2)
    mag_cp = np.linalg.norm(cp)
    cp = cp / mag_cp
    return cp


def startingpoints(robot):
    # disable the motors
    key = input("Press P to disable the motors")
    error = 0
    if key != "P":
        error = 1
    arg = {"cmd": "motor", "id": 1200, "motor": 0}
    robot.play(**arg)

    # setting Bottom Left coordinate
    key = input("Press P to set Top Left Coordinate")
    if key != "P":
        error = 1
        TL = []
        M = []
        BR = []
        return error, TL, M, BR

    lastmessage = dict(robot.sys)

    if 'x' in lastmessage:
        TL = np.array([lastmessage["x"], lastmessage["y"], lastmessage["z"]])

    # setting Top Right coordinate
    key = input("Press P to set Bottom Right Coordinate")
    if key != "P":
        error = 1
        TL = []
        M = []
        BR = []
        return error, TL, M, BR

    lastmessage = robot.sys
    if 'x' in lastmessage:
        BR = np.array([lastmessage["x"], lastmessage["y"], lastmessage["z"]])

    # setting coordinate along clockwise line perpendicular to Top Right
    key = input("Press P set Coordinate along the line clockwise of Left Bottom")
    if key != "P":
        error = 1
        TL = []
        M = []
        BR = []
        return error, TL, M, BR

    lastmessage = robot.sys
    if 'x' in lastmessage:
        M = np.array([lastmessage["x"], lastmessage["y"], lastmessage["z"]])

    # check if two coordinates are the same
    if TL[0] == M[0] and TL[1] == M[1] and TL[2] == M[2]:
        print("error-first and last point are the same")
        error = 1
    if TL[0] == BR[0] and TL[1] == BR[1] and TL[2] == BR[2]:
        print("error-first and second point are the same")
        error = 1
    if M[0] == BR[0] and M[1] == BR[1] and M[2] == BR[2]:
        print("error-second and last point are the same")
        error = 1

    # check if each array is filled
    if TL.size == 0 or M.size == 0 or BR.size == 0:
        print("error- not all coordinates set")
        error = 1

    # turn motors on
    key = input("Press P to Turn Motors On")
    if key != "P":
        error = 1
    arg = {"cmd": "motor", "id": 1201, "motor": 1}
    robot.play(**arg)
    return error, TL, M, BR

if __name__ == '__main__':
    # connects to robot
    # SET RIGHT IP ADDRESS
    robot = dorna()
    print("connecting")
    robot.connect("ip_address", 443)
    wait_id = 100
    print("done connecting")

    # if scale to window, scale =1
    # if fit to window, scale =0
    # currently does not work; keep at 1
    scale = 1

    # the drawing will become more precise by decreasing the numbers below
    # the drawing will be drawn faster by increasing the numbers below
    # recommended velocity = 50
    velocity = 50
    # recommended acceleration = 300
    acceleration = 300
    # recommended jerk = 3000
    jerk = 3000
    # recommended corner = 5
    corner = 5

    # intialize the 3 points on the plane,Left Bottom-Point along line-Top Right
    error, TL, M, BR = startingpoints(robot)

    # check if there is an error from last function called
    if(error == 1):
        print("need to restart")
        robot.close()

    # calls function to find 3rd corner for new plane
    TR = findcorner(TL, M, BR)

    # finds the a and b for linear translation of rotated paper
    T, B, width, length = transformation(TL, TR, BR)

    # perpendicular vector in unit vector form
    cp = perpendicularvector(TL, M, BR)

    # calls main function to create path
    cmds, cmds_length = svg(10).gen('cat.svg', width, length, 0, 0, T, B, cp, scale, velocity, acceleration, jerk, corner)
    command_list = []
    print("how many continous lines=", len(cmds_length))
    i = 0
    stop = True
    print("length of cmd", len(cmds))
    while stop:
        for cmd in cmds:
            for c in cmd:
                command_list.append(robot.play(True, **c))
                if len(command_list) == cmds_length[i]:

                    last_continous_point = command_list.pop()
                    status = last_continous_point.complete()
                    command_list = []
                    if(status == 2):
                        pass
                    else:
                        arg = {"cmd": "halt", "id": 1000}
                        robot.play(True, **arg)
                        break
            i += 1
            if len(cmds_length) == i:
                stop = False
    print("done")
    robot.close()
