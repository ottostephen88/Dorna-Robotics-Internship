import logging
from threading import Thread 
import time
import math
import serial
import heapq
import numpy as np
"""
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.wait_for_edge(23, GPIO.FALLING)
GPIO.wait_for_edge(24, GPIO.FALLING)
"""

class CalculationAStarAlgorithm():
    def __init__(self, arena, start_charactor, goal_charactor):
        self.arena = arena
        self.start_charactor_position = self.getCharacotorCoordinates(start_charactor)
        self.goal_charactor_position = self.getCharacotorCoordinates(goal_charactor)

    def getCharacotorCoordinates(self, search_criteria_charactor):
        for index_height, line in enumerate(self.arena):
            for index_wedth, charactor in enumerate(line):
                if charactor == search_criteria_charactor:
                    return (index_height, index_wedth)

    def heuristic(self, position):
        return ((position[0] - self.goal_charactor_position[0]) ** 2 + (position[1] - self.goal_charactor_position[1]) ** 2) ** 0.5

    def distance(self, path):
        return len(path)

    def nextCandidatePosition(self, last_passed_position):
        wall = "1"
        vertical_axis, horizontal_axis = last_passed_position
        for next_vertical, next_horizontal in zip((vertical_axis + 1, vertical_axis - 1, vertical_axis, vertical_axis, vertical_axis + 1, vertical_axis + 1, vertical_axis - 1, vertical_axis - 1), (horizontal_axis, horizontal_axis, horizontal_axis + 1, horizontal_axis - 1, horizontal_axis + 1, horizontal_axis - 1, horizontal_axis - 1, horizontal_axis + 1)):
            if self.arena[next_vertical][next_horizontal] != wall:
                yield (next_vertical, next_horizontal)

    def aStarAlgorithm(self):
        passed_list = [self.start_charactor_position]
        init_score = self.distance(passed_list) + self.heuristic(self.start_charactor_position)
        checked = {self.start_charactor_position: init_score}
        searching_heap = []
        heapq.heappush(searching_heap, (init_score, passed_list))

        while len(searching_heap) > 0:
            score, passed_list = heapq.heappop(searching_heap)
            last_passed_position = passed_list[-1]
            
            if last_passed_position == self.goal_charactor_position:
                return passed_list
            for position in self.nextCandidatePosition(last_passed_position):
                new_passed_list = passed_list + [position]
                position_score = self.distance(new_passed_list) + self.heuristic(position)
                if position in checked and checked[position] <= position_score:
                    continue
                checked[position] = position_score
                heapq.heappush(searching_heap, (position_score, new_passed_list))
        return []

    def renderPath(self, path):
        structure = [[arena_dot for arena_dot in element] for element in self.arena]
        for dot in path[1:-1]:
            structure[dot[0]][dot[1]] = "$"
        structure[path[0][0]][path[0][1]] = "S"
        structure[path[-1][0]][path[-1][1]] = "G"
        return ["".join(l) for l in structure]
    
def state_space():
    state = ['111111111111111111111111111111111111111111111111']
    #print(len(state[0]))
    # establish state array
    for i in range(70):
        state.append('1                                              1')
    state.append('111111111111111111111111111111111111111111111111')  
    return state

def change_state(state, cart1_coords, cart2_coords, ball_coords, goal_coords):
    #put goal position
    sample_str = state[ball_coords[1]]
    n = ball_coords[0]
    replacement = 'G'
    sample_str = sample_str[0:n] + replacement + sample_str[n+1: ]
    state[ball_coords[1]] = sample_str
    
    #put starting position
    sample_str = state[cart1_coords[1]]
    n = cart1_coords[0]
    replacement = 'S'
    sample_str = sample_str[0:n] + replacement + sample_str[n+1: ]
    state[cart1_coords[1]] = sample_str

    #cart 2 is an obstacle
    replacement = '1'
    for i in range(7):
        for j in range(7):
            sample_str = state[cart2_coords[1] - 4 + i]
            n = cart2_coords[0] - 4 + j
            sample_str = sample_str[0:n] + replacement + sample_str[n+1:]
            state[cart2_coords[1] - 4 + i] = sample_str
    #ball wall
    x_theta = ball_coords[0] - goal_coords[0]
    y_theta = ball_coords[1] - goal_coords[1]
    if x_theta == 0:
        theta = 90
    else:
        theta = math.atan(y_theta/x_theta)
    theta = math.degrees(theta)
    #print(theta)
    x_l, y_l = plot_circle(ball_coords[0], ball_coords[1], 4.5, theta)

    x_l = [round(num) for num in x_l]
    y_l = [round(num) for num in y_l]
    
    for i in range(len(x_l)):
        sample_str = state[y_l[i]]
        n = x_l[i]
        replacement = '1'
        sample_str = sample_str[0:n] + replacement + sample_str[n+1: ]
        state[y_l[i]] = sample_str
    
    sample_str = state[goal_coords[1]-1]
    n = goal_coords[0]
    replacement = 'Q'
    sample_str = sample_str[0:n] + replacement + sample_str[n+1: ]
    state[goal_coords[1]-1] = sample_str
    return state

def discretization(point):
    #boundaries of the arena
    # 48 and 36 because we dont want the robot to hit the wall
    #
    max_y = 515 - 48
    min_y = 25 + 48
    max_x = 370 -36
    min_x = 10 + 36
    if point[0] > max_x:
        point[0] = 370
    elif point[0] < min_x:
        point[0] = 10
    if point[1] > max_y:
        point[1] = 515
    elif point[1] < min_y:
        point[1] = 25
    #these are to transform the coordinates into the state space we have
    a1 = 72/245
    a2 = 4/15
    b1 = -360/49
    b2 = -8/3
    current_state = [int(round(point[0] * a2 + b2)),int(round(point[1] * a1 + b1))]
    #current_state = [int(round(point[0]/2)),int(round(point[1]/2))]
    return current_state

def plot_circle(x, y, radius, theta, color="-k"):
    start_degree = theta + 40
    end_degree = theta + 320
    deg = np.linspace(start_degree,end_degree,70)

    xl = [x + radius * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + radius * math.sin(np.deg2rad(d)) for d in deg]
    
    return xl, yl

def make_path(path):
    action_sequence = []
    for i in range(len(path)-1):
        y = path[i+1][0] - path[i][0]
        x = path[i][1] - path[i+1][1]
        action_sequence.append([x,y])
    return path, action_sequence

def action_sequence_condense(action_sequence):
    pop_list = []
    for i in range(len(action_sequence) - 1):
        unit_vector = []
        magnitude = (action_sequence[i + 1][0] ** 2 + action_sequence[i + 1][1] ** 2) ** .5
        unit_vector = [round(action_sequence[i + 1][0]/magnitude,2), round(action_sequence[i + 1][1]/magnitude,2)]

        magnitude2 = (action_sequence[i][0] ** 2 + action_sequence[i][1] ** 2) ** .5
        unit_vector2 = [round(action_sequence[i][0]/magnitude2,2), round(action_sequence[i][1]/magnitude2,2)]
        # if they are going in the same direction combine them
        if unit_vector == unit_vector2:
            action_sequence[i + 1][0] = action_sequence[i][0] + action_sequence[i + 1][0]
            action_sequence[i + 1][1] = action_sequence[i][1] + action_sequence[i + 1][1]
            pop_list.append(i)
    pop_list.reverse()
    #pop all duplicates
    for i in pop_list:
        action_sequence.pop(i)
    return action_sequence

def Three_scenarios(ball_coords, cart1_coords, cart2_coords, color):
    if color == "blue":
        goal1 = [24,72]
    else:
        goal1 = [24,0]
        
    """
    cart2_distance_goal = [cart2_coords[0] - goal1[0], cart2_coords[1] - goal1[1]]
    cart1_distance_goal = [cart1_coords[0] - goal1[0], cart1_coords[1] - goal1[1]]
    ball_distance_goal = [ball_coords[0] - goal1[0], ball_coords[1] - goal1[1]]
    theta_car1 = 1/math.tan(cart1_distance_goal[0],cart1_distance_goal)
    theta_car2 = 1/math.tan(cart2_distance_goal[0],cart2_distance_goal)
    theta_ball = 1/math.tan(ball_distance_goal[0],ball_distance_goal)
    """
    #other car is on a break away
    if cart2_distance_goal < cart1_distance_goal:
        pass
    # if other car is closer to ball
    
    #if we are closer to ball
    else:
        ball_coords = ball_coords
    return ball_coords

def Yaw_correction(yaw, color):
    # degrees per second
    angular_speed = 210
    
    if color == "blue":
        theta_wanted = 180
    else:
        theta_wanted = 0
    delta_theta = yaw - theta_wanted
    
    #counter clockwise
    if  delta_theta > 5:
        time = round(angular_speed / delta_theta)
        action = ["ccw", time]
    #clockwise
    elif delta_theta < -5:
        time = round(angular_speed / delta_theta)
        action = ["cw", time]
    #straight
    else:
        action = []
    return action
    
def button_function():
    while True:
        if GPIO.input(24) == True:
            print("hello")
        elif GPIO.input(23) == True:
            print("goodbye")
    return

def Parse(serial_message):
    serial_message = serial_message.split()
    return serial_message

def all_coords(serial_message):
    blue_coords = [int(serial_message[2]),int(serial_message[1])]
    green_coords = [int(serial_message[5]),int(serial_message[4])]
    ball_coords = [int(serial_message[8]),int(serial_message[7])]
    yaw = int(serial_message[10])
        
    return blue_coords, green_coords, ball_coords, yaw

def action_seq_arduino(action_sequence):
    
    print(action_sequence)
    #inches per second
    velocity = 25
    #degrees per second
    angular_velocity = 210
    #distance vector
    new_action_sequence = []
    
    for action in action_sequence:
        string = ""
        if action[0] == "cw":
            string.append("j ")
            time = ""
        elif action[0] == "cw":
            string.append("j ")
            time = ""
        else:
            distance = (action[0] ** 2 + action[1] ** 2) ** .5
            unit_vector = [round(action[0] / distance, 1), round(action[1] / distance, 1)]
            
            #forward
            if (unit_vector  == [0, 1]):
                string = string + "a "
              
            #forward right
            elif (unit_vector  == [.7, .7]):
                string = string + "b "
                
            #forward left
            elif (unit_vector  == [-.7, .7]):
                string = string + "c "
               
            #left
            elif (unit_vector  == [-1, 0]):
                string = string + "d "
               
            #right
            elif (unit_vector  == [1, 0]):
                string = string + "e "
            
            #back left
            elif (unit_vector  == [-.7, -.7]):
                string = string + "f "
            
            #backwards
            elif (unit_vector  == [0, -1]):
                string = string + "g "
        
            #back right
            elif (unit_vector  == [.7, -.7]):
                string = string + "h "
    
            # add time to the string(1000 because its in milliseconds in arduino)
            time  = round(distance * 1000 / velocity)
            time = str(time)
        string = string + time
        new_action_sequence.append(string)
    return new_action_sequence

#serial ports
ser = serial.Serial('/dev/ttyACM0',38400)
ser.reset_input_buffer()
#arduino = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
#arduino.reset_input_buffer()

if __name__ == '__main__':
    # blue or green
    color = "green"
    stop = True
    
    while stop:
        if ser.in_waiting > 0:
            
            #line = ser.readline().decode('utf-8').rstrip()
            read_serial = str(ser.readline(40))
            
            #serial_message = Parse(read_serial)
            checks if serial message is filled
            if len(serial_message) > 9:
                #checks if serial message is complete
                if serial_message[9] == 'Q':
                    blue_coords, green_coords, ball_coords, yaw = all_coords(serial_message)
                    print(blue_coords, green_coords, ball_coords, yaw)
                    if color == "blue":
                        goal_coords = [48,0]
                        cart1_coords = blue_coords
                        cart2_coords = green_coords
                        goal_direction = 180
                    else:
                        goal_coords = [48,144]
                        cart2_coords = blue_coords
                        cart1_coords = green_coords
                        goal_direction = 0
                    #fit it to arena
                    ball_coords = discretization(ball_coords)
                    cart2_coords = discretization(cart2_coords)
                    cart1_coords = discretization(cart1_coords) 
                    # put into state space
                    state = state_space()
                    state = change_state(state, cart1_coords, cart2_coords, ball_coords, goal_coords)
                    
                    #A* star algorithm
                    calculation = CalculationAStarAlgorithm(state, "S", "G")
                    path = calculation.aStarAlgorithm()
                    
                    #make path into action_sequence
                    if len(path) > 0:
                        print("\n".join(calculation.renderPath(path)))
                        path, action_sequence = make_path(path)
                    else:
                        print('failed')
                        continue
                
                    #condense action_sequence
                    action_sequence = action_sequence_condense(action_sequence)
                    #action_sequence readable for arduino
                    action_sequence = action_seq_arduino(action_sequence)
                    print(action_sequence)
                    #string = 'e 1000 \n'
                    
                    #arduino.write(string.encode())
    
    """
    #this is a thread to check button status at all times
    thread1 = Thread(target = button_function)
    thread1.start()
    """
    """
    #this is to grab receiver data and accelerometer
    thread2 = Thread(target = receiver_function)
    thread2.start()
    """
    """
    # blue or green
    color = "green"
    
    if color == "blue":
        goal_coords = [48,0]
        goal_direction = 180
    else:
        goal_coords = [48,144]
        goal_direction = 0
    goal_coords = discretization(goal_coords)
    
    #made up coordinates 
    ball_coords = [515,515]
    cart2_coords = [15,30]
    cart1_coords = [10,10]
    ball_coords = discretization(ball_coords)
    print(ball_coords)
    cart2_coords = discretization(cart2_coords)
    cart1_coords = discretization(cart1_coords)
    # depending on scenario will either play defense or offense
    """
    #ball_coords = Three_scenarios(ball_coords, cart1_coords, cart2_coords, color)
    """
    # put into state space
    state = state_space()
    state = change_state(state, cart1_coords, cart2_coords, ball_coords, goal_coords)
    
    #A* star algorithm
    calculation = CalculationAStarAlgorithm(state, "S", "G")
    path = calculation.aStarAlgorithm()
    
    #make path into action_sequence
    if len(path) > 0:
        pass
        #print("\n".join(calculation.renderPath(path)))
    else:
        print('failed')
    path, action_sequence = make_path(path)
        
    #condense action_sequence
    action_sequence = action_sequence_condense(action_sequence)
    #action_sequence readable for arduino
    action_sequence = action_seq_arduino(action_sequence)
    print(action_sequence)
    #serial stuff
    """
    #arduino = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    #write_read(action_sequence, arduino)
    """
    
