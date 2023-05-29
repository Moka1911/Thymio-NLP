"""Thymio_Controller controller."""

# Imports
from controller import Robot
import numpy as np
import speech_recognition as sr
from keras.models import load_model
import csv
import keyboard
import asyncio
import time
import math
import random


# Gloval Variables
GROUND_SENSOR_COUNT = 2
DISTANCE_SENSOR_COUNT = 7
THRESHOLD_TOUCH = 4100
walk_condition = False
follow_condition = False
model = load_model("final_model.h5")
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Get the motor values
motor_l = robot.getDevice('motor.left')
motor_r = robot.getDevice('motor.right')

motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf')) 

# Set initial velocity
motor_l.setVelocity(0)
motor_r.setVelocity(0)

# Create speech recognition object
r = sr.Recognizer()

# THIS CODE WAS BORROWED FROM CONSTRUCTIVE AI MODULE

# initialise sensors
def init_sensors():
    global distance_sensors, distance_sensors_values, ground_sensors, ground_sensors_values
    # Set up distance sensors
    distance_sensors = []
    for i in range(DISTANCE_SENSOR_COUNT):
        distance_sensors_name = 'prox.horizontal.{:d}'.format(i)
        distance_sensors.append(robot.getDistanceSensor(distance_sensors_name))
        distance_sensors[i].enable(timestep)
    ground_sensors = []
    for i in range(GROUND_SENSOR_COUNT):
        ground_sensors_name = 'prox.ground.{:d}'.format(i)
        ground_sensors.append(robot.getDistanceSensor(ground_sensors_name))
        ground_sensors[i].enable(timestep)
        
    distance_sensors_values = [0] * DISTANCE_SENSOR_COUNT
    ground_sensors_values = [0] * GROUND_SENSOR_COUNT

# read sesnors data    
def read_sensors():
    global distance_sensors, distance_sensors_values, ground_sensors, ground_sensors_values, distance_sensor_left, distance_sensor_right
    
    for i in range(GROUND_SENSOR_COUNT):
        ground_sensors_values[i] = ground_sensors[i].getValue()
    for i in range(DISTANCE_SENSOR_COUNT):
        distance_sensors_values[i] = distance_sensors[i].getValue()
    distance_sensor_left = 2 * distance_sensors_values[0] + distance_sensors_values[1] + 0.5 * distance_sensors_values[2]
    distance_sensor_right = 2 * distance_sensors_values[4] + distance_sensors_values[3] + 0.5 * distance_sensors_values[2]  
 
def randomRoam():
    #Behaviour which makes Thymio roam randomly across the area
    #walk_stability - countdown before changing dirction of rotatio
    #random_int_r and random_int_l - random values for motor speed between -1 and 1 to rotate Thymio
    #time_left - time before Thymio changing behaviour happens

    global walk_stability, motor_speed_r, motor_speed_l, time_left, random_int_r, random_int_l, distance_sensor_left, distance_sensor_right
    current_time = robot.getTime() # get current time
    
    if walk_stability < 0:   
        time_left = current_time + 1 # make time_left 1 second more than current time
        random_int_r = random.choice([-1,1])
        random_int_l = random.choice([-1,1])
        # To avoid double values of -1 or 1, which will not make Thymio turn
        #  we should check if one of the motors already is -1 or 1
        if random_int_r == -1:
            random_int_l = 1
        else:
            random_int_l = -1
            
    if time_left - current_time > 0: #make rotation last 1 seconds
        motor_r.setVelocity(random_int_r * 5.5)
        motor_l.setVelocity(random_int_l * 5.5)
        walk_stability = 140 # return countdown            
    else: #when countdown is not 0, keep Thymio move
        motor_r.setVelocity(5.5)
        motor_l.setVelocity(5.5)
        walk_stability -= 1 #decrease countdown until next turn
        
# Function to move and avoid walls
def walk():
    global  distance_sensor_left, distance_sensor_right, walk_condition
    #To avoid walls
    if distance_sensor_left > THRESHOLD_TOUCH:
        motor_r.setVelocity(-5.5)
        motor_l.setVelocity(5.5)
        
    elif distance_sensor_right > THRESHOLD_TOUCH:
        motor_r.setVelocity(5.5)
        motor_r.setVelocity(-5.5)
    
    elif  ground_sensors_values[0] < 160 or ground_sensors_values[1] < 160:
        stop()
        print("The line detected :)")
        walk_condition = False
    #Randomly roam if no walls detecte
    else:
        randomRoam()  
        
# END OF THE BORROWED CODE 

# Function to followe the line (to collect data)        
def follow_line(sensors):
    # set speed depending on the threshhold of a black color (160)
    if sensors[0] < 160 and sensors[1] < 160:
        motor_l.setVelocity(9.53)
        motor_r.setVelocity(9.53)
    elif sensors[0] < 160 and sensors[1] > 160:
        motor_l.setVelocity(-9.53)
        motor_r.setVelocity(9.53)
    elif sensors[0] > 160 and sensors[1] < 160:
        motor_l.setVelocity(9.53)
        motor_r.setVelocity(-9.53)  

# Function for straight movement
def go_straight():
        global motor_speed_l, motor_speed_r
        motor_speed_l = 4.53
        motor_speed_r = 4.53
        motor_l.setVelocity(motor_speed_l)
        motor_r.setVelocity(motor_speed_r)

# Turning thymio left        
def go_left():
    global motor_speed_l, motor_speed_r
    motor_speed_l = motor_l.getVelocity() + 0.001
    motor_speed_r = motor_r.getVelocity() + 0.001
    # Calculate the time required to turn 90 degrees
    wheel_distance = 0.052  # Distance between wheels in meters
    robot_width = 0.138    # Width of the robot in meters
    turn_radius = (robot_width / 2.0) + (wheel_distance / 2.0)
    turn_circumference = 2 * math.pi * turn_radius
    turn_distance = turn_circumference * 0.25  # Turn for 25% of the circumference
    turn_time = turn_distance / ((motor_speed_l + motor_speed_r) / 2.0)
    
    # Calculate the angle to turn
    angle_to_turn = 180.0
    
    # Reset the motor velocities
    motor_speed_l = -5.00
    motor_speed_r = 5.00
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    
    # Calculate the current angle
    current_angle = 0.0
    prev_time = robot.getTime()
    # Keep turning until the desired angle is reached
    while current_angle < angle_to_turn:
        # Calculate the time since the last iteration
        time_diff = robot.getTime() - prev_time
        
        # Update the previous time
        prev_time = robot.getTime()
        
        # Update the current angle based on the time elapsed and the motor speeds
        current_angle += ((motor_speed_r - motor_speed_l) / wheel_distance) * time_diff
        
        # Wait for a short time before the next iteration
        robot.step(timestep)
    
    # Stop the robot briefly to let it stabilize
    motor_l.setVelocity(0)
    motor_r.setVelocity(0)
    time.sleep(0.1)
    
    # Move the robot straight for a short distance
    motor_speed_l = 5.53
    motor_speed_r = 5.53
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    time.sleep(1.0)
    
    # Stop the robot
    motor_l.setVelocity(5.5)
    motor_r.setVelocity(5.5)

# Turning Thymio right
def go_right():
    global motor_speed_l, motor_speed_r
    motor_speed_l = motor_l.getVelocity() + 0.001
    motor_speed_r = motor_r.getVelocity() + 0.001    
    # Calculate the time required to turn 90 degrees
    wheel_distance = 0.052  # Distance between wheels in meters
    robot_width = 0.138    # Width of the robot in meters
    turn_radius = (robot_width / 2.0) + (wheel_distance / 2.0)
    turn_circumference = 2 * math.pi * turn_radius
    turn_distance = turn_circumference * 0.25  # Turn for 25% of the circumference
    turn_time = turn_distance / ((motor_speed_l + motor_speed_r) / 2.0)
    
    # Calculate the angle to turn
    angle_to_turn = 180.0
    
    # Reset the motor velocities
    motor_speed_l = 5.00
    motor_speed_r = -5.00
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    
    # Calculate the current angle
    current_angle = 0.0
    prev_time = robot.getTime()
    # Keep turning until the desired angle is reached
    while current_angle < angle_to_turn:
        # Calculate the time since the last iteration
        time_diff = robot.getTime() - prev_time
        
        # Update the previous time
        prev_time = robot.getTime()
        
        # Update the current angle based on the time elapsed and the motor speeds
        current_angle += ((motor_speed_l - motor_speed_r) / wheel_distance) * time_diff
        
        # Wait for a short time before the next iteration
        robot.step(timestep)
    
    # Stop the robot briefly to let it stabilize
    motor_l.setVelocity(0)
    motor_r.setVelocity(0)
    time.sleep(0.1)
    
    # Move the robot straight for a short distance
    motor_speed_l = 5.53
    motor_speed_r = 5.53
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    time.sleep(1.0)
    
    # Stop the robot
    motor_l.setVelocity(5.5)
    motor_r.setVelocity(5.5)

# Function for backward movement
def go_back():
        global motor_speed_l, motor_speed_r
        motor_speed_l = -4.53
        motor_speed_r = -4.53
        motor_l.setVelocity(motor_speed_l)
        motor_r.setVelocity(motor_speed_r)
 
# Function to set speed for motors        
def set_speed(speed_l, speed_r):
    motor_l.setVelocity(4 + speed_l)
    motor_r.setVelocity(4 + speed_r)     

# Function to stop Thymio
def stop():
    motor_l.setVelocity(0)
    motor_r.setVelocity(0)

# Function for finding a line
def find_line():
    global motor_speed_l, motor_speed_r
    motor_speed_l = motor_l.getVelocity() + 0.001
    motor_speed_r = motor_r.getVelocity() + 0.001    
    # Calculate the time required to turn 90 degrees
    wheel_distance = 0.052  # Distance between wheels in meters
    robot_width = 0.138    # Width of the robot in meters
    turn_radius = (robot_width / 2.0) + (wheel_distance / 2.0)
    turn_circumference = 2 * math.pi * turn_radius
    turn_distance = turn_circumference * 0.25  # Turn for 25% of the circumference
    turn_time = turn_distance / ((motor_speed_l + motor_speed_r) / 2.0)
    
    # Calculate the angle to turn
    angle_to_turn = random.randint(0, 180)
    
    # Reset the motor velocities
    values = [1, -1]
    random.shuffle(values)
    motor_speed_l = values[0] * 5.00
    motor_speed_r = values[1] * 5.00
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    
    # Calculate the current angle
    current_angle = 0.0
    prev_time = robot.getTime()
    # Keep turning until the desired angle is reached
    while current_angle < angle_to_turn:
        # Calculate the time since the last iteration
        time_diff = robot.getTime() - prev_time
        
        # Update the previous time
        prev_time = robot.getTime()
        
        # Update the current angle based on the time elapsed and the motor speeds
        current_angle += (abs(motor_speed_l - motor_speed_r) / wheel_distance) * time_diff
        
        # Wait for a short time before the next iteration
        robot.step(timestep)
    
    # Stop the robot briefly to let it stabilize
    motor_l.setVelocity(0)
    motor_r.setVelocity(0)
    time.sleep(0.1)
    
    # Move the robot straight for a short distance
    motor_speed_l = 5.53
    motor_speed_r = 5.53
    motor_l.setVelocity(motor_speed_l)
    motor_r.setVelocity(motor_speed_r)
    time.sleep(1.0)
    
    # Stop the robot
    motor_l.setVelocity(5.5)
    motor_r.setVelocity(5.5)    


# Create a recognizer instance
r = sr.Recognizer()

# Define a function to handle speech input
def handle_speech_input():
    
    # Use the microphone as audio source
    with sr.Microphone() as source:
        print("Listening...")
        audio = r.listen(source)
    return audio
      
def handle_key_press(event):
    global perform_speech_recognition
    # Check if the "r" key was pressed
    if event.name == 'r':
        # Call the speech recognition function
        perform_speech_recognition = True
# Continuously listen for speech input and handle it

keyboard.on_press(handle_key_press)
perform_speech_recognition = False
init_sensors()
time_left = 0
walk_stability = 140



# Main loop:

while robot.step(timestep) != -1:
    read_sensors()
    ground_sensors_np_array = np.array(ground_sensors_values).reshape(-1,2)
    if walk_condition:
        walk()
    elif follow_condition:
        prediction = model.predict(np.array([float(ground_sensors_values[0]),float(ground_sensors_values[1])]).reshape(1,2))
    
        motor_l.setVelocity(int(prediction[0][0]))
        motor_r.setVelocity(int(prediction[0][1]))
        print(prediction[0])
        print(ground_sensors_values)
            

    if perform_speech_recognition:
        try:
            # Use Speech Recognition to transcribe audio
            text = r.recognize_google(handle_speech_input())
            print("You said:", text)
        
            # Perform a task based on the recognized text
            if "go straight" in text:
                go_straight()
                walk_condition = False
                follow_condition = False
            elif "stop here" in text:
                stop()
                walk_condition = False
                follow_condition = False
            elif "go right" in text:
                go_right()
                walk_condition = False
                follow_condition = False
            elif "go left" in text:
                go_left()
                walk_condition = False
                follow_condition = False
            elif "search for line" in text:
                walk_condition = True
                follow_condition = False
            elif "follow line" in text:
                if ground_sensors_values[0] < 160 or ground_sensors_values[1] < 160:
                    follow_condition = True
                    walk_condition = False
                else:
                    print("The line is not deteced :(")
                    follow_condition = False                           
            elif "increase speed" in text:
                set_speed(motor_l.getVelocity(), motor_r.getVelocity())
            else:
                print(text)
        except sr.UnknownValueError:
            print("Sorry, I could not understand what you said.")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        perform_speech_recognition = False
       

    # CODE FOR DATA COLLECTION 
    
    #follow_line(ground_sensors_values)
    """
    with open('data.csv','a',newline='') as file:
        writer = csv.writer(file)
        writer.writerow([ground_sensors_values[0],ground_sensors_values[1],motor_l.getVelocity(),motor_r.getVelocity()])
        file.close()
    """

    # TESTING FUNCTINOS
    #prediction = model.predict(np.array([float(ground_sensors_values[0]),float(ground_sensors_values[1])]).reshape(1,2))

    #motor_l.setVelocity(int(prediction[0][0]))
    #motor_r.setVelocity(int(prediction[0][1]))
# Enter here exit cleanup code.
