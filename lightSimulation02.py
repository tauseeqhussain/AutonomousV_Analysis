
# Owned by Mario Carvalho (mnogueiramansurcarva@hawk.iit.edu) and Syed Tauseeq Hussain (shussain10@hawk.iit.edu)
# Updated: 04/19/2023

# This simulation assumes a predefined city route that is divided into segments between traffic lights.
# This script simulates the behavior of an autonomous vehicle and a human-drive vehicle when driving through this route
# The autonomous vehicle and the human-drive vehicle have different speed profiles along the route, because the speed decisions are made differently
# Autonomous vehicles can choose an optimal speed that reduces the amount of stops at traffic lights, because they may have access to traffic light schedules
# Human drivers, on the other hand, don't know traffic light schedules, hence make decisions based on the lights they see only

import numpy as np
import matplotlib.pyplot as plt
import random
import math
from scipy.optimize import fsolve
from datetime import datetime

#Simulation Parameters Set Up
time_step = 0.001 #For graphs generation
max_acceleration_ms2 = 2 #In m/s^2
human_driver_decision_distance_m = 50 #Distance before traffic light that human driver decides whether to stop or not, in meters

#Initial Street Parameters Set Up
initial_simulation_speed_mph = 0
traffic_light_duration_array = np.array([[30,3,20],[30,3,30],[20,3,25],[30,3,35],[30,3,35],[30,3,35],[20,3,10],[20,3,20],[20,3,30]]) #One row per traff light, in-row order is: Green duration, Yellow dur., Red Dur. First light is at 33rd & State
speed_limit_mph_array = np.array([25,25,30,30,40,40,30,30,30])
street_length_ft_array = np.array([1346,1320,936,466,1332,1351,234,248,920])

def traffic_light_state_generator(light_duration): #Randomly sets the initial traffic light that is on, and the time remianing for it
    light_colors = np.array(['g', 'y', 'r'])
    probabilities=np.array([0.46,0.04,0.50])

    # Randomly choose a traffic light color
    traff_light_on = np.random.choice(light_colors,p=probabilities)
    if traff_light_on == 'g':
        time_left = np.random.randint(0,light_duration[0])
    elif traff_light_on == 'y':
        time_left = np.random.randint(0,light_duration[1])
    else: #red
        time_left = np.random.randint(0,light_duration[2])

    return traff_light_on, time_left

#Conversions to SI
speed_limits_ms_array = speed_limit_mph_array * 0.44704 #Now speed is in m/s
street_length_m_array = street_length_ft_array * 0.3048 #Now street length is in meters
initial_simulation_speed_ms = initial_simulation_speed_mph * 0.44704


#--------------------------AUTONOMOUS VEHICLE SIMULATION-------------------------------------------
#Car is at one crossing, will calculate optimal speed until next crossing
current_speed_ms = initial_simulation_speed_ms
auton_simulation_time = np.zeros(1)
auton_vehicle_speed = np.zeros(1)
auton_vehicle_position = np.zeros(1)
simulation_time_accumulated = 0
for section in range(len(speed_limit_mph_array)): #One per street segment / from one traff light to another
    green_light_time = traffic_light_duration_array[section,0]
    yellow_light_time = traffic_light_duration_array[section,1]
    red_light_time = traffic_light_duration_array[section,2]
    max_acceleration = max_acceleration_ms2 
    initial_speed_ms = current_speed_ms # in m/s
    dist_to_next_light_m = street_length_m_array[section] # in m
    light_state, time_to_switch = traffic_light_state_generator(traffic_light_duration_array[section,:])
    speed_limit_ms = speed_limits_ms_array[section] #In m/s

    can_make_it = False #To be used for deciding whether car will go full speed or not
    decision_made = False #To check if calculations still need to be done
    target_speed_ms = 0 # To be used to define car speed

    #If light ahead is green, need to check if car can make it
    if light_state == 'g':
        decision_made = False
        # Check if car can make it during initial green light
        def travel_time_finder(t): #Finds maximum velocity 
            v_max = speed_limit_ms
            v0 = initial_speed_ms
            a = max_acceleration
            xf = dist_to_next_light_m
            return t - abs(v_max - v0) / a - xf / v_max + abs(v_max * v_max - v0 * v0) / (2 * a * v_max)

        t = fsolve(travel_time_finder, x0=10, xtol=1e-6)

        if (t<=time_to_switch)and(np.isreal(t))and(t>0): #Vehicle can make it
            target_speed_ms = speed_limit_ms #Travel at speed limit
            decision_made = True
        
        accumulated_time = yellow_light_time+red_light_time
        while not decision_made: #If couldn't make it during first green light
            #print(accumulated_time)
            time_to_next_green = accumulated_time + time_to_switch #Light is already green
            def v_target_finder(v_target): #Finds steady velocity to reach light
                t = time_to_next_green
                v0 = initial_speed_ms
                a = max_acceleration
                xf = dist_to_next_light_m
                return t - abs(v_target - v0) / a - xf / v_target + abs(v_target * v_target - v0 * v0) / (2 * a * v_target)

            v_target = fsolve(v_target_finder, x0=10, xtol=1e-6)

            if (v_target <= speed_limit_ms)and(np.isreal(v_target))and(v_target>0): #Vehicle can make it at beginning of green light
                target_speed_ms =  v_target
                decision_made = True
            elif (np.isreal(v_target))and(v_target>0): #Need to check if vehicle can make it during duration of green light
                def v_target_finder(v_target): #Finds steady velocity to reach light
                    t = time_to_next_green + green_light_time
                    v0 = initial_speed_ms
                    a = max_acceleration
                    xf = dist_to_next_light_m
                    return t - abs(v_target - v0) / a - xf / v_target + abs(v_target * v_target - v0 * v0) / (2 * a * v_target)

                v_target = fsolve(v_target_finder, x0=10, xtol=1e-6)
                if (v_target <= speed_limit_ms)and(np.isreal(v_target))and(v_target>0): #vehicle can make it
                    target_speed_ms =  speed_limit_ms
                    decision_made = True
                else: #Must wait until next green light
                    decision_made = False

            accumulated_time += green_light_time+yellow_light_time+red_light_time


    else: #If light ahead is red/yellow, need to find how soon will it be green
        accumulated_time = 0
        while not decision_made:
            if light_state == 'y':
                time_to_next_green = time_to_switch + red_light_time + accumulated_time
            else: #If red
                time_to_next_green = time_to_switch + accumulated_time
            
            def v_target_finder(v_target): #Finds steady velocity to reach light
                t = time_to_next_green
                v0 = initial_speed_ms
                a = max_acceleration
                xf = dist_to_next_light_m
                return t - abs(v_target - v0) / a - xf / v_target + abs(v_target * v_target - v0 * v0) / (2 * a * v_target)

            v_target = fsolve(v_target_finder, x0=10, xtol=1e-6)

            if v_target <= speed_limit_ms: #Vehicle can make it
                target_speed_ms =  v_target
                decision_made = True
            else: #Need to check if vehicle can make it during duration of next green light
                def v_target_finder(v_target): #Finds steady velocity to reach light
                    t = time_to_next_green + green_light_time
                    v0 = initial_speed_ms
                    a = max_acceleration
                    xf = dist_to_next_light_m
                    return t - abs(v_target - v0) / a - xf / v_target + abs(v_target * v_target - v0 * v0) / (2 * a * v_target)

                v_target = fsolve(v_target_finder, x0=10, xtol=1e-6)

                if v_target <= speed_limit_ms: #vehicle can make it
                    target_speed_ms =  speed_limit_ms #Will travel at speed limit
                    decision_made = True
                else: #Must wait until next green light
                    decision_made = False
                    accumulated_time += green_light_time+yellow_light_time+red_light_time


    simulation_time = np.arange(0,4*dist_to_next_light_m/target_speed_ms,time_step)
    vehicle_speed = np.zeros(len(simulation_time))
    vehicle_position = np.zeros(len(simulation_time))
    light_prediction = np.zeros(len(simulation_time)) #1 if green, 0 if yellow or red
    current_speed_ms = initial_speed_ms

    for j in range(len(simulation_time)): #Determine space, velocity, time graphs
        if abs(current_speed_ms-target_speed_ms)>0.05: #if still accelerating/deaccelerating
            if initial_speed_ms > target_speed_ms: #If deaccelerating
                vehicle_speed[j] = initial_speed_ms - max_acceleration*simulation_time[j]
            else: #If accelerating
                vehicle_speed[j] = initial_speed_ms + max_acceleration*simulation_time[j]
            current_speed_ms = vehicle_speed[j]

        else: #Speed is on target already
            vehicle_speed[j] = target_speed_ms
            current_speed_ms = vehicle_speed[j]
            
        vehicle_position[j] = vehicle_position[j-1] + vehicle_speed[j-1]*time_step

    for j in range(len(simulation_time)): #To determine traffic light schedule graph
        if light_state == 'g':
            green_shift = time_to_switch - green_light_time #Calculates at what simulation time the traffic light cycle begins - Can be negat
        elif light_state == 'y':
            green_shift = time_to_switch - green_light_time - yellow_light_time
        else: #red
            green_shift = time_to_switch - green_light_time - yellow_light_time - red_light_time

        # Calculate the cycle time and the time within the cycle
        cycle_length = green_light_time + yellow_light_time + red_light_time
        this_cycle_time = simulation_time[j]-green_shift 
        cycle_current_time = this_cycle_time % cycle_length
        # Determine which traffic light is on
        if (cycle_current_time < green_light_time):
            light_prediction[j] = dist_to_next_light_m
        else: #Cycle is yellow or red
            light_prediction[j] = -dist_to_next_light_m


    """
    #Plot speed vs time
    plt.scatter(simulation_time, vehicle_speed)
    plt.xlabel('Time / s')
    plt.ylabel('Vehicle Speed / m/s')
    plt.title('Speed profile between traffic lights - Time analysis')
    #plt.show()

    #Plot speed vs position
    plt.scatter(vehicle_position, vehicle_speed)
    plt.xlabel('Position / m')
    plt.ylabel('Vehicle Speed / m/s')
    plt.title('Speed profile between traffic lights - Space analysis')
    #plt.show()

    #Plot position vs time, with traffic light status
    plt.scatter(simulation_time, vehicle_position, color='blue', label='Position')
    plt.scatter(simulation_time, light_prediction, color='green', label='Light State')
    plt.ylim([0, None]) 
    plt.xlabel('Time / s')
    plt.ylabel('Position & Light State')
    plt.title('Vehicle position and Traffic Light vs Time')
    #plt.axvline(x=time_to_next_green)
    plt.axhline(y=dist_to_next_light_m,color='r')
    #plt.show()
    """

    #Perform Concatenations
    #Find time index where position is end of street
    last_epoch = np.where(vehicle_position>dist_to_next_light_m)
    if last_epoch[0].size > 0:
        index = last_epoch[0][0]
    else:
        index = -1
    auton_simulation_time = np.concatenate((auton_simulation_time,(simulation_time[:index]+auton_simulation_time[-1])))
    auton_vehicle_speed = np.concatenate((auton_vehicle_speed,vehicle_speed[:index]))
    auton_vehicle_position = np.concatenate((auton_vehicle_position,(vehicle_position[:index]+auton_vehicle_position[-1])))

#__________________________________________________________________________________________________


#--------------------------HUMAN DRIVER SIMULATION-------------------------------------------------
#Car is at one crossing, will calculate optimal speed until next crossing
current_speed_ms = initial_simulation_speed_ms
human_simulation_time = np.zeros(1)
human_vehicle_speed = np.zeros(1)
human_vehicle_position = np.zeros(1)
simulation_time_accumulated = 0
for section in range(len(speed_limit_mph_array)): #One per street segment / from one traff light to another
    green_light_time = traffic_light_duration_array[section,0]
    yellow_light_time = traffic_light_duration_array[section,1]
    red_light_time = traffic_light_duration_array[section,2]
    max_acceleration = max_acceleration_ms2 
    initial_speed_ms = current_speed_ms # in m/s
    dist_to_next_light_m = street_length_m_array[section] # in m
    light_state, time_to_switch = traffic_light_state_generator(traffic_light_duration_array[section,:])
    speed_limit_ms = speed_limits_ms_array[section] #In m/s


    #Create array to monitor which light is currently on
    light_state_array = np.array([light_state])
    cycle_length = green_light_time + yellow_light_time + red_light_time
    if light_state == 'g':
        green_shift = time_to_switch - green_light_time #Calculates at what simulation time the traffic light cycle begins - Can be negat
    elif light_state == 'y':
        green_shift = time_to_switch - green_light_time - yellow_light_time
    else: #red
        green_shift = time_to_switch - green_light_time - yellow_light_time - red_light_time

    #Create space, time, speed arrays
    vehicle_position = np.zeros(1)
    simulation_time = np.zeros(1)
    vehicle_speed = np.array([initial_speed_ms])

    #Perform analysis while vehicle does not reach traffic light
    while vehicle_position[-1] < dist_to_next_light_m: #While car does not reach traffic light

        #Determine which light is currently on (Updating during all time epochs)
        this_cycle_time = simulation_time[-1]-green_shift 
        cycle_current_time = this_cycle_time % cycle_length
        # Determine which traffic light is on
        if (cycle_current_time < green_light_time):
            light_state_array = np.append(light_state_array,'g') #Trick so that green light shows on positive y-axis, yellow/red on negative y-axis
        elif (cycle_current_time < green_light_time+yellow_light_time): #Cycle is yellow:
            light_state_array = np.append(light_state_array,'y')
        else: #light is red
            light_state_array = np.append(light_state_array,'r')
        light_now = light_state_array[-1]

        #Check how close car is to intersection:
        remaining_dist_to_light = dist_to_next_light_m-vehicle_position[-1]

        #Human driver decision process:

        #Human driver has to brake if getting close to light when light is not green
        if((remaining_dist_to_light<human_driver_decision_distance_m)and(light_now!='g')): 
            braking_acceleration = 0.5*(current_speed_ms*current_speed_ms)/remaining_dist_to_light #Acceleration necessary to stop before light
            current_speed_ms -= time_step*braking_acceleration
        else: #Still far from Traffic Light, can keep changing speed towards speed limit
            #Change speed towards speed limit
            if (abs(current_speed_ms - speed_limit_ms) < 0.1):
                current_speed_ms = speed_limit_ms #Speed already maximum
            elif (current_speed_ms < speed_limit_ms):
                current_speed_ms += time_step*max_acceleration
            else: #current_speed_ms > speed_limit_ms
                current_speed_ms -+ time_step*max_acceleration

        #Update time, space, and speed graphs:
        vehicle_position = np.append(vehicle_position,vehicle_position[-1]+current_speed_ms*time_step)
        simulation_time = np.append(simulation_time,simulation_time[-1]+time_step)
        vehicle_speed = np.append(vehicle_speed,current_speed_ms)

    #Perform Concatenations
    human_simulation_time = np.concatenate((human_simulation_time,(simulation_time+human_simulation_time[-1])))
    human_vehicle_speed = np.concatenate((human_vehicle_speed,vehicle_speed))
    human_vehicle_position = np.concatenate((human_vehicle_position,(vehicle_position+human_vehicle_position[-1])))

#__________________________________________________________________________________________________

#Make arrays same size
if len(auton_simulation_time)>len(human_simulation_time):
    extension_size = len(auton_simulation_time)-len(human_simulation_time)
    human_simulation_time = np.pad(human_simulation_time,(0,extension_size), mode='constant')
    human_vehicle_speed = np.pad(human_vehicle_speed,(0,extension_size), mode='constant')
    human_vehicle_position = np.pad(human_vehicle_position,(0,extension_size), mode='constant')                         
elif len(auton_simulation_time)<len(human_simulation_time):
    extension_size = len(human_simulation_time)-len(auton_simulation_time)
    auton_simulation_time = np.pad(auton_simulation_time,(0,extension_size), mode='constant')
    auton_vehicle_speed = np.pad(auton_vehicle_speed,(0,extension_size), mode='constant')
    auton_vehicle_position = np.pad(auton_vehicle_position,(0,extension_size), mode='constant')

#Full trip Graphs:

now = datetime.now()
month = str(now.month).zfill(2)
day = str(now.day).zfill(2)
hour = str(now.hour).zfill(2)
minute = str(now.minute).zfill(2)
timestamp = f"{hour}{minute}_{month}{day}"

#Plot speed vs time
plt.scatter(auton_simulation_time, auton_vehicle_speed*2.237,marker='.',label='Autonomous')
plt.scatter(human_simulation_time, human_vehicle_speed*2.237,marker='.',label='Human')
plt.legend()
plt.xlabel('Time / s')
plt.ylabel('Vehicle Speed / mph')
plt.title('Speed vs Time')
plt.ylim([0, None]) 
plt.xlim([0, None])
plt.show()


#Plot speed vs position
plt.scatter(auton_vehicle_position*3.281, auton_vehicle_speed*2.237,marker='.',label='Autonomous')
plt.scatter(human_vehicle_position*3.281, human_vehicle_speed*2.237,marker='.',label='Driver')
plt.legend()
plt.xlabel('Distance / ft')
plt.ylabel('Vehicle Speed / mph')
plt.title('Speed vs Distance')
plt.ylim([0, None]) 
plt.xlim([0, None])
total_dist_travelled_meters = 0
for k in range(len(street_length_m_array)):
    total_dist_travelled_meters += street_length_m_array[k]
    plt.axvline(x=total_dist_travelled_meters*3.281,color='r',linestyle='--') #Show where traffic lights are located
plt.show()

#Plot position vs time, with traffic light status
plt.scatter(auton_simulation_time, auton_vehicle_position*3.281, color='blue', label='Autonomous',marker='.')
plt.scatter(human_simulation_time, human_vehicle_position*3.281, color='black', label='Human',marker='.')
plt.legend()
plt.ylim([0, None]) 
plt.xlim([0, None])
plt.xlabel('Time / s')
plt.ylabel('Distance / ft')
plt.title('Distance vs Time')
total_dist_travelled_meters = 0
for k in range(len(street_length_m_array)):
    total_dist_travelled_meters += street_length_m_array[k]
    plt.axhline(y=total_dist_travelled_meters*3.281,color='r',linestyle='--') #Show where traffic lights are located
plt.show()


#Save results
file_name = f'exp\\simulation_{timestamp}.csv'
#Saved in SI units
np.savetxt(file_name, np.column_stack((auton_simulation_time,auton_vehicle_position,auton_vehicle_speed,human_simulation_time,human_vehicle_position,human_vehicle_speed)),delimiter=',')

