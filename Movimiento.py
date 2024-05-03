#El siguiente codigo es la base para el desarrollo de un line following differential robot
#Modificar las partes que estan segnaladas 

from controller import Robot, DistanceSensor, Motor
import numpy as np


#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28 # [rad/s] define la maxima velocidad de los motores

# crea una instancia del robot.
robot = Robot()

# intervalo discreto por cada paso de simulacion del mundo (world) en ms
timestep = int(robot.getBasicTimeStep())   # [ms]

# Estados (states)
states = ['forward', 'turn_right', 'turn_left'] # lista de todos los estados que tiene la maquina de estados
current_state = states[0] # Define cual sera el estado en el que partira el robot

# contador (counter): algunos estados estaran activos por un tiempo que se puede definir a traves de counter
counter = 0
COUNTER_MAX = 5 # jugar con este valor para ver que conviene


#-------------------------------------------------------
# Initialize devices

 
# Sensores de proximidad en el suelo. Esos nos daran un valor proporcional a la luz que se rebota
gs = []
gsNames = ['gs0', 'gs1', 'gs2'] # hay solamente 3 en este robot. #pregunta# donde pueden ver los nombres y mas infromacion sobre ellos? 
for i in range(3):
    gs.append(robot.getDevice(gsNames[i])) # Agrega los 3 sensores en gs
    gs[i].enable(timestep) #los habilita 


# motores    
leftMotor = robot.getDevice('left wheel motor') #similar a los sensores, pero ahora con los motores (actuadores)
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # Aqui leemos los valores de los sensores del suelo
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Imprimir los valores que nos da. Jugar con la posicion del robot en el espacio para ver como cambian los valores 
    print('Counter: '+ str(counter), gsValues[0], gsValues[1], gsValues[2]) # Definir cual es cual!


    #definir cuando la linea esta a la izquierda o a la derecha, basado en los valores de los sensores
    
    if(gsValues[0]>400):
        line_left = True
    else:
        line_left = False
    if(gsValues[2]>400):
        line_right = True
    else:
        line_right = False
     

    # Implement the line-following state machine
    if current_state == 'forward':
        # Action for the current state: update speed variables
        leftSpeed  = MAX_SPEED #si va hacia adelante, ambas velocidades angulares deben ser iguales
        rightSpeed = MAX_SPEED

        # check if it is necessary to update current_state
        if line_right == True and line_left == False:
            current_state = states[2]
        elif line_left == True and line_right == False:
            current_state = states[1]
            
    if current_state == 'turn_right':
        # Action for the current state: update speed variables
        leftSpeed = 0.8 * MAX_SPEED  
        rightSpeed = 0.4 * MAX_SPEED  

        # Ve si ha pasado el tiempo y es necesario volver al estado forward
        if (gsValues[0]<400):
            print("ENTRE!")
            current_state = states[0]

    if current_state == 'turn_left':
            # Action for the current state: update speed variables
            rightSpeed = 0.8 * MAX_SPEED  
            leftSpeed = 0.4 * MAX_SPEED  
    
            # Ve si ha pasado el tiempo y es necesario volver al estado forward
            if (gsValues[2]<400):
                print("ENTRE!")
                current_state = states[0]

    # incrementa el contador counter
    counter += 1
    
 
    print('Counter: '+ str(counter) + '. Current state: ' + current_state)

    # Fijar la velocidad de los motores que asignamos en las maquinas de estado
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
