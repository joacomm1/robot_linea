
#El siguiente codigo es la base para el desarrollo de la estimacion de la posicion (Odometria) de un robot diferencial
#Modificar las partes que estan segnaladas 

from controller import Robot, DistanceSensor, Motor
import numpy as np

 
MAX_SPEED = 6.28 # [rad/s] define la maxima velocidad de los motores

# crea una instancia del robot.
robot = Robot()


# intervalo discreto por cada paso de simulacion del mundo (world) en ms
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t =timestep/1000.0    # [s] # cambio a unidades de [s]

# Estados (states)
states = ['forward', 'turn_right', 'turn_left'] # lista de todos los estados que tiene la maquina de estados
current_state = states[0] # Define cual sera el estado en el que partira el robot

# contador (counter): algunos estados estaran activos por un tiempo que se puede definir a traves de counter
counter = 0
COUNTER_MAX = 5 # jugar con este valor para ver que conviene



# Robot pose
# Ajustar a los valores iniciales que se encuentran en la consola. 
# de la simuacion webots. 
#Posicion y orientacion
x = 0.00    # posicion en x [m]
y = 0.44    # posicion en y [m]
phi = 0.0   # orientacion [rad]

# Velocidad en los ejes. Se ira actualizando a medida que midamos los encoders
dx = 0.0   # velocidad en x [m/s]
dy = 0.0   # velocidad en y [m/s]


# Velocidad angular de las ruedas, basado en los resultados de los encoders
wl = 0.0    # Velocidad angular rueda izquierda (l de left)  [rad/s]
wr = 0.0    # Velocidad angular rueda derecha   (r de right) [rad/s]

# Velocidad lineal (en el eje x del robot) y angular
u = 0.0    # velocidad lineal  [m/s]
w = 0.0    # velocidad angular [rad/s]

# Parametros fisicos del e-puck (robot diff)
R = 0.0205    # Radio Ruedas: 20.5mm [m]
D = 0.0565    # Distancia entre ruedas: 52mm [m]


#-------------------------------------------------------
# Inicializar los dispositivos 

# Sensores de proximidad en el suelo. Esos nos daran un valor proporcional a la luz que se rebota
gs = []
gsNames = ['gs0', 'gs1', 'gs2'] # hay solamente 3 en este robot. #pregunta# donde pueden ver los nombres y mas infromacion sobre ellos? 
for i in range(3):
    gs.append(robot.getDevice(gsNames[i])) # Agrega los 3 sensores en gs
    gs[i].enable(timestep) #los habilita 


# encoders: Para la odometria ahora necesitamos esta info
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
 
oldEncoderValues = [] # aqui almacenaremos el valor antiguo de los encoders. Eso nos permitira saber cuanto avanzaron

# motores    
leftMotor = robot.getDevice('left wheel motor') #similar a los sensores, pero ahora con los motores (actuadores)
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

 
#Funciones para poder calcular la posicion, estan basadas en las ecuaciones vistas en clases
#A ser llenadas por cada estudiante. 

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    # Calcula la velocidad angular de las ruedas en [rad/s]:
    wl = (encoderValues[0]-oldEncoderValues[0])/delta_t
    wr = (encoderValues[1]-oldEncoderValues[1])/delta_t
    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    #Velocidad lineal (u) y angular(w). Ver ecuaciones en ppt  
    u = (R*wl/2)+(R*wr/2)
    w = (R*wr/D)-(R*wl/D)
    return u, w


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    # Calcula posicion y orientacion en el eje global
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    x = x_old + np.cos(phi)*u*delta_t
    y = y_old + np.sin(phi)*u*delta_t
   
    return x, y, phi
 

#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Update sensor readings
    
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
    
    #opcion para imprimir los valores de cada encoder  
    for indice, valor in enumerate(encoderValues):
        print("encoder ", indice, "valor ",valor  )    
  

    # Una forma de ver la velocidad de las ruedas (en rad/s)
    #Solo usado para confirmar y validar, no para odometria. (ya que solo es posible gracias a webots)
    left_wheel_speed = leftMotor.getVelocity()
    right_wheel_speed = rightMotor.getVelocity()
    
    # Imprime la velocidad 
    print("Left wheel speed:", left_wheel_speed, "rad/s")
    print("Right wheel speed:", right_wheel_speed, "rad/s")
    
    print(f'wl: {wl}')
    print(f'wr: {wr}')

    print(f'step time: {delta_t}')
   
    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   
    
    # Process sensor data
    line_right = gsValues[0] > 600
    line_left  = gsValues[2] > 600

    # Maquina de estado para seguir la linea 
    if current_state == 'forward':
        # Action for the current state: update speed variables
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED

        # check if it is necessary to update current_state
        if line_right and not line_left:
            current_state = 'turn_right'
            counter = 0
        elif line_left and not line_right:
            current_state = 'turn_left'
            counter = 0
            
    if current_state == 'turn_right':
        # Action for the current state: update speed variables
        leftSpeed = 0.8 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'

    if current_state == 'turn_left':
        # Action for the current state: update speed variables
        leftSpeed = 0.4 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'        

  
    
    #######################################################################
    # Localizacion del robot

    # Calcula la velocidad de las ruedas
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    
    # Calcula velocidad lineal y angular
    [u, w] = get_robot_speeds(wl, wr, R, D)
    
    # calcula la nueva posicion del robot
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

   
    
    # Actualiza el valor del encoder para el proximo ciclo
    oldEncoderValues = encoderValues
    
    # incrementa el contador counter
    counter += 1

    # To help on debugging:        
    #print('Counter: '+ str(counter), gsValues[0], gsValues[1], gsValues[2])
    #print('Counter: '+ str(counter) + '. Current state: ' + current_state)
    print(f'Sim time: {robot.getTime():.3f}  Pose: x={x:.5f} m, y={y:.5f} m, phi={phi:.5f} rad.')    


    # Fijar la velocidad de los motores que asignamos en las maquinas de estado
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    
