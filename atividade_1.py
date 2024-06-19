from turtle import clear
import numpy as np
import matplotlib.pyplot as plt

import math
import sim 
import os
import logging

logging.basicConfig(level=logging.INFO, filename="programa.log", format="%(asctime)s - %(levelname)s - %(message)s")
os.system('cls' if os.name == 'nt' else 'clear')

def andar(eixo, distancia):
    
    # calcula o ponto final para o eixo informado
    r, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) 
    origem = pos[eixo]
    destino = pos[eixo] + distancia 

    #faz o carro andar 
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 3, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 3, sim.simx_opmode_streaming + 5)

    posicao = origem
    if (posicao < destino):
        while (posicao < destino):
            r, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait) 
            posicao = pos[eixo]
    else:
        while (posicao > destino):
            r, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)  
            posicao = pos[eixo]

    pararRobo()

def curva(destino):
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 1, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, -1, sim.simx_opmode_streaming + 5) 
    
    r, orientation = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    atual = math.degrees(orientation[2])

    while (atual > destino):
        #print(math.degrees(orientation[0]), math.degrees(orientation[1]), math.degrees(orientation[2]))
        r, orientation = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        atual = math.degrees(orientation[2])
    pararRobo()

def printPositionAndOrientation():
    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)  
    a, orientation = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    print('Position: ', pos, 'Orientation: ', int(math.degrees(orientation[0])), int(math.degrees(orientation[1])), int(math.degrees(orientation[2])))

def pararRobo():
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)  
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)

def pararSimulacao():
    # Parando a simulação     
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)         
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

def readSensorData(clientId=-1, range_data_signal_id="hokuyo_range_data", angle_data_signal_id="hokuyo_angle_data"):

    laser_data = []
    # the first call should be non-blocking to avoid getting out-of-sync angle data
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id, sim.simx_opmode_streaming)

    # the second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id, sim.simx_opmode_blocking)

    # check the if both data were obtained correctly
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # unpack data from range and sensor messages
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)
        laser_data = np.array([raw_angle_data, raw_range_data]).T
    return laser_data

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
    
    # Iniciando a simulação
    # Deve usar a porta do 'continuous remote API server services' (remoteApiConnections.txt)
    # e = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Handle para o ROBÔ    
    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)     

    # Handle para as juntas das RODAS
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)    
    
    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)   
    
    # Dados do Pioneer
    larguraRobo = 0.381   # Metros
    raioRoda = 0.0975  # Metros
    v_TangencialRoda = 0.3
    v_AngularRoda = np.deg2rad(0)  
    vAngular_esquerda = 3 #v_TangencialRoda/raioRoda - (v_AngularRoda*larguraRobo)/(2*raioRoda)
    vAngular_direita = 3 #v_TangencialRoda/raioRoda + (v_AngularRoda*larguraRobo)/(2*raioRoda)
    t = 0

    # Faz andar pra frente (vAngular, distancia, eixo)
    # trecho 1

    #pararRobo()
    #sim.simxSetObjectPosition(clientID, robotHandle, -1, [0,0,0], sim.simx_opmode_oneshot_wait)
    #sim.simxSetObjectOrientation(clientID, robotHandle, -1, [0,0,1.57], sim.simx_opmode_oneshot_wait)

    laser_range_data = "hokuyo_range_data"
    laser_angle_data = "hokuyo_angle_data"
    
    print('--->>> Inicio da Simulacao <<<---')
    # trecho 1
    printPositionAndOrientation()
    andar(1, 1)
    curva(0)
    laser_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    print('laser_data', laser_data)

    # trecho 2
    printPositionAndOrientation()
    andar(0, 1)
    curva(-90)
    laser_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    print('laser_data', laser_data)

    # trecho 3
    readSensorData(clientID, laser_range_data, laser_angle_data)
    printPositionAndOrientation()
    andar(1, -1)
    curva(-179)
    laser_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    print('laser_data', laser_data)

    # trecho 4
    readSensorData(clientID, laser_range_data, laser_angle_data)
    printPositionAndOrientation()
    andar(0, -1)
    curva(90)
    laser_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    print('laser_data', laser_data)

    printPositionAndOrientation()
    readSensorData(clientID, laser_range_data, laser_angle_data)
    print('--->>> Fim <<<---')
    # todo andar pra zero

    pararSimulacao()
    exit()