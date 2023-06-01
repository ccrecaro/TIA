#!/usr/bin/python
#coding: utf8

"""
Imports
"""
import time
import math
import serial
import atexit, termios  # for keyboard reads
import sys, os          # for keyboard reads
from test_NeatoCommands import envia  # "envia" function sends commands to Neato and returns Neato response, in case we need it
import numpy as np
import math

# Inverse Kinematics Computation
def inverseKinematics(Vc, S, Tita_dot, Ts):
	velR = Vc + ( S * Tita_dot) # mm/s
	velL = Vc + (-S * Tita_dot) # mm/s
	distancia_L = (velL * 10 * Ts) # mm
	distancia_R = (velR * 10 * Ts) # mm
	maxSpeed = max(abs(velL), abs(velR))

	# Command generation and sending
	if velL == 0 and velR == 0:
		envia (ser,'SetMotor LWheelDisable RWheelDisable', 0.01,False)
		envia(ser,'SetMotor LWheelEnable RWheelEnable', 0.01,False)
	else:
		comando = 'SetMotor ' + str(distancia_L) + ' ' + str(distancia_R) + ' ' + str(maxSpeed)
		envia(ser,comando, 0.01,False)

### funcion angular angdiff
def angdiff(TH1, TH2):
	diff = TH1 - TH2
	diff = (diff + np.pi) % (2 * np.pi) - np.pi
	return diff

# Go to point
# Vector de error, distancia
def goToPoint(goal, currentPose, Kv, Kh, r, S, ts):
	currentPose_out = currentPose

	## 
	I_kine = lambda v_x, psi, r, S: [(1/r)*v_x + (1/r)*S*psi, (1/r)*v_x - (1/r)*S*psi]
	odo_vel = lambda phi_r, phi_l, r, S, ts: [(1/2)*r*(phi_r+phi_l)*ts, (r/(2*S))*(phi_r-phi_l)*ts]
	Pose_int = lambda X_ant, odo: [X_ant[0] + odo[0]*np.cos(X_ant[2]), X_ant[1] + odo[0]*np.sin(X_ant[2]), X_ant[2] + odo[1]]

	stop = False
	while not stop:
		print(currentPose)
		
		currentdiff = [goal[0] - currentPose[0], goal[1] - currentPose[1]]
		throttle = np.sqrt(currentdiff[0]**2 + currentdiff[1]**2)
		if throttle < 0.1:
			stop = True
			currentPose_out = currentPose
			return currentPose_out
		else:
			### VC
			velocity = throttle * Kv
			### THETA PUNTO
			steering = np.arctan2(currentdiff[1], currentdiff[0])
			psi = angdiff(steering, currentPose[2]) * Kh
			phi_l_r = I_kine(velocity, psi, r, S)
			deltas = odo_vel(phi_l_r[0], phi_l_r[1], r, S, ts)
			print(deltas)
			currentPose = Pose_int(currentPose, [deltas[0], deltas[1]])
		inverseKinematics(velocity, S, steering, ts)
	return currentPose_out

"""
funcion getch_nonBlock
Intenta leer de teclado.
Devuelve una lista con todas las teclas pulsadas, pendientes de ser leidas.
En la lista, encontrareis los valores numericos ASCII  de cada tecla. Ejemplo: si pulsais 'a', en la lista se anyadira un 97
Si no se ha pulsado ninguna tecla, la funcion retorna INMEDIATAMENTE con una LISTA VACIA.
"""
def getch_nonBlock():
	try:
		old_settings_in  = termios.tcgetattr(sys.stdin)
		old_settings_out = termios.tcgetattr(sys.stdout)
		new_settings_in = old_settings_in
		new_settings_in[3] = new_settings_in[3] & ~termios.ECHO & ~termios.ICANON # lflags
		new_settings_in[6][termios.VMIN] = 0  # cc array. NonCanonical behaviour values
		new_settings_in[6][termios.VTIME] = 0 # cc array. NonCanonical behaviour values
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings_in)
		
		ch_set = []
		ch = os.read(sys.stdin.fileno(), 1)
		while ch is not None and len(ch) > 0:
			ch_set.append(ord(ch[0]))
			ch = os.read(sys.stdin.fileno(), 1)

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings_in)
		termios.tcsetattr(sys.stdout, termios.TCSADRAIN, old_settings_out)

	return ch_set



# Llamada a la funcion main
if __name__ == '__main__':

	global ser
	# Open the Serial Port.
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

	envia(ser,'TestMode On', 0.2)
	envia(ser,'PlaySound 1', 0.2)
	envia(ser,'SetMotor LWheelEnable RWheelEnable', 0.2)

	# Declaracion de variables relacionadas con el robot, dimensiones, cinematica diferencial, etc.
	Ts=0.1  # cada iteracion del bucle de control se ejecutara cada Ts seg
	S = 121.5     # mm
	Vc = 0        # mm/s
	Tita_dot = 0  # rad/s
	
	#Condiciones iniciales
	poseX=0       # mm
	poseY=0       # mm
	poseTita=0    # rad

	# Inicializar variables
	

	startPose = [0, 0, 0]  # startPose [x y theta]
	puntos = [[3, 5]]  # Camino de puntos por donde pasara el robot
	goal = [0, -2]  # goal position

	Kv = 0.9  # Velocity Gain. Units (m/s)/m = [1/s]
	Kh = 4  # Head Gain. Units [1/s]
	r = 0.1  # wheels radius
	#S = 0.26  # half of the distance between the wheels' center
	ts = 0.1  # sample time
	
	"""
	#Inicialización de variables odometria del robot, para calculos posteriores de la evolución Pose
	msg = envia(ser, 'GetMotors LeftWheel RightWheel', 0.01, False).split('\n')
	odoL = int(msg[4].split(',')[1])
	odoR = int(msg[8].split(',')[1])
	"""

	startTime = time.time()
	currentTime = startTime


	print("")
	print("TIME = ",currentTime-startTime)


	### run gotopoint

	currentPose = poseTita

	points =  [3, 5]



	# Bucle para recorrer las coordenadas
	#for i in range(1, len(points)):
		# Obtener la coordenada goal y currentPose
		#goal = points(i)
		# Llamar a la función myFunction con los parámetros adecuados
		#currentPose = goToPoint(goal, currentPose, Kv, Kh, r, S, ts)

	
	# Control Loop. It executes every Ts seconds, useful for doing tasks that must be executed periodically (e.g. Forward Kinematics for Pose integration)
	while True:
		# Wait until the next Control Loop can be started
		nextTime = currentTime+Ts
		while (time.time()<nextTime):
			pass
		currentTime=currentTime+Ts

		print("")
		#print('TIME = ',currentTime-startTime)


		#CHECK KEYBOARD HIT
		#Usamos la tecla "q" para salir del programa en cualquier momento.
		#En este codigo de ejemplo, tambien se usan las teclas "wasd" para controlar el Neato
		#Usamos la tecla "z" para parar el Neato (ponemos sus velocidades a 0)
		"""
		key = getch_nonBlock()
		if key is not None and key:
			if key[-1] == ord('w'):
				Vc = Vc + 50
			if key[-1] == ord('s'):
				Vc = Vc - 50
			if key[-1] == ord('a'):
				Tita_dot = Tita_dot + (3.1415/10)
			if key[-1] == ord('d'):
				Tita_dot = Tita_dot - (3.1415/10)
			if key[-1] == ord('z'):
				Vc = 0
				Tita_dot = 0
			if key[-1] == ord('q'):
				break
		"""


		goal = points
		# Llamar a la función myFunction con los parámetros adecuados
		currentPose = goToPoint(goal, startPose, Kv, Kh, r, S, ts)



		#####################################################
		#####    RELLENAD AQUI CON VUESTRO CODIGO         ###
		##### NECESARIO PARA ODOMETRIA Y POSE INTEGRATION ###
		#####        (FORWARD KINEMATICS)        ############
		#####################################################

		#Obtencion Informacion de Sensores
		#e.g. en Neato: obtencion odometria y calculo de pose estimada

		# PODEIS USAR EL SIGUIENTE CODIGO PYTHON PARA HACER UNA LECTURA DEL VALOR DE ODOMETRIA QUE RETORNA NEATO
		# RECORDAD QUE EL VALOR DEVUELTO POR EL ROBOT ES EL VALOR EN MM DESDE EL ENCENDIDO DEL NEATO.
		# VOSOTROS NECESITAIS CALCULAR EL INCREMENTO DE MM SOLO DESDE LA ULTIMA LECTURA!!!
		msg = envia(ser, 'GetMotors LeftWheel RightWheel', 0.01, False).split('\n')
		odoL = int(msg[4].split(',')[1])
		odoR = int(msg[8].split(',')[1])
		#S = 12.15
		forwardMatrix = np.array([[0.5, 0.5], [1/2*S, -1/2*S]])

		
		#velocities = forwardMatrix @ [velR,velL]


		#### GO TO POINT



		#integrarlos, sumarles lo desplazado en x y thetha

		#Variables iniciales
		ProcNoiseD = 0.03**2
		ProcNoiseTheta=0.009**2
		Ts=0.02

		IC = [8.65, 17.2, -math.pi/2]
		V = [[ProcNoiseD, 0], [0, ProcNoiseTheta]]
		Pk0 = np.eye(3) * 0.0001
		Ls = 0
		Rs = 0
		t = 4




		
		elapsedTime=time.time()-currentTime
		print('% USAGE = ',(elapsedTime/Ts)*100)
		print("Estimated poseX = ",poseX)
		print("Estimated poseY = ",poseY)
		print("Estimated poseTita = ",poseTita)


	#Al salir del bucle, le enviamos al Neato la orden de parar. Por si acaso se estuviera moviendo aun.
	envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.01,False)
	envia(ser,'SetMotor LWheelEnable RWheelEnable', 0.01,False)

	print("Estimated poseX = ",poseX)
	print("Estimated poseY = ",poseY)
	print("Estimated poseTita = ",poseTita)
