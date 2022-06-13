#!/usr/bin/env python
# Reto Ros
# Equipo 3
# Programa de control PD para la navegación y de evasión de obstáculos
# 13 de junio de 2022

#Librerias que contienen funciones para el programa
from cmath import pi
from math import atan2, cos, sin, sqrt
from turtle import pu
import rospy, time
#Librerias de mensajes de ROS
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String, Int32, Int32MultiArray


#Creamos una clase para el puzzlebot
class Puzzlebot:
    #Funcion para inicializar todas las variables de la clase
    def __init__(self):
        self.r = 0.05
        self.l = 0.19
        self.thetha = 0
        self.x = 0
        self.y = 0
        self.wl = 0
        self.wr = 0
        self.dt = 0.01
	# Coordenadas de los Waypoints
        self.xt = [1, 2, 3, 4] 
        self.yt = [0, 0, 0, 0]
	self.obstx = 0
	self.obsty = 0
        self.goal = 0
        self.ed = 0
        self.et = 0
        self.vw = Twist()
        self.vw.linear.x=0
        self.vw.linear.y=0
        self.vw.linear.z=0
        self.vw.angular.x=0
        self.vw.angular.y=0
        self.vw.angular.z=0
	# Constantes Kp, Kd
        self.kv = 0.12
        self.kw = 0.1
	# Color identificado
	self.semafo = "X"

    #Funcion de callback cuando el topico /wr mande un mensaje    
    def whr(self, msg):
        pubr.publish(msg.data)
        self.wr = msg.data

    #Funcion de callback cuando el topico /wl mande un mensaje
    def whl(self, msg):
        publ.publish(msg.data)
        self.wl = msg.data

    #Funcion para calcular la posicion actual del robot
    def position(self):
        self.thetha = self.thetha + self.r*((self.wr-self.wl)/self.l)*self.dt
        #Mantenemos el valor de theta en un intervalo de -pi a pi
        if(self.thetha > pi):
            self.thetha = self.thetha-pi
        if(self.thetha < -pi):
            self.thetha = self.thetha+pi
        #Publicamos la posicion actual
        self.x = self.x + self.r*((self.wr+self.wl)/2)*self.dt*cos(self.thetha)
        self.y = self.y + self.r*((self.wr+self.wl)/2)*self.dt*sin(self.thetha)
        pubp.publish(self.x,self.y,self.thetha)
        #print("Posicion actualizada")
	print(self.x, self.y, self.thetha)

    #Funcion para calcular el error 
    def error(self):
        if(self.thetha >= 0):
            self.et=atan2(self.yt[self.goal],self.xt[self.goal])-self.thetha
        elif (self.thetha < 0) :
            self.et=-(atan2(self.yt[self.goal],self.xt[self.goal])+self.thetha)
        self.ed = sqrt(pow(self.xt[self.goal]-self.x,2)+pow(self.yt[self.goal]-self.y,2))
        #Si el error se hace muy pequeno lo volvemos 0

        #Publicamos el error
        pubed.publish(self.et)
        pubet.publish(self.ed)
        self.calculaVel()

    #Funcion para calcular la velocidad
    def calculaVel(self):
	self.vw.linear.x = self.ed*self.kv
	self.vw.angular.z = self.et*self.kw
	
    #Funcion para publicar la velocidad
    def pubVel(self):
        pubvel.publish(self.vw)

    #Funcion de callback para el topic del semaforo
    def semaforo(self, msg):
        if(msg.data == "Rojo"):          
	    self.semafo="Rojo"
        elif(msg.data == "Verde"):
	    self.semafo="Verde"
        else:
            self.semafo="X"
	
    # Función principal	
    def move(self):
	# Identificar si vio el objeto
	entreR = False
	entreV = False
	accessR = False
	accessV = False
	# Posición antes de rodear
	posV = 0
	posR = 0
        base = rospy.get_time()
        now=rospy.get_time()
	desp=sqrt(pow(self.xt[self.goal],2)+pow(self.yt[self.goal],2))
        while not rospy.is_shutdown():
	    now=rospy.get_time()
	    self.dt=now-base
	    if self.dt >= 0.01:
		print("Color:", self.semafo)
	        print("Waypoint:",self.goal)
	        self.position()
	        self.error() #Los argumentos son la posicion deseada ej. (0.8,0) (esta en metros)
         	self.calculaVel()

		# Si la cámara no detecta obstáculos
		if self.semafo == "X" and self.et > 0.03 or self.et < -0.03:
		    self.vw.linear.x=0
	            print("Rotando")
		if self.semafo == "X" and self.ed < desp*.05:
		    self.vw.linear.x=0
		    self.vw.angular.z=0 
		    print("Llegue") 
		    # Si ya llegó a todos los waypoints
		    if(self.goal == len(self.xt)-1):
			print(self.goal+1)
			print("Fin")
			self.x = 0
			self.y = 0
			self.thetha = 0 
			self.pubVel()
			break
		    # Ir al siguiente waypoint
		    else:
		        self.x = self.xt[self.goal]
		        self.y = self.yt[self.goal]
		        self.goal = self.goal +1
		        print("GOAL: ", self.goal)   

		# Si detecta el obstáculo rojo
		if entreR == False and self.semafo == "Rojo":
		    posR = self.x # Obtener la posición
		    accessR = True
		    accessV = False
                    entreR = True 
		# Esquivar obstáculo 
		if self.semafo == "Rojo" or self.x < (posR + 0.10) and posR != 0:
                    self.vw.linear.x =0.1
                    self.vw.angular.z =0.4 # Giro izquierda
                    print("Vuelta rojo")

		# Si detecta el obstáculo verde
		if entreV == False and self.semafo == "Verde":
		    posV = self.x # Ultima posición
                    entreV = True 
		    accessV = True
		    accessR = False
		# Esquivar obstáculo
		if self.semafo == "Verde" or self.x < (posV + 0.10) and posV != 0:
                    self.vw.linear.x =0.1
                    self.vw.angular.z =-0.3 # Giro derecha
		    print("Vuelta verde")

		# Imprimir el obstáculo que esta esquivando
		print("AccessV", accessV)
		print("AccessR", accessR)
		
		# Para regresar al eje y=0 una vez que deje de ver el color del obstaculo
	        if self.y > 0.02 or self.y < -0.02 and self.semafo == "X" and (accessR == True or accessV == True):
		    if(accessR == True):		    
                    	self.vw.angular.z=-0.05 # Giro contrario para acomodarse
                    	print("Regresando del rojo a la linea")	
		    elif (accessV == True):
	            	self.vw.angular.z=0.05 # Giro contrario para acomodarse
	            	print("Regresando del verde a la linea")   
		    else:
			print("Mal regresado a la linea")        
		    self.vw.linear.x=0.05

		self.pubVel()
		base=rospy.get_time()
	
	 
    
if __name__ == '__main__':
    try:
        #Iniciamos el puzzlebot junto con todos los topicos que necesitamos
        puzzlebot = Puzzlebot()
        rospy.init_node("control")
        pubed = rospy.Publisher("errort",Float32, queue_size=10) #Topic del error del angulo
        pubet = rospy.Publisher("errord", Float32, queue_size=10) #Topic del error de la distancia
        pubvel = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic de cmd_vel para publicar las velocidades
	pubr = rospy.Publisher("vel_wr", Float32, queue_size=10) #Opcional
        publ = rospy.Publisher("vel_wl", Float32, queue_size=10) #Opcional
	pubp = rospy.Publisher("pos", Vector3, queue_size=10) #Topic de la posicion
	rospy.Subscriber("wr", Float32, puzzlebot.whr)#Topic de wr para recibir la velocidad derecha
        rospy.Subscriber("wl", Float32, puzzlebot.whl)#Topic de wr para recibir la velocidad izquierda
        rospy.Subscriber("Mensaje_semaforo", String, puzzlebot.semaforo)#Topic del semaforo para recibir el color 

	puzzlebot.move()

    except rospy.ROSInterruptException:
        pass
