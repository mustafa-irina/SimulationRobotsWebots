"""RobotLightP controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Emitter
from controller import Receiver
from controller import DistanceSensor
from controller import LightSensor
from controller import Compass
from random import randint
import math
import struct
import socket
import sys
import threading
import time
from random import random

class Main:
    k = 0
    name = ""
    messageToAgent_cl = ''
    messageFromAgent_cl = ''
    #print(name + " I am created!!!")

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__stop = False
        self.connection = 0

    def stop(self):
        self.__stop = True

    def start(self):
        print('M.start')
        #self.name = name
        self.CreateClient()

    def reactToData(self, data):
        print("got data: " + repr(data))

    def threadVoid(self):
        #print(self.name+": Daemon created")
        while True:
            time.sleep(0.032)
            data = self.sock.recv(1024).decode('UTF-8')
            if data:
                self.messageFromAgent_cl = data
                #self.reactToData(data)
            else:
                print(sys.stderr, 'no more data from client')
                break

    def CreateClient(self):
        #k = 0
        #print(self.name+": Connecting to java part")
        new_name = self.name.replace('RL', '')
        #print("My name IS: "+new_name)
        self.k = int(new_name)
        server_address = ('localhost', (3330 + self.k))
        self.sock.connect(server_address)
        try:
            thread = threading.Thread(target=self.threadVoid, daemon=True)
            thread.start()
            while True:
                time.sleep(0.032)
                r = self.messageToAgent_cl
                #print(r + 'send msg')
                if r != '':
                    #print('sending message')
                    self.sock.send(r.encode('UTF-8'))
                    

        finally:
            #pass
            self.sock.close()

    def reactToMessage(self, msg):
        print(msg)

if __name__ == '__main__':
    print('shiiiiit started')
    # create the Robot instance.
    robot = Robot()
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    # initialize motors
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
    for i in range(4):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)

    # initialize emmiters
    emm = robot.getDevice('trans')

    #Вводим количество членов группы не включая самого робота (т.е. n-1)
    k = 24

    #initialize receiver
    rec = []
    recNames = ['rec1', 'rec2', 'rec3', 'rec4', 'rec5', 'rec6', 'rec7', 'rec8', 'rec9', 'rec10', 'rec11', 'rec12', 'rec13', 'rec14', 'rec15', 'rec16', 'rec17', 'rec18', 'rec19', 'rec20', 'rec21', 'rec22', 'rec23', 'rec24']
    for i in range(k):
        rec.append(robot.getDevice(recNames[i]))
        rec[i].enable(timestep)

    # initialize distance sensor
    ds = robot.getDevice('ds')
    ds.enable(timestep)

    # initialize motors
    ls = []
    lsNames = ['ls1', 'ls2', 'ls3', 'ls4']
    for i in range(4):
        ls.append(robot.getDevice(lsNames[i]))
        ls[i].enable(timestep)
    # initialize distance sensor
    com = robot.getDevice('com')
    com.enable(timestep)

    #Начальное значение на двигатели
    leftSpeed = 0
    rightSpeed = 0
    # Переменные для задания обхода препятствия
    avoidObstacleCounter = 0
    counter = 0
    j = 0
    p = 0
    M = Main()
    #print(robot.getName())
    M.name = robot.getName()
    thread = threading.Thread(target=M.start, daemon=True)
    thread.start()
    #print('im here!')
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        #print('im here!2')
        #print (robot.getName())
        #Ищем максимум из датчиков
        light = []
        for i in range(4):
            light.append(ls[i].getValue()) #здесь готовим инфу для агента
            #print (str(light[i]) + 'light_i')
        messageToAgent = ""
        #print(messageToAgent + 'msg to agent0')
        for i in range(4):
            #print('loop1')
            messageToAgent += str(light[i]) + ';'
        messageToAgent += str(ds.getValue()) + ';'
        messageToAgent += str(counter) + ';'
        messageToAgent += str(avoidObstacleCounter) + ';'
        messageToAgent += str(p) + '\n'
        print(messageToAgent + 'msg to agent')
        M.messageToAgent_cl = messageToAgent

        messageFromAgent = M.messageFromAgent_cl
        print('msg from agent: '+messageFromAgent)
        data = []
        if messageFromAgent != '':
            data = messageFromAgent.split(';')
            leftSpeed = float(data[0])
            rightSpeed = float(data[1])
            counter = float(data[2])
            avoidObstacleCounter = float(data[3])
            p = float(data[4])
            if counter != 0:
                leftSpeed = -2
                rightSpeed = 2
                counter -= 1
            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(rightSpeed)
            wheels[2].setVelocity(leftSpeed)
            wheels[3].setVelocity(rightSpeed)


        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()

        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)


    # Enter here exit cleanup code.
