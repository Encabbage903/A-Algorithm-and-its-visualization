# _*_ coding utf-8 _*_
# wiritter：ENcabbage903
# this code is to find the shortest route in a binary map based on A* algorithm

import numpy as np
import random
import sys
import time
import math
from PyQt5.QtWidgets import QWidget,QLabel,QApplication,QMessageBox
from PyQt5.QtGui import QPainter,QColor
from PyQt5 import QtCore

# Create a binary map with random BFS to make sure every point except obstacles is reachable
# size means the size of the map
# p_ob means the percentage of obstacles
def Create_Map(Size,P_ob):
    MM = Size[0]
    NN = Size[1]
    Map = np.ones([MM,NN])
    Map[:,0] = 0
    X = [0,0]
    V = np.array([[0, 1],[1, 0],[0, -1],[-1, 0]])
    Num_Path = NN
    Open = []
    while Num_Path < MM * NN * (1 - P_ob):
        Add = []
        for v in V:
            x = list(X + v)
            if x[0] in range(MM) and x[1] in range(NN) and Map[x[0],x[1]] == 1:
                Open.append(x)
                Add.append(x)
        if Add:
            next = int(random.random()*len(Add))
            X = Add.pop(next)
            Map[X[0],X[1]] = 0
            Num_Path += 1
        else:
            next = int(random.random()*len(Open))
            X = Open.pop(next)
            if Map[X[0],X[1]] == 1:
                Map[X[0],X[1]] = 0
                Num_Path += 1
    return Map

def Astar(Map,Start_Point,End_Point ,H_Type):
    class Node:
        def __init__(self,pos):
            self.Pos = pos
            self.G = 0
            self.H = 0
            self.F = 0
            self.Parent = []

    def Cal_H(x ,y ,type):
        h = 0.0
        if type == 'Manhattan':
            h = sum(abs(x - np.array(y)))
        elif type == 'Chess':
            h = max(abs(x - np.array(y)))
        elif type == 'Geometry':
            h = math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)
        return h

    V = np.array([[0,1],[0,-1],[1,0],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]])
    MM = np.shape(Map)[0]
    NN = np.shape(Map)[1]
    X = Node(Start_Point)
    Route = []
    X.G = 0
    X.H = abs(End_Point[0]- Start_Point[0])+ abs(End_Point[1] - Start_Point[1])
    X.F = X.G + Cal_H(Start_Point ,End_Point ,H_Type)
    Open = []
    Close = [X.Pos]
    Open_Pos = []
    while X.Pos != End_Point:
        for v in V:
            x = list(X.Pos + v)
            if x[0] in range(MM) and x[1] in range(NN) and Map[x[0],x[1]]!=1 and x not in Close:
                G = 0.0
                if type == 'Manhattan':
                    G = sum(abs(v))
                elif type == 'Chess':
                    G = max(abs(v))
                elif type == 'Geometry':
                    G = math.sqrt(v[0]** 2 + v[1]** 2)
                if x not in Open_Pos:
                    node = Node(x)
                    node.G = G + X.G
                    node.H = Cal_H(x ,End_Point ,H_Type)
                    node.F = node.G + node.H
                    node.Parent = X
                    Open.append(node)
                    Open_Pos.append(x)
                else:
                    for i in range(len(Open)):
                        if Open[i].Pos == x and Open[i].G > X.G + 1.0:
                            Open[i].G = X.G + G
                            Open[i].Parent = X
        for i in range(len(Open)):
            get = Open[i]
            j  = i - 1
            while j >= 0 and Open[j].F > get.F:
                Open[j+1] = Open[j]
                j -= 1
            Open[j+1] = get
        X = Open.pop(0)
        Open_Pos.remove(X.Pos)
        Close.append(X.Pos)
    while X.Parent:
        Route.insert(0,X.Pos)
        X = X.Parent
    return Route

class Window(QWidget):
    def __init__(self,Map,Route):
        super().__init__()
        self.mm = np.shape(Map)[0]
        self.nn = np.shape(Map)[1]
        self.map = Map
        self.route = Route
        self.num_step = len(self.route)
        self.pos = Route[0]
        self.edge = 600
        self.p_ob = 100*len(np.where(Map==1)[0]) / (self.mm*self.nn)
        self.setWindowTitle('A* Algorithm')
        self.initUI()

    def initUI(self):
        self.setGeometry(100,100,self.edge,self.edge)
        self.show()

    def paintEvent(self,e):
        qp = QPainter()
        qp.begin(self)
        delta = int(self.edge / self.mm)
        for i in range(self.mm):
            for j in range(self.nn):
                if self.map[i,j] == 0 or self.map[i,j] == 4:
                    color = QColor(255,255,255)
                elif self.map[i,j] == 1:
                    color = QColor(0, 0, 0)
                elif self.map[i,j] == 2:
                    color = QColor(0,255,0)
                elif self.map[i,j] == 3:
                    color = QColor(255, 0, 0)
                qp.setBrush(color)
                qp.drawRect(delta*i,delta*j,delta,delta)
                if self.map[i,j] == 4:
                    color = QColor(0, 255, 0)
                    qp.setBrush(color)
                    qp.drawRect(int(delta * (i+0.1)),int(delta * (j+0.1)),int(delta*0.8),int(delta*0.8))
        qp.end()

    def mousePressEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            self.map[0,0] = 0
            while self.route:
                time.sleep(0.12)
                self.map[self.pos[0],self.pos[1]] = 0
                self.pos = self.route.pop(0)
                self.map[self.pos[0], self.pos[1]] = 4
                self.repaint()

            QMessageBox.about(self, 'info', 'the number of movements：' + str(self.num_step))

if __name__ == '__main__':
    H_type = 'Manhattan' #'Manhattan' ,'Chess' ,'Geometry'

    Map = Create_Map([20,20],0.25)
    index = np.where(Map == 0)
    Start_Point = [0, 0]
    End_Point = [0 ,0]
    # choose an end_point randomly
    while End_Point == Start_Point:
        e = int(random.random() * len(index[0]))
        End_Point = [index[0][e],index[1][e]]
    Map[Start_Point[0],Start_Point[1]] = 2
    Map[End_Point[0],End_Point[1]] = 3
    route = Astar(Map, Start_Point,End_Point ,H_type)

    print('click mouse to start!')

    app = QApplication(sys.argv)
    w = Window(Map,route)
    sys.exit(app.exec_())

