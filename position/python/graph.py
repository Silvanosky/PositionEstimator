import json
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d

class Graph:

    def __init__(self, file='tmp.gr'):
        self.file = file
        self.num = 0
        self.id = []
        self.x = []
        self.y = []
        self.z = []

    def show(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(self.x, self.y, self.z, 'black')
        plt.show()

    def add_point(self, x, y, z):
        self.id.append(self.num)
        self.num += 1
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def add_points(self, x, y, z):
        x_length = len(x)
        if x_length != len(y) and x_length != len(z):
            raise Exception('Error length')
        for i in range(x_length):
            self.id.append(self.num)
            self.num += 1
        self.x += x
        self.y += y
        self.z += z

    def save(self):
        data = json.dumps(dict(zip(self.id, zip(self.x, self.y, self.z))), indent=4)
        with open(self.file, 'w') as file:
            file.write(data)

    def load(self):
        with open(self.file, 'r') as file:
            data = json.loads(file.read())
        self.id = list(data.keys())
        for i in self.id:
            self.x.append(data[i][0])
            self.y.append(data[i][1])
            self.z.append(data[i][2])
