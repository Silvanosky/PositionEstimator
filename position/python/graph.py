import pylab as pl
import json

class Graph:

    def __init__(self, file='tmp.gr'):
        self.file = file
        self.x = []
        self.y = []

    def print(self):
        x = pl.array(self.x)
        y = pl.array(self.y)
        pl.plot(x, y)
        pl.show()

    def add_point(self, x, y):
        self.x.append(x)
        self.y.append(y)

    def add_points(self, x, y):
        self.x += x
        self.y += y

    def save(self):
        data = json.dumps(dict(zip(self.x, self.y)), indent=4)
        with open(self.file, 'w') as file:
            file.write(data)

    def load(self):
        with open(self.file, 'r') as file:
            data = json.loads(file.read())
        self.x = list(data.keys())
        self.y = list(data.values())
