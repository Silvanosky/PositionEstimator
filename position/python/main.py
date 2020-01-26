from graph import Graph
from parser import Parser

Parser.Parse('test6.p', 'test6.gr')
graph = Graph('test6.gr')
graph.load()
graph.show()
