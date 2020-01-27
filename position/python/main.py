from graph import Graph
from parser import Parser
import sys

Parser.Parse('test' + sys.argv[1] + '.p', 'test' + sys.argv[1]  + '.gr')
graph = Graph('test' + sys.argv[1] + '.gr')
graph.load()
graph.show()
