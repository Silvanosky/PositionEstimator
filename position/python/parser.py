from graph import Graph

class Parser:
    
    def Parse(file='', save=''):
        if file == '' or save == '':
            raise Exception('No file provided')
        with open(file, 'r') as f:
            line = f.readline()
            graph = Graph(save)
            while line != '':
                position = line.split(';')
                graph.add_point(float(position[0]), float(position[1]), float(position[2]))
                line = f.readline()
            graph.save()
