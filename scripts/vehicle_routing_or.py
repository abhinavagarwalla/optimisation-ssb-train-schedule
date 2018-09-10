from parse_input_route_graph import *
import math
import sys

#Input File
INPUT_MODEL = "sample_files/sample_scenario.json"
ROUTE_GRAPH_FOLDER = "route_graphs/*.graphml"

class BasicTrainProblem:
    def __init__(self):
        self._trains = ServiceIntention(INPUT_MODEL)
        self.num_trains = 1    #self._trains.get_number_service_intentions()

        self.routes = RouteGraph(INPUT_MODEL, ROUTE_GRAPH_FOLDER)

        # self.routes_graph = nx.convert_node_labels_to_integers(self.routes.get_global_graph())
        print(self.routes.get_global_graph(), self.routes.get_service_intention_graph('111'))
        self.routes_graph = nx.convert_node_labels_to_integers(self.routes.get_service_intention_graph('111'))

        self.locations = self.routes_graph.nodes()
        self.num_locations = len(self.locations)
        # print(self.locations)

        self.original_labels = list(zip(self.locations, self.routes.get_service_intention_graph('111').nodes()))
        print(self.original_labels)
        self.time_as_distance = nx.floyd_warshall(self.routes_graph)

        for sx in range(self.num_locations):
            for ey in range(self.num_locations):
                if math.isinf(self.time_as_distance[sx][ey]):
                    self.time_as_distance[sx][ey] = 100000000#sys.maxsize

        self.start_depot = 0  #'(1_beginning)'
        self.end_depot = 12
        self._time_windows = None
    
    def get_time_as_distance(self, start_node, end_node):
        return self.time_as_distance[start_node][end_node]

# a = BasicTrainProblem()
# for i in range(12):
#     for j in range(12):
#         print("From {} to {}: {}\n".format(a.original_labels[i], a.original_labels[j], a.get_time_as_distance(i, j)))