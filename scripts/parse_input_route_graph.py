import json
import networkx as nx
import glob
import re

#Input File
INPUT_MODEL = "sample_files/sample_scenario.json"
ROUTE_GRAPH_FOLDER = "route_graphs/*.graphml"

class RouteGraph:
    def __init__(self, INPUT_MODEL, ROUTE_GRAPH_FOLDER):
        self.input_model = json.loads(open(INPUT_MODEL).read())
        self.route_graph_model = glob.glob(ROUTE_GRAPH_FOLDER)
        self.service_intention_graphs = dict()
        self.global_graph = None
        self.generate_service_intention_graph()
        self.generate_global_graph()

    def generate_service_intention_graph(self):
        for graph in self.route_graph_model:
            sid = graph.split('.')[0].split('-')[-1]
            self.service_intention_graphs[sid] = nx.read_graphml(graph)
            # print(self.service_intention_graphs[sid], self.service_intention_graphs[sid].edges)
    
    def generate_global_graph(self):
        self.global_graph = nx.compose_all(self.service_intention_graphs.values())
        # print(self.global_graph, self.global_graph.edges)

    def get_service_intention_graph(self, id):
        if id in self.service_intention_graphs.keys():
            return self.service_intention_graphs[id]
        raise Exception("Id not found")
    
    def get_global_graph(self):
        return self.global_graph
    
    def get_travelling_time(self, start_node, end_node):
        if (start_node, end_node) in self.global_graph.edges:
            return self.global_graph.edges[(start_node, end_node)]["running_time"]
        else:
            return -1
        return None
        
# a = RouteGraph(INPUT_MODEL, ROUTE_GRAPH_FOLDER)
# # gp = a.get_service_intention_graph('113')
# gp = a.get_global_graph()

# # for start_node, end_node, data in gp.edges(data=True):
# #     print(data["running_time"], data["resource"])

# print("getting travelling itme")
# print(a.get_travelling_time('(M2)', '(M1)'))

class Resource:
    def __init__(self, INPUT_MODEL):
        self.input_model = json.loads(open(INPUT_MODEL).read())
        self.release_time_dict = {}
        self.parse_release_times()
    
    def parse_release_times(self):
        res = self.input_model['resources']
        for resiter in res:
            self.release_time_dict[resiter['id']] = resiter['release_time']
    
    def get_release_time(self, resource_id):
        if resource_id in self.release_time_dict.keys():
            return self.release_time_dict[resource_id]
        raise Exception("Id not found")

# a = Resource(INPUT_MODEL)
# print(a.get_release_time('A1'))

class ServiceIntention:
    def __init__(self, INPUT_MODEL):
        self.input_model = json.loads(open(INPUT_MODEL).read())
        self.section_requirements_dict = {}
        self.parse_section_requirements()
    
    def parse_section_requirements(self):
        res = self.input_model["service_intentions"]
        for service_iter in range(len(res)):
            for resiter in res[service_iter]["section_requirements"]:
                self.section_requirements_dict[(res[service_iter]['id'], resiter["section_marker"])] = {}
                for k, v in resiter.items():
                    if k == "section_marker":
                        continue
                    self.section_requirements_dict[(res[service_iter]['id'], resiter["section_marker"])][k] = v
        print(self.section_requirements_dict)
    
    def get_section_requirements(self, service_id, section_marker):
        if (service_id, section_marker) in self.section_requirements_dict.keys():
            return self.section_requirements_dict[(service_id, section_marker)]
        raise Exception("Id not found")

a = ServiceIntention(INPUT_MODEL)
print(a.get_section_requirements(111, 'A'))