import json
import networkx as nx
import glob
import re
import math
import sys
import itertools
import ast
import isodate

#Input File
# INPUT_MODEL = "sample_files/sample_scenario.json"
# ROUTE_GRAPH_FOLDER = "route_graphs/*.graphml"

class RouteGraph:
    def __init__(self, INPUT_MODEL, ROUTE_GRAPH_FOLDER):
        self.input_model = json.loads(open(INPUT_MODEL).read())
        self.route_graph_model = glob.glob(ROUTE_GRAPH_FOLDER)
        self.service_intention_graphs = dict()
        self.global_graph = None
        self.generate_service_intention_graph()
        self.generate_global_graph()

        self._number_of_sections = {}
        self._number_of_service_intention_graphs = None
        self._incoming_outgoing_edges = {}
        self._section_travelling_times = {}
        self.populate_parameters()

    def generate_service_intention_graph(self):
        for graph in self.route_graph_model:
            sid = graph.split('.')[0].split('-')[-1]
            print("Reading graph for service-", sid)
            self.service_intention_graphs[sid] = nx.read_graphml(graph)
            # print(self.service_intention_graphs[sid], self.service_intention_graphs[sid].edges)
    
    def generate_global_graph(self):
        self.global_graph = nx.compose_all(self.service_intention_graphs.values())

    def get_service_intention_graph(self, id):
        if id in self.service_intention_graphs.keys():
            return self.service_intention_graphs[id]
        raise Exception("Id not found")
    
    def get_global_graph(self):
        return self.global_graph
    
    def get_travelling_time(self, start_node, end_node):
        if (start_node, end_node) in self.global_graph.edges():
            return self.global_graph.edges[(start_node, end_node)]["running_time"]
        else:
            return -1
        return None
    
    def edge_to_track_wise_sequence_number(self, service_intention, start_node, end_node):
        if service_intention in self.service_intention_graphs.keys():
            if (start_node, end_node) in self.service_intention_graphs[service_intention].edges():
                return self.service_intention_graphs[service_intention].edges[(start_node, end_node)]["track_wise_sequence_number"]
        return None
    
    def section_marker_to_track_wise_sequence_number(self, service_intention, section_marker):
        edge_list = []
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'section_marker' in data.keys() and data["section_marker"] == section_marker:
                    edge_list.append(self.service_intention_graphs[service_intention].edges[(start_node, end_node)]["track_wise_sequence_number"])
            return edge_list
        return None

    def section_marker_to_edge(self, service_intention, section_marker):
        edge_list = []
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'section_marker' in data.keys() and data["section_marker"] == section_marker:
                    edge_list.append((start_node, end_node))
            return edge_list
        return None
    
    def track_wise_sequence_number_to_section_marker(self, service_intention, sequence_number):
        sequence_number = int(sequence_number)
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'track_wise_sequence_number' in data.keys() and data['track_wise_sequence_number'] == sequence_number:
                    if 'section_marker' in data.keys():
                        return data['section_marker']
                    else:
                        return None
        return None
    
    def track_wise_sequence_number_to_route_path_id(self, service_intention, sequence_number):
        sequence_number = int(sequence_number)
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'track_wise_sequence_number' in data.keys() and data['track_wise_sequence_number'] == sequence_number:
                    if 'route_path_id' in data.keys():
                        return data['route_path_id']
                    else:
                        return None
        return None
    
    def track_wise_sequence_number_to_edge(self, service_intention, sequence_number):
        sequence_number = int(sequence_number)
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'track_wise_sequence_number' in data.keys() and data['track_wise_sequence_number'] == sequence_number:
                    return (start_node, end_node)
        return None
    
    def track_wise_sequence_number_to_sequence_number(self, service_intention, track_wise_sequence_number):
        track_wise_sequence_number = int(track_wise_sequence_number)
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'track_wise_sequence_number' in data.keys() and data['track_wise_sequence_number'] == track_wise_sequence_number:
                    if 'sequence_number' in data.keys():
                        return data['sequence_number']
                    else:
                        return None
        return None
    
    def get_sections_with_resource(self, service_intention):
        edge_list = {}
        if service_intention in self.service_intention_graphs.keys():
            for start_node, end_node, data in self.service_intention_graphs[service_intention].edges(data = True):
                if 'resource' in data.keys():
                    resource_data = ast.literal_eval(data['resource'])
                    for r in resource_data:
                        if r['resource'] not in edge_list.keys():
                            edge_list[r['resource']] = []
                        edge_list[r['resource']].append((start_node, end_node))
            return edge_list
        return None

    def populate_parameters(self):
        self._number_of_service_intention_graphs = len(self.service_intention_graphs)

        self._number_of_sections = {k:v.number_of_edges() for k, v in self.service_intention_graphs.items()}

        for k, v in self.service_intention_graphs.items():
            edge_list = []
            for node in v.nodes:
                in_edges = list(v.in_edges(node))
                if not len(in_edges):
                    continue
                out_edges = list(v.out_edges(node))
                if not len(out_edges):
                    continue
                # edge_unions = list(itertools.product(in_edges, out_edges))
                edge_list.append((in_edges, out_edges))
            self._incoming_outgoing_edges[k] = edge_list
        
        for k, v in self.service_intention_graphs.items():
            self._section_travelling_times[k] = {}
            for start_node, end_node, data in v.edges(data = True):
                self._section_travelling_times[k][(start_node, end_node)] = data["weight"]

# a = RouteGraph(INPUT_MODEL, ROUTE_GRAPH_FOLDER)
# print(a.)
# gp = a.get_service_intention_graph('113')
# gp = a.get_global_graph()

# for start_node, end_node, data in gp.edges(data=True):
#     print(data["running_time"], data["resource"])

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
            self.release_time_dict[resiter['id']] = int(isodate.parse_duration(resiter['release_time']).total_seconds())
    
    def get_release_time(self, resource_id):
        if resource_id in self.release_time_dict.keys():
            return self.release_time_dict[resource_id]
        raise Exception("Id not found")

# a = Resource(INPUT_MODEL)
# print(a.get_release_time('A1'))

class ServiceIntention:
    def __init__(self, INPUT_MODEL):
        self.input_model = json.loads(open(INPUT_MODEL).read())
        self.number_service_intentions = 0
        self._latest_requirements = {}
        self._earliest_requirements = {}
        self.section_requirements_dict = {}
        self.parse_section_requirements()
    
    def parse_section_requirements(self):
        res = self.input_model["service_intentions"]
        for service_iter in range(len(res)):
            self.number_service_intentions += 1
            for resiter in res[service_iter]["section_requirements"]:
                if str(res[service_iter]['id']) not in self.section_requirements_dict.keys():
                    self.section_requirements_dict[str(res[service_iter]['id'])] = []
                self.section_requirements_dict[str(res[service_iter]['id'])].append(resiter)
        # print(self.section_requirements_dict)
    
    def get_number_service_intentions(self):
        return self.number_service_intentions
    
    def get_section_requirements(self, service_id):
        if service_id in self.section_requirements_dict.keys():
            return self.section_requirements_dict[service_id]
        raise Exception("Id not found")
    
    def to_seconds(self, time_str):
        hr, minu, sec = map(int, time_str.split(':'))
        return hr*60*60 + minu*60 + sec
    
    # [sections], entry_latest, weight_entry, exit_latest, weight_exit
    def get_latest_requirements(self):
        for key, value in self.section_requirements_dict.items():
            self._latest_requirements[key] = []
            for v in value:
                self._latest_requirements[key].append((v['section_marker'],
                    None if "entry_latest" not in v.keys() else self.to_seconds(v['entry_latest']),
                    None if "entry_latest" not in v.keys() else v['entry_delay_weight'],
                    None if "exit_latest" not in v.keys() else self.to_seconds(v['exit_latest']),
                    None if "exit_latest" not in v.keys() else v['exit_delay_weight']))

        return self._latest_requirements
    
    # [sections], entry_earliest, exit_earliest
    def get_earliest_requirements(self):
        for key, value in self.section_requirements_dict.items():
            self._earliest_requirements[key] = []
            for v in value:
                self._earliest_requirements[key].append((v['section_marker'],
                    None if "entry_earliest" not in v.keys() else self.to_seconds(v['entry_earliest']),
                    None if "exit_earliest" not in v.keys() else self.to_seconds(v['exit_earliest'])))

        return self._earliest_requirements

# a = ServiceIntention(INPUT_MODEL)
# print(a.get_latest_requirements())
# print(a.get_earliest_requirements())

class TrainProblemConstrained:
    def __init__(self, input_model, route_graph_folder):
        self._trains = ServiceIntention(input_model)
        self._routes = RouteGraph(input_model, route_graph_folder)
        self._resource = Resource(input_model)

        self.num_trains = self._trains.get_number_service_intentions()
        self.num_sections = self._routes._number_of_sections

    def latest_requirements(self):
        ##Convert section marker to edge names here
        return self._trains.get_latest_requirements()
    
    def earliest_requirements(self):
        ##Convert section marker to edge names here
        return self._trains.get_earliest_requirements()

    def section_unions(self):
        return self._routes._incoming_outgoing_edges

    def minimum_running_times(self):
        #Add waiting time for sections here
        run_times = self._routes._section_travelling_times
        waiting_times = {}
        for k,v in self._trains.section_requirements_dict.items():
            waiting_times[k] = {}
            for sec in v:
                if 'min_stopping_time' in sec:
                    edge = self._routes.section_marker_to_edge(k, sec['section_marker'])
                    if edge is not None:
                        for e in edge:
                            waiting_times[k][e] = int(isodate.parse_duration(sec['min_stopping_time']).total_seconds())

        for service, times in run_times.items():
            if service in waiting_times.keys():
                for edge in waiting_times[service].keys():
                    if edge in times.keys():
                        run_times[service][edge] += waiting_times[service][edge]
        return run_times

    def edge_to_track_wise_sequence_number(self, service_intention, start_node, end_node):
        return self._routes.edge_to_track_wise_sequence_number(service_intention, start_node, end_node)
    
    def section_marker_to_track_wise_sequence_number(self, service_intention, section_marker):
        return self._routes.section_marker_to_track_wise_sequence_number(service_intention, section_marker)

    def section_marker_to_edge(self, service_intention, section_marker):
        return self._routes.section_marker_to_edge(service_intention, section_marker)

    def track_wise_sequence_number_to_section_marker(self, service_intention, sequence_number):
        return self._routes.track_wise_sequence_number_to_section_marker(service_intention, sequence_number)

    def track_wise_sequence_number_to_edge(self, service_intention, sequence_number):
        return self._routes.track_wise_sequence_number_to_edge(service_intention, sequence_number)

    def track_wise_sequence_number_to_sequence_number(self, service_intention, track_wise_sequence_number):
        return self._routes.track_wise_sequence_number_to_sequence_number(service_intention, track_wise_sequence_number)
   
    def track_wise_sequence_number_to_route_path_id(self, service_intention, track_wise_sequence_number):
        return self._routes.track_wise_sequence_number_to_route_path_id(service_intention, track_wise_sequence_number)

    def if_section_marker_referenced_in_section_requirements(self, service_intention, track_wise_sequence_number):
        section_marker = self.track_wise_sequence_number_to_section_marker(service_intention, track_wise_sequence_number)
        req = self._trains.get_section_requirements(service_intention)
        secmark_list = [i['section_marker'] for i in req]
        if section_marker in secmark_list:
            return True
        return False

    def to_actual_time(self, time):
        time = int(time)
        hr = int(math.floor(time/3600))
        minu  = int((time%3600)/60)
        sec = int((time%3600)%60)
        res =  "{:02d}:{:02d}:{:02d}".format(hr, minu, sec)
        return res

    def get_sections_with_resource(self, service_intention):
        res = self._routes.get_sections_with_resource(service_intention)
        return res

    def get_common_resources(self, resouce_dict_1, resource_dict_2):
        common = resouce_dict_1.keys() & resource_dict_2.keys()
        return common

    def get_release_time(self, resource):
        if resource in self._resource.release_time_dict.keys():
            return self._resource.release_time_dict[resource]
        return None

    def get_path_from_section(self, section_form_path):
        start_edge = []
        end_edge = []
        for section_iter in section_form_path:
            if len(start_edge) == 0:
                start_edge.append(section_iter)
                current_end_edge = section_iter
                continue
            if(current_end_edge[1] == section_iter[0]):
                current_end_edge = section_iter
            else:
                end_edge.append(current_end_edge)
                start_edge.append(section_iter)
                current_end_edge = section_iter
        end_edge.append(current_end_edge)
        return list(zip(start_edge, end_edge))

# a = TrainProblemConstrained(INPUT_MODEL, ROUTE_GRAPH_FOLDER)
# print(a.latest_requirements())
# print(a.earliest_requirements())
# print(a.section_unions())
# print(a.minimum_running_times())

class BasicTrainProblemORTools:
    def __init__(self, input_model, route_graph_folder):
        self._trains = ServiceIntention(input_model)
        self.num_trains = 1    #self._trains.get_number_service_intentions()

        self.routes = RouteGraph(input_model, route_graph_folder)

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

        st = self._trains.get_section_requirements('111')
        start_time = self.to_seconds(st[0]["entry_earliest"])
        end_time = self.to_seconds(st[-1]["exit_latest"])
        self.time_windows = []
        for t in range(len(st)):
            if t==0 or t == len(st)-1:
                self.time_windows.append((start_time, end_time))
            else:
                if "exit_earliest" in st[t].keys():
                    self.time_windows.append((self.to_seconds(st[t]["exit_earliest"]), end_time))
        print(self.time_windows)

    def to_seconds(self, time_str):
        hr, minu, sec = map(int, time_str.split(':'))
        return hr*60*60 + minu*60 + sec
    
    def get_time_as_distance(self, start_node, end_node):
        return self.time_as_distance[start_node][end_node]

# a = BasicTrainProblem()