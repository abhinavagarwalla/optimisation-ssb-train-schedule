import json
import networkx as nx
from networkx.drawing.nx_agraph import write_dot, graphviz_layout
import time
import pathlib
import tempfile
import matplotlib.pyplot as plt
import isodate

def from_node_id(route_path, route_section, index_in_path):
    if "route_alternative_marker_at_entry" in route_section.keys() and \
            route_section["route_alternative_marker_at_entry"] is not None and \
            len(route_section["route_alternative_marker_at_entry"]) > 0:
                return "(" + str(route_section["route_alternative_marker_at_entry"][0]) + ")"
    else:
        if index_in_path == 0:  # can only get here if this node is a very beginning of a route
            return "(" + str(route_section["sequence_number"]) + "_beginning)"
        else:
            return "(" + (str(route_path["route_sections"][index_in_path - 1]["sequence_number"]) + "->" +
                          str(route_section["sequence_number"])) + ")"


def to_node_id(route_path, route_section, index_in_path):
    if "route_alternative_marker_at_exit" in route_section.keys() and \
            route_section["route_alternative_marker_at_exit"] is not None and \
            len(route_section["route_alternative_marker_at_exit"]) > 0:

                return "(" + str(route_section["route_alternative_marker_at_exit"][0]) + ")"
    else:
        if index_in_path == (len(route_path["route_sections"]) - 1): # meaning this node is a very end of a route
            return "(" + str(route_section["sequence_number"]) + "_end" + ")"
        else:
            return "(" + (str(route_section["sequence_number"]) + "->" +
                          str(route_path["route_sections"][index_in_path + 1]["sequence_number"])) + ")"

def generate_route_graphs(scenario):
    start_time = time.time()

    # now build the graph. Nodes are called "previous_FAB -> next_FAB" within lineare abschnittsfolgen and "AK" if
    # there is an Abschnittskennzeichen 'AK' on it
    route_graphs = dict()
    for route in scenario["routes"]:

        # set global graph settings
        G = nx.DiGraph(route_id = route["id"], name="Route-Graph for route "+str(route["id"]))

        track_wise_sequence_number = 0
        # add edges with data contained in the preprocessed graph
        for path in route["route_paths"]:
            for (i, route_section) in enumerate(path["route_sections"]):
                sn = route_section['sequence_number']
                rt = route_section['minimum_running_time']
                res = str(route_section['resource_occupations'])
                sm = ''
                if 'section_marker' in route_section.keys():
                    if len(route_section['section_marker']):
                        sm = route_section['section_marker'][0]
                py = 0.0
                if route_section['penalty'] is not None:
                    py = route_section['penalty']
                
                track_wise_sequence_number += 1
                print("Adding Edge from {} to {} with sequence number {}".format(from_node_id(path, route_section, i), to_node_id(path, route_section, i), sn))
                print(sn, rt, res, sm, py)
                G.add_edge(from_node_id(path, route_section, i),
                           to_node_id(path, route_section, i),
                           weight = int(isodate.parse_duration(rt).total_seconds()),
                           sequence_number=sn,
                           running_time = rt,
                           resource = res,
                           section_marker = sm,
                           penalty = py,
                           track_wise_sequence_number = track_wise_sequence_number,
                           route_path_id = str(path["id"]))

        route_graphs[route["id"]] = G

    print("Finished building fahrweg-graphen in {} seconds".format(str(time.time() - start_time)))
    return route_graphs


def save_graph(route_graphs, output_folder):
    for k, route_graph in route_graphs.items():
        print("Running save graph for route_graph: ", k)
        for node in route_graph.nodes():
            route_graph.node[node]['label'] = node

        edge_labels = {}
        for node1, node2, data in route_graph.edges(data=True):
            edge_labels[(node1, node2)] = data['sequence_number'] 

        for edge in route_graph.edges():
            route_graph.edges[edge]['label'] = edge_labels[edge]

        pos = nx.spring_layout(route_graph)
        nx.draw(route_graph, pos, edge_color='black', width=1, linewidths=1, node_size=500, node_color='pink', alpha=0.9)
        nx.draw_networkx_edge_labels(route_graph,pos,edge_labels=edge_labels,font_color='red')
        nx.write_graphml(route_graph, output_folder + "graph-"+str(k)+".graphml")
        #plt.show()

# scratch######################################

if __name__ == "__main__":
    problem_str = "sample_scenario"
    scenario = "sample_files/" + problem_str + ".json"
    output_folder = "route_graphs/" + problem_str + "/"
    with open(scenario) as fp:
        scenario = json.load(fp)
    route_graphs = generate_route_graphs(scenario)
    save_graph(route_graphs, output_folder)