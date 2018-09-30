from __future__ import print_function
import collections
from ortools.sat.python import cp_model
import sys
from ortools.constraint_solver import pywrapcp
from parse_input_route_graph import TrainProblemConstrained
import math
import json
import re
import copy
import itertools
import numpy as np

#Input File
problem_string = "sample_scenario"
INPUT_MODEL = "sample_files/" + problem_string + ".json"
ROUTE_GRAPH_FOLDER = "route_graphs/" + problem_string + "/*.graphml"
OUTPUT_MODEL_FILE = "sample_files/" + problem_string + "_solution_ours.json"

problem = TrainProblemConstrained(INPUT_MODEL, ROUTE_GRAPH_FOLDER)

num_tracks = problem.num_sections
section_union_data = problem.section_unions()

# [sections], entry_latest, weight_entry, exit_latest, weight_exit
latest_requirements = problem.latest_requirements()

earliest_requirements = problem.earliest_requirements()

# Total waiting time on each section
section_time = problem.minimum_running_times()

entry_earliest = {k:v[0][1] for k,v in earliest_requirements.items()}
exit_latest = {k:v[-1][-2] for k,v in latest_requirements.items()}

def main():
  # Create the model and solver.
  model = cp_model.CpModel()
  solver = cp_model.CpSolver()
  
  solver = pywrapcp.Solver("sample_problem")
  model = solver

  # Create the variables.
  section_visit_bool = {service_intention_number: [model.IntVar(0, 1, "x{}({})".format(service_intention_number, i)) for i in range(num_tracks[service_intention_number])] for service_intention_number in num_tracks.keys()}

  section_entry_time = {service_intention_number: [model.IntVar(entry_earliest[service_intention_number], exit_latest[service_intention_number], "s{}({})".format(service_intention_number, i)) for i in range(num_tracks[service_intention_number])] for service_intention_number in num_tracks.keys()}

  section_exit_time = {service_intention_number: [model.IntVar(entry_earliest[service_intention_number], exit_latest[service_intention_number], "d{}({})".format(service_intention_number, i)) for i in range(num_tracks[service_intention_number])] for service_intention_number in num_tracks.keys()}
  
  # list(zip(section_visit_bool, section_entry_time, section_exit_time, earliest_info, latest_info)))
 
  print("Num_tracks: ", num_tracks)
  print("Section Union Data: ", section_union_data)
  print("Latest requirements: ", latest_requirements)
  # print("Earliest requirements: ", earliest_requirements)
  # print("Section times: ", section_time)

  # Constraints  
  all_vars = []
  for service_intention in num_tracks.keys():
    
    x = section_visit_bool[service_intention]
    s = section_entry_time[service_intention]
    d = section_exit_time[service_intention]

    # Start-End constraint
    start_marker = latest_requirements[service_intention][0][0]
    start_edges = problem.section_marker_to_track_wise_sequence_number(service_intention, start_marker)

    end_marker = latest_requirements[service_intention][-1][0]
    end_edges = problem.section_marker_to_track_wise_sequence_number(service_intention, end_marker)

    print(start_edges, end_edges)
    print(start_marker, end_marker)
    print([problem.track_wise_sequence_number_to_edge(service_intention, i) for i in start_edges])
    print([problem.track_wise_sequence_number_to_edge(service_intention, i) for i in end_edges])
    # exit(9)
    start_constaint = sum(x[index-1] for index in start_edges)
    end_constraint = sum(x[index-1] for index in end_edges)
    model.Add(start_constaint == 1)
    model.Add(end_constraint == 1)

    # Path constraints
    for constraint in section_union_data[service_intention]:
      lhs = sum([ x[problem.edge_to_track_wise_sequence_number(service_intention, edges[0], edges[1])-1] for edges in constraint[0] ])
      rhs = sum([ x[problem.edge_to_track_wise_sequence_number(service_intention, edges[0], edges[1])-1] for edges in constraint[1] ])
      model.Add(lhs==rhs)

    # Loss function
    for constraint in latest_requirements[service_intention]:
      loss = 0
      add_constraint = False
      edges = problem.section_marker_to_track_wise_sequence_number(service_intention, constraint[0])

      if constraint[1] is not None:
        add_constraint = True
        for section in edges:
          loss += s[section-1]*constraint[2]*(s[section-1]-solver.Max(s[section-1], constraint[1]))

      if constraint[3] is not None:
        add_constraint = True
        for section in edges:
          loss += s[section-1]*constraint[4]*(d[section-1]-solver.Max(d[section-1], constraint[3]))
      
      if add_constraint:
        solver.Add(loss==0)
  
    # Time constraints
    for section in section_time[service_intention].keys():
      edge = problem.edge_to_track_wise_sequence_number(service_intention, section[0], section[1])
      solver.Add(d[edge - 1] >= s[edge - 1] + section_time[service_intention][section])
  
    # Earliest time constraint
    for constraint in earliest_requirements[service_intention]:
      sections = problem.section_marker_to_track_wise_sequence_number(service_intention, constraint[0])
      for section in sections:

        if constraint[1] is not None:
          solver.Add(s[section-1]>=constraint[1])
        
        if constraint[2] is not None:
          solver.Add(d[section-1]>=constraint[2])
  
    # Consistent entry-exit time constraint
    for constraint in section_union_data[service_intention]:
      start = constraint[0]
      end = constraint[1]
      for i in start:
        for j in end:
          si = problem.edge_to_track_wise_sequence_number(service_intention, i[0], i[1])
          sj = problem.edge_to_track_wise_sequence_number(service_intention, j[0], j[1])
          solver.Add(x[si-1]*x[sj-1]*(d[si-1]-s[sj-1])==0)

    all_vars = all_vars+x+s+d

  # service_intention_keys = list(num_tracks.keys())
  # service_combinations = itertools.combinations(service_intention_keys, 2)
  # #Could be made more efficient
  # for service_combination in service_combinations:
  #   service_intention_1 = service_combination[0]
  #   service_intention_2 = service_combination[1]
  #   resource_sections_1 = problem.get_sections_with_resource(service_intention_1)
  #   resource_sections_2 = problem.get_sections_with_resource(service_intention_2)
  #   common_resource = problem.get_common_resources(resource_sections_1, resource_sections_2)

  #   for resource in common_resource:
  #     # print("resources: ", resource, resource_sections_1.keys())
  #     section1 = resource_sections_1[resource]
  #     section2 = resource_sections_2[resource]

  #     #join sections to form a path
  #     # print("Section-1", section1)
  #     # print("Section-2", section2)

  #     ## combining edges to form a path
  #     path_section1 = problem.get_path_from_section(section1)
  #     path_section2 = problem.get_path_from_section(section2)
      
  #     path_combinations = itertools.product(path_section1, path_section2)
  #     for pathc in path_combinations:
  #       p1, p2 = pathc[0], pathc[1]
  #       p1_start_edge = problem.edge_to_sequence_number(service_intention_1, p1[0][0], p1[0][1])
  #       p1_end_edge = problem.edge_to_sequence_number(service_intention_1, p1[1][0], p1[1][1])
  #       p2_start_edge = problem.edge_to_sequence_number(service_intention_2, p2[0][0], p2[0][1])
  #       p2_end_edge = problem.edge_to_sequence_number(service_intention_2, p2[1][0], p2[1][1])
  #       a = section_entry_time[service_intention_1][p1_start_edge - 1]
  #       b = section_entry_time[service_intention_2][p2_start_edge - 1]
  #       c = section_exit_time[service_intention_1][p1_end_edge - 1]
  #       d = section_exit_time[service_intention_2][p2_end_edge - 1]
  #       dR = problem.get_release_time(resource)
  #       # print(common_resource, resource_sections_1, dR)
  #       model.Add((a >= b)*(a) >= (a >= b)*(d + dR))
  #       model.Add((a <= b)*(b) >= (a <= b)*(c + dR))

  db = solver.Phase(all_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
  solutions_limit = solver.SolutionsLimit(1)
  solver.Solve(db, solutions_limit)

  count=0
  print("Solution", '\n')

  while solver.NextSolution():
    count += 1
    write_output_to_json(all_vars, problem)
    break
  print("Number of solutions: ", count)
  print("should exit")

def write_output_to_json(all_vars, problem):
  ans = {}
  for v in all_vars:
    # print(v)
    if(str(v)[0] == "x"):
      service_key = re.split(r'[()]', str(v)[1:])[0]
      if service_key not in ans.keys():
        ans[service_key] = {}
      section_key = re.split(r'[()]', str(v)[1:])[1]
      if section_key not in ans[service_key].keys():
        ans[service_key][section_key] = {}
    else:
      section_key = re.split(r'[()]', str(v)[1:])[1]

    if(str(v)[0] == "x"):
      ans[service_key][section_key][str(v)[0]] = int(re.split(r'[()]', str(v)[1:])[-2])
    else:
      ans[service_key][section_key][str(v)[0]] = problem.to_actual_time(re.split(r'[()]', str(v)[1:])[-2])

  ## Add sequence_number according to entry_times
  entry_times = {}
  for service_intention in ans.keys():
    entry_times[service_intention] = np.sort(np.unique([v['s'] for v in ans[service_intention].values()], return_index=True, axis=0)[0])
  
  ## Add logic to show section_marker only if required in section_requirements in service_intentions 
  print(ans)
  
  with open(OUTPUT_MODEL_FILE, mode='w') as outfile:
    outputdump = {"problem_instance_label": "SBB_challenge_sample_scenario_with_routing_alternatives",
                  "problem_instance_hash": -1254734547,
                  "hash": 1538680897,
                  "train_runs": []}
    for service_intention_id, val  in ans.items():
      data = dict()
      data['service_intention_id'] = service_intention_id
      data["train_run_sections"] = []
      sqn = 1
      for section_key, info in val.items():
        data2 = {}
        if(info['x']==1):
          data2['entry_time'] = info['s']
          data2['exit_time'] = info['d']
          data2['route'] = int(service_intention_id)
          data2['route_section_id'] = str(service_intention_id) + "#" + str(problem.track_wise_sequence_number_to_sequence_number(service_intention_id, int(section_key)+1))
          data2['sequence_number'] = entry_times[service_intention_id].tolist().index(data2['entry_time']) + 1
          sqn += 1
          # sections = problem.section_marker_to_track_wise_sequence_number(service_intention_id, problem.track_wise_sequence_number_to_section_marker(service_intention_id, str(int(section_key)+1)))
          data2['route_path'] = problem.track_wise_sequence_number_to_route_path_id(service_intention_id, int(section_key)+1)
          data2['section_requirement'] = None
          if problem.if_section_marker_referenced_in_section_requirements(service_intention_id, int(section_key)+1):
            data2['section_requirement'] = problem.track_wise_sequence_number_to_section_marker(service_intention_id, int(section_key)+1)
          data['train_run_sections'].append(copy.deepcopy(data2))
      outputdump['train_runs'].append(copy.deepcopy(data))
    json.dump(outputdump, outfile, indent=4)

if __name__ == '__main__':
  main()