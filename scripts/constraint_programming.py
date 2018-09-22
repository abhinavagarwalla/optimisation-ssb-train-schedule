from __future__ import print_function
import collections
from ortools.sat.python import cp_model
import sys
from ortools.constraint_solver import pywrapcp
from parse_input_route_graph import TrainProblemConstrained

#Input File
INPUT_MODEL = "sample_files/sample_scenario.json"
ROUTE_GRAPH_FOLDER = "route_graphs/*.graphml"

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
  print("Earliest requirements: ", earliest_requirements)
  print("Section times: ", section_time)

  # Constraints  
  all_vars = []
  for service_intention in num_tracks.keys():
    
    x = section_visit_bool[service_intention]
    s = section_entry_time[service_intention]
    d = section_exit_time[service_intention]

    # Start-End constraint
    start_marker = latest_requirements[service_intention][0][0]
    start_edges = problem.section_marker_to_sequence_number(service_intention, start_marker)

    end_marker = latest_requirements[service_intention][0][0]
    end_edges = problem.section_marker_to_sequence_number(service_intention, end_marker)

    start_constaint = sum(x[index] for index in start_edges)
    end_constraint = sum(x[index] for index in end_edges)
    model.Add(start_constaint == 1)
    model.Add(end_constraint == 1)

    # Path constraints
    for constraint in section_union_data[service_intention]:
      lhs = sum([ x[problem.edge_to_sequence_number(service_intention, edges[0], edges[1])-1] for edges in constraint[0] ])
      rhs = sum([ x[problem.edge_to_sequence_number(service_intention, edges[0], edges[1])-1] for edges in constraint[1] ])
      model.Add(lhs==rhs)

    # Loss function
    for constraint in latest_requirements[service_intention]:
      loss = 0
      add_constraint = False
      edges = problem.section_marker_to_sequence_number(service_intention, constraint[0])

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
      edge = problem.edge_to_sequence_number(service_intention, section[0], section[1])
      solver.Add(d[edge - 1] >= s[edge - 1] + section_time[service_intention][section])
  
    for constraint in earliest_requirements[service_intention]:
      sections = problem.section_marker_to_sequence_number(service_intention, constraint[0])
      for section in sections:

        if constraint[1] is not None:
          solver.Add(s[section-1]>=constraint[1])
        
        if constraint[2] is not None:
          solver.Add(d[section-1]>=constraint[2])
  
    # Consistency Constraints
    for constraint in section_union_data[service_intention]:
      start = constraint[0]
      end = constraint[1]
      for i in start:
        for j in end:
          si = problem.edge_to_sequence_number(service_intention, i[0], i[1])
          sj = problem.edge_to_sequence_number(service_intention, j[0], j[1])
          solver.Add(x[si-1]*x[sj-1]*(d[si-1]-s[sj-1])==0)

    all_vars = all_vars+x+s+d

  db = solver.Phase(all_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
  solutions_limit = solver.SolutionsLimit(1)
  solver.Solve(db, solutions_limit)

  count=0
  print("Solution", '\n')

  while solver.NextSolution():
    count += 1
    for v in all_vars:
      print('%s = %i' % (v, v.Value()), end = ', ')
    break
  print("Number of solutions: ", count)
  print("should exit")

if __name__ == '__main__':
  main()