from __future__ import print_function
import collections
from ortools.sat.python import cp_model
import sys
from ortools.constraint_solver import pywrapcp
import math
import json
import re
import copy

def to_seconds(hr, min, sec):
    return hr*60*60+min*60+sec

def to_actual_time(time):
    time = int(time)
    hr = int(math.floor(time/3600))
    min  = int((time%3600)/60)
    sec = int((time%3600)%60)
    res =  "{:02d}:{:02d}:{:02d}".format(hr, min, sec)
    return res

# section_requirement = {
#     '1':{
#         'num_tracks': 14,
#         'section_union_data': []
# #     }
# }

# TODO Rearrange data format

num_tracks = 14
section_union_data = [
    ([1,2,3], [4]),
    ([4], [5]),
    ([5], [6, 7]),
    ([6], [10, 11]),
    ([11], [12]),
    ([10], [13]),
    ([12,13], [14]),
    ([7], [8]),
    ([8], [9])
]

# [sections], entry_latest, weight_entry, exit_latest, weight_exit
latest_info1 = [
    ([9, 14], None, 0, to_seconds(8,50,00), 1)
]
latest_info2 = [
    ([9, 14], None, 0, to_seconds(8,16,00), 1)
]

earliest_info1 = [
    ([1, 2, 3], to_seconds(8, 20, 00), None),
    ([5], None, to_seconds(8, 30, 00))
]

earliest_info2 = [
    ([1, 2, 3], to_seconds(7, 50, 00), None),
]

# Total waiting time on each section
section_time = [ 53, 53, 53, 32, 32+3*60, 32, 32, 32, 32, 32, 32, 32, 32, 32 ]


entry_earliest1 = to_seconds(8, 20, 00)
exit_latest1 = to_seconds(8, 50, 00)

entry_earliest2 = to_seconds(7, 50, 00)
exit_latest2 = to_seconds(8, 16, 00)


def main():
  # Create the model and solver.
  model = cp_model.CpModel()
  solver = cp_model.CpSolver()
  
  solver = pywrapcp.Solver("simple_example")
  model = solver

  # Create the variables.
  num_tracks = 14
  x1 = [model.IntVar(0, 1, "x1(%d)"%i) for i in range(num_tracks)]
  x2 = [model.IntVar(0, 1, "x2(%d)"%i) for i in range(num_tracks)]

  s1 = [model.IntVar(entry_earliest1, exit_latest1, 's%d'%i) for i in range(num_tracks)]
  d1 = [model.IntVar(entry_earliest1, exit_latest1, 'd%d'%i) for i in range(num_tracks)]

  s2 = [model.IntVar(entry_earliest2, exit_latest2, 's%d'%i) for i in range(num_tracks)]
  d2 = [model.IntVar(entry_earliest2, exit_latest2, 'd%d'%i) for i in range(num_tracks)]

#   s = [model.NewBoolVar("x_%d"%i) for i in range(num_tracks)]
#   e = []
  # Constraints
  
  all_vars = []
  for x, s, d, earliest_info, latest_info in [(x1, s1, d1, earliest_info1, latest_info1), (x2, s2, d2, earliest_info2, latest_info2)]:
    # Start-End constraint
    model.Add(x[0]+x[1]+x[2]==1)

    # Path constraints
    for constraint in section_union_data:
      lhs = sum([ x[i-1] for i in constraint[0] ])
      rhs = sum([ x[i-1] for i in constraint[1] ])
      model.Add(lhs==rhs)

    # Loss function
    for constraint in latest_info:
        loss = 0
        sections = constraint[0]
  
        if constraint[1]:
            for section in sections:
              loss += s[section-1]*constraint[2]*(s[section-1]-solver.Max(s[section-1], constraint[1]))
  
        if constraint[3]:
            for section in sections:
              loss += s[section-1]*constraint[4]*(d[section-1]-solver.Max(d[section-1], constraint[3]))
        
        print(loss)
        solver.Add(loss==0)
  
    # Time constraints
    for section in range(num_tracks):
      solver.Add(d[section-1] >= s[section-1] + section_time[section-1])
  
    for constraint in earliest_info:
        for section in constraint[0]:
          if constraint[1]:
            solver.Add(s[section-1]>=constraint[1])
          
          if constraint[2]:
            solver.Add(d[section-1]>=constraint[2])
  
      # Consistency Constraints
    for constraint in section_union_data:
        start = constraint[0]
        end = constraint[1]
        for i in start:
          for j in end:
            solver.Add(x[i-1]*x[j-1]*(d[i-1]-s[j-1])==0)

    all_vars = all_vars+x+s+d

  # Create the constraints.
#   model.Add(x != y)
  # Call the solver.

#   solution_printer = SolutionPrinter(x)
#   status = solver.SearchForAllSolutions(model, solution_printer)
#   print('\nNumber of solutions found: %i' % solution_printer.SolutionCount())

  db = solver.Phase(all_vars, solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
  solutions_limit = solver.SolutionsLimit(1)
  solver.Solve(db, solutions_limit)

  count=0
  print("Solution", '\n')

  ans = {}
  while solver.NextSolution():
    count += 1
    for v in all_vars:
      if(str(v)[0] == "x"):
        service_key = re.split(r'[()]', str(v)[1:])[0]
        if service_key not in ans.keys():
          ans[service_key] = {}
        section_key = re.split(r'[()]', str(v)[1:])[1]
        if section_key not in ans[service_key].keys():
          ans[service_key][section_key] = {}
      else:
        section_key = re.split(r'[()]', str(v)[1:])[0]

      if(str(v)[0] == "x"):
        ans[service_key][section_key][str(v)[0]] = int(re.split(r'[()]', str(v)[1:])[-2])
      else:
        ans[service_key][section_key][str(v)[0]] = to_actual_time(re.split(r'[()]', str(v)[1:])[-2])

    print(ans)
    
    with open('sample_submission.json', mode='w') as outfile:
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
            data2['route_section_id'] = str(service_intention_id) + "#" + str(int(section_key)+1)
            data2['sequence_number'] = sqn
            sqn += 1
            data2['route_path'] = None
            data2['section_requirement'] = None
            data['train_run_sections'].append(copy.deepcopy(data2))
        outputdump['train_runs'].append(copy.deepcopy(data))
      json.dump(outputdump, outfile)

    break        
  print("Number of solutions: ", count)
  print("should exit")

class SolutionPrinter(cp_model.CpSolverSolutionCallback):
  """Print intermediate solutions."""

  def __init__(self, variables):
    self.__variables = variables
    self.__solution_count = 0

  def NewSolution(self):
    self.__solution_count += 1
    for v in self.__variables:
      print('%s = %i' % (v, self.Value(v)), end = ', ')
    # print()

  def SolutionCount(self):
    return self.__solution_count

if __name__ == '__main__':
  main()