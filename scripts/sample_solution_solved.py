from __future__ import print_function
import collections
from ortools.sat.python import cp_model
import sys
from ortools.constraint_solver import pywrapcp
import math
import json
import re


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
  data = dict()
  store_x = dict()
  store_s = dict()
  store_d = dict()

  while solver.NextSolution():
    count += 1
    for v in all_vars:
      if(str(v)[0] == "x"):
        store_x[str(v)] =  v.Value()
      elif(str(v)[0] == "s"):
        temp = re.split(r'[()]', str(v)[1:])
        store_s[temp[0]] = to_actual_time(v.Value())
      elif(str(v)[0] == "d"):
        temp = re.split(r'[()]', str(v)[1:])
        store_d[temp[0]] = to_actual_time(v.Value())
      print(v)
    print(store_x)
    print(store_s)
    print(store_d)
    for key, value in store_x.items():
      #print(key,value)
      if(value == 1):
        temp = re.split(r'[()]', key)
        data['entry_time'] = store_s[temp[1]]
        data['exit_time'] = store_d[temp[1]]
        data['route'] = "11" + (temp[0])[1:]
        data['route_section_id'] = "11" + (temp[0])[1:] + "#" + "__"
        with open('data.json', 'w') as outfile:
          json.dump(data, outfile)
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