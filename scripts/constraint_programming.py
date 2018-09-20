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
 
  # Constraints  
  all_vars = []
  for service_intention in num_tracks.keys():
    
    x = section_visit_bool[service_intention]
    s = section_entry_time[service_intention]
    d = section_exit_time[service_intention]
    earliest_info = earliest_requirements[service_intention]
    latest_info = latest_requirements[service_intention]

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

  while solver.NextSolution():
    count += 1
    for v in all_vars:
      print('%s = %i' % (v, v.Value()), end = ', ')
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