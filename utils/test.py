from __future__ import print_function
import collections
from ortools.sat.python import cp_model


def to_seconds(hr, min, sec):
    return hr*60*60+min*60+sec

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

latest_info = [
    ([9, 14], None, 0, to_seconds(8,50,00), 1)
]

entry_earliest = to_seconds(8, 20, 00)
exit_latest = to_seconds(8, 50, 00)


def main():
  # Create the model and solver.
  model = cp_model.CpModel()
  solver = cp_model.CpSolver()
  # Create the variables.
  num_tracks = 14
  x = [model.NewBoolVar("x%d"%i) for i in range(num_tracks)]

#   s = [model.NewIntVar(entry_earliest, exit_latest, 's%d'%i) for i in range(num_tracks)]
#   d = [model.NewIntVar(entry_earliest, exit_latest, 's%d'%i) for i in range(num_tracks)]


#   s = [model.NewBoolVar("x_%d"%i) for i in range(num_tracks)]
#   e = []
  # Constraints

  # Start-End constraint
  model.Add(x[0]+x[1]+x[2]==1)

  # Path constraints
  for constraint in section_union_data:
    lhs = sum([ x[i-1] for i in constraint[0] ])
    rhs = sum([ x[i-1] for i in constraint[1] ])
    model.Add(lhs==rhs)

  # Loss function
#   for constraint in latest_info:
#       loss = 0
#       sections = constraint[0]

#       if constraint[1]:
#           for section in sections:
#             loss += s[section]*solver.Max

    
#       if constraint[3]:
#           pass
      
#       print(loss)
#       exit(0)


  # Create the constraints.
#   model.Add(x != y)
  # Call the solver.
  solution_printer = SolutionPrinter(x)
  status = solver.SearchForAllSolutions(model, solution_printer)
  print('\nNumber of solutions found: %i' % solution_printer.SolutionCount())

class SolutionPrinter(cp_model.CpSolverSolutionCallback):
  """Print intermediate solutions."""

  def __init__(self, variables):
    self.__variables = variables
    self.__solution_count = 0

  def NewSolution(self):
    self.__solution_count += 1
    for v in self.__variables:
      print('%s = %i' % (v, self.Value(v)), end = ', ')
    print()

  def SolutionCount(self):
    return self.__solution_count

if __name__ == '__main__':
  main()