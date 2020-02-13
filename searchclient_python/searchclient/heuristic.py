from abc import ABCMeta, abstractmethod
import math

class Heuristic(metaclass=ABCMeta):

    def __init__(self, initial_state: 'State'):
        self.goal_locations = []
        #Save locations of goals like e.g. (row,col, a)
        for row in range(initial_state.MAX_ROW):
            for col in range(initial_state.MAX_COL):
                if initial_state.goals[row][col] is not None:
                    self.goal_locations.append((row,col,initial_state.goals[row][col]))
  

    def h(self, state: 'State') -> 'int':
        # distance from box to nearest goal
        distances = []

        for row in range(state.MAX_ROW):
            for col in range(state.MAX_COL):
                if state.boxes[row][col] is not None:
                    goals_of_same_type = filter(lambda goal: goal[2].upper() == state.boxes[row][col], self.goal_locations)

                    # sorted_distances = sorted(map(lambda goal: math.sqrt(math.pow(goal[1] - state.agent_col, 2)+ math.pow(goal[0] - state.agent_row,2)), goals_of_same_type))
                    sorted_distances = sorted(map(lambda goal: abs(goal[1] - col) + abs(goal[0] - row), goals_of_same_type))

                    distances.append(sorted_distances[0])

        return sum(distances)

    # def h(self, state: 'State') -> 'int':
    #     # Straight Line distance
    #     for goal in self.goal_locations:
    #         a = goal[1] - state.agent_col
    #         b = goal[0] - state.agent_row
    #         return math.sqrt(math.pow(a, 2)+ math.pow(b,2))
    
    @abstractmethod
    def f(self, state: 'State') -> 'int': pass
    
    @abstractmethod
    def __repr__(self): raise NotImplementedError


class AStar(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)
    
    def f(self, state: 'State') -> 'int':
        return state.g + self.h(state)
    
    def __repr__(self):
        return 'A* evaluation'


class WAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int'):
        super().__init__(initial_state)
        self.w = w
    
    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)
    
    def __repr__(self):
        return 'WA* ({}) evaluation'.format(self.w)


class Greedy(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)
    
    def f(self, state: 'State') -> 'int':
        return self.h(state)
    
    def __repr__(self):
        return 'Greedy evaluation'

