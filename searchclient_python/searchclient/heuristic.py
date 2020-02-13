from abc import ABCMeta, abstractmethod
import math
import sys

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
        min_distance_agent_to_box = state.MAX_ROW + state.MAX_COL
        
        for row in range(state.MAX_ROW):
            for col in range(state.MAX_COL):
                if state.boxes[row][col] is not None:
                    #Box not already in goal position
                    if(state.goals[row][col] != state.boxes[row][col].lower()):
                        goals_of_same_type = filter(lambda goal: goal[2].upper() == state.boxes[row][col], self.goal_locations)
    
                        # sorted_distances = sorted(map(lambda goal: math.sqrt(math.pow(goal[1] - col, 2)+ math.pow(goal[0] - row,2)), goals_of_same_type))
                        sorted_distances = sorted(map(lambda goal: abs(goal[1] - col) + abs(goal[0] - row), goals_of_same_type))
    
                        distances.append(sorted_distances[0])
                        
                        distance_to_agent=abs(row-state.agent_row) + abs(col-state.agent_col)
                        min_distance_agent_to_box = min(min_distance_agent_to_box, distance_to_agent)
                    
        return sum(distances) +1/1000000*min_distance_agent_to_box

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

