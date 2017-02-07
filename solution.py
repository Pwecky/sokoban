#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
    #IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    count = 0
    if state.restrictions is None:
        for box in state.boxes:
            min_dist = float('inf')

            #check if box is in a viable storage location already (min_dist = 0)
            if box in state.storage:
                continue

            for store in state.storage:
                dist = abs(box[0]-store[0]) + abs(box[1]-store[1])
                if dist < min_dist:
                    min_dist = dist

            count += min_dist

    else:
        for box in state.boxes:
            min_dist = float('inf')
            index = state.boxes[box]

            #check if box is in a viable storage location already (min_dist = 0)
            if box in state.restrictions[index]:
                continue

            for store in state.restrictions[index]:
                dist = abs(box[0]-store[0]) + abs(box[1]-store[1])
                if dist < min_dist:
                    min_dist = dist

            count += min_dist

    return count

def heur_alternate(state):
    #IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    #This heuristic function will calculate the cost based on the manhattan_distance but will consider deadlock states
    #and the location of the robot.
    count = 0

    if state.restrictions is None:
        for box in state.boxes:
            min_dist = float('inf')

            #check if box is in a viable storage location already (min_dist = 0)
            if box in state.storage:
                continue

            #obstacles
            o_r = ((box[0]+1, box[1]) in state.obstacles)
            o_d = ((box[0], box[1]-1) in state.obstacles)
            o_u = ((box[0], box[1]+1) in state.obstacles)
            o_l = ((box[0]-1, box[1]) in state.obstacles)

            #walls
            w_r = (box[0]+1 == state.width)
            w_d = (box[1]-1 == -1)
            w_u = (box[1]+1 == state.height)
            w_l = (box[0]-1 == -1)

            #deadlock detection: boxes at corners of obstacles and walls
            if (o_r or w_r) and (o_d or w_d):
                count = float('inf')
                break
            elif (o_r or w_r) and (o_u or w_u):
                count = float('inf')
                break
            elif (o_l or w_l) and (o_u or w_u):
                count = float('inf')
                break
            elif (o_l or w_l) and (o_d or w_d):
                count = float('inf')
                break

            on_right = False
            on_left = False
            on_top = False
            on_bottom = False

            for store in state.storage:
                if store[0] == state.width-1:
                    on_right = True

                elif store[0] == 0:
                    on_left = True

                elif store[1] == 0:
                    on_bottom = True

                elif store[1] == state.height-1:
                    on_top = True

            #deadlock detection: boxes against a wall with no storage along the wall
            if not on_right and w_r:
                count = float('inf')
                break
            elif not on_left and w_l:
                count = float('inf')
                break
            elif not on_bottom and w_d:
                count = float('inf')
                break
            elif not on_top and w_u:
                count = float('inf')
                break

            #boxes
            b_r = ((box[0]+1, box[1]) in state.boxes)
            b_d = ((box[0], box[1]-1) in state.boxes)
            b_u = ((box[0], box[1]+1) in state.boxes)
            b_l = ((box[0]-1, box[1]) in state.boxes)

            #deadlock detection: two consecutive boxes against a wall
            if (b_d or b_u) and (w_r or w_l):
                count = float('inf')
                break
            elif (b_r or b_l) and (w_d or w_u):
                count = float('inf')
                break

            for store in state.storage:
                dist = (abs(state.robot[0]-box[0]) + abs(state.robot[1]-box[1])) + (abs(box[0]-store[0]) + abs(box[1]-store[1]))
                dist += (b_r + b_d + b_u + b_l) + (o_r + o_d + o_u + o_l) + (w_r + w_d + w_u + w_l)
                if dist < min_dist:
                    min_dist = dist

            count += min_dist

    else:
        for box in state.boxes:
            min_dist = float('inf')
            index = state.boxes[box]

            #check if box is in a viable storage location already (min_dist = 0)
            if box in state.restrictions[index]:
                continue

            #obstacles
            o_r = ((box[0]+1, box[1]) in state.obstacles)
            o_d = ((box[0], box[1]-1) in state.obstacles)
            o_u = ((box[0], box[1]+1) in state.obstacles)
            o_l = ((box[0]-1, box[1]) in state.obstacles)

            #walls
            w_r = (box[0]+1 == state.width)
            w_d = (box[1]-1 == -1)
            w_u = (box[1]+1 == state.height)
            w_l = (box[0]-1 == -1)

            #deadlock detection: boxes at corners of obstacles and walls
            if (o_r or w_r) and (o_d or w_d):
                count = float('inf')
                break
            elif (o_r or w_r) and (o_u or w_u):
                count = float('inf')
                break
            elif (o_l or w_l) and (o_u or w_u):
                count = float('inf')
                break
            elif (o_l or w_l) and (o_d or w_d):
                count = float('inf')
                break

            on_right = False
            on_left = False
            on_top = False
            on_bottom = False

            for store in state.restrictions[index]:
                if store[0] == state.width-1:
                    on_right = True

                elif store[0] == 0:
                    on_left = True

                elif store[1] == 0:
                    on_bottom = True

                elif store[1] == state.height-1:
                    on_top = True

            #deadlock detection: boxes against a wall with no storage along the wall
            if not on_right and w_r:
                count = float('inf')
                break
            elif not on_left and w_l:
                count = float('inf')
                break
            elif not on_bottom and w_d:
                count = float('inf')
                break
            elif not on_top and w_u:
                count = float('inf')
                break

            #boxes
            b_r = ((box[0]+1, box[1]) in state.boxes)
            b_d = ((box[0], box[1]-1) in state.boxes)
            b_u = ((box[0], box[1]+1) in state.boxes)
            b_l = ((box[0]-1, box[1]) in state.boxes)

            #deadlock detection: two consecutive boxes against a wall
            if (b_d or b_u) and (w_r or w_l):
                count = float('inf')
                break
            elif (b_r or b_l) and (w_d or w_u):
                count = float('inf')
                break

            for store in state.restrictions[index]:
                dist = (abs(state.robot[0]-box[0]) + abs(state.robot[1]-box[1])) + (abs(box[0]-store[0]) + abs(box[1]-store[1]))
                dist += (b_r + b_d + b_u + b_l) + (o_r + o_d + o_u + o_l) + (w_r + w_d + w_u + w_l)
                if dist < min_dist:
                    min_dist = dist

            count += min_dist

    return count

def fval_function(sN, weight):
    #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight*sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
    #IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    #initialize search engine
    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    #final stores the final result
    final = False

    #costbound is of the form: (g_bound,h_bound,g_plus_h_bound), initially set to infinity
    costbound = (float('inf'), float('inf'), float('inf'))
    time_remaining = timebound
    saved_time = os.times()[0]

    while time_remaining > 0:
        #result is SokobanState object
        result = se.search(time_remaining, costbound)

        if result:
            time_remaining -= (os.times()[0] - saved_time)
            saved_time = os.times()[0]
            #g_bound is result's gval
            costbound = (result.gval, float('inf'), float('inf'))
            final = result
        else:
            return final

    return final

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
    #IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    #wrap fval function
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))

    #initialize search engine
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)

    #final stores the final result
    final = False

    #costbound is of the form: (g_bound,h_bound,g_plus_h_bound), initially set to infinity
    costbound = (float('inf'), float('inf'), float('inf'))
    time_remaining = timebound
    saved_time = os.times()[0]

    while time_remaining > 0:
        #result is SokobanState object
        result = se.search(time_remaining, costbound)

        if result:
            time_remaining -= (os.times()[0] - saved_time)
            saved_time = os.times()[0]
            #g_bound + h_bound is result's gval
            costbound = (float('inf'), float('inf'), result.gval)
            final = result
        else:
            return final

    return final

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 



