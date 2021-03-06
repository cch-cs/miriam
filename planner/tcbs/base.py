import logging
import numpy as np

from tools import ColoredLogger

logging.setLoggerClass(ColoredLogger)

def astar_base(start, condition, heuristic, get_children, cost, goal_test):
    _, start = cost(condition, start)  # it may have collisions

    closed = []
    open = [start]
    g_score = {}
    g_score[start] = 0
    f_score = {}
    f_score[start] = heuristic(condition, start)

    f_score_open = np.array([])
    f_score_open = np.append(f_score_open, f_score[start])

    while len(open) > 0:
        # the node in openSet having the lowest fScore[] value
        current = argmin_f_open(open, f_score_open)

        if goal_test(condition, current):
            return current

        i_rm = open.index(current)
        open.remove(current)
        f_score_open = np.delete(f_score_open, i_rm)

        closed.append(current)
        children = get_children(condition, current)
        for neighbor in children:
            if neighbor in closed:
                continue  # Ignore the neighbor which is already evaluated.
            # The distance from start to a neighbor
            c, neighbor = cost(condition, neighbor)
            if c >= 99999:
                closed.append(neighbor)
                continue  # This is not part of a plan

            tentative_g_score = c

            append = True
            if neighbor not in open:  # Discover a new node
                open.append(neighbor)
            elif tentative_g_score >= g_score[neighbor]:
                continue  # This is not a better path.
            else:
                append = False
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = g_score[neighbor]
            f_score[neighbor] += heuristic(condition, neighbor)
            if append:
                f_score_open = np.append(f_score_open, f_score[neighbor])

    raise RuntimeError("Can not find a solution")


def argmin_f_open(open_list, f_score_open):
    assert len(open_list) == len(f_score_open), "Lengths must be equal"
    return open_list[np.argmin(f_score_open)]
