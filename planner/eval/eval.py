import numpy as np
import matplotlib.pyplot as plt

from planner.cbs_ext.plan import plan, plot_results, generate_config
from tools import load_map


def eval(_map, agents, jobs, fname, display=False, finished_blocking=True):
    from planner.milp.milp import plan_milp
    grid = np.repeat(_map[:, ::2, np.newaxis], 100, axis=2)

    config = generate_config()
    config['filename_pathsave'] = fname
    config['finished_agents_block'] = finished_blocking

    print("Problem:")
    print(str(jobs))
    print(str(agents))

    print("plan")
    res_agent_job, res_agent_idle, res_paths = plan(agents, jobs, [], [], grid, config, plot=display)

    print("minlp")
    minlp_res_agent_job, minlp_res_paths = plan_milp(agents, jobs, grid, config)

    print("TCBS")
    print("agent_job: " + str(res_agent_job))
    print("paths: " + str(res_paths))
    costs_tcbs = get_costs(res_paths, jobs, res_agent_job, display)

    print("MINLP")
    print("agent_job: " + str(minlp_res_agent_job))
    print("paths: " + str(minlp_res_paths))
    if display:
        plot_results([], minlp_res_paths, agents, minlp_res_agent_job, plt.figure(), grid, [], jobs)
    costs_minlp = get_costs(minlp_res_paths, jobs, minlp_res_agent_job, display)
    return costs_tcbs, costs_minlp


def get_costs(paths, jobs, agent_job, display=True):
    if not paths:
        return np.inf
    costs = np.zeros(len(jobs))
    for ia, paths_for_agent in enumerate(paths):
        ip = 0
        for ij in agent_job[ia]:
            if paths_for_agent[ip][-1][0:2] == jobs[ij][1]:  # alloc job
                costs[ij] = paths_for_agent[ip][-1][2]  # t
            elif paths_for_agent[ip + 1][-1][0:2] == jobs[ij][1]:  # not pre-alloc
                ip += 1
                costs[ij] = paths_for_agent[ip][-1][2]  # t
            else:
                assert False, "Problem in assignment"
            ip += 2
    print("Costs:\n(per job:)")
    print(costs)
    print("(total:)")
    print(sum(costs))
    return sum(costs)


def random_landmark(landmarks, taken):
    assert len(taken) < len(landmarks), "All are taken"
    i_lm = np.random.choice(range(len(landmarks)))
    while (i_lm in taken):
        i_lm = np.random.choice(range(len(landmarks)))
    taken.add(i_lm)
    return landmarks[i_lm]


def random_jobs(n, landmarks):
    taken = set()
    jobs = []
    for i in range(n):
        jobs.append(
            (random_landmark(landmarks, taken),
             random_landmark(landmarks, taken),
             0)
        )
    return jobs

# -------
def corridor():
    _map = load_map('corridor.png')
    agents = [(0, 0),
              (0, 1)]
    jobs = [((5, 0), (5, 2), 0),
            ((4, 2), (4, 0), 0),
            ((3, 0), (3, 2), 0),
            ]
    eval(_map, agents, jobs, 'corridor.pkl')

# -------
def mr_t():
    _map = load_map('mr_t.png')
    agents = [(5, 3),
              (2, 1)]
    jobs = [((4, 3), (4, 1), 0),
            ((3, 1), (3, 3), 0)]
    eval(_map, agents, jobs, 'mr_t.pkl', finished_blocking=True)
# Results with finished agents as obstacle:
# CBS EXT
# agent_job: ((0, 1), ())
# paths: [([(5, 3, 0), (4, 3, 1)], [(4, 3, 2), (5, 3, 3), (5, 2, 4), (5, 1, 5), (4, 1, 6)], [(4, 1, 7), (3, 1, 8)], [(3, 1, 9), (4, 1, 10), (5, 1, 11), (5, 2, 12), (5, 3, 13), (4, 3, 14), (3, 3, 15)]), ([(2, 1, 0), (2, 1, 1), (2, 1, 2), (2, 1, 3), (2, 1, 4), (2, 1, 5), (2, 1, 6), (2, 1, 7), (2, 1, 8), (2, 1, 9), (2, 1, 10), (2, 1, 11), (2, 1, 12), (2, 1, 13), (2, 1, 14), (2, 1, 15)],)]
# Costs:
# (per job:)
# [  6.  15.]
# (total:)
# 21.0
# MILP
# agent_job: [(0,), (1,)]
# paths: [([(5, 3, 0), (4, 3, 1)], [(4, 3, 2), (5, 3, 3), (5, 2, 4), (5, 1, 5), (4, 1, 6)], [(4, 1, 7), (4, 1, 8), (4, 1, 9), (4, 1, 10), (4, 1, 11), (4, 1, 12), (4, 1, 13), (4, 1, 14), (4, 1, 15), (4, 1, 16), (4, 1, 17), (4, 1, 18), (4, 1, 19), (4, 1, 20)]), ([(2, 1, 0), (3, 1, 1)], [(3, 1, 2), (2, 1, 3), (2, 0, 4), (1, 0, 5), (0, 0, 6), (0, 1, 7), (0, 2, 8), (0, 3, 9), (0, 4, 10), (0, 5, 11), (0, 6, 12), (0, 7, 13), (1, 7, 14), (2, 7, 15), (2, 6, 16), (2, 5, 17), (2, 4, 18), (2, 3, 19), (3, 3, 20)])]
# Costs:
# (per job:)
# [  6.  20.]
# (total:)
# 26.0

# -------
def c():
    _map = load_map('c.png')
    agents = [(3, 3),
              (6, 5)]
    jobs = [((4, 3), (4, 5), 0),
            ((5, 5), (5, 3), 0)]
    eval(_map, agents, jobs, 'c.pkl')


# -------
def line():
    _map = load_map('line.png')
    agents = [(0, 0),
              (6, 0)]
    jobs = [((1, 0), (6, 0), 0),
            ((5, 0), (1, 0), 0),
            ]
    eval(_map, agents, jobs, 'line.pkl')


# -------
def h():
    _map = load_map('h.png')
    agents = [(0, 0),
              (2, 2)]
    jobs = [((0, 1), (2, 0), 0),
            ((2, 1), (0, 2), 0),
            ]
    eval(_map, agents, jobs, 'h.pkl')


# -------
def i():
    _map = load_map('I.png')
    agents = [(0, 2),
              (4, 1)]
    jobs = [((0, 0), (0, 3), 0),
            ((1, 3), (1, 0), 0),
            ((3, 0), (3, 3), 0),
            ((4, 3), (4, 0), 0),
            ]
    eval(_map, agents, jobs, 'I.pkl')


# -------
def s():
    _map = load_map('S.png')
    agents = [(3, 4),
              (6, 0),
              (2, 0)]
    jobs = [((0, 4), (0, 0), 0),
            ((2, 4), (8, 0), 0),
            ((7, 0), (3, 4), 0),
            ]
    eval(_map, agents, jobs, 'S.pkl')


# -------
def ff():
    jobs = random_jobs(2, [(0, 0), (2, 0), (2, 6), (4, 6), (6, 2), (7, 7)])
    _map = load_map('ff.png')
    agents = [(0, 0),
              (2, 6),
              (7, 7)]
    return eval(_map, agents, jobs, 'ff.pkl', finished_blocking=False, display=False)


if __name__ == "__main__":
    n_samples = 10
    res = np.zeros([n_samples, 2])
    for i_sample in range(n_samples):
        print("#" * 80)
        print("%d / %d" % (i_sample, n_samples))
        res[i_sample, :] = ff()
    print(res)
    print(res[:, 1] - res[:, 0])
