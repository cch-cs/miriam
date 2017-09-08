import os

from planner.cbs_ext.plan import generate_config, plan, alloc_threads
from planner.cbs_ext_test import get_data_colission, get_data_random
from planner.eval.eval import get_costs
from tools import benchmark


def tcbsnn_for_comparison(config):
    print("Testing with number_nearest=" + str(config['number_nearest']))
    print("Testing with all_collisions=" + str(config['all_collisions']))
    agent_pos, grid, idle_goals, jobs = config['params']
    if 'milp' in config:
        print("milp")
        from planner.milp.milp import plan_milp
        res_agent_job, res_paths = plan_milp(agent_pos, jobs, grid, config)
    elif 'greedy' in config:
        print("greedy")
        from planner.greedy.greedy import plan_sc
        res_agent_job, res_paths = plan_sc(agent_pos, jobs, grid, config)
    else:
        res_agent_job, res_agent_idle, res_paths = plan(agent_pos, jobs, [], idle_goals, grid, config)
    print(res_agent_job)
    return get_costs(res_paths, jobs, res_agent_job, True)


def test_tcbsnn_comparison():
    fname = '/tmp/random.pkl'
    try:
        os.remove(fname)  # cleaning from maybe last run
    except FileNotFoundError:
        pass

    config_opt = generate_config()
    config_opt['params'] = get_data_random(map_res=8,
                                           map_fill_perc=20,
                                           agent_n=3,
                                           job_n=4,
                                           idle_goals_n=0)
    config_opt['filename_pathsave'] = fname

    config_milp = config_opt.copy()
    config_milp['milp'] = 1

    config_greedy = config_opt.copy()
    config_greedy['greedy'] = 1

    config_nn = config_opt.copy()
    config_nn['number_nearest'] = 2

    config_col = config_nn.copy()
    config_col['all_collisions'] = True

    configs = [config_milp, config_greedy, config_col, config_nn, config_opt]
    ts, ress = benchmark(tcbsnn_for_comparison, configs)

    for conf in configs:
        if ress[configs.index(conf)]:
            assert ress[configs.index(config_opt)] <= \
                   ress[configs.index(conf)], \
                "optimal planner should be better then:\n------\n" + str(conf)

    os.remove(fname)

if __name__ == "__main__":
    test_tcbsnn_comparison()
