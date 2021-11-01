import json
import sqlite3
import numpy as np
import matplotlib.pyplot as plt


# filepath = 'narrow8/n8'

# general parameters
resolution = 200


def get_plot_data(cur, times, max_cost, ci_left, ci_right):

    medians = np.zeros(len(times))
    quantile5 = np.zeros(len(times))
    quantile95 = np.zeros(len(times))

    for i in range(len(times)):
        data = np.array(cur.execute(
            "SELECT a.best_cost FROM (SELECT MAX(time), best_cost FROM {0} WHERE time<={1} GROUP BY runid) a".
            format('progress', times[i])).fetchall()).flatten()
        if data.size == 0:
            medians[i] = max_cost
            quantile5[i] = max_cost
            quantile95[i] = max_cost
        else:
            data = np.where(data == None, max_cost, data)
            medians[i] = np.median(data)
            quantile5[i] = np.percentile(data, ci_left, interpolation='nearest')
            quantile95[i] = np.percentile(data, ci_right, interpolation='nearest')

    return [medians, quantile5, quantile95]

def get_start_index(medians, times, max_cost):
    for i in range(len(times)):
        if medians[i] < max_cost:
            return i
    return len(times)


def store_optimization_data(filepath):
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    times = np.logspace(np.log10(data["info"]["min_time"]["optimization"]), np.log10(data["info"]["max_time"]["optimization"]), resolution)
    max_cost = data["info"]["max_cost"]
    ci_left = data["info"]["ci_left"]
    ci_right = data["info"]["ci_right"]

    con = sqlite3.connect(filepath + '_spacetime.db')
    cur = con.cursor()
    spacetime_rrt = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["spacetime"]["median"] = spacetime_rrt[0].tolist()
    data["spacetime"]["quantile5"] = spacetime_rrt[1].tolist()
    data["spacetime"]["quantile95"] = spacetime_rrt[2].tolist()

    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtstar_tb[i]) + '.db')
        cur = con.cursor()
        rrt_star = get_plot_data(cur, times, max_cost, ci_left, ci_right)
        data["rrtstar" + str(rrtstar_tb[i])]["median"] = rrt_star[0].tolist()
        data["rrtstar" + str(rrtstar_tb[i])]["quantile5"] = rrt_star[1].tolist()
        data["rrtstar" + str(rrtstar_tb[i])]["quantile95"] = rrt_star[2].tolist()

    with open(filepath + '.json','w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot_optimization(ax, data, colors, linestyles, markerstyles):

    min_time = data["info"]["min_time"]["optimization"]
    max_time = data["info"]["max_time"]["optimization"]
    times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)
    max_cost = data["info"]["max_cost"]

    ax.set_xscale('log')
    ax.set_yscale('linear')
    ax.set_xlim(min_time, max_time)
    ax.set_ylim(0.0, max_cost)

    spacetime_median = data["spacetime"]["median"]
    spacetime_q5 = data["spacetime"]["quantile5"]
    spacetime_q95 = data["spacetime"]["quantile95"]
    start = get_start_index(spacetime_median, times, max_cost)
    ax.plot(times[start:], spacetime_median[start:], color=colors['spacetime'], label='ST-RRT*')
    ax.fill_between(times[start:], spacetime_q5[start:], spacetime_q95[start:], color=colors['spacetime'],
                     alpha=0.5)



    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        rrtstar_median = data["rrtstar" + str(rrtstar_tb[i])]["median"]
        rrtstar_q5 = data["rrtstar" + str(rrtstar_tb[i])]["quantile5"]
        rrtstar_q95 = data["rrtstar" + str(rrtstar_tb[i])]["quantile95"]
        start = get_start_index(rrtstar_median, times, max_cost)
        plt.plot(times[start:], rrtstar_median[start:], color=colors['rrtstar'], label='RRT* ' + str(rrtstar_tb[i]), linestyle=linestyles[i])
        plt.fill_between(times[start:], rrtstar_q5[start:], rrtstar_q95[start:], color=colors['rrtstar'],
                         alpha=0.5)

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        rrtconnect_point = data["rrtconnect" + str(rrtconnect_tb[i])]["point"]
        time_errors, cost_errors = get_errors(rrtconnect_point)
        plt.errorbar(rrtconnect_point["time"][0], rrtconnect_point["cost"][0], cost_errors, time_errors,
                     c=colors['rrtconnect'], marker=markerstyles[i], ms=10, lw=0.5)

    # for i in range(len(rrtstar_tb)):
    #     rrtstar_point = data["rrtstar" + str(rrtstar_tb[i])]["point"]
    #     time_errors, cost_errors = get_errors(rrtstar_point)
    #     plt.errorbar(rrtstar_point["time"][0], rrtstar_point["cost"][0], cost_errors, time_errors,
    #                  c=colors['rrtstar'], marker=markerstyles[i], ms=10, lw=0.5)
    #
    # spacetime_point = data["spacetime"]["point"]
    # time_errors, cost_errors = get_errors(spacetime_point)
    # plt.errorbar(spacetime_point["time"][0], spacetime_point["cost"][0], cost_errors, time_errors,
    #              c=colors['spacetime'], marker=markerstyles[0], ms=10, lw=0.5)

    ax.grid(True, which="both", ls='--')
    ax.set_xlabel("run time [s]")
    ax.set_ylabel("solution cost")


def get_errors(point):
    time_errors = np.array([[point["time"][0] - point["time"][1]],
                            [point["time"][2] - point["time"][0]]])
    cost_errors = np.array([[point["cost"][0] - point["cost"][1]],
                            [point["cost"][2] - point["cost"][0]]])
    return time_errors, cost_errors


if __name__ == '__main__':
    filepath = '2/a1'
    # store_data(filepath)
    # plot(filepath)