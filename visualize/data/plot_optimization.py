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

    con = sqlite3.connect(filepath + '_multimodal.db')
    cur = con.cursor()
    multimodal_planner = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["multimodal"]["median"] = multimodal_planner[0].tolist()
    data["multimodal"]["quantile5"] = multimodal_planner[1].tolist()
    data["multimodal"]["quantile95"] = multimodal_planner[2].tolist()

    con = sqlite3.connect(filepath + '_KOMO.db')
    cur = con.cursor()
    KOMO_planner = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["KOMO"]["median"] = KOMO_planner[0].tolist()
    data["KOMO"]["quantile5"] = KOMO_planner[1].tolist()
    data["KOMO"]["quantile95"] = KOMO_planner[2].tolist()

    con = sqlite3.connect(filepath + '_BITstar.db')
    cur = con.cursor()
    BITstar_planner = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["BITstar"]["median"] = BITstar_planner[0].tolist()
    data["BITstar"]["quantile5"] = BITstar_planner[1].tolist()
    data["BITstar"]["quantile95"] = BITstar_planner[2].tolist()

    # con = sqlite3.connect(filepath + '_FMT.db')
    # cur = con.cursor()
    # FMT_planner = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    # data["FMT"]["median"] = FMT_planner[0].tolist()
    # data["FMT"]["quantile5"] = FMT_planner[1].tolist()
    # data["FMT"]["quantile95"] = FMT_planner[2].tolist()

    con = sqlite3.connect(filepath + '_LBTRRT.db')
    cur = con.cursor()
    LBTRRT_planner = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["LBTRRT"]["median"] = LBTRRT_planner[0].tolist()
    data["LBTRRT"]["quantile5"] = LBTRRT_planner[1].tolist()
    data["LBTRRT"]["quantile95"] = LBTRRT_planner[2].tolist()

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

    multimodal_median = data["multimodal"]["median"]
    multimodal_q5 = data["multimodal"]["quantile5"]
    multimodal_q95 = data["multimodal"]["quantile95"]
    start = get_start_index(multimodal_median, times, max_cost)
    ax.plot(times[start:], multimodal_median[start:], color=colors['multimodal'], label='MOMO')
    ax.fill_between(times[start:], multimodal_q5[start:], multimodal_q95[start:], color=colors['multimodal'],
                     alpha=0.5)

    KOMO_median = data["KOMO"]["median"]
    KOMO_q5 = data["KOMO"]["quantile5"]
    KOMO_q95 = data["KOMO"]["quantile95"]
    start = get_start_index(KOMO_median, times, max_cost)
    ax.plot(times[start:], KOMO_median[start:], color=colors['KOMO'], label='KOMO')
    ax.fill_between(times[start:], KOMO_q5[start:], KOMO_q95[start:], color=colors['KOMO'],
                     alpha=0.5)

    BITstar_median = data["BITstar"]["median"]
    BITstar_q5 = data["BITstar"]["quantile5"]
    BITstar_q95 = data["BITstar"]["quantile95"]
    start = get_start_index(BITstar_median, times, max_cost)
    ax.plot(times[start:], BITstar_median[start:], color=colors['BITstar'], label='BIT*')
    ax.fill_between(times[start:], BITstar_q5[start:], BITstar_q95[start:], color=colors['BITstar'],
                     alpha=0.5)

    # FMT_median = data["FMT"]["median"]
    # FMT_q5 = data["FMT"]["quantile5"]
    # FMT_q95 = data["FMT"]["quantile95"]
    # start = get_start_index(FMT_median, times, max_cost)
    # ax.plot(times[start:], FMT_median[start:], color=colors['FMT'], label='FMT')
    # ax.fill_between(times[start:], FMT_q5[start:], FMT_q95[start:], color=colors['FMT'],
    #                  alpha=0.5)

    LBTRRT_median = data["LBTRRT"]["median"]
    LBTRRT_q5 = data["LBTRRT"]["quantile5"]
    LBTRRT_q95 = data["LBTRRT"]["quantile95"]
    start = get_start_index(LBTRRT_median, times, max_cost)
    ax.plot(times[start:], LBTRRT_median[start:], color=colors['LBTRRT'], label='LBTRRT')
    ax.fill_between(times[start:], LBTRRT_q5[start:], LBTRRT_q95[start:], color=colors['LBTRRT'],
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
    # multimodal_point = data["multimodal"]["point"]
    # time_errors, cost_errors = get_errors(multimodal_point)
    # plt.errorbar(multimodal_point["time"][0], multimodal_point["cost"][0], cost_errors, time_errors,
    #              c=colors['multimodal'], marker=markerstyles[0], ms=10, lw=0.5)

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