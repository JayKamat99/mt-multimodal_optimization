import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import json

# colors
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'
ibm_green = '#24A148'

col_MOMO = ibm_blue
col_rrt_connect = ibm_red
col_rrt_star = ibm_orange
col_PathSimplifier_planner = ibm_red
col_BITstar_planner = ibm_violet
col_FMT_planner = ibm_orange
col_LBTRRT_planner = ibm_green

# filepath = 'narrow8/n8'

# general parameters
resolution = 200


def get_from_progress(cur, times):
    count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()[0][0]
    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time "
            "FROM {0} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < {1}".format(
                'progress', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100
    return percentages


def get_percentages_first_solution(cur, times):
    count = cur.execute("SELECT COUNT(*) FROM {}".format('runs')).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE time_first_solution < {1}".format('runs', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages


def get_percentages_time(cur, times):
    id = cur.execute("SELECT id FROM {} WHERE name='geometric_RRTConnect'".format('plannerConfigs')).fetchall()[0][0]
    count = cur.execute("SELECT COUNT(*) FROM {} WHERE plannerid={}".format('runs', id)).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE plannerid={2} AND time < {1}".format('runs', times[i], id)).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages


def store_success_data(filepath):
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    # plot MOMO
    times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
                        resolution)

    con = sqlite3.connect(filepath + '_multimodal.db')
    cur = con.cursor()
    # MOMO = get_percentages_first_solution(cur, times)
    MOMO = get_from_progress(cur, times)

    # plot PathSimplifier_Planner
    times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
                        resolution)

    con = sqlite3.connect(filepath + '_PathSimplifier.db')
    cur = con.cursor()
    # PathSimplifier_Planner = get_percentages_first_solution(cur, times)
    PathSimplifier_Planner = get_from_progress(cur, times)

    # plot BITstar_Planner
    times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
                        resolution)

    con = sqlite3.connect(filepath + '_BITstar.db')
    cur = con.cursor()
    # BITstar_Planner = get_percentages_first_solution(cur, times)
    BITstar_Planner = get_from_progress(cur, times)

    # # plot FMT_Planner
    # times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
    #                     resolution)

    # con = sqlite3.connect(filepath + '_FMT.db')
    # cur = con.cursor()
    # # FMT_Planner = get_percentages_first_solution(cur, times)
    # FMT_Planner = get_from_progress(cur, times)

    # plot LBTRRT_Planner
    times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
                        resolution)

    con = sqlite3.connect(filepath + '_LBTRRT.db')
    cur = con.cursor()
    # LBTRRT_Planner = get_percentages_first_solution(cur, times)
    LBTRRT_Planner = get_from_progress(cur, times)

    rrt_connect = []
    rrt_star = []

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtconnect_tb[i]) + '.db')
        cur = con.cursor()
        rrt_connect.append(get_percentages_time(cur, times))

    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtstar_tb[i]) + '.db')
        cur = con.cursor()
        rrt_star.append(get_from_progress(cur, times))

    data["multimodal"] = {
        "success": MOMO.tolist()
    }

    data["PathSimplifier"] = {
        "success": PathSimplifier_Planner.tolist()
    }

    data["BITstar"] = {
        "success": BITstar_Planner.tolist()
    }

    # data["FMT"] = {
    #     "success": FMT_Planner.tolist()
    # }

    data["LBTRRT"] = {
        "success": LBTRRT_Planner.tolist()
    }

    for i in range(len(rrtconnect_tb)):
        data["rrtconnect" + str(rrtconnect_tb[i])] = {
            "success": rrt_connect[i].tolist()
        }

    for i in range(len(rrtstar_tb)):
        data["rrtstar" + str(rrtstar_tb[i])] = {
            "success": rrt_star[i].tolist()
        }

    with open(filepath + '.json', 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot_success(ax, data, colors, linestyles):

    min_time = data["info"]["min_time"]["success"]
    max_time = data["info"]["max_time"]["success"]
    times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)


    ax.set_xscale('log')
    ax.set_yscale('linear')
    ax.set_xlim(min_time, max_time)
    ax.set_ylim(0.0, 100.0)

    MOMO = data["multimodal"]["success"]
    ax.plot(times, MOMO, color=colors["multimodal"], linestyle='-', label='MOMO')

    PathSimplifier_Planner = data["PathSimplifier"]["success"]
    ax.plot(times, PathSimplifier_Planner, color=colors["PathSimplifier"], linestyle='-', label='PathSimplifier')

    BITstar_Planner = data["BITstar"]["success"]
    ax.plot(times, BITstar_Planner, color=colors["BITstar"], linestyle='-', label='BIT*')

    # FMT_Planner = data["FMT"]["success"]
    # ax.plot(times, FMT_Planner, color=colors["FMT"], linestyle='-', label='FMT')

    LBTRRT_Planner = data["LBTRRT"]["success"]
    ax.plot(times, LBTRRT_Planner, color=colors["LBTRRT"], linestyle='-', label='LBTRRT')

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        rrt_connect = data["rrtconnect" + str(rrtconnect_tb[i])]["success"]
        ax.plot(times, rrt_connect, color=colors["rrtconnect"], linestyle=linestyles[i],
                        label='RRTConnect ' + str(rrtconnect_tb[i]))
    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        rrt_star = data["rrtstar" + str(rrtstar_tb[i])]["success"]
        ax.plot(times, rrt_star, color=colors["rrtstar"], linestyle=linestyles[i], label='RRT* ' + str(rrtstar_tb[i]))

    ax.grid(True, which="both", ls='--')
    ax.set_ylabel("success [\%]")

def store_info(filepath, info):
    data = {}
    if os.path.isfile(filepath + '.json'):
        with open(filepath + '.json', 'r') as jsonfile:
            data = json.load(jsonfile)

    data["info"] = info

    with open(filepath + '.json', 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


if __name__ == '__main__':
    filepath = '2/a1'
    # store_info()
    # store_data()
    # plot()

    # tables = cur.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name").fetchall()
    # for table in tables:
    #     print("\nNAME: " + table[0] + "\n")
    #     cur.execute("SELECT * FROM {}".format(table[0])).fetchall()
    #     names = list(map(lambda x: x[0], cur.description))
    #     print(names)
    #
    # print(4 * '\n')
    # planners = cur.execute("SELECT id, name FROM {}".format('plannerConfigs')).fetchall()
    # for planner in planners:
    #     print("Planner {} has ID {}".format(planner[1], planner[0]))
    #
    # print(4 * '\n')
    # experiments = cur.execute("SELECT id, name, timelimit, runcount FROM {}".format('experiments')).fetchall()
    # for experiment in experiments:
    #     print(experiment)
    #
    # print(4 * '\n')
    # runs = cur.execute("SELECT id, experimentid, plannerid, time, simplified_correct_solution FROM {}".format('runs')).fetchall()
    # for run in runs:
    #     print(run)
    #
    # print(4 * '\n')
    # count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()
    # print(count)
    # best_times = cur.execute(
    #     "SELECT runid, MIN(time) FROM {} WHERE best_cost<>'NONE' GROUP BY runid".format('progress')).fetchall()
    # print(best_times)
    #
    # percentage = cur.execute(
    #     "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time FROM {} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < 2.0".format(
    #         'progress')).fetchall()
    # print(percentage)
