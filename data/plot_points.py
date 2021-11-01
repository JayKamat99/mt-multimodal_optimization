import json
import sqlite3
import numpy as np

# filepath = 'narrow8/n8'

def get_from_progress(cur, ci_left, ci_right):
    pair = np.array(cur.execute("SELECT MIN(time), best_cost FROM {0} WHERE best_cost<>'NONE' GROUP BY runid".format('progress')).fetchall())
    if pair.size == 0:
        return None
    split = np.split(pair, 2, axis=1)
    times = split[0]
    costs = split[1]
    return calculate_points(times, costs, ci_left, ci_right)

def get_from_runs(cur, ci_left, ci_right):
    id = cur.execute("SELECT id FROM {} WHERE name='geometric_RRTConnect'".format('plannerConfigs')).fetchall()[0][0]
    # t = cur.execute("SELECT time, cost, status FROM {0} WHERE plannerid={1}".format('runs', id)).fetchall()
    pair = np.array(cur.execute("SELECT time, cost FROM {0} WHERE plannerid={1} AND status=6".format('runs', id)).fetchall())
    if pair.size == 0:
        return None
    split = np.split(pair, 2, axis=1)
    times = split[0]
    costs = split[1]
    return calculate_points(times, costs, ci_left, ci_right)


def calculate_points(times, costs, ci_left, ci_right):
    data = {"time": [np.median(times), np.percentile(times, ci_left, interpolation='nearest'),
                     np.percentile(times, ci_right, interpolation='nearest')],
            "cost": [np.median(costs), np.percentile(costs, ci_left, interpolation='nearest'),
                     np.percentile(costs, ci_right, interpolation='nearest')]}
    return data

def max_point(max_time, max_cost):
    data = {"time": [max_time, max_time,
                     max_time],
            "cost": [max_cost, max_cost, max_cost]}
    return data

def store_points(filepath):
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    ci_left = data["info"]["ci_left"]
    ci_right = data["info"]["ci_right"]
    max_time = data["info"]["max_time"]["optimization"]
    max_cost = data["info"]["max_cost"]

    # con = sqlite3.connect(filepath + '_spacetime.db')
    # cur = con.cursor()
    # space_time_rrt = get_from_progress(cur, ci_left, ci_right)
    # data["spacetime"]["point"] = space_time_rrt

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtconnect_tb[i]) + '.db')
        cur = con.cursor()
        rrt_connect = get_from_runs(cur, ci_left, ci_right)
        if rrt_connect is None:
            rrt_connect = max_point(max_time, max_cost)
        data["rrtconnect" + str(rrtconnect_tb[i])]["point"] = rrt_connect

    # rrtstar_tb = data["info"]["rrtstar_tb"]
    # for i in range(len(rrtstar_tb)):
    #     con = sqlite3.connect(filepath + '_' + str(rrtstar_tb[i]) + '.db')
    #     cur = con.cursor()
    #     rrt_star = get_from_progress(cur, ci_left, ci_right)
    #     if rrt_star is None:
    #         rrt_star = max_point(max_time, max_cost)
    #     data["rrtstar" + str(rrtstar_tb[i])]["point"] = rrt_star

    with open(filepath + '.json','w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


if __name__ == '__main__':
    filepath = '2/cost'
    store_points(filepath)
