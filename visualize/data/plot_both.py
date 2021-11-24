import matplotlib.pyplot as plt
import json
from matplotlib.lines import Line2D

from plot_success_rate import store_info, store_success_data, plot_success
from plot_points import store_points
from plot_optimization import store_optimization_data, plot_optimization

# colors
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

color_dict = {
    'multimodal': ibm_blue,
    'rrtconnect': ibm_red,
    'rrtstar': ibm_yellow
}

linestyles = ['solid', 'dashed', 'dotted']
markerstyles = ['.', 'x', 'v']

filepath = 'Benchmarks/benchmark'

info = {
        "min_time": {
            "success": 0.001,
            "optimization": 0.001
        },
        "max_time": {
            "success": 30,
            "optimization": 30
        },
        "max_cost": 10,
        "ci_left": 39,
        "ci_right": 59,
        # "rrtconnect_tb": [1, 2, 4],
        # "rrtstar_tb": [1, 2, 4]
        "rrtconnect_tb": [],
        "rrtstar_tb": [1]
    }

def run():
    store()
    plot()

def store():
    store_info(filepath, info)
    store_success_data(filepath)
    store_points(filepath)
    store_optimization_data(filepath)

def plot():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    # plt.style.use('../../test.mplstyle')
    fig, axs = plt.subplots(2, 1, sharex='col')
    plot_success(axs[0], data, color_dict, linestyles)
    plot_optimization(axs[1], data, color_dict, linestyles, markerstyles)

    # create handles for the legend
    handles = []
    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        handles.append(Line2D([], [], color='black', marker='', ls=linestyles[i], label=rrtconnect_tb[i]))
    for i in range(len(rrtconnect_tb)):
        handles.append(Line2D([], [], color='black', marker=markerstyles[i], ls='', label=rrtconnect_tb[i]))
    axs[1].legend(handles=handles, loc='upper left', title='$t_{ub}$')


    plt.show()
    # plt.savefig('n8_bounded.pdf', format='pdf', dpi=300, bbox_inches='tight')



if __name__ == '__main__':
    run()


