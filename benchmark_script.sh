#!/bin/bash
# A bash script to run the benchmark and save the log files

# shellcheck disable=SC2164
cd build
make -j4
cd ../visualize

# ../build/t_manip # run the algorithm on PathOptimizerKOMO and visualize the paths

../build/t_manip "../examples/Models/2D_arm.g" "PathOptimizerKOMO" "true" # run the executable
# ../build/t_manip "../examples/Models/kuka_multimodal.g" "PathOptimizerKOMO" "true" # run the executable
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathOptimizerKOMO.log -d data/Benchmarks/benchmark_multimodal.db # make the database file out of the log file

# ../build/t_manip "../examples/Models/kuka_multimodal.g" "KOMO" "true" # run the executable
# ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_KOMO.log -d data/Benchmarks/benchmark_KOMO.db # make the database file out of the log file

../build/t_manip "../examples/Models/2D_arm.g" "PathSimplifier" "true"
# ../build/t_manip "../examples/Models/kuka_multimodal.g" "PathSimplifier" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathSimplifier.log -d data/Benchmarks/benchmark_PathSimplifier.db # make the database file out of the log file

../build/t_manip "../examples/Models/2D_arm.g" "RRTstar" "true"
# ../build/t_manip "../examples/Models/kuka_multimodal.g" "RRTstar" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_RRTstar.log -d data/Benchmarks/benchmark_1.db # make the database file out of the log file

# Run the plot_both.py file to visualize the results using the command python3 visualize/data/plot_both.py
# you might need to make a few changes to the code based on the requirements
