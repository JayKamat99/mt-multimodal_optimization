#!/bin/bash
# A bash script to run the benchmark and save the log files

# shellcheck disable=SC2164
cd build
make -j4
cd ../visualize

# ../build/t_manip # run the algorithm on PathOptimizerKOMO and visualize the paths

# Set model here
# MODEL="1_kuka_shelf"
# MODEL="4_kuka_box"
# MODEL="5_disc_obstacle"
MODEL="6_rectangle_opening"


../build/t_manip "../examples/Models/$MODEL.g" "PathOptimizerKOMO" "true" # run the executable
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathOptimizerKOMO.log -d data/Benchmarks/benchmark_multimodal.db # make the database file out of the log file
cp data/Benchmarks/benchmark_multimodal.db $MODEL/benchmark_multimodal.db

# ../build/t_manip "../examples/Models/$MODEL.g" "KOMO" "true" # run the executable
# ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_KOMO.log -d data/Benchmarks/benchmark_KOMO.db # make the database file out of the log file

../build/t_manip "../examples/Models/$MODEL.g" "PathSimplifier" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathSimplifier.log -d data/Benchmarks/benchmark_PathSimplifier.db

../build/t_manip "../examples/Models/$MODEL.g" "RRTstar" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_RRTstar.log -d data/Benchmarks/benchmark_1.db

../build/t_manip "../examples/Models/$MODEL.g" "BITstar" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_BITstar.log -d data/Benchmarks/benchmark_BITstar.db

../build/t_manip "../examples/Models/$MODEL.g" "LBTRRT" "true"
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_LBTRRT.log -d data/Benchmarks/benchmark_LBTRRT.db

# ../build/t_manip "../examples/Models/$MODEL.g" "FMT" "true"
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_FMT.log -d data/Benchmarks/benchmark_FMT.db

# Run the plot_both.py file to visualize the results using the command python3 visualize/data/plot_both.py
# you might need to make a few changes to the code based on the requirements
