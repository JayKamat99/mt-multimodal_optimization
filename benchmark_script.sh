#!/bin/bash
# A bash script to run the benchmark and save the log files

# shellcheck disable=SC2164
cd build
make -j4
cd ../visualize

# ../build/t_manip # run the algorithm on PathOptimizerKOMO and visualize the paths

# Set model here
# MODEL="../examples/Models/1_kuka_shelf.g"
# MODEL="../examples/Models/2_Two_Pandas.g"
# MODEL="../examples/Models/3_TwoMobileManipulators.g"
MODEL="../examples/Models/4_kuka_box.g"
# MODEL="../examples/Models/5_disc_obstacle.g"
# MODEL="../examples/Models/6_rectangle_opening.g"
# MODEL="../examples/Models/7_disc_rooms.g"

# ../build/t_manip $MODEL "PathOptimizerKOMO" "true" # run the executable
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathOptimizerKOMO.log -d data/Benchmarks/benchmark_multimodal.db # make the database file out of the log file

../build/t_main $MODEL "PKOMO" "true" # run the executable
python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PKOMO.log -d data/Benchmarks/benchmark_multimodal.db # make the database file out of the log file

# ../build/t_manip $MODEL "KOMO" "true" # run the executable
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_KOMO.log -d data/Benchmarks/benchmark_KOMO.db # make the database file out of the log file

#../build/t_manip $MODEL "PathSimplifier" "true"
#python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_PathSimplifier.log -d data/Benchmarks/benchmark_PathSimplifier.db

# ../build/t_manip $MODEL "RRTstar" "true"
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_RRTstar.log -d data/Benchmarks/benchmark_1.db

# ../build/t_manip $MODEL "BITstar" "true"
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_BITstar.log -d data/Benchmarks/benchmark_BITstar.db

# ../build/t_manip $MODEL "LBTRRT" "true"
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_LBTRRT.log -d data/Benchmarks/benchmark_LBTRRT.db

# ../build/t_manip $MODEL "FMT" "true"
# python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/benchmark_FMT.log -d data/Benchmarks/benchmark_FMT.db

# Run the plot_both.py file to visualize the results using the command python3 visualize/data/plot_both.py
# you might need to make a few changes to the code based on the requirements
