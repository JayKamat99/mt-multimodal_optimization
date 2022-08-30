# !/bin/bash
# A bash script to run the benchmark and save the log files

cd build
make -j4
cd ../visualize

date=$(date +"%F")
time=$(date +"%T")
TIME=$date\_$time

# Set FILE here
FILE="manipulationSequence"
# for FILE in {"manipulationSequence","manipulationSequence2"}
# do
    PLANNER="sktp"
    # PLANNER="SequentialKOMO"
    # for PLANNER in {"RRTstar","LBTRRT"}
    # do
        ../build/s_main "../examples/sequential/$FILE.txt" true $PLANNER
        python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Sequential/Benchmarks/$FILE/logs/benchmark_$PLANNER.log -d data/Sequential/Benchmarks/$FILE/benchmark_$PLANNER.db # make the database file out of the log file
        # cp -r data/Sequential/Benchmarks/$FILE data/Sequential/Benchmarks/Records/$FILE\_$TIME

    # done
    cd ../ompl_benchmark_plotter
    ./ompl_benchmark_plotter.py ../visualize/data/Sequential/Benchmarks/$FILE/*.db -s --max-time 180 --max-cost 40 --title-name $FILE 
    cd ../visualize
# done