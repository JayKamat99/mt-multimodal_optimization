# !/bin/bash
# A bash script to run the benchmark and save the log files

cd build
make -j4
cd ../visualize

date=$(date +"%F")
time=$(date +"%T")
TIME=$date\_$time

# Set model here
# MODEL="1_kuka_shelf"
# MODEL="2_Two_Pandas"
# MODEL="3_TwoMobileManipulators"
# MODEL="4_kuka_box"
# MODEL="5_disc_obstacle"
# MODEL="6_rectangle_opening"
# MODEL="7_disc_rooms"
# ../build/t_main "../examples/Models/$MODEL.g" "FMT" "true" # run the 

for MODEL in {"2_Two_Pandas","3_TwoMobileManipulators"}
# for MODEL in {"1_kuka_shelf","4_kuka_box"}
# for MODEL in {"5_disc_obstacle","7_disc_rooms"}
do
    for PLANNER in {"BITKOMO","KOMO"}
    do
        ../build/t_main "../examples/Models/$MODEL.g" $PLANNER "true" # run the 
    done
    for PLANNER in {"BITKOMO","KOMO","RRTstar","BITstar","LBTRRT","FMT"}
    do 
        ../build/t_main "../examples/Models/$MODEL.g" $PLANNER "true" # run the executable
        python3 ../ompl/scripts/ompl_benchmark_statistics.py data/Benchmarks/$MODEL/logs/benchmark_$PLANNER.log -d data/Benchmarks/$MODEL/benchmark_$PLANNER.db # make the database file out of the log file
    done
    cd ../ompl_benchmark_plotter
    ./ompl_benchmark_plotter.py ../visualize/data/Benchmarks/$MODEL/*.db -s --max-time 180 --max-cost 40
    cd ../visualize
    cp -r data/Benchmarks/$MODEL data/Benchmarks/Records/$MODEL\_$TIME
done