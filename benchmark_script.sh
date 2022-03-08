# !/bin/bash
# A bash script to run the benchmark and save the log files

# Remember to make necessary edits to main.cpp before running this script

cd build
make -j4
cd ../IROS

date=$(date +"%F")
time=$(date +"%T")
TIME=$date\_$time

# Set model here
# MODEL="1_kuka_shelf"
MODEL="2_Two_Pandas"
# MODEL="3_TwoMobileManipulators"
# MODEL="4_kuka_box"
# MODEL="5_disc_obstacle"
# MODEL="6_rectangle_opening"
# MODEL="7_disc_rooms"
# MODEL="8_TwoMobileManipulators_hard"
# MODEL="9_TwoMobileRobots_hard"
# MODEL="10_MobileManipulator"

for PLANNER in {"BITKOMO","BITstar","KOMO"}
do
    ../build/t_main "../examples/Models/$MODEL.g" $PLANNER "true"
    python3 ../ompl/scripts/ompl_benchmark_statistics.py Benchmarks/$MODEL/logs/benchmark_$PLANNER.log -d Benchmarks/$MODEL/benchmark_$PLANNER.db # make the database file out of the log file
    cp -r Benchmarks/$MODEL Benchmarks/Records/$MODEL\_$TIME
done