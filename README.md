# BITKOMO: Combining Sampling and Optimization for Fast Convergence in Optimal Motion Planning
BITKOMO, an optimal motion planner, combines Batch Informed Trees (BIT*) and K-th Order Markov Optimizer (KOMO) for fast convergnce. This work is submitted to IROS 2022.
This branch is created to for the readers of our paper to verify the results. 
The benchmark files used for generating the plots can be found in "IROS/Benchmarks". Refer to the text file present in the same folder to reproduce the exact same plots.

## Visualization
Navigate to "ompl_benchmark_plotter" and run:
```
  ./ompl_benchmark_plotter.py pathToDatabaseFile.db -s
```

## Quick Start

```
git clone --recurse-submodules git@github.com:JayKamat99/mt-multimodal_optimization.git
# OR, if you don't have a github account:
git clone --recurse-submodules https://github.com/JayKamat99/mt-multimodal_optimization.git
cd mt-multimodal_optimization

#Move to rai to install all dependencies for rai
cd rai
make -j1 printUbuntuAll    # for your information: what the next step will install
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
cd ..

# Create and move to the build folder
mkdir build
cd build

# Run cmake and then make
cmake .. 
make -j4
```

You are now ready to explore the execulables in the build folder. You might also want to check how our algorithm compares with other planners for that run `./benchmark_script.sh` in the root folder.

The database files will be stored in "IROS/Benchmarks". You might have to create the necessary folders.