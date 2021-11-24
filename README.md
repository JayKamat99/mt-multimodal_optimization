# mt-multimodal_optimization
This is my masters thesis project. The aim of the project is to asymptotically enumerate all locally optimal solutions for a manipulation task and hence find the globally optimal solution. 

## Quick Start

```
git clone --recurse-submodules git@github.com:JayKamat99/mt-multimodal_optimization.git
# OR, if you don't have a github account:
git clone --recurse-submodules https://github.com/JayKamat99/mt-multimodal_optimization.git
cd mt-multimodal_optimization

# Create and move to the build folder
mkdir build
cd build

# Run cmake and then make
cmake .. 
make -j4
```

You are now ready to explore the execulables in the build folder. You might also want to check how our algorithm compares with other planners for that run `./benchmark_script.sh` in the root folder.

## Visualization
The database files will be stored in (visualize/data/Benchmarks) You may visualize them by running the python file plot_both.py in (visualize/data) using:
```
python3 plot_both.py
```