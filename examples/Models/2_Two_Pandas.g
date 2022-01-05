Include = '../../rai-robotModels/panda/panda.g'
Include = '../../rai-robotModels/kuka_drake/kuka.g'

#Edit panda_link0 { X=<T t(0. -2. 0) d(90 0 0 1)> gains=[1 1] ctrl_limits=[1 1 1] ctrl_H=10 body }
Edit panda_link0 { X=<T t(0. -1. 0) d(90 0 0 1)> }