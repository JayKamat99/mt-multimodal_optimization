Include = '../../rai-robotModels/kuka_drake/kuka.g'
Include = '../../rai-robotModels/objects/shelf.g'

Edit shelf { X=<T t(0. -0.8 0) d(90 0 0 1)> gains=[1 1] ctrl_limits=[1 1 1] ctrl_H=10 body }