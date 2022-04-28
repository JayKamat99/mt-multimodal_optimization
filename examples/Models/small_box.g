base_box { X:[.6 .4 0.02] }

small_box (base_box) { A:<t(0 0 0) d(90 0 0 1)>  shape:box mass=.1 size=[0.2 0.2 0.04], contact, color = [0 1 1] }
shape object(small_box){ shape:marker rel=<T t(0 0.12 0)> size=[.1 .1 .1 0] }
