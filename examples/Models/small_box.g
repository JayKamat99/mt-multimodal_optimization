base_box { X:[.3 .5 0.02] }

small_box (base_box) { A:<t(0 0 0) d(0 0 0 1)>  shape:ssBox mass=.1 size=[0.5 0.2 0.04 0.01] contact, color = [0 1 1] }
shape object(small_box){ shape:marker rel=<T t(0 0 0) d(0 0 0 1)> size=[.1 .1 .1 0] }