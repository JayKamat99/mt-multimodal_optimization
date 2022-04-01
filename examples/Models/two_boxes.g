body box { X=<T t(.4 0 .175) d(0 0 0 1)> fixed }
mobileBase (box) { joint:transXYPhi }

shape(box){ shape:ssBox color=[.7 .3 0 0.5] rel=<T t(0  0 0) d(45 1 1 0)> size=[.3 .3 .35 .02] contact }
shape(mobileBase){ shape:sphere color=[0 .3 .7 0.5] rel=<T t(0  0 0)> size=[.1] contact }