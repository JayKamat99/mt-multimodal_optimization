body box { X=<T t(.4 0 .15) d(0 0 0 1)> fixed }

shape(box){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  .15 0.)> size=[.3 .05 .3 .02] contact }
shape(box){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  -.15 0.)> size=[.3 .05 .3 .02] contact }
shape(box){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(.15  0  0.) d(90 0 0 1)> size=[.3 .05 .3 .02] contact }
shape(box){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.15  0  0.) d(90 0 0 1)> size=[.3 .05 .3 .02] contact }
shape(box){ shape:ssBox color=[0 .3 .7] rel=<T t(0. 0  -.15) d(90 1 0 0)> size=[.3 .05 .3 .02] contact }

target(box){ shape:sphere color=[1. 0 0] rel=<T t(0 0 0)> size=[.07] }
