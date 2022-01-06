body openinig { X=<T t(.4 0 0) d(0 0 0 1)> fixed }

shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(0  .4 0.)> size=[1.2 .1 .1 .02] contact }
shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(0  -.4 0.)> size=[1.2 .1 .1 .02] contact }
shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(.6  0  0.) d(90 0 0 1)> size=[.8 .1 .1 .02] contact }
shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(-.6  0  0.) d(90 0 0 1)> size=[.8 .1 .1 .02] contact }
shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(0  0.25  0.) d(90 0 0 1)> size=[.35 .1 .1 .02] contact }
shape(openinig){ shape:ssBox color=[0 .3 .7] rel=<T t(0  -0.25  0.) d(90 0 0 1)> size=[.35 .1 .1 .02] contact }