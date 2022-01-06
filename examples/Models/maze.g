body maze { X=<T t(.4 0 0) d(0 0 0 1)> fixed }

shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  .4 0.)> size=[1.2 .05 .05 .02] contact }
shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  -.4 0.)> size=[1.2 .05 .05 .02] contact }
shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(.6  0  0.) d(90 0 0 1)> size=[.8 .05 .05 .02] contact }
shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.6  0  0.) d(90 0 0 1)> size=[.8 .05 .05 .02] contact }
shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.2  0.15  0.) d(90 0 0 1)> size=[.5 .05 .05 .02] contact }
shape(maze){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(.2  -0.15  0.) d(90 0 0 1)> size=[.5 .05 .05 .02] contact }