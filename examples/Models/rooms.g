body rooms { X=<T t(.4 0 0) d(0 0 0 1)> fixed }

shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  .4 0.)> size=[1.3 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  -.4 0.)> size=[1.3 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(0  0  0.) d(90 0 0 1)> size=[.9 .15 .05 .02] contact }
#shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.6  0  0.) d(90 0 0 1)> size=[.9 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.6  0.3  0.) d(90 0 0 1)> size=[.25 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(-.6  -0.3  0.) d(90 0 0 1)> size=[.25 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(.6  0.3  0.) d(90 0 0 1)> size=[.25 .15 .05 .02] contact }
shape(rooms){ shape:ssBox color=[0 .3 .7 0.5] rel=<T t(.6  -0.3  0.) d(90 0 0 1)> size=[.25 .15 .05 .02] contact }