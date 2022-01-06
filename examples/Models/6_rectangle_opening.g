Include = 'opening.g'

world {}

mobileBase (world) { joint:transXYPhi, ctrl_H:0.01 }
bot (mobileBase) { rel = <t(1 0 0) d(90 0 0 1)>  shape:box mass=.1 size=[.1 .25 .04] contact }
#eye (mobileBase) { X = <t(0.08 0 0) d(0 1 0 0)> shape:cylinder  size=[0.02 0.02] color:[1 0 0]}

#target (maze) {X=<T t(.8  -0.3  0.)> shape:cylinder  size=[0.02 0.02] color:[1 0 0]}