Include = 'rooms.g'

world {}

mobileBase (world) { joint:transXYPhi, ctrl_H:0.01 }
bot (mobileBase) { rel = <t(1 0 0) d(90 0 0 1)>  shape:cylinder mass=.1 size=[0 0 0.01 0.1] contact }
eye (mobileBase) { X = <t(0.08 0 0) d(0 1 0 0)> shape:cylinder  size=[0.02 0.02] color:[1 0 0]}