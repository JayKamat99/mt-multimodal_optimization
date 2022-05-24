base_bot { X:[0 0 0.02] }

#bot (base_bot) { joint:transXYPhi Q:<T t(0 0 0) d(90 0 0 1)>  shape:cylinder mass=.1 size=[0.04 0.1] contact color=[ 0 0 1] }
bot (base_bot) { joint:transXYPhi Q:<T t(0 0 0) d(90 0 0 1)>  shape:sphere mass=.1 size=[0.1] color=[ 0 0 1], contact }
endeff (bot){ shape:marker Q:<T t(-0.12 0 0)> size=[.1 .1 .1 0] }

#eye (base) {A:<t(0.08 0 0) d(0 1 0 0)> shape:cylinder  size=[0.02 0.02] color:[1 0 0]} shape:box mass=.1 size=[0.2 0.2 0.2] contact, color = [0 1 1] }