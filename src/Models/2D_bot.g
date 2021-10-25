world {}
base { A:<t(1 0 0) d(90 0 0 1)>  shape:cylinder mass=.1 size=[0 0 0.01 0.1] contact }
eye (base) {X = <t(0.08 0 0) d(0 1 0 0)> shape:cylinder  size=[0.02 0.02] color:[1 0 0]}
joint (world base) { joint:transXYPhi, A:<t(0 0 0) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 0)> }
Obstacle1 { X = <t(0.5 0.5 0) d(90 0 0 1)>  shape:cylinder mass=.1 size=[0 0 0.01 0.15] contact, color:[0.5 0.5 0.5]}
