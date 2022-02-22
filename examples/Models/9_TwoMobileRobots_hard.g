World {}

Prefix: "l_"
Include: 'MovingDisc.g'
Prefix: "r_"
Include: 'MovingDisc.g'

Edit l_base{ X:<t(.5 0 0) d(90 0 0 1)> }
Edit r_base{ X:<t(-.5 0 0) d(-90 0 0 1)> }

l_target (World) {shape:sphere X:<t(1 0 0)> color:[1 0 0] size=[0.1] }
r_target (World) {shape:sphere X:<t(-1 0 0)> color:[1 0 0] size=[0.1] }

#obstacle (World) { shape:cylinder, Q:<t(0 0.1 0)>, size:[1 .1], color:[.3 0 .3] contact }
#obstacle (World) { shape:cylinder, Q:<t(0.5 0.7 0)>, size:[1 .1], color:[.3 0 .3] contact }
#obstacle (World) { shape:cylinder, Q:<t(-0.4 -0.8 0)>, size:[1 .1], color:[.3 0 .3] contact }
#obstacle (World) { shape:cylinder, Q:<t(-0.3 -0.3 0)>, size:[1 .1], color:[.3 0 .3] contact }
#obstacle (World) { shape:cylinder, Q:<t(-0.5 0.8 0)>, size:[1 .1], color:[.3 0 .3] contact }

obstacleBlock (World) { shape:ssBox, X:<t(0. 0. 0.25) d(0 0 0 1)>, size:[.1 1. 0.5 .02], color:[.3 0 .3] contact }
obstacleBlock (World) { shape:ssBox, X:<t(0. 0.5 0.25) d(90 0 0 1)>, size:[.1 .4 0.5 .02], color:[.3 0 .3] contact }
obstacleBlock (World) { shape:ssBox, X:<t(0. -0.5 0.25) d(90 0 0 1)>, size:[.1 .4 0.5 .02], color:[.3 0 .3] contact }

# Obstacle box
box (World) { shape:ssBox, Q:<t(0  1 0.5)>, size:[3 .05 1 .02], color:[.3 0 .3 .2] contact }
box (World) { shape:ssBox, Q:<t(0  -1 0.5)>, size:[3 .05 1 .02], color:[.3 0 .3 .2] contact }
box (World) { shape:ssBox, Q:<t(1.5  0 0.5)>, size:[.05 2 1 .02], color:[.3 0 .3 .2] contact }
box (World) { shape:ssBox, Q:<t(-1.5  0 0.5)>, size:[.05 2 1 .02], color:[.3 0 .3 .2] contact }
#box (world) { shape:ssBox color=[0 .3 .7 0.2] rel=<T t(0  .25 0.)> size=[.5 .05 .3 .02] contact }
