world{}

Base (world) {X=<t(0 0 0) d(90 0 1 0)> }
arm1 { shape:capsule, mass:1, size:[.2 .02] contact}
arm2 { shape:capsule, mass:1, size:[.2 .02] contact}
arm3 { shape:capsule, mass:1, size:[.2 .02] contact}
arm4 { shape:capsule, mass:1, size:[.2 .02] contact}
arm5 { shape:capsule, mass:1, size:[.2 .02] contact}

joint1 (Base arm1) { joint:hingeX, A:<t(0 0 0) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 .25) >}
joint2 (arm1 arm2) { shape:sphere, size:[0.05], joint:hingeX, A:<t(0 0 .15) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 .15) > }
joint3 (arm2 arm3) { shape:sphere, size:[0.05], joint:hingeX, A:<t(0 0 .15) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 .15) > }
joint4 (arm3 arm4) { shape:sphere, size:[0.05], joint:hingeX, A:<t(0 0 .15) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 .15) > }
joint5 (arm4 arm5) { shape:sphere, size:[0.05], joint:hingeX, A:<t(0 0 .15) d(0 1 0 0)>, Q:<d(0 1 0 0)>, B:<t(0 0 .15) > }

gripper (arm5){
    shape:marker, size:[.5], color:[.9 .9 .5],
    Q:<t(0 0 .1)>  
}

goal (world) {X=<t(0 -1 0) d(90 0 1 0)> }

#ball {shape:sphere, mass:1, size:[.05] contact, color:[0 1 0], X = <t(0 -1 0)> }

Obstacle {shape:sphere, mass:1, size:[.15] contact, color:[1 0 0], X = <t(0 -0.8 0)> }

#Obstacle {shape:sphere, mass:1, size:[.15] contact, color:[1 0 0], X = <t(0.6 -0.6 0)> }
