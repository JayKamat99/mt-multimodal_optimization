Include = '/home/jay/git/optimization-course/rai-robotModels/kuka_drake/kuka.g'

ball {shape:sphere, mass:1, size:[.05], color:[0 1 0], X = <t(0.5 0.5 0.5)> }

Obstacle1 {shape:sphere, mass:1, size:[.1] contact, color:[1 0 0], X = <t(.25 .5 .75)> }

Obstacle2 {shape:sphere, mass:1, size:[.1] contact, color:[1 0 0], X = <t(.25 -0.05 .75)> }

Obstacle3 {shape:sphere, mass:1, size:[.1] contact, color:[1 0 0], X = <t(0.5 -0.2 0.4)> }
