four_rooms { X:[0 0 0.02] }

side (four_rooms) { Q:<t(1 0 0) d(90 0 0 1)>  shape:ssBox mass=.1 size=[2.1 .1 0.04 0.01] contact }
side (four_rooms) { Q:<t(-1 0 0) d(90 0 0 1)>  shape:ssBox mass=.1 size=[2.1 .1 0.04 0.01] contact }
side (four_rooms) { Q:<t(0 1 0) d(0 0 0 1)>  shape:ssBox mass=.1 size=[2.1 .1 0.04 0.01] contact }
#side (four_rooms) { Q:<t(0 -1 0) d(0 0 0 1)>  shape:ssBox mass=.1 size=[2.1 .1 0.04 0.01] contact }
side (four_rooms) { Q:<t(.6 0 0) d(20 0 0 1)>  shape:ssBox mass=.1 size=[.9 .1 0.04 0.01] contact }
side (four_rooms) { Q:<t(-.6 0 0) d(-20 0 0 1)>  shape:ssBox mass=.1 size=[.9 .1 0.04 0.01] contact }