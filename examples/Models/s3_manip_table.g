world {}

Include: '../../rai-robotModels/panda/panda.g'
Edit panda_link0{ X:<t(-.3 0 .65)> }

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[1. 1. .1 .02], color:[.3 .3 .3]
}

table2 (world){
    shape:ssBox, Q:<t(0.01 0. .7)>, size:[.4 .4 .1 .02], color:[0. 0. .3] contact
}

obstacle (table2){
    shape:ssBox, Q:<t(.2 0 .2) d(90 0 1 0)>, size:[.5 .5 .01 .02], color:[0. 0. .3 0.5] contact
}
obstacle (table2){
    shape:ssBox, Q:<t(.1 .2 .2) d(90 1 0 0)>, size:[.25 .5 .01 .02], color:[0. 0. .3 0.5] contact
}
obstacle (table2){
    shape:ssBox, Q:<t(.1 -.2 .2) d(90 1 0 0)>, size:[.25 .5 .01 .02], color:[0. 0. .3 0.5] contact
}


box (table){
  shape:ssBox, Q:<t(.4 .1 .08) d(120 0 0 1)>, size:[.2 .06 .06 .02], color:[.4 .8 .4] contact
  joint:rigid
}

# Fix the joints
Edit panda_hand>panda_finger_joint1(panda_hand_joint) 	{  Q:<0 0.05 0.0584 0.707107 0 0 .707107> }
Edit panda_hand>panda_finger_joint2(panda_hand_joint) 	{  Q:<0 -0.05 0.0584 0.707107 0 0 -.707107> }

Edit panda_finger_joint1     {q:.05 joint:rigid }
Edit panda_finger_joint2     {q:.05 joint:rigid }

gripper (panda_joint7){
    shape:marker, size:[.03], color:[.9 .9 .5],
    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>  
}

#target (table) { Q:<t(0.1 0 0.3)> shape:marker size=[.2] color=[0 .5 0] }