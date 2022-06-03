world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[1. 1. .1 .02], color:[.3 .3 .3]
}

winebase (table){
  shape:cylinder, Q:<t(.4 .1 .14) d(120 0 0 1)>, size:[.16 .06], color:[.4 .8 .4] contact
  joint:rigid
}
winenose (winebase){
  shape:cylinder, Q:<t(0 0 .13) d(120 0 0 1)>, size:[.1 .03], color:[.4 .8 .4] contact
}

Include: 'beerBox.g'
Edit box { X:<t(.03 0. .66)>}

Include: '../../rai-robotModels/panda/panda.g'
Edit panda_link0{ X:<t(-.3 0 .66)> }

# Fix the joints
Edit panda_hand>panda_finger_joint1(panda_hand_joint) 	{  Q:<0 0.05 0.0584 0.707107 0 0 .707107> }
Edit panda_hand>panda_finger_joint2(panda_hand_joint) 	{  Q:<0 -0.05 0.0584 0.707107 0 0 -.707107> }

Edit panda_finger_joint1     {q:.05 joint:rigid }
Edit panda_finger_joint2     {q:.05 joint:rigid }

gripper (panda_joint7){
    shape:marker, size:[.03], color:[.9 .9 .5],
    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>  
}