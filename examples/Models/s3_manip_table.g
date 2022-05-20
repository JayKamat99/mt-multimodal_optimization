world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[1. 1. .1 .02], color:[.3 .3 .3] contact
}

table2 (world){
    shape:ssBox, Q:<t(0 0. .7)>, size:[.5 .5 .1 .02], color:[0. 0. .3] contact
}

box (table){
  shape:ssBox, Q:<t(.4 .1 .08) d(120 0 0 1)>, size:[.2 .06 .06 .02], color:[.4 .8 .4] contact
  joint:rigid
}


Include: '../../rai-robotModels/panda/panda.g'
Edit panda_link0{ X:<t(-.3 0 .65)> }

gripper (panda_joint7){
    shape:marker, size:[.03], color:[.9 .9 .5],
    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>  
}

#target (table) { Q:<t(0 0 0.08)> shape:marker size=[.2] color=[0 .5 0] }