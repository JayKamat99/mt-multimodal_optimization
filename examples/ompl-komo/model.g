

world{}

box_part1(world) { joint:free shape:box, size:[.3,.1,.1]  , color:[.2,.2,.2] contact }
box_part2(box_part1) {Q:<t(.1 0 .22)> shape:box size:[.1,.1,.3], color:[.1,.1,.1] contact  }
box_part3(box_part1) {Q:<t(.1 0 .41)> shape:box size:[.12,.12,.05], color:[.3,.1,.1] contact  }
box_part4(box_part1) {Q:<t(-.2 0 0)> shape:box size:[.05,.12,.12], color:[.3,.1,.1] contact  }

box_frame { shape:marker, size:[.4]  , color:[1,0,0] }

obstacle { X:<t(.5 .5 .5)> shape:sphere  size:[.2]  , color:[.5,.5,0] contact }
obstacle2 { X:<t(1 1 1)> shape:sphere  size:[.2]  , color:[.5,.5,0] contact }


box_part1_goal { X:<t(1.5 1.5 1.5)>  shape:box, size:[.3,.1,.1]  , color:[1,0.2,0.2, 0.5] }
box_part2_goal(box_part1_goal) {Q:<t(.1 0 .22)>  shape:box size:[.1,.1,.3], color:[1,.2,.2, 0.5]  }
box_part3(box_part1_goal) {Q:<t(.1 0 .41)> shape:box size:[.12,.12,.05], color:[1,.2,.2, .5] }
box_part4(box_part1_goal) {Q:<t(-.2 0 0)> shape:box size:[.05,.12,.12], color:[1,.2,.2, .5] }
box_goal_frame { shape:marker, size:[.4]  , color:[1,0,0] }



