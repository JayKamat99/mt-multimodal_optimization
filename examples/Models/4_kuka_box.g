world {}

Include = '../../rai-robotModels/kuka_drake/kuka.g'
Include = 'box.g'

table (world){ shape:ssBox, Q:<t(0.6 -0.5 .9)>, size:[1. 2. .1 .02], color:[.3 .3 .3 0.5] contact }