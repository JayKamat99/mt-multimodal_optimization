base_origin { X:[0 0 0] }

base (base_origin){
    joint:transXYPhi,
    shape:cylinder size:[.1 .1], contact }

Include: '../../rai-robotModels/panda/panda.g'

Edit panda_link0 (base)  { Q:<t(0 0 .05) d(90 0 0 1)> }
