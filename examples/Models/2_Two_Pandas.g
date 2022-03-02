world {}

### table

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[3. 2. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

### two pandas

Prefix: "l_"
Include: '../../rai-robotModels/panda/panda.g'

Prefix: "r_"
Include: '../../rai-robotModels/panda/panda.g'

Prefix!

# Fix the joints
Edit l_panda_finger_joint1     { joint:rigid }
Edit l_panda_finger_joint2     { joint:rigid }

Edit r_panda_finger_joint1     { joint:rigid }
Edit r_panda_finger_joint2     { joint:rigid }

Edit l_panda_link0 (table) { Q:<t(-.5 0 .1) d(0 0 0 1)> }
Edit r_panda_link0 (table) { Q:<t( .5 0 .1) d(180 0 0 1)> }

Edit l_panda_link0_1{ color:[.9 .9 0] }
Edit l_panda_link1_1{ color:[.9 .9 0] }
Edit l_panda_link2_1{ color:[.9 .9 0] }
Edit l_panda_link3_1{ color:[.9 .9 0] }
Edit l_panda_link4_1{ color:[.9 .9 0] }
Edit l_panda_link5_1{ color:[.9 .9 0] }
Edit l_panda_link6_1{ color:[.9 .9 0] }
Edit l_panda_link7_1{ color:[.9 .9 0] }
Edit l_panda_hand_1{ color:[.9 .9 0] }