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

Edit l_panda_link0 (table) { Q:<t(-.5 0 .1) d(0 0 0 1)> }
Edit r_panda_link0 (table) { Q:<t( .5 0 .1) d(180 0 0 1)> }