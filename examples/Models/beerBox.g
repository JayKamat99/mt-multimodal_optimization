body box { shape:ssBox, size:[.44 .44 .02 .01], color:[.7 .7 .7] contact}

#beerStationary (box){ shape:cylinder, Q:<t(.0 .0 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(-.14 .0 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(.14 .0 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(.0 -.14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(.0 .14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(-.14 .14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(.14 .14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(.14 -.14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }
beerStationary (box){ shape:cylinder, Q:<t(-.14 -.14 .11) d(0 0 0 1)>, size:[.2 .06], color:[1 1 .4] contact, joint:rigid }

targetspace(box) { Q:<t(.0 .0 .0)> shape:ssBox, size:[.16 .16 .02 .01], color:[.7 .7 .7] }
