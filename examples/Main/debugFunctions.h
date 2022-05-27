void checkCollision(rai::Configuration &C)
{
    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1, 1, 1, 1);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
    komo.run_prepare(0);

    auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);

    ObjectiveTypeA ot;
    nlp->getFeatureTypes(ot);
    std::cout << ot << std::endl;

    arr phi;
    arr x_query = arr{0, 0, 0};
    nlp->evaluate(phi, NoArr, x_query);
    std::cout << phi << std::endl;

    x_query = arr{0.5, 0.5, 0.5};
    nlp->evaluate(phi, NoArr, x_query);
    std::cout << phi << std::endl;

    size_t N = 20;
    for (size_t i = 0; i < N; i++) {
        arr pose = rand(3);

        nlp->evaluate(phi, NoArr, pose);
        std::cout << pose << " " << phi << std::endl;
        komo.view(true);
    }
}

void calculateCollision(rai::Configuration C)
{
    KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);
    auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
    ObjectiveTypeA ot;
    nlp->getFeatureTypes(ot);
    arr phi;
    arr config = C.getJointState();
    nlp->evaluate(phi, NoArr, config);
    std::cout << config << " " << phi(0) << std::endl;
}

void visualizeModel(std::string filename = "../examples/Models/s1_2d_manip.g" )
{
    rai::Configuration C;
    C.addFile(filename.c_str());

    // visualize environment
    C.watch(true);
}

// void visualizeSequence(std::string filename, std::vector<arrA> &trajectories)
// {
//     rai::Configuration C;
//     C.addFile(filename.c_str());
//     KOMO komo;
//     komo.verbose = 0;
//     komo.setModel(C, true);
    
//     arrA configs = trajectories.at(0);
//     komo.setTiming(2, configs.N, 5., 2);
// 	komo.add_qControlObjective({}, 1, 1.);

//     //use configs to initialize with waypoints
// 	komo.initWithWaypoints(configs, configs.N, false);
//     komo.run_prepare(0);
// 	komo.plotTrajectory();

// 	rai::ConfigurationViewer V;
// 	V.setPath(C, komo.x, "result", true);
//     V.playVideo();
// }

void runOnlyKOMO()
{
    rai::Configuration C("../examples/Models/model2.g");

    //  rai::ConfigurationViewer V;
    //  V.setConfiguration(C, "initial model", false);

    KOMO komo;

    komo.setModel(C, false);
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
    komo.addSquaredQuaternionNorms();

    //grasp
    komo.addSwitch_stable(1., 2., "table", "gripper", "box");
    komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
    komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
    komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

    // komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    // komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

    //place
    komo.addSwitch_stable(2., -1., "gripper", "table", "box", false);
    // komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
    komo.addObjective({2.}, FS_distance, {"box", "table"}, OT_eq/* , {1e2}, {0,0,.08} */); //arr({1,3},{0,0,1e2})
    komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

    komo.verbose = 4;
    komo.optimize();
    //  komo.checkGradients();

    komo.view(true, "optimized motion");
    for(uint i=0;i<2;i++) komo.view_play(true);
    // V.setPath(komo.getPath_frames(), "optimized motion", true);
    // for(uint i=0;i<2;i++) V.playVideo(true);
}


void sampleGoalsandDisplay(rai::Configuration C, std::string &ref1, std::string &ref2, transition transition_ = pick)
{
    arr goalConfig;
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C);
    arr tergetReference;

    komo.setTiming(1,1,1,1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
    bool feasible = false;
    while (!feasible)
    {
        arr pos_ref2 = C.getFrameState(C.getFrameIDs({"object"/* (rai::String)ref2 */})).resize(3);
        std::cout << "pos_ref2: " << pos_ref2 << "\nCSpace_dim: " << C.getJointStateDimension() << std::endl; 
        arr randomConfig_ = pos_ref2 - 0.5 + 2*0.5*rand(C.getJointStateDimension());
        komo.initWithConstant(randomConfig_);
        std::cout << komo.getConfiguration_q(0)<< std::endl;
        std::cout << "ref1, ref2: " << ref2.c_str() <<  ", " <<ref1.c_str() << std::endl;
        komo.addObjective({1,1},FS_distance,{ref2.c_str(),ref1.c_str()}, OT_eq);
        // komo.addObjective({1,1},FS_distance,{ref2.c_str(),ref1.c_str()}, OT_eq, {}, {0,0,0});
        komo.run_prepare(0);
        // komo.animateOptimization = 2;
        komo.optimize();
        komo.view(true);
        std::cout << komo.x << std::endl;
        feasible = true;
    }
    
    goalConfig.append(komo.x);
    // komo.view(true);
}

// void sequentialPlan(std::string filename = "../examples/Models/s1_2d_manip.g")
// {
//     rai::Configuration C;
//     C.addFile(filename.c_str());

//     // visualize environment
//     C.watch(true);

//     std::string ref1 = "endeff", ref2 = "object";
//     arr pickConfig = getGoalConfig(C, ref1, ref2);
//     std::cout << "pickConfig: " << pickConfig << std::endl;

//     // checkCollision(C);
//     arrA pickTrajectory = solveMotion(C, pickConfig);
//     visualizePath(C, pickTrajectory);

//     // set the joint state for the keyframe, and attach the frame
//     C.setJointState(pickTrajectory.last());
//     C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str()));
//     C.watch(true);

//     ref1 = "object", ref2 = "target";
//     arr placeConfig = getGoalConfig(C, ref1, ref2);
//     std::cout << "placeConfig: " << placeConfig << std::endl;

//     // checkCollision(C);
//     arrA placeTrajectory = solveMotion(C, placeConfig);
//     visualizePath(C, placeTrajectory);
//     C.setJointState(placeTrajectory.last());
//     C.watch(true);

//     C.attach(C.getFrame("world"), C.getFrame(ref1.c_str()));
//     C.watch(true);

//     C.setJointState({0,0,0});
//     C.watch(true);
// }