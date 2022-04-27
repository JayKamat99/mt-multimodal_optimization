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

void sequentialPlan(std::string filename = "../examples/Models/s1_2d_manip.g")
{
    rai::Configuration C;
    C.addFile(filename.c_str());

    // visualize environment
    C.watch(true);

    std::string ref1 = "endeff", ref2 = "object";
    arr pickConfig = getTargetConfig(C, ref1, ref2);
    std::cout << "pickConfig: " << pickConfig << std::endl;

    // checkCollision(C);
    arrA pickTrajectory = solveMotion(C, pickConfig);
    visualizePath(C, pickTrajectory);

    // set the joint state for the keyframe, and attach the frame
    C.setJointState(pickTrajectory.last());
    C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str()));
    C.watch(true);

    ref1 = "object", ref2 = "target";
    arr placeConfig = getTargetConfig(C, ref1, ref2);
    std::cout << "placeConfig: " << placeConfig << std::endl;

    // checkCollision(C);
    arrA placeTrajectory = solveMotion(C, placeConfig);
    visualizePath(C, placeTrajectory);
    C.setJointState(placeTrajectory.last());
    C.watch(true);

    C.attach(C.getFrame("world"), C.getFrame(ref1.c_str()));
    C.watch(true);

    C.setJointState({0,0,0});
    C.watch(true);
}