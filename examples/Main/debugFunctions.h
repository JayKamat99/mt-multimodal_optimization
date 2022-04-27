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