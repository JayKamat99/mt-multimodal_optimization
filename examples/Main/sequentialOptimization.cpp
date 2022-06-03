/**
 * @file sequentialOptimization.cpp
 * @author Jay Kamat
 * @brief This code is to run the examples only using KOMO to see where the planner fails 
 *          and what are the advantages of using KOMO.
 * @version 0.1
 * @date 2022-06-02
 * 
 * @copyright Copyright (c) 2022
 * 
**/

#include<KOMO/komo.h>
#include<Kin/viewer.h>

void KOMO_solve(std::vector<std::string> inputs)
{
    // Set filename (Model) and totalPhases
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    // make a KOMO object and write down the whole action sequence
    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(totalPhases+1, 15, 5, 2);
    komo.add_qControlObjective({}, 2);
    komo.add_collision(true, 0.01);


    // Define the KOMO problem by iterating over action the sequence
    int phase = 0;
    while (phase < totalPhases)
    {
        std::string ref1 = inputs.at(2+phase*2), ref2 = inputs.at(3+phase*2);

        if (phase == 0)
            komo.addSwitch_stable(phase+1, phase+2., "", ref1.c_str(),ref2.c_str());
        else if(phase%2 == 0)
            komo.addSwitch_stable(phase+1, phase+2., "", ref1.c_str(),ref2.c_str(),false);
        else
            komo.addSwitch_stable(phase+1, phase+2., "", ref2.c_str(),ref1.c_str(), false);
        komo.addObjective({phase+1.}, FS_distance, {ref1.c_str(),ref2.c_str()}, OT_eq, {1e2});
        komo.addObjective({phase+1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

        if(phase%2 == 0) //pick
        {
            std::cout << "pick" << std::endl;
            komo.addObjective({phase+1.}, FS_scalarProductXX, {ref1.c_str(),ref2.c_str()}, OT_eq, {1e2}, {0.});
        }
        else //place
        {
            std::cout << "place" << std::endl;
            komo.addObjective({phase+1.}, FS_aboveBox, {ref2.c_str(),ref1.c_str()}, OT_ineq, {1e2});
        }
        phase++;
    }

    komo.verbose = 4;
    komo.optimize();
    //  komo.checkGradients();

    komo.view(true, "optimized motion");
    for(uint i=0;i<2;i++) komo.view_play(true);
}

int main(int argc, char ** argv)
{
    // Default inputs
    std::string inputFile = "../examples/Main/manipulationSequence2.txt";

    // Don't like the default inputs? You can input the inputs you want!
    switch(argc) {
        case 2:
        inputFile = argv[1];
    }

    // Read file to get inputs
    std::string line;
    ifstream myfile (inputFile);
    std::vector<std::string> inputs;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            inputs.push_back(line);
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        return 0;
    }

    KOMO_solve(inputs);
}