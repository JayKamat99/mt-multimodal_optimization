#include <ompl/geometric/PathOptimizer.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>

namespace ob = ompl::base;

void ompl::geometric::PathOptimizer::setSI(base::SpaceInformationPtr si)
{
    si_ = std::move(si);
}

void ompl::geometric::PathOptimizer::displayPath(PathGeometric &path, const std::string &txt) const
{
	std::cout << "We are in PathOptimizer!!" << std::endl;
    arrA configs;
	//To copy the path to arrA Configs from states.
	const base::StateSpace *space(si_->getStateSpace().get());
	for (auto state : path.getStates())
	{
		arr config;
		std::vector<double> reals;
		space->copyToReals(reals, state);
		for (double r : reals)
		{
			config.append(r);
		}
		configs.append(config);
	}

    // Create a text string, which is used to output the text file
    std::string filename;
    ifstream MyReadFile("/home/jay/git/optimization-course/examples/Models/Configuration.txt");
    getline (MyReadFile, filename);
    MyReadFile.close(); 

    // setup KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);

    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 1, 2.);

    komo.initWithWaypoints(configs, path.getStateCount(), false);

    for (int i=0; i<=7; i++){
        std::string frame = "iiwa_link_"+std::to_string(i)+"_1";
        if (txt == "Repeated")
            komo.pathConfig.getFrame(frame.c_str())->setColor({1,1,0});
        else if (txt == "New Solution")
            komo.pathConfig.getFrame(frame.c_str())->setColor({0,1,0});
        else if (txt == "Invalid")
            komo.pathConfig.getFrame(frame.c_str())->setColor({1,0,0});
    }
    for (int j=0; j<10 ;j++)
        komo.displayPath(txt.c_str(), false);
}