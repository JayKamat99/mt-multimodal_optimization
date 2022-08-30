#include <path/SequentialKOMO.h>
#include <thread>
#include <chrono>

#define PI 3.14

struct ValidityCheckWithKOMO
{
    private:
        int C_Dimension;
        std::shared_ptr<KOMO> komo;
        KOMO::Conv_KOMO_SparseNonfactored nlp;
    public:
    ValidityCheckWithKOMO(std::shared_ptr<KOMO> &komo) : komo(komo) , nlp(*komo){
        C_Dimension = nlp.getDimension();
    }
    bool check(const ompl::base::State *state)
    {
        const auto *State = state->as<ompl::base::RealVectorStateSpace::StateType>();

        arr x_query;
        for (unsigned int i = 0; i < C_Dimension; i++)
        {
            x_query.append((*State)[i]);
        }
        arr phi;
        nlp.evaluate(phi, NoArr, x_query);
        return std::abs(phi(0)) < 0.01;
    }
};

ompl::geometric::SequentialKOMO::SequentialKOMO(const base::SpaceInformationPtr &si, std::string name) : base::Planner(si, name)
{
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
	C_Dimension = si_->getStateDimension();
}

void ompl::geometric::SequentialKOMO::setup()
{
	Planner::setup();
	// See if we have an optimization objective
	if (!Planner::pdef_->hasOptimizationObjective())
	{
		OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
					Planner::getName().c_str());
		Planner::pdef_->setOptimizationObjective(
			std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
	}
	bestCost = std::numeric_limits<double>::infinity();
}

ompl::geometric::SequentialKOMO::~SequentialKOMO()
{
	freeMemory();
}

void ompl::geometric::SequentialKOMO::freeMemory()
{
}

void ompl::geometric::SequentialKOMO::clear()
{
	// std::cout << "Clear called" << std::endl;
	Planner::clear();
	bestCost = std::numeric_limits<double>::infinity();
	iteration = 0;
}

std::shared_ptr<KOMO> ompl::geometric::SequentialKOMO::setupKOMO()
{
	// initialize the configuration
	rai::Configuration C(inputs.at(0).c_str());
	startConfig = C.getJointState();

	int totalPhases = stoi(inputs.at(1));

	auto komo_ = std::make_shared<KOMO>();
	komo_->setModel(C, true);
	komo_->setTiming(totalPhases, stepsPerPhase, 5, 2);
	komo_->verbose = 0;

	int phase = 0;
	while (phase < totalPhases)
	{
		std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

		if (phase == 0)
			komo_->addSwitch_stable(1, 2., "", ref1.c_str(), ref2.c_str());
		else if (phase % 2 == 0)
			komo_->addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str(), false);
		else
			komo_->addSwitch_stable(phase + 1, phase + 2., "", ref2.c_str(), ref1.c_str(), false);
		komo_->addObjective({phase + 1.}, FS_distance, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2});
		komo_->addObjective({phase + 1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

		if (phase % 2 == 0) // pick
		{
			komo_->addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
		}
		else // place
		{
			komo_->addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
		}
		phase++;
	}

	komo_->add_qControlObjective({}, 1);
	komo_->add_collision(true, 0.01);

	return komo_;
}

double ompl::geometric::SequentialKOMO::calcCost(arrA configs)
{
	double cost = 0;
	// std::cout << configs.N/stepsPerPhase << std::endl;
	for (int currentPhase=0; currentPhase<configs.N/stepsPerPhase; currentPhase++)
	{
		std::cout << "Enters here" << std::endl;
		// setup space, setup optimization objective, calculate cost
		// Get configuration
		rai::Configuration C(inputs.at(0).c_str());
		for(int phase = 0; phase <currentPhase; phase++)
		{
			std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
			C.setJointState(configs(phase*stepsPerPhase).resize(C_Dimension));
			if (phase % 2 == 0)
				C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
			else
				C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
			phase++;
		}

		// setup KOMO and collision checking
		auto komo = std::make_shared<KOMO>();
		komo->setModel(C, true);
		komo->setTiming(1, 1, 1, 1);
		komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
		komo->run_prepare(2);

		//Construct the state space we are planning in
		auto space(std::make_shared<ompl::base::RealVectorStateSpace>(C_Dimension));

		ompl::base::RealVectorBounds bounds(C_Dimension);
		bounds.setLow(-PI);
		bounds.setHigh(PI);
		space->setBounds(bounds);

		//create space information pointer
		ompl::base::SpaceInformationPtr subspace_si(std::make_shared<ompl::base::SpaceInformation>(space));
		auto checker = std::make_shared<ValidityCheckWithKOMO>(komo);

		subspace_si->setStateValidityChecker([checker](const ompl::base::State *state) {
			return checker->check(state);
		});

		subspace_si->setup();

		/* Convert arrA to PathGeometric to get the cost and to check validity */
        std::vector<const base::State*> states;
        for (int i=0; i<configs.N; i++)
        {
            std::vector<double> reals;
			arr config = configs(i).resize(C_Dimension);
            for (double r : config){
                reals.push_back(r);
            }
            ompl::base::State* state = subspace_si->allocState();
            space->copyFromReals(state, reals);
            states.push_back(state);
        }

		auto path = std::make_shared<geometric::PathGeometric>(subspace_si, states);

        if (path->check()){ 
			cost += path->cost(pdef_->getOptimizationObjective()).value();
			std::cout << "cost: " << cost << std::endl;
		}
		else{
			std::cout << "collision checking failed" << std::endl;
			return INFINITY;
		}
	}
	return cost;
}

ompl::base::PlannerStatus ompl::geometric::SequentialKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    bool isValid = false;
	std::cout << "Starting KOMO, bestCost = " << bestCost << std::endl;

	while (!ptc){
		iteration++;
		// Setup KOMO using inuts
		auto komo_ = setupKOMO();
		// komo_->initWithConstant(startConfig);

		komo_->run_prepare(0);
		komo_->optimize();
		arrA configs = komo_->getPath_q();

		komo_->view(false);
		komo_->view_play(false);

		// Check if path is valid. Also calculate cost
		double cost = calcCost(configs); // Returns INFINITY if path is invalid. Else returns cost of sequence
		std::cout << "returned cost: " << cost << std::endl;

		if (cost<bestCost){
			this->bestCost = cost;
			std::cout << "best cost updated: " << bestCost << std::endl;
			// std::cout << "Iteration: " << iteration << std::endl;
			// break;
		}
	}

	if (bestCost < INFINITY) {
		std::cout << "Exact solution" << std::endl;
		pdef_->addSolutionPath(path, false, 0.0, getName());
		// Pause for a 10th of a second
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return base::PlannerStatus::EXACT_SOLUTION;
	}
	else return base::PlannerStatus::INFEASIBLE;
}