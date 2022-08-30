#include <path/naiveSequentialPlanner.h>

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
    bool check(const ob::State *state)
    {
        const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

        arr x_query;
        for (unsigned int i = 0; i < C_Dimension; i++)
        {
            x_query.append((*State)[i]);
        }
        arr phi;
        nlp.evaluate(phi, NoArr, x_query);
        return std::abs(phi(0)) < tol;
    }
};

namespace ompl
{
	namespace geometric
	{

		naiveSequentialPlanner::naiveSequentialPlanner(const base::SpaceInformationPtr &si, std::string name) : base::Planner(si, name)
		{
			addPlannerProgressProperty("best cost REAL", [this]
									   { return bestCostProgressProperty(); });
			C_Dimension = si->getStateDimension();
		}

		void naiveSequentialPlanner::setup()
		{
			Planner::setup();

			// Check if we have a problem definition
			if (static_cast<bool>(Planner::pdef_))
			{
				// We do, do some initialization work.
				// See if we have an optimization objective
				if (!Planner::pdef_->hasOptimizationObjective())
				{
					OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
								Planner::getName().c_str());
					Planner::pdef_->setOptimizationObjective(
						std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
				}
				// No else, we were given one.

				// Initialize the best cost found so far to be infinite.
				bestCost = Planner::pdef_->getOptimizationObjective()->infiniteCost();

				// If the problem definition *has* a goal, make sure it is of appropriate type
				if (static_cast<bool>(Planner::pdef_->getGoal()))
				{
					if (!Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
					{
						OMPL_ERROR("%s::setup() BIT* currently only supports goals that can be cast to a sampleable "
								   "goal region.",
								   Planner::getName().c_str());
						// Mark as not setup:
						Planner::setup_ = false;
						return;
					}
					// No else, of correct type.
				}
				// No else, called without a goal. Is this MoveIt?
			}
		}

		naiveSequentialPlanner::~naiveSequentialPlanner()
		{
			freeMemory();
			std::cout << "naiveSequentialPlanner deleted" << std::endl;
		}

		void naiveSequentialPlanner::freeMemory()
		{
		}

		void naiveSequentialPlanner::clear()
		{
			std::cout << "naiveSequentialPlanner: clear called" << std::endl;
			Planner::clear();
			bestCost = Planner::pdef_->getOptimizationObjective()->infiniteCost();
		}

		arrA naiveSequentialPlanner::copyPathToArrA(std::shared_ptr<ompl::geometric::PathGeometric> path, ompl::base::PlannerPtr planner)
		{
			path->interpolate(15);
			arrA configs;
			for (auto state : path->getStates())
			{
				arr config;
				std::vector<double> reals;
				planner->getSpaceInformation()->getStateSpace()->copyToReals(reals, state);
				for (double r : reals){
					config.append(r);
				}
				configs.append(config);
			}
		}

		void naiveSequentialPlanner::visualizePath(arrA &configs, int currentPhase, arrA sequence)
		{	
			// initialize the configuration
			rai::Configuration C(inputs.at(0).c_str());
			for(int phase = 0; phase <currentPhase; phase++)
			{
				std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
				C.setJointState(sequence(phase).resize(C_Dimension));
				if (phase % 2 == 0)
					C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
				else
					C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
				phase++;
			}
			
			KOMO komo;
			komo.verbose = 0;
			komo.setModel(C, true);

			komo.setTiming(1., configs.N, 5., 2);
			komo.add_qControlObjective({}, 1, 1.);

			// use configs to initialize with waypoints
			komo.initWithWaypoints(configs, configs.N, false);
			komo.run_prepare(0);
			komo.plotTrajectory();

			rai::ConfigurationViewer V;
			V.setPath(C, komo.x, "result");
			V.playVideo(false);
		}

		arrA naiveSequentialPlanner::sampleKeyframeSequence(ompl::base::PlannerTerminationCondition ptc)
		{
			arrA keyFrames;

			bool keyframesValid = false;

			// Set Configuration
			rai::Configuration C;
			C.addFile(filename.c_str());

			while (!keyframesValid && !ptc)
			{
				KOMO komo;
				komo.verbose = 0;
				komo.setModel(C, true);
				komo.setTiming(totalPhases, 1, 5, 2);

				int phase = 0;
				while (phase < totalPhases)
				{
					std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

					if (phase == 0)
						komo.addSwitch_stable(1, 2., "", ref1.c_str(), ref2.c_str());
					else if (phase % 2 == 0)
						komo.addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str(), false);
					else
						komo.addSwitch_stable(phase + 1, phase + 2., "", ref2.c_str(), ref1.c_str(), false);
					komo.addObjective({phase + 1.}, FS_distance, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2});
					komo.addObjective({phase + 1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

					if (phase % 2 == 0) // pick
					{
						// std::cout << "pick" << std::endl;
						komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
					}
					else // place
					{
						// std::cout << "place" << std::endl;
						komo.addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
					}
					phase++;
				}

				// komo.add_qControlObjective({}, 1); // or put a small weight
				komo.add_collision(true, 0.01);

				komo.run_prepare(0);
				komo.optimize(1);
				// komo.view();
				// komo.view_play();
				keyFrames = komo.getPath_q();

				// A betterway for doing this would be to explicitly check for collision

				rai::Graph R = komo.getReport(false);
				double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
				// std::cout << constraint_violation << std::endl;

				if (constraint_violation < maxConstraintViolationKOMO)
				{
					keyframesValid = true;
				} // No Else
			}
			if (!keyframesValid)
			{
				return nullArrA;
			}
			return keyFrames;
		}

		ompl::base::PlannerPtr naiveSequentialPlanner::initPlanner(int currentPhase, arrA sequence)
		{
			// initialize the planner variable
			std::shared_ptr<ompl::base::Planner> planner;

			// initialize the configuration
			rai::Configuration C(inputs.at(0).c_str());
			for(int phase = 0; phase <currentPhase; phase++)
			{
				std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
				C.setJointState(sequence(phase).resize(C_Dimension));
				if (phase % 2 == 0)
					C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
				else
					C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
				phase++;
			}

			auto komo = std::make_shared<KOMO>();
			komo->setModel(C, true);
			komo->setTiming(1, 1, 1, 1);
			komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
			komo->run_prepare(2);

			//Construct the state space we are planning in
			auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

			ob::RealVectorBounds bounds(C_Dimension);
			bounds.setLow(-PI);
			bounds.setHigh(PI);
			space->setBounds(bounds);

			//create space information pointer
			ob::SpaceInformationPtr subplanner_si(std::make_shared<ob::SpaceInformation>(space));
			auto checker = std::make_shared<ValidityCheckWithKOMO>(komo);

			subplanner_si->setStateValidityChecker([checker](const ob::State *state) {
				return checker->check(state);
			});

			ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition_ext>(subplanner_si));

			// create start and goal states. These states might change from example to example
			ob::ScopedState<> start(space);
			for (unsigned int i = 0; i < C_Dimension; i++)
			{
				start[i] = komo->getConfiguration_q(0).elem(i);
			}
			pdef->addStartState(start);

			// Get goals
			auto goalStates = std::make_shared<ob::GoalStates>(subplanner_si);
			
			arr goal_ = sequence(currentPhase).resize(C_Dimension);
			ob::ScopedState<> goal(space);
			for (unsigned int i = 0; i < C_Dimension; i++)
			{
			goal[i] = goal_(i);
			}
			goalStates->addState(goal);
			pdef->setGoal(goalStates);

			auto BITstar_planner = std::make_shared<og::BITstar>(subplanner_si);
			BITstar_planner->setPruning(false);
			planner = BITstar_planner;

			planner->setProblemDefinition(pdef);
			planner->setup();
			return planner;
		}

		ompl::base::PlannerStatus naiveSequentialPlanner::solve(const base::PlannerTerminationCondition &ptc)
		{
			/****************************************************************/
			// Check that Planner::setup_ is true, if not call this->setup()
			Planner::checkValidity();

			// Assert setup succeeded
			if (!Planner::setup_)
			{
				throw ompl::Exception("%s::solve() failed to set up the planner. Has a problem definition been set?",
									  Planner::getName().c_str());
			}
			// No else

			// Make sure you have inputs defined

			OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

			/****************************************************************/
			// Initialize
			filename = inputs.at(0);
			totalPhases = stoi(inputs.at(1));
			bool exactSolutionFound{false};

			/****************************************************************/
			// Parameters
			double timeout = 1;

			/****************************************************************/
			// Begin Algorithm

			while (!ptc)
			{
				arrA sequence = sampleKeyframeSequence(ptc);
				if (sequence == nullArrA)
				{
					std::cout << "gets nullArrA" << std::endl;
					// break;
				}
				std::cout << sequence << std::endl;
				std::queue<arrA> solutionPath;
				
				// Plan
				double cost = 0;
				bool solutionFound = true; // Turned to false if a solution to a path does not exist
				for (int currentPhase = 0; currentPhase<sequence.N; currentPhase++)
				{
					// std::cout << "currentPhase: " << currentPhase << std::endl;
					auto planner = initPlanner(currentPhase, sequence);
					// std::cout << "Planner Initialized" << std::endl;
					// Define a new ptc, "or" it with the main ptc
					auto local_ptc = plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(timeout));
					ob::PlannerStatus solved;
					solved = planner->solve(local_ptc);
					if (!solved)
					{
						solutionFound = false;
						break;
					}
					if (solved)
					{
						// Get solutioncost and add to current cost
						auto obj = planner->getProblemDefinition()->getOptimizationObjective();
						auto path = planner->getProblemDefinition()->getSolutionPath();
						cost += path->cost(obj).value();
						auto pathGeo = std::static_pointer_cast<ompl::geometric::PathGeometric>(path);
						// arrA configs = copyPathToArrA(pathGeo, planner);
						pathGeo->interpolate(15);
						arrA configs;
						for (auto state : pathGeo->getStates())
						{
							arr config;
							std::vector<double> reals;
							planner->getSpaceInformation()->getStateSpace()->copyToReals(reals, state);
							for (double r : reals){
								config.append(r);
							}
							configs.append(config);
						}
						solutionPath.push(configs);
					}
					timeout = timeout*0.2;
				}
				if (solutionFound)
				{
					// Compare current solution with bestCost
					if (cost < bestCost.value())
					{
						bestCost = (ompl::base::Cost)(cost);
					}
					// Print the solution
					for (int i=0; i<sequence.N; ++i)
					{
						visualizePath(solutionPath.front(),i,sequence);
						solutionPath.pop();
					}
					exactSolutionFound = true;
				}
			}
			if (exactSolutionFound)
			{
				return ompl::base::PlannerStatus::EXACT_SOLUTION;
			}
			else
			{
				return ompl::base::PlannerStatus::TIMEOUT;
			}
		}
	}
}