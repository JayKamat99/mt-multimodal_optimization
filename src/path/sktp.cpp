#include <path/sktp.h>

namespace ompl
{
	namespace geometric
	{

		sktp::sktp(const base::SpaceInformationPtr &si) : base::Planner(si, "sktp")
		{
			addPlannerProgressProperty("best cost REAL", [this] { return bestCostProgressProperty(); });
			C_Dimension = si->getStateDimension();
		}

		void sktp::setup()
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

		sktp::~sktp()
		{
			freeMemory();
		}

		void sktp::freeMemory()
		{
		}

		void sktp::clear()
		{
			Planner::clear();
			bestCost = Planner::pdef_->getOptimizationObjective()->infiniteCost();
		}

		sktp::keyframeNode::keyframeNode(arr state, std::shared_ptr<keyframeNode> parent)
		{
			this->state = state;
			this->parent = parent;
			if(parent == nullptr)
			{
				this->bestCostToCome = 0;
				this->costToComeHeuristic = 0;
				this->level = 0;
			}
			else
			{
				this->C_Dimension = this->parent->C_Dimension;
				this->bestCostToCome = INFINITY;
				this->costToComeHeuristic = this->parent->costToComeHeuristic+ this->distFromNode(parent);
				this->level = this->parent->level + 1;
			}
			this->costToGoHeuristic = INFINITY;
			this->is_new = true;
		}

		double sktp::keyframeNode::distFromNode(std::shared_ptr<keyframeNode> node)
		{
			return euclideanDistance(this->state.resize(C_Dimension), node->state.resize(C_Dimension));
		}

		double sktp::keyframeNode::calcDistHeuristic()
		{
			if (this->parent == nullptr)
				return 0;
			else 
				return (this->parent->get_costToComeHeuristic() + this->distFromNode(this->parent));
		}

		void sktp::keyframeNode::add_child(std::shared_ptr<keyframeNode> child, bool updateCosts = false)
		{
			this->childern.push_back(child);
			if (updateCosts)
			{
				if (this->costToGoHeuristic > child->costToGoHeuristic + this->distFromNode(child))
				{
					this->costToGoHeuristic = child->costToGoHeuristic + this->distFromNode(child);
				}
				// No else
			}
			// No else
		}

		ob::PlannerStatus sktp::keyframeNode::plan()
		{
			// attempt to solve the problem
			ob::PlannerStatus solved;
			solved = planner->solve(.5);
			return solved;
		}

		void sktp::visualize(std::shared_ptr<og::sktp::keyframeNode> &node)
		{
			// get configuration

			int currentPhase = node->get_level();
			// std::cout << "currentPhase: " << currentPhase << std::endl; 
			auto node_ = node;
			std::stack<arr> pathUntilNow;
			for (int i=0; i<currentPhase; i++)
			{
				pathUntilNow.push(node_->get_state().resize(C_Dimension));
				node_ = node_->get_parent();
			}

			// initialize the configuration
			rai::Configuration C(inputs.at(0).c_str());
			for(int phase = 0; phase <currentPhase; phase++)
			{
				std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
				C.setJointState(pathUntilNow.top());
				// std::cout << pathUntilNow.top() << std::endl;
				pathUntilNow.pop();
				if (phase % 2 == 0)
					C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
				else
					C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
				phase++;
			}


			// initialize KOMO object.

			KOMO komo;
			komo.verbose = 0;
			komo.setModel(C, true);

			auto path = static_cast<og::PathGeometric &>(*node->get_planner()->getProblemDefinition()->getSolutionPath());
			arrA configs;
			for (auto state : path.getStates())
			{
				arr config;
				std::vector<double> reals;
				node->get_planner()->getSpaceInformation()->getStateSpace()->copyToReals(reals, state);
				for (double r : reals){
					config.append(r);
				}
				configs.append(config);
			}
			
			komo.setTiming(1., configs.N, 5., 2);
			komo.add_qControlObjective({}, 1, 1.);

			//use configs to initialize with waypoints
			komo.initWithWaypoints(configs, configs.N, false);
			komo.run_prepare(0);
			komo.plotTrajectory();

			rai::ConfigurationViewer V;
			V.setPath(C, komo.x, "result", true);
			V.playVideo(true, 1.);
			
		}

		std::shared_ptr<og::sktp::keyframeNode> sktp::makeRootNode(std::vector<std::string> inputs)
		{

			std::string filename = inputs.at(0);

			// Get init Configuration
			rai::Configuration C(filename.c_str());

			// make a root node and return it!
			auto root = std::make_shared<keyframeNode>(C.getJointState(),nullptr);
			root->set_dimension(C.getJointStateDimension());
			return root;
		}

		arrA sktp::sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start)
		{
			arrA keyFrames;
			std::string filename = inputs.at(0);
    		int totalPhases = stoi(inputs.at(1));

			int currentPhase = start->get_level();
			// std::cout << "currentPhase: " << currentPhase << std::endl;
			if (currentPhase == totalPhases)
			return {{}}; // Returns null

			bool keyframesValid = false;

			// Set Configuration
			rai::Configuration C;
			C.addFile(filename.c_str());

			while(!keyframesValid)
			{
				KOMO komo;
				komo.verbose = 0;
				komo.setModel(C, true);
				komo.setTiming(totalPhases, 1, 5, 2);
				komo.addObjective({(double)currentPhase}, FS_qItself, {}, OT_eq, {}, start->get_state().resize(this->getSpaceInformation()->getStateDimension()));

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

				komo.add_qControlObjective({}, 1);
				komo.add_collision(true, 0.01);

				komo.run_prepare(0);
				komo.optimize();
				keyFrames = komo.getPath_q();
				// komo.view(true);
				// komo.view_play(true);

				rai::Graph R = komo.getReport(false);
				double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
				// std::cout << constraint_violation << std::endl;

				if (constraint_violation < maxConstraintViolationKOMO){
					keyframesValid = true;
				}
			}
			return keyFrames;
		}

		/**
		 * @brief This adds the keyframe sequence to the keyframe tree starting from the start node
		 * @param sequence 
		 * @param start 
		 */
		void sktp::addToTree(arrA sequence, std::shared_ptr<keyframeNode> start)
		{
			std::stack<std::shared_ptr<keyframeNode>> keyframeStack;
			int currentPhase = start->get_level();
			keyframeStack.push(start);
			for(int i=currentPhase; i<sequence.N; i++)
			{
				auto node = std::make_shared<keyframeNode>(sequence(i), keyframeStack.top());
				keyframeStack.push(node);
				// std::cout << keyframeStack.top() << std::endl;
			}
			keyframeStack.top()->setasleaf();
			while(keyframeStack.size()>1)
			{
				auto child = keyframeStack.top();
				// std::cout << child << std::endl;
				keyframeStack.pop();
				keyframeStack.top()->add_child(child, true);
			}
		}

        void sktp::growTree(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start)
		{
			int excess_kids = start->get_children().size()%branchingFactor;
			int sequencesToSample = this->branchingFactor;
			std::queue<std::shared_ptr<sktp::keyframeNode>> nodesToExpand;
			if(excess_kids != 0)
			{
				sequencesToSample = this->branchingFactor-excess_kids;
			}
			// No Else

			for(int i=0; i<sequencesToSample; i++)
			{
				arrA sequence = sampleKeyframeSequence(inputs, start);
				addToTree(sequence, start);
			}

			
		}

		void sktp::initPlanner(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node)
		{
			// initialize the planner variable
			std::shared_ptr<ompl::base::Planner> planner;

			// get state history until now
			int currentPhase = node->get_level();
			// std::cout << "currentPhase: " << currentPhase << std::endl; 
			auto node_ = node;
			std::stack<arr> pathUntilNow;
			for (int i=0; i<currentPhase; i++)
			{
				pathUntilNow.push(node_->get_state().resize(C_Dimension));
				node_ = node_->get_parent();
			}

			// initialize the configuration
			rai::Configuration C(inputs.at(0).c_str());
			for(int phase = 0; phase <currentPhase; phase++)
			{
				std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
				C.setJointState(pathUntilNow.top());
				// std::cout << pathUntilNow.top() << std::endl;
				pathUntilNow.pop();
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

			std::vector<arr> goal_;
			for (auto child:node->get_children())
			{
				goal_.push_back(child->get_state().resize(C_Dimension));
			}

			for (unsigned int j = 0; j < goal_.size(); j++)
			{
				ob::ScopedState<> goal(space);
				for (unsigned int i = 0; i < C_Dimension; i++)
				{
				goal[i] = goal_.at(j)(i);
				}
				goalStates->addState(goal);
				// std::cout << "goal " << j << " = " << goal << std::endl;
			}

			pdef->setGoal(goalStates);

			switch (subPlanner)
			{
			case (BITstar):
				auto BITstar_planner = std::make_shared<og::BITstar>(subplanner_si);
				BITstar_planner->setPruning(false);
				BITstar_planner->setStopOnGoalStateUpdate(true);
				planner = BITstar_planner;
				break;
			}
			planner->setProblemDefinition(pdef);
			planner->setup();
			node->set_planner(planner);
		}

		void sktp::addNewGoals(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node)
		{
			// Get goals
			auto planner = node->get_planner();
			auto subplanner_si = planner->getSpaceInformation();
			auto goalStates = std::make_shared<ob::GoalStates>(subplanner_si);

			auto space = subplanner_si->getStateSpace();

			std::vector<arr> goal_;
			for (auto child:node->get_children())
			{
				goal_.push_back(child->get_state().resize(C_Dimension));
			}

			// std::cout << "goal_.size(): " << goal_.size() << std::endl;
			for (unsigned int j = 0; j < goal_.size(); j++)
			{
				ob::ScopedState<> goal(space);
				for (unsigned int i = 0; i < C_Dimension; i++)
				{
				goal[i] = goal_.at(j)(i);
				}
				goalStates->addState(goal);
				// std::cout << "goal " << j << " = " << goal << std::endl;
			}

			planner->getProblemDefinition()->setGoal(goalStates);
		}

		ompl::base::PlannerStatus sktp::solve(const base::PlannerTerminationCondition &ptc)
		{
			// Make sure I have the necessary inputs. This is the place you define filename and total phases. Also you need to do the sanity check here.
			
			// Check that Planner::setup_ is true, if not call this->setup()
            Planner::checkValidity();

			// Assert setup succeeded
            if (!Planner::setup_)
            {
                throw ompl::Exception("%s::solve() failed to set up the planner. Has a problem definition been set?",
                                      Planner::getName().c_str());
            }
            // No else

			OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

			auto root = makeRootNode(inputs);
			growTree(inputs,root); // This samples keyframe sequences starting from node and adds to the tree. Also, builds a tree from there on.
			auto node = root;
			while(!ptc)
			{
				if (node->is_new)
				{
					initPlanner(node);
					node->is_new = false;
				}//No else

				auto solved = node->plan();
				#ifdef VISUALIZE
					visualize(node);
				#endif
				// bool solved = true;

				if (solved)
				{
					// move to the node whose solution you have.
					// For that, we first need to to figure out which node it is
					auto intermediate_solutionPath = static_cast<og::PathGeometric &>(*node->get_planner()->getProblemDefinition()->getSolutionPath());
					auto finalState = intermediate_solutionPath.getState(intermediate_solutionPath.getStateCount()-1);
					arr state;	std::vector<double> reals;
					node->get_planner()->getSpaceInformation()->getStateSpace()->copyToReals(reals, finalState);
					for (double r : reals)
					{
						state.append(r);
					}
					// std::cout << state << std::endl;

					// Now, scroll through every child node to figure out which node to expand
					for (auto child:node->get_children())
					{
						if(child->get_state().resize(C_Dimension) == state)
						{
							node = child;
							break;
						}
					}
				}

				else
				{
					std::cout << "planner on level " << node->get_level() << " could not find a solution" << std::endl;
					// Check if this faliure is at the root node. If yes, we only need to sample more sequences.
					if (node->get_parent() != nullptr)
					{
						node->penalty++; // I need to change this penalty++ to addPenalty. This function would add penalty to the goal inside the previous planner.
						// I can possibly penalize it's neighbours as well.
						node = node->get_parent();
						while (node->penalty > 1 && node->get_parent() != nullptr)
						{
							node->penalty++;
							node = node->get_parent();
						}
					}// No else, you can't do anything if you are already at the root node
				}

				// If you have reached the end you need to restart from the root
				if (node->get_level() == stoi(inputs.at(1)))
				{
					// get previous nodes to reclaim the whole path.
					// Along the way, you get to the root node anyway! So I don't need to define node = root again
					std::stack<std::shared_ptr<sktp::keyframeNode>> parentNodes;
					while(node->get_parent() != nullptr)
					{
						parentNodes.push(node->get_parent());
						node = node->get_parent();
					}

					// get the path from the nodes
					arrA solutionPath_arrA;
					og::PathGeometric solutionPath(this->getSpaceInformation());

					ompl::base::Cost pathCost;
					while(!parentNodes.empty())
					{
						auto intermediate_solutionPath = static_cast<og::PathGeometric &>(*parentNodes.top()->get_planner()->getProblemDefinition()->getSolutionPath());
						// intermediate_solutionPath.print(std::cout);
						auto optObj = parentNodes.top()->get_planner()->getProblemDefinition()->getOptimizationObjective();
						pathCost = optObj->combineCosts(pathCost,intermediate_solutionPath.cost(optObj));
						solutionPath.append(intermediate_solutionPath); // Todo: This does not work yet!
						// solutionPath.print(std::cout);
						arrA configs;
						for (auto state : intermediate_solutionPath.getStates())
						{
							arr config;
							std::vector<double> reals;
							node->get_planner()->getSpaceInformation()->getStateSpace()->copyToReals(reals, state);
							for (double r : reals){
								config.append(r);
							}
							configs.append(config);
						}
						solutionPath_arrA.append(configs);
						parentNodes.pop();
					}
					if (pathCost.value()<bestCost.value())
					{
						bestCost = pathCost;
						// std::cout << "bestCost: " << bestCost.value() << std::endl;
					}

					std::cout << solutionPath_arrA << std::endl;
					// break;
					// This solutionPath_arrA is a solution and must be added to pdef
					// komo.optimize();
				}
			}

			return ompl::base::PlannerStatus::ABORT;
		}
	}
}