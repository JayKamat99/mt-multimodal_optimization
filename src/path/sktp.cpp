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

		sktp::keyframeNode::keyframeNode(arr configuration, std::shared_ptr<keyframeNode> parent)
		{
			this->configuration = configuration;
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
			this->bestCostHeuristic = INFINITY;
			this->is_new = true;
		}

		double sktp::keyframeNode::distFromNode(std::shared_ptr<keyframeNode> node)
		{
			return euclideanDistance(this->configuration.resize(C_Dimension), node->configuration.resize(C_Dimension));
		}

		void sktp::keyframeNode::add_child(std::shared_ptr<keyframeNode> child)
		{
			this->childern.push_back(child);
			// Since new child is added costToGoHeuristic can be updated. We check if the new child node offers any promises
			if (this->costToGoHeuristic > child->costToGoHeuristic + this->distFromNode(child))
			{
				this->costToGoHeuristic = child->costToGoHeuristic + this->distFromNode(child);
			}
			// No else, this new child node does not promise us anything
		}

		ob::PlannerStatus sktp::keyframeNode::plan()
		{
			// attempt to solve the problem
			ob::PlannerStatus solved;
			std::cout << "level: " << level << std::endl;
			solved = planner->solve(.5);
			calls++;
			return solved;
		}

		void sktp::keyframeNode::update_bestCostHeuristic() // This function is called everytime a new child is added
		{
			if (bestCostHeuristic > distFromNode(this->childern.back()))
			{
				bestCostHeuristic = distFromNode(this->childern.back());
			}
		}

		void sktp::keyframeNode::update_bestCost(og::PathGeometricPtr pathGeo, std::shared_ptr<keyframeNode> child) // This function is called everytime a new child is added
		{
			if (bestCost < pathGeo->cost(planner->getProblemDefinition()->getOptimizationObjective()).value())
			{
				bestCost = pathGeo->cost(planner->getProblemDefinition()->getOptimizationObjective()).value();
				closestChild = child;
			}
		}

		double sktp::calcPriority(std::shared_ptr<og::sktp::keyframeNode> &node) const
		{
			double bestCost = node->get_bestCost(); 
			double bestCostHeuristic = node->get_bestCostHeuristic();
			int attempts = node->get_attempts();
			int k = 1;
			double ratio = bestCost/bestCostHeuristic;
			double priority = (1-std::exp(-ratio*k)) / (1+std::exp(-ratio*k)) / std::sqrt(attempts);
			return priority;
		}

		void sktp::visualizePath(arrA &configs, std::shared_ptr<og::sktp::keyframeNode> &node)
		{
			// get configuration

			int currentPhase = node->get_level();
			// std::cout << "currentPhase: " << currentPhase << std::endl; 
			auto node_ = node;
			std::stack<arr> pathUntilNow;
			for (int i=0; i<currentPhase; i++)
			{
				pathUntilNow.push(node_->get_configuration().resize(C_Dimension));
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
			V.setPath(C, komo.x, "result", true);
			V.playVideo();
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
				pathUntilNow.push(node_->get_configuration().resize(C_Dimension));
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

        void sktp::updateSolution(std::shared_ptr<og::sktp::keyframeNode> node)
		{
			// std::cout << "I enter the updateSolution function" << std::endl;

			// move to the best goal that has been reached.
			auto node_ = node->get_closestChild();

			std::stack<std::shared_ptr<sktp::keyframeNode>> solutionSequence; // Initialize the stack
			solutionSequence.push(node_);

			// get previous nodes to reclaim the whole path.
			while(node_->get_parent() != nullptr)
			{
				solutionSequence.push(node_->get_parent());
				node_ = node_->get_parent();
			}

			// get the path from the nodes
			arrA solutionPath_arrA;
			og::PathGeometric solutionPath(this->getSpaceInformation());

			ompl::base::Cost pathCost;
			std::shared_ptr<keyframeNode> currentNode = solutionSequence.top(); solutionSequence.pop();
			std::shared_ptr<keyframeNode> childNode;
			while(!solutionSequence.empty())
			{
				childNode = solutionSequence.top();
				auto pdef_ext = std::static_pointer_cast<ompl::base::ProblemDefinition_ext>(currentNode->get_planner()->getProblemDefinition());
				// std::cout << "I reach 1" << std::endl;

				// std::cout << "currentState: " << currentNode->get_state() << std::endl;
				// std::cout << "childState_update: " << childNode->get_state()->as<ob::RealVectorStateSpace::StateType>() << std::endl;

				auto intermediate_solutionPath = pdef_ext->getPath(childNode->get_state());
				intermediate_solutionPath->interpolate(15);
				// std::cout << "I reach 1.5" << std::endl;
				// The above line gives a segmentation fault. That is natural because, I am not saving the solution paths inside of problem definitions!
				intermediate_solutionPath->print(std::cout);
				auto optObj = currentNode->get_planner()->getProblemDefinition()->getOptimizationObjective();
				pathCost = optObj->combineCosts(pathCost,intermediate_solutionPath->cost(optObj));
				// std::cout << "I reach 2" << std::endl;
				// solutionPath.append(*intermediate_solutionPath); // Todo: This does not work yet!
				// solutionPath.print(std::cout);
				arrA configs;
				for (auto state : intermediate_solutionPath->getStates())
				{
					arr config;
					std::vector<double> reals;
					currentNode->get_planner()->getSpaceInformation()->getStateSpace()->copyToReals(reals, state);
					for (double r : reals){
						config.append(r);
					}
					configs.append(config);
				}
				visualizePath(configs, currentNode);
				solutionPath_arrA.append(configs);
				currentNode = childNode;
				solutionSequence.pop();
			}
			if (pathCost.value()<bestCost.value())
			{
				bestCost = pathCost;
				// std::cout << "bestCost: " << bestCost.value() << std::endl;
			}

			std::cout << solutionPath_arrA << std::endl;

			// Set solution using pdef.

			// std::cout << "I exit the updateSolution function" << std::endl;
		}


		std::shared_ptr<og::sktp::keyframeNode> sktp::makeRootNode(std::vector<std::string> inputs)
		{

			std::string filename = inputs.at(0);

			// Get init Configuration
			rai::Configuration C(filename.c_str());

			// make a root node and return it!
			auto root = std::make_shared<keyframeNode>(C.getJointState(),nullptr);
			root->set_dimension(C.getJointStateDimension());
			root->pathExists = true; // You are at the root!
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
			return nullArrA; // Returns null

			bool keyframesValid = false;
			int failure = 0;

			// Set Configuration
			rai::Configuration C;
			C.addFile(filename.c_str());

			while(!keyframesValid)
			{
				KOMO komo;
				komo.verbose = 0;
				komo.setModel(C, true);
				komo.setTiming(totalPhases, 1, 5, 2);
				komo.addObjective({(double)currentPhase}, FS_qItself, {}, OT_eq, {}, start->get_configuration().resize(this->getSpaceInformation()->getStateDimension()));

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
				else{
					failure++;
					if (failure >= 2*branchingFactor)
						return nullArrA;
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
				keyframeStack.top()->add_child(child);
				keyframeStack.top()->update_bestCostHeuristic();
			}
		}

        void sktp::growTree(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start)
		{
			int excess_kids = start->get_children().size()%branchingFactor;
			int sequencesToSample = this->branchingFactor;
			std::queue<std::shared_ptr<sktp::keyframeNode>> nodesToExpand;
			sequencesToSample = this->branchingFactor-excess_kids;

			for(int i=0; i<sequencesToSample; i++)
			{
				arrA sequence = sampleKeyframeSequence(inputs, start); // Returns null if it can't find a sequence after sufficient tries
				if (sequence == nullArrA)
				{
					std::cout << "I am breaking!" << std::endl;
					break;
				}
				addToTree(sequence, start);
			}	
		}

		void sktp::initPlanner(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node)
		{
			// initialize the planner variable
			std::shared_ptr<ompl::base::Planner> planner;

			// get state history until now
			int currentPhase = node->get_level();
			auto node_ = node;
			std::stack<arr> pathUntilNow;
			for (int i=0; i<currentPhase; i++)
			{
				pathUntilNow.push(node_->get_configuration().resize(C_Dimension));
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
			if (node->get_parent() == nullptr)
			{
				std::cout << "initializing root!" << std::endl;
				node->set_state(start.get());
			}
			pdef->addStartState(node->get_state());

			// Get goals
			auto goalStates = std::make_shared<ob::GoalStates>(subplanner_si);

			for (auto child:node->get_children())
			{
				arr goal_ = child->get_configuration().resize(C_Dimension);
				ob::ScopedState<> goal(space);
				for (unsigned int i = 0; i < C_Dimension; i++)
				{
				goal[i] = goal_(i);
				}
				goalStates->addState(goal);
			}

			pdef->setGoal(goalStates);

			switch (subPlanner)
			{
			case (BITstar):
				auto BITstar_planner = std::make_shared<og::BITstar>(subplanner_si, "BITstar_sktp");
				BITstar_planner->setPruning(false);
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
			auto subplanner = node->get_planner();
			auto subplanner_si = subplanner->getSpaceInformation();
			auto goalStates = std::make_shared<ob::GoalStates>(subplanner_si);

			auto space = subplanner_si->getStateSpace();

			for (auto child:node->get_children())
			{
				arr goal_ = child->get_configuration().resize(C_Dimension);
				ob::ScopedState<> goal(space);
				for (unsigned int i = 0; i < C_Dimension; i++)
				{
				goal[i] = goal_(i);
				}
				goalStates->addState(goal);
			}

			subplanner->getProblemDefinition()->setGoal(goalStates);
		}

		ompl::base::PlannerStatus sktp::solve(const base::PlannerTerminationCondition &ptc)
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
			// Begin Algorithm
			srand(std::time(0)); // Generate a random number
			std::priority_queue<std::pair<std::shared_ptr<sktp::keyframeNode>, double>> visitedNodes; //Initialize the priority queue to determine which node to expand
			auto root = makeRootNode(inputs); // Make root node

			while (root->get_children().size() == 0) // basically keep searching for a solution sequence until you get a solution.
			{
				growTree(inputs,root); // This samples keyframe sequences starting from node and adds to the tree.
			}

			auto node = root;
			while(!ptc)
			{
				if (node->is_new) // This is the first time you are visiting the node.
				{
					initPlanner(node); // Intializes the ompl planner for the space associated with that node
					node->is_new = false;
				}//No else

				// 20% of the times add more goals otherwise, only try planning a motion // make this your parameter (k < %)
				if (rand()%100 < 20)
				{
					std::cout << "I am growing tree!" << std::endl;
					growTree(inputs,node);
					addNewGoals(node);
					continue;
				}
				// No else, continue with the planning

				auto solved = node->plan(); // Try finding a plan. This runs for one BIT* batch

				// get the pdef as pdef_ext. This extension helps us get path to a given goal.
				auto pdef_ext = std::static_pointer_cast<ompl::base::ProblemDefinition_ext>(node->get_planner()->getProblemDefinition());

				if (solved)
				{
					/****************************************************************/
					// Add the new goals reached to the visitedNodes priority queue. This node can be (will be) chosen later to be planned on.
					
					auto reachedGoals = pdef_ext->get_reachedGoals();
					std::cout << "reachedGoals:" << reachedGoals.size() << std::endl;
					while (node->markedGoals < reachedGoals.size()) // We don't check compare goals that have already been accounted for
					{
						auto goal = reachedGoals.at(node->markedGoals);
						// Check which keframe this goal belongs to and acordingly set the ompl state pointer for that node
						for (auto child:node->get_children())
						{
							if (child->pathExists == true) // There is no point checking a node if it has already been reached and documented
								continue;

							int i = 0; 
							while (i<C_Dimension)
							{
								if (child->get_configuration()(i) != (*goal->as<ob::RealVectorStateSpace::StateType>())[i])
									break; // This is not the right keyframe
								i++;								
							}

							if (i==C_Dimension) // This child state is in among the reached goals
							{
								child->pathExists = true; // Mark this node as reached
								child->set_state(goal);
								std::cout << "This goal has been reached!" << std::endl;
								if (child->get_level() < stoi(inputs.at(1))) // Add the child to visited nodes vector as long as it is not at the last level
									visitedNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(child,calcPriority(child)));
							}
						}
						node->markedGoals++;
					}

					/****************************************************************/
					// For all the child nodes to which path exists, we call for the path and use that to update the bestCost for the node.
					for (auto child:node->get_children())
					{
						if (child->pathExists)
						{
							auto pathGeo = pdef_ext->getPath(child->get_state());
							std::cout << "currentState: " << node->get_state() << std::endl;
							std::cout << "childState: " << child->get_state()->as<ob::RealVectorStateSpace::StateType>() << std::endl;
							pathGeo->print(std::cout);
							node->update_bestCost(pathGeo, child); // Checks if this is a better path than the path that already exists. Sets bestCost, closestChild
						} // No else, you cannot do anything if a path doesn not exist
					}
					
					/****************************************************************/
					// Have we elready fond a solution?
					// If this is the second to last node, it means that we have a solution!! return/update it! otherwise skip this step
					if (node->get_level() == stoi(inputs.at(1))-1)
					{
						updateSolution(node);
					} // No else, don't do anything if you are not on the last node

				} // No else, do nothing if unsolved

				// No matter if we have solved or not we need to push the node we used back into the priorityQueue with updated values
				visitedNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(node, calcPriority(node)));

				// Choose the next node. This is our real contribution.
				std::cout << "size:" << visitedNodes.size() << std::endl;
				node = visitedNodes.top().first;
				visitedNodes.pop();

				#ifdef VISUALIZE
					visualize(node);
				#endif
			}

			if (bestCost.value() < INFINITY)
				return ompl::base::PlannerStatus::EXACT_SOLUTION;
			else 
				return ompl::base::PlannerStatus::TIMEOUT;
		}
	}
}