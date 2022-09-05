#include <path/sktp.h>

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

	bool check(arr jointState)
    {
        arr phi;
        nlp.evaluate(phi, NoArr, jointState);
        return std::abs(phi(0)) < tol;
    }
};


namespace ompl
{
	namespace geometric
	{

		sktp::sktp(const base::SpaceInformationPtr &si, std::string name) : base::Planner(si, name)
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
                bestCost = Planner::pdef_->getOptimizationObjective()->infiniteCost().value();

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
			std::cout << "sktp deleted" << std::endl;
		}

		void sktp::freeMemory()
		{
		}

		void sktp::clear()
		{
			std::cout << "sktp: clear called" << std::endl;
			Planner::clear();
			bestCost = Planner::pdef_->getOptimizationObjective()->infiniteCost().value();
		}

		sktp::keyframeNode::keyframeNode(arr configuration, std::shared_ptr<keyframeNode> parent)
		{
			this->configuration = configuration;
			this->parent = parent;
			if(parent == nullptr)
			{
				this->costToCome = 0;
				this->costToComeHeuristic = 0;
				this->level = 0;
			}
			else
			{
				this->C_Dimension = this->parent->C_Dimension;
				this->costToCome = INFINITY;
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

		void sktp::keyframeNode::update_costToCome()
		{
			auto pdef_ext = std::static_pointer_cast<ompl::base::ProblemDefinition_ext>(parent->get_planner()->getProblemDefinition());
			costToCome = parent->get_costToCome() + pdef_ext->getCostToReach(state);
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

		ob::PlannerStatus sktp::keyframeNode::plan(const base::PlannerTerminationCondition &ptc)
		{
			// attempt to solve the problem
			ob::PlannerStatus solved;
			solved = planner->solve(ptc); // We pass ptc. This planner continues until ptc or until batch exhausts or until other conditions are met
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

		void sktp::keyframeNode::update_bestCost(double cost) // This function is called everytime a new child is added
		{
			this->bestCost = cost;
		}

		double sktp::calcPriority(std::shared_ptr<og::sktp::keyframeNode> &node) const
		{
			double bestCost = node->get_bestCost(); 
			double bestCostHeuristic = node->get_bestCostHeuristic();
			int attempts = node->get_attempts();
			int k = 1;
			double ratio = (bestCost-bestCostHeuristic)/bestCostHeuristic;
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

		std::shared_ptr<og::sktp::keyframeNode> sktp::get_bestGoal(std::shared_ptr<og::sktp::keyframeNode> root)
		{
			// Search for the best solution from here. Use BFS algorithm. This can be later improved to a better algorithm
			double bestCost = INFINITY;
			std::shared_ptr<og::sktp::keyframeNode> bestGoal;
			std::queue<std::shared_ptr<keyframeNode>> queue;
			queue.push(root);
			while (!queue.empty())
			{
				auto node = queue.front(); queue.pop();
				if (node->get_level() < stoi(inputs.at(1)))
				{
					for (auto child : node->get_children())
					{
						queue.push(child);
					}
				}
				else // This is the goal node
				{
					// Document cost
					double cost = node->get_costToCome();
					if (cost <bestCost)
					{
						bestGoal = node;
					} // No else
				}
			}
			return bestGoal;
		}

		void sktp::addSolution(std::shared_ptr<og::sktp::keyframeNode> node)
		{
			// mark that parents have already found a solution to the goal
			while(node->hasSolution == false)
			{
				node->hasSolution = true;
				node = node->get_parent();
				if (node == nullptr)
					break;
			}
		}

        void sktp::updateSolution(std::shared_ptr<og::sktp::keyframeNode> node)
		{
			// Use graphs search to find best path from here to the goal
			std::shared_ptr<og::sktp::keyframeNode> bestGoal = get_bestGoal(node);
			// Check if this gives a better solution then what is already present
			if (bestGoal->get_costToCome() < bestCost)
			{
				// update bestCost and bestGoalNode
				bestCost = bestGoal->get_costToCome();
				bestGoalNode = bestGoal;
			}
		}


		std::shared_ptr<og::sktp::keyframeNode> sktp::makeRootNode(std::vector<std::string> inputs)
		{

			std::string filename = inputs.at(0);

			// Get init Configuration
			rai::Configuration C(filename.c_str());

			// make a root node and return it!
			auto root = std::make_shared<keyframeNode>(C.getJointState(),nullptr);
			root->set_dimension(C.getJointStateDimension());
			root->reached = true; // You are at the root!
			return root;
		}

		bool sktp::isConfigValid(rai::Configuration &C)
		{
			// create checker
			// check

			auto komo = std::make_shared<KOMO>();
			komo->setModel(C, true);
			komo->setTiming(1, 1, 1, 1);
			komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
			komo->run_prepare(2);

			auto checker = std::make_shared<ValidityCheckWithKOMO>(komo);

			return checker->check(C.getJointState());
		}

		bool sktp::checkKeyframes(arrA keyFrames)
		{
			// get configuration
			// check configuration
			// get and check subsequent configurations
			// return true if check is successful

			rai::Configuration C(inputs.at(0).c_str());

			if (!isConfigValid(C))
			{
				return false;
			}

			for(int phase = 0; phase < keyFrames.N; phase++)
			{
				std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);
				C.setJointState(keyFrames(phase).resize(C_Dimension));
				if (phase % 2 == 0)
					C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
				else
					C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place

				if (!isConfigValid(C))
				{
					return false;
				}
			}
			return true;
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

				// komo.add_qControlObjective({}, 1);
				komo.add_collision(true, 0.01);

				komo.run_prepare(0);
				komo.optimize(1); // Try 1, 2, 0.5
				keyFrames = komo.getPath_q();
				// komo.view(true);
				// komo.view_play(true);

				keyframesValid = checkKeyframes(keyFrames);
				if (!keyframesValid){
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
					// std::cout << "I am breaking!" << std::endl;
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
				// std::cout << "initializing root!" << std::endl;
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
			// Prameters
			int sampleProbability = 20; // New children are sampled 20% of the times


			/****************************************************************/
			// Begin Algorithm
			srand(std::time(0)); // Generate a random number
			std::priority_queue<std::pair<std::shared_ptr<sktp::keyframeNode>, double>> activeNodes; //Initialize the priority queue to determine which node to expand
			auto root = makeRootNode(inputs); // Make root node

			// growTree function takes quite some time. You need to do something about it
			while (root->get_children().size() == 0 && !ptc) // basically keep searching for a solution sequence until you get a solution.
			{
				growTree(inputs,root); // This samples keyframe sequences starting from node and adds to the tree.
			}

			auto node = root;
			while(!ptc)
			{
				if (node->is_new) // This is the first time you are visiting the node. Initialize it!
				{
					initPlanner(node); // Intializes the ompl planner for the space associated with that node
					node->is_new = false;
				}//No else

				// p% of the times add more goals, otherwise, only try planning a motion
				if (rand()%100 < sampleProbability)
				{
					growTree(inputs,node);
					addNewGoals(node);
					continue;
				}
				// No else, continue with the planning

				auto solved = node->plan(ptc); // Try finding a plan. This runs for one BIT* batch (TODO)

				// get the pdef as pdef_ext. This extension helps us get path to a given goal.
				auto pdef_ext = std::static_pointer_cast<ompl::base::ProblemDefinition_ext>(node->get_planner()->getProblemDefinition());

				if (solved)
				{
					/****************************************************************/
					// Add the new goals reached to the activeNodes priority queue. This node can be (will be) chosen later to be planned on.
					
					auto reachedGoals = pdef_ext->get_reachedGoals();
					// std::cout << "reachedGoals:" << reachedGoals.size() << std::endl;
					while (node->markedGoals < reachedGoals.size()) // We don't check / compare goals that have already been accounted for
					{ // TODO: Check this  works as intended, i.e., goals remain in the same order
						auto i_goal = reachedGoals.at(node->markedGoals);
						// Check which keframe this intermeiate goal belongs to and acordingly set the ompl state pointer for that node
						for (auto child:node->get_children())
						{
							if (child->reached == true) // There is no point checking a node if it has already been reached and documented
								continue;

							int i = 0; 
							while (i<C_Dimension)
							{
								if (child->get_configuration()(i) != (*i_goal->as<ob::RealVectorStateSpace::StateType>())[i])
									break; // This is not the right keyframe
								i++;								
							}

							if (i==C_Dimension) // This child state is in among the reached goals
							{
								child->reached = true; // Mark this node as reached
								child->set_state(i_goal);
								child->update_costToCome();
								if (child->get_level() < stoi(inputs.at(1))) // Add the child to the activeNodes vector as long as it is not at the last level
								{
									// sktp and sktpRandom only differ at this stage // TODO: check calcPriority function
									if (Planner::getName() == "sktp")
										activeNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(child,calcPriority(child)));
									else if (Planner::getName() == "sktpRandom")
										activeNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(child, rand()));
								}
								else // it is the last node! We have a new solution!
								{
									addSolution(child); // Marks nodes as having a solution
								}
							}
						}
						node->markedGoals++;
					}

					/****************************************************************/
					// update bestCosts of all child nodes
					// For all the child nodes to which path exists, we get the cost to update the bestCost for the node.
					for (auto child:node->get_children())
					{
						if (child->reached)
						{
							double cost = pdef_ext->getCostToReach(child->get_state());
							if (node->get_bestCost()>cost)
							{
								node->update_bestCost(cost);
								// We also update the cost of the tolat path if there is a feasible path from here.
							} // No else
							// Update cost to come of every child node
							child->update_costToCome(); 
						} // No else, you cannot do anything if a path doesn not exist
					}

					/****************************************************************/
					// Since, we have solved the node, we need to check if the goal / path to goal is updated
					if (node->hasSolution)
					{
						updateSolution(node);	
					}

				} // No else, do nothing if unsolved

				// No matter if we have solved or not we need to push the node we used back into the priorityQueue with updated values.
				// This again depends on the planner
				if (Planner::getName() == "sktp")
					activeNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(node,calcPriority(node)));
				else if (Planner::getName() == "sktpRandom")
					activeNodes.push(std::make_pair<std::shared_ptr<sktp::keyframeNode>&, double>(node, rand()));

				// Choose the next node. This is our real contribution.
				// std::cout << "size:" << activeNodes.size() << std::endl;
				node = activeNodes.top().first;
				activeNodes.pop();
				std::cout << "bestCost: " << bestCost << std::endl;
			}

			std::cout << "bestCost: " << bestCost << std::endl;

			if (bestCost < INFINITY)
			{
				std::cout << "Exact Solution" << std::endl;
				return ompl::base::PlannerStatus::EXACT_SOLUTION;
			}
			else 
				return ompl::base::PlannerStatus::TIMEOUT;
		}
	}
}