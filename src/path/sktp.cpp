#include <path/sktp.h>

namespace ompl
{
	namespace geometric
	{

		sktp::sktp(const base::SpaceInformationPtr &si) : base::Planner(si, "KOMO")
		{
			addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
		}

		void sktp::setup()
		{
			Planner::setup();
			bestCost = std::numeric_limits<double>::infinity();
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
			bestCost = std::numeric_limits<double>::infinity();
		}

		sktp::keyframeNode::keyframeNode(arr state, std::shared_ptr<keyframeNode> parent)
		{
			this->state = state;
			this->parent = parent;
			if(parent == nullptr)
			{
				this->bestCostToCome = 0;
				this->costToComeHeuristic = 0;
			}
			else
			{
				this->C_Dimension = this->parent->C_Dimension;
				this->bestCostToCome = INFINITY;
				this->costToComeHeuristic = this->parent->costToComeHeuristic+ this->distFromNode(parent);
			}
			this->costToGo = INFINITY;
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
				if (this->costToGo > child->costToGo + this->distFromNode(child))
				{
					this->costToGo = child->costToGo + this->distFromNode(child);
				}
				// No else
			}
			// No else
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

		int sktp::getCurrentPhase(std::shared_ptr<keyframeNode> node)
		{
			int phase = 0;
			while (node->get_parent() != nullptr)
			{
				node = node->get_parent();
				phase ++;
			}
			return phase;
		}

		arrA sktp::sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start)
		{
			arrA keyFrames;
			std::string filename = inputs.at(0);
    		int totalPhases = stoi(inputs.at(1));

			int currentPhase = getCurrentPhase(start);
			std::cout << "currentPhase: " << currentPhase << std::endl;

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

					// std::cout << "ref1,ref2: " << ref1 << ref2 << std::endl;

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

				if (constraint_violation < 1){
					keyframesValid = true;
				}
			}
			return keyFrames;
		}

		void sktp::addToTree(arrA sequence, std::shared_ptr<keyframeNode> start)
		{
			std::stack<std::shared_ptr<keyframeNode>> keyframeStack;
			int currentPhase = getCurrentPhase(start);
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
			for(int i=0; i<this->branchingFactor; i++)
			{
				arrA sequence = sampleKeyframeSequence(inputs, start);
				addToTree(sequence, start);
			}
		}

		ompl::base::PlannerStatus sktp::solve(const base::PlannerTerminationCondition &ptc)
		{
			// Make sure I have the necessary inputs. This is the place you define filename and total phases. Also you need to do the sanity check here.

			auto node = makeRootNode(inputs);
			while(!ptc)
			{
				growTree(inputs,node); // This samples keyframe sequences starting from node and adds to the tree
				
				// Describe planner parameters
				auto planner(std::make_shared<og::BITstar>(this->getSpaceInformation()));
				node->set_planner(planner);
				node->plan();
			}

			return ompl::base::PlannerStatus::ABORT;
		}
	}
}