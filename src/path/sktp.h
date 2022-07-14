#ifndef OMPL_GEOMETRIC_sktp_
#define OMPL_GEOMETRIC_sktp_

/**
 * @file sktp.h
 * @author Jay Kamat
 * @brief Short for sequential keyframe tree planner
 * @version 0.1
 * @date 2022-07-11
 * 
 * @copyright Copyright (c) 2022
 * 
**/

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <limits>
#include <stack>
#include <KOMO/komo.h>
#include <Kin/kin.h>

// Planners
#include <ompl/geometric/planners/informedtrees/BITstar.h>

namespace ompl
{
    namespace geometric
    {

        OMPL_CLASS_FORWARD(sktp);

        class sktp : public base::Planner
        {
        public: 
            sktp(const base::SpaceInformationPtr &si);
            virtual ~sktp() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            enum SUBPLANNER{
                BITstar
            };

            void set_subPlanner(SUBPLANNER subPlanner) {this->subPlanner = subPlanner;}
            void set_branchingFactor(uint branchingFactor) {this->branchingFactor = branchingFactor;}
            void set_inputs(std::vector<std::string> inputs) {this->inputs = inputs;}

        protected:
            // PLanner Variables
            std::vector<std::string> inputs;
            SUBPLANNER subPlanner;
            uint branchingFactor;
			double bestCost = std::numeric_limits<double>::infinity();
            int C_Dimension;

            void freeMemory();

			std::string bestCostProperty()
            {
                return std::to_string(bestCost);
            }
            void clear() override;

            void setup() override;

            // This is the structure of every leaf in the keyframe tree. The backbone of this algorithm
            class keyframeNode
            {
            private:
                int C_Dimension;
                arr state;
                std::shared_ptr<keyframeNode> parent;
                std::vector<std::shared_ptr<keyframeNode>> childern;
                std::shared_ptr<ompl::base::Planner> planner;
                double costToComeHeuristic; // lower-bound cost to reach the node
                double bestCostToCome; // cost of the best path to node until now
                double costToGo; // lower-bound cost to reach the final goal.
                double calcDistHeuristic(); // calculates eucledian distance from parent and adds it to the best cost to parent if available, else to the dist heuristic to the parent
                double distFromNode(std::shared_ptr<keyframeNode> node);
            public:
                keyframeNode(arr state, std::shared_ptr<keyframeNode> parent);
                ~keyframeNode() = default;
                arr get_state() {return this->state;}
                std::shared_ptr<keyframeNode> get_parent() {return this->parent;}
                double get_costToComeHeuristic()  {return this->costToComeHeuristic;}
                // double get_bestCostToCome() {return this->bestCostToCome;}
                void add_child(std::shared_ptr<keyframeNode> child, bool updateCosts);
                void setasleaf() {this->costToGo = 0;}
                void set_dimension(int C_Dimension) {this->C_Dimension = C_Dimension;}
                void set_planner(std::shared_ptr<ompl::base::Planner> planner) {this->planner = planner;}
                void plan() {planner->solve(1.0);}
            };

            std::shared_ptr<og::sktp::keyframeNode> makeRootNode(std::vector<std::string> inputs);

            void growTree(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
		    arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
            int getCurrentPhase(std::shared_ptr<keyframeNode> node);
			void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start);
        };  
    } // namespace geometric
} //namespace ompl

#endif