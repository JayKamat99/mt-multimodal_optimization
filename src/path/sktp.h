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

#include <cstdlib>
#include <time.h>

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <limits>
#include <stack>
#include <set>
#include <KOMO/komo.h>
#include <Kin/kin.h>
#include <Kin/viewer.h>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>

#include <ompl/base/ProblemDefinition_ext.h>

// Planners
#include <ompl/geometric/planners/informedtrees/BITstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.14159
#define tol 1e-2
#define nullArrA (arrA){{}}

namespace ompl
{
    namespace geometric
    {

        OMPL_CLASS_FORWARD(sktp);

        class sktp : public base::Planner
        {
        public: 
            sktp(const base::SpaceInformationPtr &si, std::string name = "sktp");
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
            int C_Dimension;

            void freeMemory();


            void clear() override;

            void setup() override;

            // This is the structure of every leaf in the keyframe tree. The backbone of this algorithm
            class keyframeNode
            {
            private:
                int C_Dimension;
                arr configuration;
                const ompl::base::State* state; // This value is defined only when it is reached
                std::shared_ptr<keyframeNode> parent;
                std::vector<std::shared_ptr<keyframeNode>> childern;
                std::shared_ptr<keyframeNode> closestChild;
                std::shared_ptr<ompl::base::Planner> planner;
                double costToComeHeuristic; // lower-bound cost to reach the node from the root
                double costToGoHeuristic; // lower-bound cost to reach the final goal.
                double bestCost; // cost of te best path found until now from this node to it's children
                double bestCostHeuristic; // Least distance between this node and it's children
                double costToCome; // cost of the best path to node until now
                double costFromParent; // cost to come from parent to this child.
                double distFromNode(std::shared_ptr<keyframeNode> node);
                int calls = 0;
                uint level;
            public:
                bool hasSolution{false}; // True if a feasible path from this node has already been found
                bool reached{false}; // Notes of there is a valid to thi node
                int markedGoals = 0;
                keyframeNode(arr configuration, std::shared_ptr<keyframeNode> parent);
                ~keyframeNode() = default;
                arr get_configuration() {return this->configuration;}
                const ob::State* get_state() {return this->state;}
                std::shared_ptr<keyframeNode> get_parent() {return this->parent;}
                std::shared_ptr<keyframeNode> get_closestChild() {return this->closestChild;}
                double get_costToComeHeuristic()  {return this->costToComeHeuristic;}
                double get_costFromParent()  {return this->costFromParent;}
                void update_bestCostHeuristic();
                double get_costToCome() {return this->costToCome;}
                void update_costToCome();
                std::vector<std::shared_ptr<keyframeNode>> get_children() {return this->childern;}
                void add_child(std::shared_ptr<keyframeNode> child);
                void setasleaf() {this->costToGoHeuristic = 0;}
                void set_dimension(int C_Dimension) {this->C_Dimension = C_Dimension;}
                void set_planner(std::shared_ptr<ompl::base::Planner> planner) {this->planner = planner;}
                void set_state(const ob::State* state) {if (reached) this->state = state;}
                void update_bestCost(double cost);
                std::shared_ptr<ompl::base::Planner> get_planner() {return planner;}
                ob::PlannerStatus plan(const base::PlannerTerminationCondition &ptc);
                uint get_level() {return level;}
                bool is_new{true};
                double get_bestCost() {return bestCost;}
                double get_bestCostHeuristic() {return bestCostHeuristic;}
                int get_attempts() {return calls;}
            };

            std::shared_ptr<og::sktp::keyframeNode> makeRootNode(std::vector<std::string> inputs);

            /* Get the best goal from the given node */
            std::shared_ptr<og::sktp::keyframeNode> get_bestGoal(std::shared_ptr<og::sktp::keyframeNode> node);

            /* If there are no children, growTree adds branchingFactor children, else adds children to make the number multiples of branchingFactor. 
            Does not add any child if it takes more than 2*branchingFator tries to get an answer. */
            void growTree(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
		    arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
            int getCurrentPhase(std::shared_ptr<keyframeNode> node);
			void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start);

		    void initPlanner(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);
            void addNewGoals(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);

            void updateSolution(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);
            void addSolution(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);

            void visualize(std::shared_ptr<og::sktp::keyframeNode> &node);
            void visualizePath(arrA &configs, std::shared_ptr<og::sktp::keyframeNode> &node);

            double calcPriority(std::shared_ptr<og::sktp::keyframeNode> &node) const;

            /* Progress Properties */
            /** \brief Retrieve the best exact-solution cost found as a planner-progress property. */
			double bestCost;
            std::string bestCostProgressProperty() const
            {
                return ompl::toString(bestCost);
            }
            std::shared_ptr<keyframeNode> bestGoalNode; // Retrieve solution from this bestGoalNode

            bool checkKeyframes(arrA keyFrames);
            bool isConfigValid(rai::Configuration& C);
        };  
    } // namespace geometric
} //namespace ompl

#endif