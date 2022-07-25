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
#include <Kin/viewer.h>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>

// Planners
#include <ompl/geometric/planners/informedtrees/BITstar.h>

// #define VISUALIZE

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.14159
#define tol 1e-2

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
    void debug()
    {
        std::cout << nlp.getDimension() << std::endl;
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
            std::shared_ptr<ValidityCheckWithKOMO> checker_;

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
                arr state;
                std::shared_ptr<keyframeNode> parent;
                std::vector<std::shared_ptr<keyframeNode>> childern;
                std::shared_ptr<ompl::base::Planner> planner;
                double costToComeHeuristic; // lower-bound cost to reach the node
                double bestCostToCome; // cost of the best path to node until now
                double costToGoHeuristic; // lower-bound cost to reach the final goal.
                double calcDistHeuristic(); // calculates eucledian distance from parent and adds it to the best cost to parent if available, else to the dist heuristic to the parent
                double distFromNode(std::shared_ptr<keyframeNode> node);
                uint level;
            public:
                keyframeNode(arr state, std::shared_ptr<keyframeNode> parent);
                ~keyframeNode() = default;
                arr get_state() {return this->state;}
                std::shared_ptr<keyframeNode> get_parent() {return this->parent;}
                double get_costToComeHeuristic()  {return this->costToComeHeuristic;}
                // double get_bestCostToCome() {return this->bestCostToCome;}
                std::vector<std::shared_ptr<keyframeNode>> get_children() {return this->childern;}
                void add_child(std::shared_ptr<keyframeNode> child, bool updateCosts);
                void setasleaf() {this->costToGoHeuristic = 0;}
                void set_dimension(int C_Dimension) {this->C_Dimension = C_Dimension;}
                void set_planner(std::shared_ptr<ompl::base::Planner> planner) {this->planner = planner;}
                std::shared_ptr<ompl::base::Planner> get_planner() {return planner;}
                ob::PlannerStatus plan();
                int penalty;
                uint get_level() {return level;}
                bool is_new{true};
            };

            std::shared_ptr<og::sktp::keyframeNode> makeRootNode(std::vector<std::string> inputs);

            void growTree(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
		    arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr<keyframeNode> start);
            int getCurrentPhase(std::shared_ptr<keyframeNode> node);
			void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start);

		    void initPlanner(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);
            void addNewGoals(std::shared_ptr<ompl::geometric::sktp::keyframeNode> node);

            void visualize(std::shared_ptr<og::sktp::keyframeNode> &node);

            /* Progress Properties */
            /** \brief Retrieve the best exact-solution cost found as a planner-progress property. */
			ompl::base::Cost bestCost;
            std::string bestCostProgressProperty() const
            {
                return ompl::toString(bestCost.value());
            }

        };  
    } // namespace geometric
} //namespace ompl

#endif