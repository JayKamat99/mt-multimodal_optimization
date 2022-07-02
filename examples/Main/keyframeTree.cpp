/**
 * todo
 * 1) A tree class
 * 2) Expanding from a random point
 * 3) Replicate the method
 * 
 */

// necessary
#include <fstream>

// komo includes
#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include "keyframeTree.h"

std::vector<std::shared_ptr<keyframeNode>> leafNodes;

arrA solveMotion(rai::Configuration &C, std::vector<arr> goal_)
{
    arrA solvedPath = {{1,2,3},{3,4,5}};
    return solvedPath;
}

template <typename T>
void printVector(T &vec)
{
    for (const auto &i : vec)
    {
        std::cout << i << std::endl;
    }
    std::cout << '\n';
}

std::shared_ptr<keyframeNode> makeRootNode(std::vector<std::string> inputs)
{

    std::string filename = inputs.at(0);

    // Get init Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());
    C_Dimension = C.getJointStateDimension();

    // make a root node and return it!
    return std::make_shared<keyframeNode>(C.getJointState(),nullptr);
}

arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr <keyframeNode> start = nullptr)
{
    arrA keyFrames;
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

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

            std::cout << "ref1,ref2: " << ref1 << ref2 << std::endl;

            if (phase % 2 == 0) // pick
            {
                std::cout << "pick" << std::endl;
                komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
            }
            else // place
            {
                std::cout << "place" << std::endl;
                komo.addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
            }
            phase++;
        }

        komo.add_qControlObjective({}, 1);
        komo.add_collision(true, 0.01);

        komo.run_prepare(0);
        komo.optimize();
        keyFrames = komo.getPath_q();
        komo.view(true);
        komo.view_play(true);

        rai::Graph R = komo.getReport(false);
        double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
        std::cout << constraint_violation << std::endl;

        if (constraint_violation < 1){
            keyframesValid = true;
        }
    }
    return keyFrames;
}

arrA planMotion(std::vector<std::string> &inputs, arrA keyFrames)
{
    arrA finalPath;

    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    int phase = 0;
    while (phase < totalPhases)
    {
        std::cout << "phase: " << phase << std::endl;
        std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

        transition transition_{pick};
        if (phase % 2 != 0)
        {
            transition_ = place;
        }

        std::vector<arr> goalConfigs;
        arr goalKeyFrame = keyFrames(phase);
        std::cout << goalKeyFrame << std::endl;
        goalConfigs.push_back(goalKeyFrame.resize(C_Dimension));
        arrA intermediatePath = solveMotion(C, goalConfigs);
        if (intermediatePath == Null_arrA)
        {
            // Your grow tree function comes in here.
        }
        finalPath.append(intermediatePath);
        if (phase % 2 == 0)
            C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
        else
            C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
        phase++;
    }
    return finalPath;
}

void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start)
{
    auto prevNode_ptr = start;
    for(arr keyframe:sequence)
    {
        auto node = std::make_shared<keyframeNode>(keyframe, prevNode_ptr);
        prevNode_ptr = node;
        std::cout << "now " << std::endl;
        std::cout << prevNode_ptr->state << "    " << prevNode_ptr->parent << std::endl;
    }
    leafNodes.push_back(prevNode_ptr);
    std::cout << prevNode_ptr->state << "    " << prevNode_ptr->parent << std::endl;
    std::cout << leafNodes.front()->state << "\t" << leafNodes.front()->parent->parent->state << std::endl;
}

void growTree(std::vector<std::string> &inputs, std::shared_ptr <keyframeNode> start = nullptr)
{
    // for(int i=0; i<3; i++)
    // {
        arrA sequence = sampleKeyframeSequence(inputs, start);
        std::cout << "sequence" << sequence << std::endl;
        addToTree(sequence, start);
    // }
}

arrA getBestSequence() // Need to change this to return the best sequence instead of the first.
{
    auto node_ptr = leafNodes.front();
    std::cout << "I reach here 1" << std::endl;
    arrA sequence;
    std::cout << node_ptr->state << std::endl;
    sequence.append(node_ptr->state);
    std::cout << "I reach here 2" << std::endl;
    while(node_ptr->parent != nullptr)
    {
        node_ptr = node_ptr->parent;
        sequence.append(node_ptr->state);
        std::cout << sequence << std::endl;
    }
    std::cout << "I reach here 3" << std::endl;

    sequence.reverse();
    std::cout << "sequence : " << sequence << std::endl;
    return sequence;
}

int main(int argc, char **argv)
{
    // Default inputs
    std::string inputFile = "../examples/Main/manipulationSequence.txt";
    std::string planner_ = "BITstar";

    // Don't like the default inputs? You can input the inputs you want!
    switch (argc)
    {
    case 3:
        planner_ = argv[2];
    case 2:
        inputFile = argv[1];
    }

    // Read file to get inputs
    std::string line;
    ifstream myfile(inputFile);
    std::vector<std::string> inputs;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            inputs.push_back(line);
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        return 0;
    }

    // build a  start node
    auto root = makeRootNode(inputs);
    std::cout << "root: " << root->state << "\t" << root->parent << std::endl;

    // Get sampled path
    growTree(inputs, root);
    std::cout << "growTree done" << std::endl;
    arrA keyFrames = getBestSequence();
    std::cout << keyFrames << std::endl;
    arrA finalPath = planMotion(inputs, keyFrames);
}