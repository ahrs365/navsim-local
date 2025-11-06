

#include <guidance_planner/graph.h>

#include <guidance_planner/types/types.h>

#include <guidance_planner/utils.h>

#include <ros_tools/logging.h>

namespace GuidancePlanner
{

    void Graph::Initialize(const SpaceTimePoint::TVector &start, const Goal &goal)
    {
        nodes_.emplace_back(-1, SpaceTimePoint(start, 0), NodeType::GUARD); // Add start
        start_node_ = &nodes_.back();

        goal_nodes_.clear();
        nodes_.emplace_back(-2, SpaceTimePoint(goal.pos, Config::N), NodeType::GUARD); // Add goal
        goal_nodes_.push_back(&nodes_.back());
    }

    void Graph::Initialize(const SpaceTimePoint::TVector &start, std::vector<Goal> &goals)
    {
        nodes_.emplace_back(-1, SpaceTimePoint(start, 0), NodeType::GUARD); // Add start
        start_node_ = &nodes_.back();

        goal_nodes_.clear();
        for (auto &goal : goals) // Create multiple goals
        {
            nodes_.emplace_back(-(int)goal_nodes_.size() - 2, SpaceTimePoint(goal.pos, Config::N), NodeType::GOAL); // Add goal
            goal_nodes_.push_back(&nodes_.back());
            goal.node = &nodes_.back();
        }
    }

    Node *Graph::AddNode(const Node &node)
    {

        nodes_.emplace_back(node);
        auto *result = &(nodes_.back());

        return result;
    }

    std::vector<Node *> Graph::GetSharedNeighbours(const std::vector<Node *> &nodes)
    {
        ROSTOOLS_ASSERT(nodes.size() == 2, "Expected 2 guards, but the number of nodes are not 2"); // Function only checks neighbours shared between 2 nodes

        std::vector<Node *> shared_neighbours;

        for (auto &neighbour : nodes[0]->neighbours_) // For neighbours of the first node
        {
            for (auto &other_neighbour : nodes[1]->neighbours_) // For neighbours of the second node
            {
                if (neighbour->id_ == other_neighbour->id_ ||
                    (neighbour->type_ == NodeType::GOAL &&
                     other_neighbour->type_ == NodeType::GOAL)) // If neighbours are the same, add them to the shared neighbours
                    shared_neighbours.push_back(neighbour);
            }
        }

        return shared_neighbours;
    }

    int Graph::GetNodeID()
    {
        int return_id = current_id_;
        current_id_++;
        return return_id;
    }

    void Graph::Visualize()
    {
        // Visualization disabled in non-ROS builds
    }

    void Graph::Clear()
    {
        nodes_.clear();
        current_id_ = 0;
    }

    void Graph::Print()
    {
        for (auto &node : nodes_)
        {
            std::cout << "Node: " << node << std::endl;
        }
    }
} // namespace GuidancePlanner