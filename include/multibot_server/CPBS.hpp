#pragma once

#include <stack>
#include <unordered_set>

#include "multibot_util/MAPF_Util.hpp"
#include "multibot_server/CPBS_Node.hpp"
#include "multibot_server/AA_SIPP.hpp"
#include "multibot_server/AA_SIPP_Motion.hpp"
#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"
#include "multibot_server/Instance_Manager.hpp"

using namespace Low_Level_Engine;
using namespace Instance;

namespace High_Level_Engine
{
    namespace CPBS
    {
        struct StringPair_HashFunc
        {
            std::size_t operator()(const std::pair<std::string, std::string> &_stringPair) const
            {
                auto firstString = std::hash<std::string>{}(_stringPair.first);
                auto secondString = std::hash<std::string>{}(_stringPair.second);

                std::size_t seed = 0;
                seed = seed ^ (firstString + 0x9e3779b9 + (seed << 6) + (seed >> 2));
                seed = seed ^ (secondString + 0x9e3779b9 + (seed << 6) + (seed >> 2));

                return seed;
            }
        }; // struct StringPair_HashFunc

        class Solver : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            std::pair<Path::PathSet, bool> solve();

        private:
            std::pair<Position::Coordinates, Position::Coordinates> restrict_searchSpace();

        private:
            bool generateRoot();
            std::pair<Node *, bool> generateChild(
                Node *_parent, const std::string &_low, const std::string &_high);

            void updateNode(Node *_curNode);
            bool validateResult();

            Node *initChild(
                Node *_parent, const std::string &_low, const std::string &_high);
            void topologicalSort(std::vector<std::string> &_list);
            void topologicalSortUtil(
                const std::string &_agentName, std::vector<std::string> &_list);
            std::vector<std::string> getHigherPriorityAgents(
                const std::string &_target, const std::vector<std::string> &_list);
            std::vector<std::string> getLowerPriorityAgents(
                const std::string &_target, const std::vector<std::string> &_list);

        public:
            void update(const InstanceMsg &_msg)
            {
                agents_ = _msg.first;
            }

        private:
            std::unordered_map<std::string, AgentInstance::Agent> agents_;

            std::stack<Node *> open_;
            std::unordered_set<std::pair<std::string, std::string>, StringPair_HashFunc> priority_graph_;

            Path::PathSet paths_;

            std::shared_ptr<AA_SIPP::Planner> planner_;
            std::shared_ptr<AA_SIPP::Map_Utility> map_utility_ = std::make_shared<AA_SIPP::Map_Utility>();
            std::shared_ptr<AA_SIPP::Motion> motion_manager_ = std::make_shared<AA_SIPP::Motion>(map_utility_);
            std::shared_ptr<AA_SIPP::ConflictChecker> conflict_checker_ = std::make_shared<AA_SIPP::ConflictChecker>(motion_manager_);

        public:
            Solver(std::shared_ptr<Instance_Manager> _instance_manager);
            ~Solver() {}
        }; // class Solver

    } // namespace CPBS
} // namespace High_Level_Engine