#pragma once

#include "multibot_util/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace High_Level_Engine
{
    namespace CPBS
    {
        struct Node
        {
        public:
            Path::PathSet pathSet_;

            double cost_;
            Time::TimePoint makeSpan_;

            std::list<std::pair<std::string, std::string>> conflicts_;
            std::pair<std::string, std::string> conflict_;
            std::string higher_agentName_;

            Node *parent_;

            unsigned int ignoredConflictNum_ = 0;

        public:
            Node &operator=(const Node &_other)
            {
                this->pathSet_ = _other.pathSet_;

                this->cost_ = _other.cost_;
                this->makeSpan_ = _other.makeSpan_;

                this->conflicts_ = _other.conflicts_;
                this->conflict_ = _other.conflict_;
                this->higher_agentName_ = _other.higher_agentName_;

                return *this;
            }

            friend std::ostream &operator<<(std::ostream &_os, const Node &_node)
            {
                _os << "Cost:     " << _node.cost_ << " / "
                    << "Makespan: " << _node.makeSpan_.count() << "s" << std::endl;

                for (const auto &singlePath : _node.pathSet_)
                {
                    std::cout << singlePath.second << std::endl;
                    std::cout << std::endl;
                }

                return _os;
            }
        
        public:
            Node()
            {
                pathSet_.clear();
                conflicts_.clear();

                parent_ = nullptr;
            }
        }; // struct Node

    } // namespace CPBS
} // namespace High_Level_Engine