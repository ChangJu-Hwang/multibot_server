#pragma once

#include "multibot_util/MAPF_Util.hpp"
#include "multibot_server/AA_SIPP.hpp"

using namespace Low_Level_Engine;

namespace High_Level_Engine
{
    namespace CPBS
    {
        class Solver
        {
        public:
            void solve();
        
        private:
            std::shared_ptr<AA_SIPP::Planner> planner_;

        public:
            Solver();
            ~Solver() {}
        }; // class Solver
    } // namespace CPBS
} // namespace High_Level_Engine