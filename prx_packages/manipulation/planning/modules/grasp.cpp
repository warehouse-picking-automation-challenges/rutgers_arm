/**
 * @file grasp.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/grasp.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace manipulation
        {
            grasp_t::grasp_t()
            {
                release_mode = -1;
                grasping_mode = -1;
                grasp_score = 0.0;
                tie_breaking_score = 0.0;
            }

            grasp_t::grasp_t(const util::config_t& conf, double open, double grasp, double score, double tie_breaking)
            {
                relative_config = conf;
                release_mode = open;
                grasping_mode = grasp;
                grasp_score = score;
                tie_breaking_score = tie_breaking;
            }

            void grasp_t::clear()
            {
                release_mode = -1;
                grasping_mode = -1;
                relative_config.zero();
            }

            grasp_t& grasp_t::operator=(const grasp_t& g)
            {
                relative_config = g.relative_config;
                release_mode = g.release_mode;
                grasping_mode = g.grasping_mode;
                grasp_score = g.grasp_score;
                tie_breaking_score = g.tie_breaking_score;
                return *this;
            }

            bool grasp_t::operator==(const grasp_t& g)
            {
                return (release_mode == g.release_mode && grasping_mode == g.grasping_mode && relative_config.is_approximate_equal(g.relative_config));
            }

            bool grasp_t::operator < (const grasp_t& rhs) const
            {
                if (std::fabs(grasp_score - rhs.grasp_score) <= PRX_ZERO_CHECK)
                    return tie_breaking_score > rhs.tie_breaking_score;
                return (grasp_score < rhs.grasp_score);
            }
            bool grasp_t::operator > (const grasp_t& rhs) const
            {
                // PRX_PRINT ("Left grasp score: " << grasp_score << " vs: " << rhs.grasp_score, PRX_TEXT_CYAN);
                // PRX_PRINT ("Left tie breaking score: " << tie_breaking_score << " vs: " << rhs.tie_breaking_score, PRX_TEXT_CYAN);
                if (std::fabs(grasp_score - rhs.grasp_score) <= PRX_ZERO_CHECK)
                    return tie_breaking_score < rhs.tie_breaking_score;
                return (grasp_score > rhs.grasp_score);
            }

            std::ostream& operator<<(std::ostream& os, const grasp_t& grasp)
            {
                os << "-";
                os << std::endl << "  relative_config: " << grasp.relative_config.serialize();
                os << std::endl << "  grasp_mode:  " << grasp.grasping_mode;
                os << std::endl << "  release_mode:  " << grasp.release_mode;
                if (grasp.grasp_score >= 0.0)
                {
                    os << std::endl << "  grasp_score:  " << grasp.grasp_score;
                }
                if (grasp.tie_breaking_score >= 0.0)
                {
                    os << std::endl << "  tie_breaking_score:  " << grasp.tie_breaking_score;
                }

                os << std::endl;
            }
        }

    }
}
