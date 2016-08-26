/**
 * @file prm_star_statistics.hpp
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
#pragma once

#ifndef PRX_RRG_STAR_STATISTICS_HPP
#define	PRX_RRG_STAR_STATISTICS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/statistics/statistics.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @anchor rrg_statistics_t
         *
         * This class gathers statistics for the \ref rrg_t sampling-based motion planner.
         * It gathers information on the size of computed roadmaps.
         *
         * @brief <b> Probabilistic Roadmap Method statistics class. </b>
         *
         * @author Athanasios Krontiris
         */
        class rrg_statistics_t : public util::statistics_t
        {

          public:
            rrg_statistics_t();
            virtual ~rrg_statistics_t();

            /**
             * @copydoc util::statistics_t::get_statistics() 
             */
            virtual std::string get_statistics() const;

            /** @copydoc util::statistics_t::serialize(std::ofstream ) const */
            virtual bool serialize(std::ofstream& stream) const;

            /** @brief The recorded number of vertices from the RRG. */
            double num_vertices;
            /** @brief The recorded number of edges from the RRG. */
            double num_edges;
            /** @brief The connected components of the graph constructed from the RRG.*/
            int num_connected_components;
            /** @brief The number of vertices in each connected component.*/
            std::vector<unsigned> cc_sizes;

        };

    }
}

#endif
