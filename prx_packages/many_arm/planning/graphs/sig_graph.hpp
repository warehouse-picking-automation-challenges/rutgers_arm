/**
 * @file sig.hpp
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

#ifndef PRX_SIG_GRAPH_HPP
#define PRX_SIG_GRAPHd_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

namespace prx
{
    namespace util
    {
        class space_point_t;
    }

    namespace packages
    {
        namespace multi_arm
        {
            /**
             *
             */
            class sig_node_t : public util::undirected_node_t
            {
              public:
                sig_node_t()
                {
                    sampling_bounds.resize(3);
                }

                ~sig_node_t()
                {
                }

                const std::vector< util::bounds_t >& get_bounds()
                {
                    return sampling_bounds;
                }

                virtual void serialize(std::ofstream& output_stream, const util::space_t* space)
                {
                    //Don't need this to output right now... not building these programatically
                }

                virtual void deserialize(std::ifstream& input_stream, const util::space_t* space )
                {
                    undirected_node_t::deserialize(input_stream, space);

                    double xmin, xmax, ymin, ymax, zmin, zmax;

                    input_stream >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;

                    // PRX_DEBUG_COLOR("Read state bounds: " << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax, PRX_TEXT_CYAN);
                    sampling_bounds[0].set_bounds(xmin, xmax);
                    sampling_bounds[1].set_bounds(ymin, ymax);
                    sampling_bounds[2].set_bounds(zmin, zmax);
                }


              protected:
                std::vector< util::bounds_t > sampling_bounds; //Bounds for stable grasp sampling for this arm
            };

            /**
             *
             */
            class sig_edge_t : public util::undirected_edge_t
            {
              public:
                sig_edge_t()
                {
                    interaction_bounds.resize(3);
                }

                ~sig_edge_t()
                {
                }

                const std::vector<util::bounds_t>& get_bounds()
                {
                    return interaction_bounds;
                }

                virtual void serialize(std::ofstream& output_stream)
                {
                    //Don't need this to output right now... not building these programatically
                }

                virtual void deserialize(std::ifstream& input_stream)
                {
                    undirected_edge_t::deserialize(input_stream);

                    double xmin, xmax, ymin, ymax, zmin, zmax;

                    input_stream >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;

                    // PRX_DEBUG_COLOR("Read bounds: " << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax, PRX_TEXT_LIGHTGRAY);

                    interaction_bounds[0].set_bounds(xmin, xmax);
                    interaction_bounds[1].set_bounds(ymin, ymax);
                    interaction_bounds[2].set_bounds(zmin, zmax);
                }


            protected:

                //Members
                std::vector< util::bounds_t > interaction_bounds; //Bounds which describe the range interactions can happen

            };
        }
    }
}

#endif //PRX_MA_AUTOMATON_HPP


