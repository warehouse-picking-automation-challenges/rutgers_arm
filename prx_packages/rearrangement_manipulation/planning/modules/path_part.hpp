/**
 * @file full_path.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_FULL_PATH_HPP
#define	PRX_FULL_PATH_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"

namespace prx
{
    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * A class that maintains all the appropriate information for each part of the rearrangement path. 
             * It will store the position from/to for each object that participate in the rearrangement plan. 
             * We are able to use this information to construct a path for rearranging the objects.
             * 
             * @author Athanasios Krontiris
             */
            class path_part_t
            {

              public:
                unsigned id;
                unsigned pose_from;
                unsigned pose_to;
                unsigned index_from;
                unsigned index_to;
                std::set<unsigned> constraints;
                bool has_plan;

                path_part_t()
                {
                    id = 0;
                    pose_from = 0;
                    pose_to = 0;
                    index_from = 0;
                    index_to = 0;
                    has_plan = false;
                }

                path_part_t(unsigned obj_id, unsigned pose_from, unsigned pose_to, unsigned index_from, unsigned index_to)
                {
                    id = obj_id;
                    this->pose_from = pose_from;
                    this->pose_to = pose_to;
                    this->index_from = index_from;
                    this->index_to = index_to;
                }

                path_part_t(const path_part_t& other)
                {
                    id = other.id;
                    pose_from = other.pose_from;
                    pose_to = other.pose_to;
                    index_from = other.index_from;
                    index_to = other.index_to;
                    has_plan = other.has_plan;
                    constraints = other.constraints;
                }

                void clear()
                {
                    id = 0;
                    pose_from = 0;
                    pose_to = 0;
                    index_from = 0;
                    index_to = 0;
                    has_plan = false;
                    constraints.clear();
                }

                void init(unsigned obj_id, unsigned pose_from, unsigned pose_to)
                {
                    id = obj_id;
                    this->pose_from = pose_from;
                    this->pose_to = pose_to;
                }

                void update(const std::set<unsigned>& constraints, unsigned index_from, unsigned index_to)
                {
                    this->constraints = constraints;
                    this->index_from = index_from;
                    this->index_to = index_to;
                }

                void merge_to_end(const path_part_t& p)
                {
                    pose_to = p.pose_to;
                    index_to = p.index_to;
                    constraints.insert(p.constraints.begin(),p.constraints.end());
                }

                void reset(unsigned obj_id, unsigned pose_from, unsigned pose_to, unsigned index_from, unsigned index_to)
                {
                    id = obj_id;
                    this->pose_from = pose_from;
                    this->pose_to = pose_to;
                    this->index_from = index_from;
                    this->index_to = index_to;
                }

                bool is_constrained_by(unsigned pose)
                {
                    return pose_from == pose || pose_to == pose || constraints.count(pose) == 1;
                }

                void reverse()
                {
                    unsigned tmp = pose_from;
                    pose_from = pose_to;
                    pose_to = tmp;
                    tmp = index_from;
                    index_from = index_to;
                    index_to = tmp;
                }

                std::string print() const
                {
                    std::stringstream output(std::stringstream::out);
                    output << id << ")  from/to:" << pose_from << " -> " << pose_to << "    indices: " << index_from << " -> " << index_to << " C: ";

                    foreach(unsigned i, constraints)
                    {
                        output << i << " , ";
                    }
                    return output.str();
                }
            };
        }
    }
}

#endif
