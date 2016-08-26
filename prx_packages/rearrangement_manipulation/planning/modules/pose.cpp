/**
 * @file pose.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/pose.hpp"
#include "prx/simulation/trajectory.hpp"
#include <fstream>
#include <sstream>
// #include <bits/stl_set.h>

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            pose_t::pose_t()
            {
                state = NULL;
            }

            pose_t::pose_t(unsigned id)
            {
                state = NULL;
                pose_id = id;
            }

            pose_t::~pose_t() { }

            void pose_t::clear(const space_t* object_space, const space_t* manip_space, const space_t* transfer_space)
            {
                object_space->free_point(state);

                foreach(state_t* s, ungrasped_set)
                {
                    manip_space->free_point(s);
                }
                ungrasped_set.clear();

                foreach(state_t* s, retracted_set)
                {
                    manip_space->free_point(s);
                }
                retracted_set.clear();

                foreach(state_t* s, grasped_set)
                {
                    transfer_space->free_point(s);
                }
                grasped_set.clear();

                foreach(state_t* s, grasped_retracted_set)
                {
                    transfer_space->free_point(s);
                }
                grasped_retracted_set.clear();

                transfer_reaching_plans.clear();
                transfer_retracting_plans.clear();
                transit_reaching_plans.clear();
                transit_retracting_plans.clear();
                constraints.clear();
            }

            // void pose_t::remove_set(unsigned index)
            // {
            //     ungrasped_set.erase(ungrasped_set.begin() + index);
            //     retracted_set.erase(retracted_set.begin() + index);
            //     grasped_set.erase(grasped_set.begin() + index);
            //     reaching_plans.erase(reaching_plans.begin() + index);
            // }

            bool pose_t::equal(const util::space_t* object_state_space, const pose_t & other, double precision) const
            {
#ifndef NDEBUG
                double dist = object_state_space->equal_points(state, other.state, precision);
                PRX_DEBUG_COLOR("dists: " << dist, PRX_TEXT_BLUE);
#endif
                return object_state_space->equal_points(state, other.state, precision);
            }

            bool pose_t::equal(const util::space_t* object_state_space, const sim::state_t * other) const
            {
#ifndef NDEBUG                
                double dist = object_state_space->distance(state, other);
                PRX_DEBUG_COLOR("dists: " << dist, PRX_TEXT_BLUE);
#endif
                return object_state_space->equal_points(state, other, 1e-3);
            }

            void pose_t::add_plans(plan_t& reaching_plan, plan_t& retracting_plan)
            {
                transfer_reaching_plans.push_back(reaching_plan);
                transfer_retracting_plans.push_back(retracting_plan);

                transit_reaching_plans.push_back(reaching_plan);
                foreach(plan_step_t step, transit_reaching_plans.back())                
                {
                    step.control->memory.back() = 0;                
                }

                transit_retracting_plans.push_back(retracting_plan);
                foreach(plan_step_t step, transit_retracting_plans.back())                
                {
                    step.control->memory.back() = 0;                
                }
            }

            unsigned pose_t::size()
            {
                PRX_ASSERT(grasped_set.size() == ungrasped_set.size());
                return ungrasped_set.size();
            }

            state_t* pose_t::get_state_at(int index, bool is_transfer_mode)
            {
                if(is_transfer_mode)
                    return grasped_set[index];
                return ungrasped_set[index];
            }

            state_t* pose_t::get_retracted_state_at(int index, bool is_transfer_mode)
            {
                if(is_transfer_mode)
                    return grasped_retracted_set[index];
                return retracted_set[index];
            }

            void pose_t::get_reaching_plan_at(sim::plan_t& plan, int index, bool is_transfer_mode)
            {
                if(is_transfer_mode)
                    plan = transfer_reaching_plans[index];
                else
                    plan = transit_reaching_plans[index];
            }

            void pose_t::get_retracting_plan_at(sim::plan_t& plan, int index, bool is_transfer_mode)
            {
                if(is_transfer_mode)
                    plan = transfer_retracting_plans[index];
                else
                    plan = transit_retracting_plans[index];
            }

            void pose_t::serialize(std::ofstream& fout, const space_t* manip_space, const util::space_t* transfer_space, const space_t* object_space, unsigned prec)
            {
                fout << object_space->serialize_point(state, prec) << std::endl;
                fout << ungrasped_set.size() << std::endl;

                foreach(state_t* st, ungrasped_set)
                {
                    fout << manip_space->serialize_point(st, prec) << std::endl;
                }

                foreach(state_t* st, retracted_set)
                {
                    fout << manip_space->serialize_point(st, prec) << std::endl;
                }

                foreach(state_t* st, grasped_set)
                {
                    fout << transfer_space->serialize_point(st, prec) << std::endl;
                }
                
                foreach(state_t* st, grasped_retracted_set)
                {
                    fout << transfer_space->serialize_point(st, prec) << std::endl;
                }

                fout << constraints.size() << std::endl;
                if( constraints.size() > 0 )
                {
                    foreach(unsigned c, constraints)
                    {
                        fout << c << " ";
                    }
                    fout << std::endl;
                }
            }

            void pose_t::serialize(std::ofstream& fout, const space_t* object_space, unsigned prec)
            {
                fout << object_space->serialize_point(state, prec) << std::endl;

                fout << constraints.size() << std::endl;
                if( constraints.size() > 0 )
                {

                    foreach(unsigned c, constraints)
                    {
                        fout << c << " ";
                    }
                    fout << std::endl;
                }
            }            

            void pose_t::deserialize(std::ifstream& fin, const space_t* manip_state_space, const space_t* manip_control_space, const util::space_t* transfer_space, const space_t* object_space)
            {
                unsigned size = 0;
                state = object_space->deserialize_point(fin);
                PRX_DEBUG_COLOR("Read pose: " << object_space->print_point(state, 16), PRX_TEXT_GREEN);
                fin >> size;

                ungrasped_set.resize(size);
                retracted_set.resize(size);
                grasped_set.resize(size);
                grasped_retracted_set.resize(size);
                for( unsigned i = 0; i < size; ++i )
                    ungrasped_set[i] = manip_state_space->deserialize_point(fin);

                for( unsigned i = 0; i < size; ++i )
                    retracted_set[i] = manip_state_space->deserialize_point(fin);

                for( unsigned i = 0; i < size; ++i )
                    grasped_set[i] = transfer_space->deserialize_point(fin);
                
                for( unsigned i = 0; i < size; ++i )
                    grasped_retracted_set[i] = transfer_space->deserialize_point(fin);

//                fin >> size;
//                reaching_plans.resize(size);
//                for( unsigned i = 0; i < size; ++i )
//                {
//                    reaching_plans[i].link_control_space(manip_control_space);
//                    reaching_plans[i].read_from_stream(fin);
//                }

                fin >> size;
                unsigned c;
                for( unsigned i = 0; i < size; ++i )
                {
                    fin >> c;
                    constraints.insert(c);
                }
            }
            
            void pose_t::deserialize(std::ifstream& fin, const space_t* object_space)
            {
                unsigned size = 0;
                state = object_space->deserialize_point(fin);
                PRX_DEBUG_COLOR("Read pose: " << object_space->print_point(state, 16), PRX_TEXT_GREEN);

                fin >> size;
                unsigned c;
                for( unsigned i = 0; i < size; ++i )
                {
                    fin >> c;
                    constraints.insert(c);
                }
            }


        }
    }
}
