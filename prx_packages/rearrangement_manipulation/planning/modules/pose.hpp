/**
 * @file pose.hpp
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

#ifndef PRX_POSE_HPP
#define	PRX_POSE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"
#include "prx/simulation/plan.hpp"

namespace prx
{
    namespace sim
    {
        class plan_t;
        class trajectory_t;
    }

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            /**
             * A brief description of this class (no tag required).
             * 
             * A more detailed description of this class with multiple paragraphs delimited by blank lines. 
             */
            class pose_t
            {

              public:
                pose_t();
                pose_t(unsigned id);
                virtual ~pose_t();
                
                void clear(const util::space_t* object_space, const util::space_t* manip_space, const util::space_t* transfer_space);                               
                
                //void remove_set(unsigned index);
                
                bool equal(const util::space_t* object_state_space, const pose_t & other, double precision = PRX_DISTANCE_CHECK) const;

                bool equal(const util::space_t* object_state_space, const sim::state_t * other) const;

                /**
                 * Adds the transfer plans and it will convert the plans for the transit version. 
                 */
                void add_plans(sim::plan_t& reaching_plan, sim::plan_t& retracting_plan);

                unsigned size();            

                /**
                 *  Returns the grasped/ungrasped point at the index. By default will return the released point.
                 */
                sim::state_t* get_state_at(int index, bool is_transfer_mode = false);              

                /**
                 *  Returns the retracted_grasped/retracted point at the index. By default will return the retracted point.
                 */
                sim::state_t* get_retracted_state_at(int index, bool is_transfer_mode = false);              

                /**
                 *  Returns the corresponding reaching plan at the index. By default will return the transit_reaching_plan.
                 */
                void get_reaching_plan_at(sim::plan_t& plan, int index, bool is_transfer_mode = false);

                /**
                 *  Returns the corresponding retracting plan at the index. By default will return the transit_retracting_plan.
                 */
                void get_retracting_plan_at(sim::plan_t& plan, int index, bool is_transfer_mode = false);
                
                void serialize(std::ofstream& fout, const util::space_t* manip_space, const util::space_t* transfer_space, const util::space_t* object_space, unsigned prec = 10);
                
                void serialize(std::ofstream& fout, const util::space_t* object_space, unsigned prec = 10);
                
                void deserialize(std::ifstream& fin, const util::space_t* manip_space, const util::space_t* manip_control_space, const util::space_t* transfer_space, const util::space_t* object_space);
                
                void deserialize(std::ifstream& fin, const util::space_t* object_space);
                
                
                //The id for this pose.
                unsigned pose_id;
                //The state of the object on this pose.
                sim::state_t* state;
                //This is the ungrapsed set of states. 
                std::vector < sim::state_t* > ungrasped_set;
                //This is the retracted set of states. 
                std::vector < sim::state_t* > retracted_set;
                //This is the set for the states that the manipulator can grasp the object on that pose.
                std::vector < sim::state_t* > grasped_set;
                //This is the set for the states for the retracted set with an object in the manipulator. It will be used in the version with sensing.
                std::vector < sim::state_t* > grasped_retracted_set;
                //These are the sets of the trajectories and plans for reaching and retract the 
                //object from the given pose;
                std::vector< sim::plan_t > transit_reaching_plans;
                std::vector< sim::plan_t > transit_retracting_plans;

                std::vector< sim::plan_t > transfer_reaching_plans;
                std::vector< sim::plan_t > transfer_retracting_plans;
                
                std::set<unsigned> constraints;
            };
        }
    }
}

#endif
