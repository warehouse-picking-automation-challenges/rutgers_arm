// /**
//  * @file manipulation_specification.hpp
//  * 
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  * 
//  * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
//  * 
//  * Email: pracsys@googlegroups.com
//  */

// #pragma once

// #ifndef PRX_MANIPULATION_SPECIFICATION_HPP
// #define	PRX_MANIPULATION_SPECIFICATION_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/planning/problem_specifications/specification.hpp"

// namespace prx
// {
//     namespace util
//     {
//         class parameter_reader_t;
//         class distance_metric_t;
//     }

//     namespace plan
//     {
//         class world_model_t;
//         class sampler_t;
//         class validity_checker_t;
//         class local_planner_t;        
//     }

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {
//             /**
//              * @anchor cloud_manipulation_specification_t
//              *
//              * 
//              * @Author Athanasios Krontiris
//              */
//             class cloud_manipulation_specification_t : public plan::specification_t
//             {

//               public:

//                 cloud_manipulation_specification_t();

//                 virtual ~cloud_manipulation_specification_t();

//                 /**
//                  * @brief Initialize the specification from input parameters.
//                  *
//                  * @param reader The reader for the parameters.
//                  * @param template_reader A template reader for reading template parameters 
//                  */
//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

//                 *
//                  * @brief Clear the plan and trajectory of this query.
//                  *
//                  * Clears the plan and the trajectory of the planning query in order to 
//                  * reuse the planning query with the same start and stopping criterion. 
                 
//                 virtual void clear();

//                 /**
//                  * Prepare the planning specification to be linked to the children
//                  * 
//                  * @brief Prepare the planning specification to be linked to the children
//                  */
//                 virtual void setup(plan::world_model_t * const model);


//                 /**
//                  * @brief Links the state and control spaces that the planning query is working over.
//                  *
//                  * Links the state and control spaces that the planning query is working over.
//                  * 
//                  * @param state_space The state space.
//                  * @param control_space The control space.
//                  */
//                 virtual void link_spaces(const util::space_t* new_state_space, const util::space_t* new_control_space);


//                 /** @brief The z coordinate that the object will be on a surface. */
//                 double z_on_table;
//                 /** @brief The max tries to grasp a pose*/
//                 int max_tries;
//                 /** @brief The different ways to grasp a pose*/
//                 unsigned max_different_grasps;
//                 /** @brief The distance that the manipulator will move after leaving the cup on the ground */
//                 double retract_distance;
//                 /** @brief The distance that the manipulator will raised the cup from the ground*/
//                 double raise_distance;
//                 /** @brief The number poses that will be added in the graph*/
//                 unsigned num_poses;
//                 /** @brief The safe position where the manipulator will return without colliding with anything */
//                 std::vector<double> safe_position;
//             };
//         }
//     }
// }

// #endif
