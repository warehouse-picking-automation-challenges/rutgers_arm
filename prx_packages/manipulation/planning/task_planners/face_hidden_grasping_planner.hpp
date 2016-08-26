// /**
//  * @file grasping_planner.hpp
//  *
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  *
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
//  *
//  * Email: pracsys@googlegroups.com
//  */

// #pragma once

// #ifndef PRX_FACE_HIDDEN_GRASPING_PLANNER_HPP
// #define	PRX_FACE_HIDDEN_GRASPING_PLANNER_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/utilities/parameters/parameter_reader.hpp"
// #include "prx/utilities/definitions/sys_clock.hpp"
// #include "prx/utilities/boost/hash.hpp"
// #include "prx/utilities/math/configurations/config.hpp"
// #include "prx/utilities/spaces/space.hpp"

// #include "prx/simulation/plan.hpp"
// #include "prx/simulation/trajectory.hpp"

// #include "prx/planning/planner.hpp"
// #include "prx/planning/problem_specifications/specification.hpp"
// #include "prx/planning/queries/query.hpp"

// #include "planning/manipulation_world_model.hpp"
// #include "planning/queries/grasping_query.hpp"
// #include "planning/task_planners/grasping_planner.hpp"

// namespace prx
// {
//     namespace packages
//     {
//         namespace manipulation
//         {
//             /**
//              * Modifies the grasp planner for restricting based on the face of the object on the surface it's resting on.
//              *
//              * @autors Zakary Littlefield
//              */
//             class face_hidden_grasping_planner_t : public grasping_planner_t
//             {

//               public:

//                 face_hidden_grasping_planner_t();
//                 virtual ~face_hidden_grasping_planner_t();

//                 /**
//                  * @copydoc planner_t::setup()
//                  */
//                 virtual void setup();

//                 /**
//                  * @copydoc planner_t::resolve_query()
//                  */
//                 virtual void resolve_query();

//                 virtual int nr_grasps(std::string context_name,movable_body_plant_t* object);            

//               protected:

//                 int determine_face_down(movable_body_plant_t* object);


//                 util::hash_t<std::string, std::vector<std::vector<grasp_entry_t> > >  face_grasps;

//             };
//         }
//     }
// }


// #endif
