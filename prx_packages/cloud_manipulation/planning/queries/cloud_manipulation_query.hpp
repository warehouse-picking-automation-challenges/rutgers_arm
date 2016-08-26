// /**
//  * @file manipulation_query.hpp
//  * 
//  * @copyright Software License Agreement (BSD License)
//  * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
//  * All Rights Reserved.
//  * For a full description see the file named LICENSE.
//  * 
//  * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
//  * 
//  * Email: pracsys@googlegroups.com
//  */
// #pragma once

// #ifndef PRX_MANIPULATION_QUERY_HPP
// #define	PRX_MANIPULATION_QUERY_HPP

// #include "prx/utilities/definitions/defs.hpp"
// #include "prx/simulation/plan.hpp"
// #include "prx/simulation/state.hpp"
// #include "prx/simulation/trajectory.hpp"
// #include "prx/planning/queries/motion_planning_query.hpp"

// namespace prx
// {
//     namespace util
//     {
//         class parameter_reader_t;
//     }

//     namespace plan
//     {
//         class stopping_criteria_t;
//     }

//     namespace packages
//     {
//         namespace cloud_manipulation
//         {

//             /**
//              * @anchor cloud_manipulation_query_t
//              *
//              * This class represents a problem instance to be solved by a manipulator task planner.
//              * It is a self-contained class which holds the starts/goals poses for the objects.
//              *
//              * @brief <b> General query for manipulation task planners. </b>
//              *
//              * @author Athanasios Krontiris
//              */
//             class cloud_manipulation_query_t : public plan::motion_planning_query_t
//             {

//               public:

//                 cloud_manipulation_query_t();
//                 virtual ~cloud_manipulation_query_t();

//                 /**
//                  * @brief Initialize the planing query from input parameters.
//                  *
//                  * @param reader The reader for the parameters.
//                  * @param template_reader A template reader for reading template parameters 
//                  */
//                 virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

//                 *
//                  * @brief Returns the initial poses for the objects.
//                  *
//                  * @return The initial poses for the objects.
                 
//                 virtual const std::vector< std::vector<double> >* get_initial_poses() const;
//                 virtual const std::vector<double>& get_initial_pose() const;

//                 /**
//                  * @brief Returns the target poses for the objects.
//                  *
//                  * @return The target poses for the objects.
//                  */
//                 virtual const std::vector< std::vector<double> >* get_target_poses() const;
//                 virtual const std::vector<double>& get_target_pose() const;

//                 /**
//                  * @brief Returns the extra poses for the objects.
//                  *
//                  * @return The extra poses for the objects.
//                  */
//                 virtual const std::vector< std::vector<double> >* get_extra_poses() const;

//                 /**
//                  * Returns the total number of all the poses in this query.
//                  * 
//                  * @return The total number of all the poses in this query.
//                  */
//                 virtual unsigned total_poses() const;

//                 /**
//                  * Returns the total number of both the initial and goal poses. 
//                  * 
//                  * @return The total number of both the initial and goal poses. 
//                  */
//                 virtual unsigned important_poses() const;
                
//                 virtual void set_initial_poses(std::vector< std::vector<double> > init_poses);
                
//                 virtual void set_target_poses(std::vector< std::vector<double> > tgt_poses);

//               protected:
//                 /** @brief Vector containing all possible initial poses in double form */
//                 std::vector< std::vector<double> > initial_poses;
//                 /** @brief Vector containing all possible target poses in double form */
//                 std::vector< std::vector<double> > target_poses;
//                 /** @brief Vector containing all extra poses in double form */
//                 std::vector< std::vector<double> > extra_poses;
//             };
//         }
//     }
// }

// #endif

