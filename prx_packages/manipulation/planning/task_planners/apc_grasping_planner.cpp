/**
 * @file apc_grasping_planner.cpp
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

#include "planning/task_planners/apc_grasping_planner.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/grasping_query.hpp"
#include "planning/specifications/grasping_specification.hpp"
#include "planning/modules/object_constraints_checker.hpp"

#include <yaml-cpp/yaml.h>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>


#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::apc_grasping_planner_t, prx::plan::planner_t)

namespace fs = boost::filesystem;
using namespace boost::algorithm;

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {

            apc_grasping_planner_t::apc_grasping_planner_t()
            {

            }

            apc_grasping_planner_t::~apc_grasping_planner_t()
            {
            }

            void apc_grasping_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                grasping_planner_t::init(reader, template_reader);
                PRX_PRINT("Initializing APC Grasping planner ...", PRX_TEXT_CYAN);
                directions = {"_x+_y+","_x-_y+","_y+_x+","_y-_x+","_z+_y+","_z-_y+",
                                    "_x+_y-","_x-_y-","_y+_x-","_y-_x-","_z+_y-","_z-_y-",
                                    "_x+_z+","_x-_z+","_y+_z+","_y-_z+","_z+_x+","_z-_x+",
                                    "_x+_z-","_x-_z-","_y+_z-","_y-_z-","_z+_x-","_z-_x-"}; 
            }

            void apc_grasping_planner_t::setup()
            {

                PRX_INFO_S("Setup grasping planner ...");
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);

                foreach(movable_body_plant_t* object, objects)
                {

                    std::string object_type =  object->get_object_type();
                    for (std::vector<std::pair<std::string,std::string> >::iterator iter = data_folders.begin(); iter != data_folders.end(); ++iter)
                    {
                        for (int j = 0; j < 24; ++j)
                        {
                            std::string data_folder = iter->second;
                            std::string hash_string = object_type+iter->first+directions[j];
                            if(grasps[hash_string].size()==0)
                            {
                                fs::path p(data_folder);
                                fs::directory_iterator end_iter;
                                for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
                                {
                                    if (fs::is_regular_file(dir_itr->status()))
                                    {
                                        std::string file_name = dir_itr->path().string();
                                        if(contains(file_name,object_type) && contains(file_name,directions[j]))
                                        {
                                            PRX_PRINT("Opening grasp file: " << file_name, PRX_TEXT_CYAN);
                                            YAML::Node doc = YAML::LoadFile(file_name);
                                            for(unsigned i=0;i<doc.size();i++)
                                            {
                                                config_t config;
                                                YAML::Node node = doc[i]["relative_config"];
                                                config.set_position(node[0].as<double>(),node[1].as<double>(),node[2].as<double>());
                                                config.set_xyzw_orientation(node[3].as<double>(),node[4].as<double>(),node[5].as<double>(),node[6].as<double>());
                                                int release_mode = 1;
                                                if(doc[i]["release_mode"])
                                                    release_mode = doc[i]["release_mode"].as<int>();
                                                int grasp_mode = doc[i]["grasp_mode"].as<int>();
                                                grasps[hash_string].push_back(grasp_t(config,release_mode,grasp_mode));
                                            }
                                        }

                                    }
                                }
                            }
                        }
                    }
                }
            }

            bool apc_grasping_planner_t::evaluate_the_query()
            {
                PRX_PRINT("Current manipulation context: "<<manipulation_model->get_current_context(),PRX_TEXT_RED);
                PRX_PRINT("Current manipulation state: "<<manipulation_model->get_state_space()->print_memory(2),PRX_TEXT_RED);
                bool found_solution = false;
                unsigned grasp_counter = 0;
                unsigned successes = 0;

                // Get the correct grasp database based on front and top faces
                std::string top_front_face;

                // Get the three closest databases
                std::vector<std::string> alternative_faces;
                determine_top_and_front_faces(grasping_query->object, top_front_face, alternative_faces);
                PRX_PRINT ("TOP FRONT FACE: " << top_front_face, PRX_TEXT_RED);
                //We have to iterate over all of the grasps in the database and test them all
                bool all_faces = false;
                int count=0;
                // Iterate over the three closest databases
                hash_t<std::string,int> reasons;
                std::vector<grasp_t> concatenated_grasp_database;
                foreach(std::string current_topfront_face, alternative_faces)
                {
                    if(all_faces || count==0)
                    {   
                        PRX_DEBUG_COLOR("Grasp counter : " << grasp_counter, PRX_TEXT_GREEN);
                        std::vector<grasp_t> current_grasp_database = grasps[grasping_query->object->get_object_type()+manipulation_model->get_current_manipulation_info()->full_arm_context_name+current_topfront_face];
                        concatenated_grasp_database.insert(concatenated_grasp_database.end(), current_grasp_database.begin(), current_grasp_database.end());
                        count++;
                    }
                }

                std::vector<bool> non_duplicate_grasp_markers;
                prune_database(concatenated_grasp_database, top_front_face, non_duplicate_grasp_markers);

                grasp_counter = -1;
                foreach(grasp_t grasp, concatenated_grasp_database)
                {
                    if(!non_duplicate_grasp_markers[++grasp_counter])
                        continue;
                    //PRX_DEBUG_COLOR ("Before world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
                    bool temp_solution = evaluate_the_grasp( &grasp );
                    found_solution = temp_solution || found_solution;
                    successes+=temp_solution;
                    if (!temp_solution)
                    {

                        PRX_PRINT("Failure: " << grasping_query->reason_for_failure, PRX_TEXT_RED);
                        if(reasons.find(grasping_query->reason_for_failure)==reasons.end())
                        {
                            reasons[grasping_query->reason_for_failure]=0;
                        }
                        reasons[grasping_query->reason_for_failure]++;
                    }
                }




          //       foreach(std::string current_topfront_face, alternative_faces)
          //       {
          //           if(all_faces || grasp_counter==0)
          //           {

          //               PRX_PRINT("Grasp face : " << current_topfront_face, PRX_TEXT_GREEN);
          //               // Evaluate each grasp
          //               foreach(grasp_t grasp, grasps[grasping_query->object->get_object_type()+manipulation_model->get_current_manipulation_info()->full_arm_context_name+current_topfront_face])
          //               {
		        //             PRX_DEBUG_COLOR("Grasp counter : " << grasp_counter, PRX_TEXT_GREEN);
		        //             //PRX_DEBUG_COLOR ("Before world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_MAGENTA);
		        //             bool temp_solution = evaluate_the_grasp( &grasp );
		        //             found_solution = temp_solution || found_solution;
		        //             successes+=temp_solution;
		        //             if (!temp_solution)
		        //             {

		        //                 PRX_PRINT("Failure: " << grasping_query->reason_for_failure, PRX_TEXT_RED);
          //                       if(reasons.find(grasping_query->reason_for_failure)==reasons.end())
          //                       {
          //                           reasons[grasping_query->reason_for_failure]=0;
          //                       }
          //                       reasons[grasping_query->reason_for_failure]++;
		        //             }
		        //             //PRX_DEBUG_COLOR ("AFTER world model state: " << manipulation_model->get_full_state_space()->print_memory(3), PRX_TEXT_LIGHTGRAY);
		        //             ++grasp_counter;
		        //         }
		        //     }
		        //     count++;
		        // }
                PRX_INFO_S("Grasping Planner Successes: "<<successes<<"/"<<grasp_counter);
                foreach(std::string name, reasons | boost::adaptors::map_keys)
                {
                    PRX_INFO_S("Reason: "<<name<<" : "<<reasons[name]);
                }
                
                return found_solution;
            }

            void apc_grasping_planner_t::prune_database(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, std::vector<bool>& non_duplicate_grasp_markers)
            {
                // Ensure that the non-duplicate markers has the same cardinality as the grasp database
                if(non_duplicate_grasp_markers.size() != grasp_database.size())
                {
                    // Initially all entries are marked is non-duplicates
                    non_duplicate_grasp_markers.resize(grasp_database.size(), true);
                }

                // Check each grasp entry if it is a duplicate
                for (unsigned i = 0; i < grasp_database.size(); ++i)
                {
                    // If the current grasp is not already marked as a duplicate entry
                    if (non_duplicate_grasp_markers[i])
                    {
                        const grasp_t* current_grasp = &grasp_database[i];
                        // We now check all the remaining grasps
                        for (unsigned j = i+1; j < grasp_database.size(); ++j)
                        {
                            // Evaluate the next grasp
                            const grasp_t* grasp_to_evaluate = &grasp_database[j];

                            // First we check the grasp modes
                            if (grasp_to_evaluate->grasping_mode == current_grasp->grasping_mode)
                            {
                                if (grasp_to_evaluate->release_mode == current_grasp->release_mode)
                                {
                                    // The modes are the same, now we check the relative config
                                    if(grasp_to_evaluate->relative_config.is_approximate_equal(current_grasp->relative_config))
                                    {
                                        // Mark the grasp entry j to be a duplicate entry 
                                        non_duplicate_grasp_markers[j] = false;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            void apc_grasping_planner_t::determine_top_and_front_faces(movable_body_plant_t* object, std::string& top_front_face, std::vector<std::string>& alternative_faces)
            {
                alternative_faces.clear();

                if(original_object_state == NULL)
                    original_object_state = object->get_state_space()->alloc_point();
                else
                    object->get_state_space()->copy_to_point(original_object_state);

                /** Get configuration of object **/
                config_t config;
                object->get_configuration(config);

                PRX_PRINT ("Object config in determine faces: " << config, PRX_TEXT_GREEN);

                /** Calculate rotation to apply to axis **/
                quaternion_t quat;
                quat.zero();
                quat/=config.get_orientation();

                /** Find Top Axis **/
                vector_t top_axis(0,0,1);
                std::string top_face;
                determine_face(top_axis, quat, top_face);

                /** Find Front Axis **/
                vector_t front_axis(-1,0,0);
                std::string front_face;
                determine_face(front_axis, quat, front_face);

                /** We've found the closest database, let's remember it now **/
                top_front_face = top_face+front_face;

                /** We want the first database evaluate to be the closest one **/
                alternative_faces.push_back(top_front_face);

                /** Let's add the remaining "nearest" databases **/
                if (front_face == "_x-" || front_face == "_x+")
                {
                    if (top_face == "_z-" || top_face == "_z+")
                    {
                        alternative_faces.push_back(top_face+"_y+");
                        alternative_faces.push_back(top_face+"_y-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_z+");
                        alternative_faces.push_back(top_face+"_z-");
                    }

                }
                else if (front_face == "_y-" || front_face == "_y+")
                {
                    if (top_face == "_z-" || top_face == "_z+")
                    {
                        alternative_faces.push_back(top_face+"_x+");
                        alternative_faces.push_back(top_face+"_x-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_z+");
                        alternative_faces.push_back(top_face+"_z-");
                    }

                }
                else if (front_face == "_z-" || front_face == "_z+")
                {
                    if (top_face == "_x-" || top_face == "_x+")
                    {
                        alternative_faces.push_back(top_face+"_y+");
                        alternative_faces.push_back(top_face+"_y-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_x+");
                        alternative_faces.push_back(top_face+"_x-");
                    }

                }


            }


            void apc_grasping_planner_t::determine_face(const vector_t& axis, const quaternion_t& rotation, std::string& face)
            {
                /** Find Top Axis **/
                vector_t rotated_axis = rotation.qv_rotation(axis);
                double max_val = fabs(rotated_axis[0]);
                int max_index = 0;
                face = "_x";
                if(fabs(rotated_axis[1]) > max_val)
                {
                    max_val = fabs(rotated_axis[1]);
                    max_index = 1;
                    face = "_y";
                }
                if(fabs(rotated_axis[2]) > max_val)
                {
                    max_val = fabs(rotated_axis[2]);
                    max_index = 2;
                    face = "_z";
                }
                // PRX_INFO_S(max_index);
                if(rotated_axis[max_index]<0)
                {
                    face+= "-";
                }
                else
                {
                    face+= "+";
                }
            }
        }
    }
}
