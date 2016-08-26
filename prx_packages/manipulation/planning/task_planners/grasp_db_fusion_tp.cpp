/**
 * @file grasp_db_fusion_tp.cpp
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


#include "planning/task_planners/grasp_db_fusion_tp.hpp"

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

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_db_fusion_tp_t, prx::plan::planner_t)

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

            grasp_db_fusion_tp_t::grasp_db_fusion_tp_t()
            {
            }

            grasp_db_fusion_tp_t::~grasp_db_fusion_tp_t()
            {
            }

            void grasp_db_fusion_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                apc_grasping_planner_t::init(reader, template_reader);
                PRX_PRINT("Initializing Grasp DB Fusion Grasping planner ...", PRX_TEXT_CYAN);

                back_plane = parameters::get_attribute_as<double>("back_plane", reader, template_reader);
                bottom_plane = parameters::get_attribute_as<double>("bottom_plane", reader, template_reader);
                bounding_box = parameters::get_attribute_as<std::vector< double > >("bounding_box", reader, template_reader);
                back_plane_offset = parameters::get_attribute_as< double >("back_plane_offset", reader, template_reader);
                bottom_plane_offset = parameters::get_attribute_as< double >("bottom_plane_offset", reader, template_reader);
                output_data_folder = parameters::get_attribute_as< std::string >("output_data_folder", reader, template_reader);
            }

            void grasp_db_fusion_tp_t::setup()
            {

                PRX_INFO_S("Setup grasping planner ...");
                manipulation_model->get_objects(objects);

                PRX_ASSERT( !objects.empty() && objects.size() == 1 );
                movable_body_plant_t* object = objects.front();

                std::string object_type =  object->get_object_type();
                for (std::vector<std::pair<std::string,std::string> >::iterator iter = data_folders.begin(); iter != data_folders.end(); ++iter)
                {
                    std::string data_folder = iter->second;
                    fs::path p(data_folder);
                    fs::directory_iterator end_iter;
                    for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
                    {
                        if (fs::is_regular_file(dir_itr->status()))
                        {
                            std::string file_name = dir_itr->path().string();
                            if(contains(file_name,object_type))
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
                                    grasps[object_type].push_back(grasp_t(config,release_mode,grasp_mode));
                                }
                            }

                        }
                    }
                
                }
            }

            void grasp_db_fusion_tp_t::resolve_query()
            {

                PRX_PRINT("Current manipulation context: "<<manipulation_model->get_current_context(),PRX_TEXT_RED);
                PRX_PRINT("Current manipulation state: "<<manipulation_model->get_state_space()->print_memory(2),PRX_TEXT_RED);

                // Get the object that we are evaluating over

                // Get the correct grasp database based on front and top faces
                std::string fixed_database_orientation;


                std::vector<quaternion_t> object_quats;
                create_quaternion_rotations(object_quats);

                // Get the object and the manipulator
                movable_body_plant_t* object = objects.front();

                // Start state
                state_t* start_state = manipulation_model->get_state_space()->alloc_point();
                state_t* result_state = manipulation_model->get_state_space()->alloc_point();
                state_t* object_state = object->get_state_space()->alloc_point();
                config_t goal_config, object_pose;
                manipulation_model->get_end_effector_local_config(ee_local_config);

                plan_t dummy_plan(manipulation_model->get_control_space());

                // Iterate over the three closest databases
                for( unsigned i = 0; i < directions.size(); ++i )
                {
                    std::string current_topfront_face = directions[i];

                    // ---- Rotate the object based on the current_topfront_face rotation

                    PRX_PRINT ("object_quats[i] : " << object_quats[i], PRX_TEXT_CYAN);
                    object_pose.set_orientation( object_quats[i] );

                    // ---- Make sure object is not colliding

                    // Get the top axis
                    char top_axis = current_topfront_face[1];
                    // Compute the correct ASCII value by subtracting 'x'
                    int top_dimension = top_axis - 'x';

                    // Get the front axis
                    char front_axis = current_topfront_face[4];
                    // Compute the correct ASCII value by subtracting 'x'
                    int front_dimension = front_axis - 'x';

                    PRX_PRINT("Front: "<<front_axis<<" "<<front_dimension,PRX_TEXT_MAGENTA);
                    PRX_PRINT("Top: "<<top_axis<<" "<<top_dimension,PRX_TEXT_MAGENTA);

                    // Set the object position
                    object_pose.set_position(back_plane - back_plane_offset - bounding_box[front_dimension]/2.0, 0.0, bottom_plane + bottom_plane_offset + PRX_ZERO_CHECK + bounding_box[top_dimension]/2.0);

                    // Actually move the object
                    object->set_configuration(object_pose);
                    PRX_PRINT ("1 Object config in resolve query: " << object_pose, PRX_TEXT_BLUE);
                    object->get_configuration(object_pose);
                    PRX_PRINT ("2 Object config in resolve query: " << object_pose<<" for direction: "<<current_topfront_face, PRX_TEXT_BLUE);

                    // SANITY CHECK
                    std::vector<std::string> alternative_faces;
                    std::string top_front_face;
                    determine_top_and_front_faces(object, top_front_face, alternative_faces);
                    PRX_PRINT ("Function face: " << top_front_face << " vs: Iterator Face: " << current_topfront_face, PRX_TEXT_RED);
                    PRX_ASSERT( top_front_face == current_topfront_face );

                    // Loop over each grasp in the full-database
                    foreach(grasp_t grasp, grasps[object->get_object_type()])
                    {

                        //Compute the grasping configuration                
                        goal_config = ee_local_config;
                        // PRX_PRINT("1: "<<goal_config,PRX_TEXT_RED);
                        goal_config.relative_to_global( grasp.relative_config );
                        // PRX_PRINT("2: "<<goal_config,PRX_TEXT_RED);
                        goal_config.relative_to_global( object_pose );

                        manipulation_model->IK( result_state, start_state, goal_config );
                        result_state->memory.back() = grasp.release_mode;
                        manipulation_model->push_state( result_state );
                        //manipulation_model->engage_grasp(dummy_plan, grasp.release_mode, true);
                        //manipulation_model->get_state_space()->copy_to_point(result_state);

                        if ( manipulation_model->valid_state( result_state ) )
                        {
                            grasps[current_topfront_face].push_back(grasp);
                            // PRX_PRINT("Valid state. Object position: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                            // PRX_PRINT("-----Config: "<< grasp.relative_config,PRX_TEXT_BLUE);
                            // PRX_PRINT("3: "<<goal_config,PRX_TEXT_RED);
                        }
                        else//0, 0, -0.0625, 0, 0, 0.707107, -0.707107
                        {
                            // PRX_PRINT("Invalid state. Object position: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                        }
                    }

                    // Serialize database
                    std::vector<bool> non_duplicate_grasp_markers;
                    PRX_PRINT(grasps[current_topfront_face].size(),PRX_TEXT_BLUE);
                    prune_database(grasps[current_topfront_face], current_topfront_face, non_duplicate_grasp_markers);
                    bool serialize_success = serialize_non_duplicate_grasps(grasps[current_topfront_face], current_topfront_face, object->get_object_type(), output_data_folder, non_duplicate_grasp_markers);

                    PRX_ASSERT( serialize_success );

                }

                // // DEBUG

                // for( unsigned i = 0; i < directions.size(); ++i )
                // {
                //     std::string current_topfront_face = directions[i];

                //     PRX_PRINT ("Built Database: " << current_topfront_face << " with size: " << grasps[current_topfront_face].size(), PRX_TEXT_MAGENTA);
                // }
                
                PRX_PRINT ("Finished building fused databases", PRX_TEXT_GREEN);

                exit(1);
            }

            // Helper function that will serialize the entire grasp database
            bool grasp_db_fusion_tp_t::serialize_all_grasps(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, const std::string& object_type, const std::string& output_folder)
            {
                std::vector<bool> dummy_duplicate_marks(grasp_database.size(), true);

                return serialize_non_duplicate_grasps(grasp_database, top_front_face, object_type, output_folder, dummy_duplicate_marks);

            }

            bool grasp_db_fusion_tp_t::serialize_non_duplicate_grasps(const std::vector<grasp_t>& grasp_database, const std::string& top_front_face, const std::string& object_type, const std::string& output_folder, std::vector<bool>& non_duplicate_grasp_markers)
            {

                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1<< "/" << output_folder << "/";
                dir += (s1.str());
                boost::filesystem::path output_dir (dir);
                if (!boost::filesystem::exists(output_dir))
                {
                    boost::filesystem::create_directories( output_dir );
                }

                dir += object_type+top_front_face+".yaml";
                std::ofstream fout;
                fout.open(dir.c_str());

                PRX_ASSERT( fout.good() );

                int good_grasp_count = 0;

                for( unsigned i = 0; i < grasp_database.size(); ++i)
                {
                    // Verify that the current grasp is a non duplicate
                    if (non_duplicate_grasp_markers[i])
                    {
                        // Serialize the corresponding grasp
                        fout << grasp_database[i];
                        good_grasp_count++;
                    }
                }
                PRX_PRINT("Serialized "<<good_grasp_count,PRX_TEXT_CYAN);
                fout.close();
            }

            void grasp_db_fusion_tp_t::create_quaternion_rotations( std::vector< util::quaternion_t >& output_quats )
            {

                quaternion_t _0(0,0,0);
                quaternion_t x45(PRX_PI/4,0,0);
                quaternion_t x90(PRX_PI/2,0,0);
                quaternion_t x135(3*PRX_PI/2.0,0,0);
                quaternion_t x180(PRX_PI,0,0);
                quaternion_t xneg45(-PRX_PI/4,0,0);
                quaternion_t xneg90(-PRX_PI/2,0,0);
                quaternion_t xneg135(-3*PRX_PI/2,0,0);

                quaternion_t y45(0,PRX_PI/4,0);
                quaternion_t y90(0,PRX_PI/2,0);
                quaternion_t y135(0,3*PRX_PI/2.0,0);
                quaternion_t y180(0,PRX_PI,0);
                quaternion_t yneg45(0,-PRX_PI/4,0);
                quaternion_t yneg90(0,-PRX_PI/2,0);
                quaternion_t yneg135(0,-3*PRX_PI/2,0);

                quaternion_t z45(0,0,PRX_PI/4);
                quaternion_t z90(0,0,PRX_PI/2);
                quaternion_t z135(0,0,3*PRX_PI/2.0);
                quaternion_t z180(0,0,PRX_PI);
                quaternion_t zneg45(0,0,-PRX_PI/4);
                quaternion_t zneg90(0,0,-PRX_PI/2);
                quaternion_t zneg135(0,0,-3*PRX_PI/2);

                output_quats.resize(24);

                output_quats[0] = z90*yneg90;
                output_quats[1] = z90*y90;
                output_quats[2] = z180*x90;
                output_quats[3] = z180*xneg90;
                output_quats[4] = z90;
                output_quats[5] = z90*y180;
                output_quats[6] = zneg90*yneg90;
                output_quats[7] = zneg90*y90;
                output_quats[8] = x90;
                output_quats[9] = xneg90;
                output_quats[10] = zneg90;
                output_quats[11] = zneg90*y180;
                output_quats[12] = yneg90;
                output_quats[13] = z180*y90;
                output_quats[14] = zneg90*x90;
                output_quats[15] = z90*xneg90;
                output_quats[16] = z180;
                output_quats[17] = y180;
                output_quats[18] = z180*yneg90;
                output_quats[19] = y90;
                output_quats[20] = z90*x90;
                output_quats[21] = zneg90*xneg90;
                output_quats[22] = _0;
                output_quats[23] = z180*y180;

                int index =0;
                foreach(quaternion_t& quat, output_quats)
                {
                    PRX_INFO_S(index<<" "<<quat<<" "<<directions[index]);
                    index++;
                }

            }
        }
    }
}
