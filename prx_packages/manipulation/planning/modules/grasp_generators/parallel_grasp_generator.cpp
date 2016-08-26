/**
 * @file parallel_grasp_generator.cpp
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


#include "planning/modules/grasp_generators/parallel_grasp_generator.hpp"

#include "simulation/workspace_trajectory.hpp"

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
#include "prx/utilities/goals/multiple_goal_states.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <pluginlib/class_list_macros.h>

#include <algorithm>


PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::parallel_grasp_generator_t, prx::packages::manipulation::grasp_generator_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {
            parallel_grasp_generator_t::parallel_grasp_generator_t()
            {
            }

            parallel_grasp_generator_t::~parallel_grasp_generator_t()
            {
            }
            void parallel_grasp_generator_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                grasp_generator_t::init(reader, template_reader);

                default_ee_normal.resize(3);
                default_ee_normal = parameters::get_attribute_as<vector_t>("default_ee_normal", reader, template_reader);
                manual_grasp_mode = parameters::get_attribute_as<unsigned>("manual_grasp_mode", reader, template_reader);
                manual_release_mode = parameters::get_attribute_as<unsigned>("manual_release_mode", reader, template_reader);
                manual_angle_increment = parameters::get_attribute_as<double>("manual_angle_increment", reader, template_reader, 0.0);
                // nr_grasps_to_sample = parameters::get_attribute_as<unsigned>("nr_grasps_to_sample", reader, template_reader, 100);
                total_nr_grasps = parameters::get_attribute_as<unsigned>("total_nr_grasps", reader, template_reader, 100);
                grasp_entire_mesh = parameters::get_attribute_as<bool>("grasp_entire_mesh", reader, template_reader, false);


                PRX_DEBUG_COLOR("ee_normal: " << default_ee_normal << " and increment: " << manual_angle_increment, PRX_TEXT_MAGENTA);


                if(parameters::has_attribute("restricted_normals", reader, template_reader))
                {
                    foreach( const parameter_reader_t* r, parameters::get_list("restricted_normals", reader, template_reader) )
                    {
                        restricted_normals.push_back(parameters::get_attribute_as<vector_t>("normal", r, template_reader));
                        PRX_DEBUG_COLOR( "Added restricted normal:  " << restricted_normals.back(), PRX_TEXT_LIGHTGRAY );
                    }
                }

                if(parameters::has_attribute("preferred_normals", reader, template_reader))
                {
                    foreach( const parameter_reader_t* r, parameters::get_list("preferred_normals", reader, template_reader) )
                    {
                        preferred_normals.push_back(parameters::get_attribute_as<vector_t>("normal", r, template_reader));
                        PRX_DEBUG_COLOR( "Added preferred normal:  " << preferred_normals.back(), PRX_TEXT_LIGHTGRAY );
                    }
                }

                limiting_normal = parameters::get_attribute_as<bool>("limiting_normal", reader, template_reader, false);
                hand_long_axis = parameters::get_attribute_as<vector_t>("hand_long_axis", reader, template_reader, vector_t(1,0,0));
                limiting_axis = parameters::get_attribute_as<vector_t>("limiting_axis", reader, template_reader, vector_t(1,0,0));


            }
            const std::vector<grasp_t>&  parallel_grasp_generator_t::compute_grasps(const util::geometry_t* ee_geom, const util::config_t& ee_local_config, movable_body_plant_t* object)
            {
                local_ee_config = ee_local_config;
                PRX_ASSERT(manipulation_model != NULL);
                // if (grasp_entire_mesh)
                // {
                //     return compute_grasps_on_entire_mesh(ee_geom, object);
                // }
                // else
                // {
                    return compute_grasps_on_descriptors(ee_geom, object);
                // }
            }



            const std::vector<grasp_t>&  parallel_grasp_generator_t::compute_grasps_on_descriptors(const util::geometry_t* ee_geom, movable_body_plant_t* object)
            {
                PRX_DEBUG_COLOR("Computing grasps over graph descriptors", PRX_TEXT_BLUE);

                config_t object_config;
                object->get_configuration(object_config);
                PRX_WARN_S ("Object configuration: " << object_config);

                double shelf_bottom [4] = { 0.6, 0.87, 1.1, 1.33 };

                double obj_x, obj_y, obj_z;
                object_config.get_position(obj_x, obj_y, obj_z);

                // for (int i = 0; i < 4; ++i)
                // {
                //     if (obj_z - shelf_bottom[i] >= 0.04 && obj_z - shelf_bottom[i] <= 0.1)
                //     {
                //         vector_t temp_vec = preferred_normals[0];
                //         preferred_normals[0] = preferred_normals[1];
                //         preferred_normals[1] = temp_vec;
                //         break;                             
                //     }     
                // } 


                std::string hash_string = object->get_object_type();
                PRX_DEBUG_COLOR("Object type: " << hash_string, PRX_TEXT_MAGENTA);

                

                // TOOD: Make these end-effector specific
                const grasp_descriptor_t* descriptor = object->get_grasp_descriptor();
                const std::vector< grasp_volume_t* >& grasp_volumes = descriptor->get_volumes();
                const std::vector< grasp_surface_t* >& grasp_surfaces = descriptor->get_surfaces();

                if(grasp_volumes.empty())
                {
                    PRX_ERROR_S("No grasp volume for "<<end_effector_name);
                }

                // for (unsigned grasp_index = 0; grasp_index < nr_grasps_to_sample; ++grasp_index)
                std::string old_context = manipulation_model->get_current_context();
                state_t* old_state = manipulation_model->get_state_space()->alloc_point();
                manipulation_model->use_context(ffee_context_name);
                state_t* ffee_state;
                if (!ffee_context_name.empty())
                {
                    ffee_state = manipulation_model->get_state_space()->alloc_point();
                }

                int max_tries=1000;
                for (unsigned grasp_index = 0; grasp_index < total_nr_grasps && --max_tries>0 && grasp_volumes.size()>0; )
                {
                    // PRX_INFO_S("-----------------------------");
                    grasp_volume_t* volume = grasp_volumes[uniform_int_random(0,grasp_volumes.size()-1)];

                    config_t volume_config = volume->relative_config;

                    double z_sample = uniform_random(-volume->height/2,volume->height/2);

                    // PRX_WARN_S("Sampled from "<<volume->height<<", "<<z_sample);


                    vector_t sample_point(0,0,z_sample);
                    double angle = uniform_random(0,360)*PRX_PI/180;
                    quaternion_t rotation(vector_t(0,0,1), angle);
                    bool flip = uniform_random(0,1)>0.5;
                    quaternion_t random_flip(0,(flip?0:1),0,(flip?1:0));
                    quaternion_t sample_orientation = rotation * random_flip * quaternion_t(0.7071, 0, 0, 0.7071);
                    // quaternion_t ortho_quat(0.7071, 0, 0, 0.7071);

                    config_t sample_config(sample_point, sample_orientation);
                    sample_config.relative_to_global(volume_config);
                    config_t grasp_config = sample_config;
                    

                    config_t global_grasp_config = grasp_config;
                    global_grasp_config.relative_to_global(object_config);

                    bool add_grasp = true;
                    if(limiting_normal)
                    {
                        vector_t gripper_axis = hand_long_axis;
                        //vector_t evaluation_axis = gripper_axis;
                        vector_t rotated_axis = global_grasp_config.get_orientation().qv_rotation(gripper_axis);
                        

                        if(rotated_axis.dot_product(limiting_axis)<0.7017)
                        {
                            // PRX_INFO_S("Limiting normal:: "<<rotated_axis<<" vs "<<gripper_axis<<" computer from "<<true_rotation);
                            // PRX_WARN_S("Computed grasp config "<<grasp_config<<" was rejected by limiting normals during nr_tries.");
                            add_grasp=false;
                        }
                    }
                    
                    if(add_grasp)
                    {
                        PRX_PRINT("Generated grasp: "<<grasp_config, PRX_TEXT_MAGENTA);
                        ++grasp_index;
                        double distance_to_object_center = global_grasp_config.get_position().distance(object_config.get_position());
                        
		               if (!ffee_context_name.empty())
		                {

		                    config_t test_config = local_ee_config;
		                    test_config.relative_to_global(grasp_config);
		                    test_config.relative_to_global(object_config);
		                    
		                    config_to_state(grasp_config, ffee_state);

		                    if (manipulation_model->valid_state(ffee_state))
		                    {
		                        PRX_DEBUG_COLOR("Found a valid grasp!", PRX_TEXT_CYAN);
		                        grasps[ hash_string ].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode,0,distance_to_object_center));
		                    }
		                    else
		                    {
		                        PRX_DEBUG_COLOR("INVALID GRASP!", PRX_TEXT_RED);
		                    }
		                }
		                else
		                {
		                    grasps[ hash_string ].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode,0,distance_to_object_center));
		                }
                    }
                    
                }

                // Restore old context and state
                if (!ffee_context_name.empty())
                {
                    manipulation_model->get_state_space()->free_point(ffee_state);
                }
                manipulation_model->use_context(old_context);
                manipulation_model->get_state_space()->copy_from_point(old_state);
                manipulation_model->get_state_space()->free_point(old_state);


                

                std::vector<grasp_t> grasp_vector = grasps[hash_string];
                std::sort(grasp_vector.begin(), grasp_vector.end(), std::greater<grasp_t>());


                if (save_grasps_to_file)
                {
                    PRX_DEBUG_COLOR("\n\n---Saving grasps to file----\n\n", PRX_TEXT_CYAN);
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string filename(w);
                    filename+="/prx_output/" + end_effector_name + "_" + hash_string +".yaml";

                    std::ofstream fout;
                    fout.open(filename.c_str());

                    for(unsigned grasp_counter = 0; grasp_counter < grasp_vector.size(); ++grasp_counter)
                    {
                        fout << grasp_vector[grasp_counter];
                        // PRX_INFO_S("Grasp: "<<grasp_vector[grasp_counter]);
                    }

                    fout.close();
                }

                return grasps[hash_string];
            }

            const std::vector<grasp_t>&  parallel_grasp_generator_t::compute_grasps_on_entire_mesh(const util::geometry_t* ee_geom, movable_body_plant_t* object)
            {
                PRX_DEBUG_COLOR("Computing grasps over entire mesh", PRX_TEXT_GREEN);

                config_t object_config;
                object->get_configuration(object_config);

                geom_map_t object_geoms; object->update_phys_geoms(object_geoms);
                std::string hash_string = object->get_object_type();


                if (manual_angle_increment <= PRX_ZERO_CHECK)
                {
                    PRX_WARN_S ("Manual angle increment was not set. Setting to 15.0");
                    manual_angle_increment = 15.0;
                }

                foreach(geometry_t obj_geom, object_geoms | boost::adaptors::map_values)
                {
                    std::vector<vector_t> normal_vectors;
                    std::vector<vector_t> centroids;
                    std::vector<double> surface_areas;
                    const trimesh_t* trimesh = obj_geom.get_trimesh();
                    trimesh->get_all_normal_vectors(normal_vectors, centroids, surface_areas, 100);
                    for (unsigned i = 0; i < normal_vectors.size(); ++i)
                    {

                        // Check if the normal in the global cooridnate frame is valid
                        bool valid_normal = true;
                        vector_t rotated_test_normal = object_config.get_orientation().qv_rotation(normal_vectors[i]);
                        for (int normal_index = 0; normal_index < restricted_normals.size() && valid_normal; ++normal_index)
                        {
                            //PRX_WARN_S ("Restricted normal: " << restricted_normals[normal_index]);
                            // Rotate restricted normal
                            vector_t rotated_restricted_normal(restricted_normals[normal_index]);
                            //object_config.get_orientation().qv_rotation(rotated_restricted_normal);

                            // Check if dot product >= 0.95
                            if (rotated_restricted_normal.dot_product(rotated_test_normal) >= 0.95)
                            {
                                // PRX_DEBUG_COLOR("Rotated restricted: " << rotated_restricted_normal << " vs rotated test: " << rotated_test_normal, PRX_TEXT_RED);
                                // PRX_DEBUG_COLOR("INVALID!", PRX_TEXT_RED);
                                valid_normal = false;
                            }
                        }
                        if (valid_normal)
                        {
                            quaternion_t rotated_normal;
                            rotated_normal.compute_rotation(default_ee_normal, normal_vectors[i]*-1.0);
                            for (double angle = 0; angle < 360; angle+=manual_angle_increment)
                            {
                                quaternion_t incremental_rotation(normal_vectors[i], angle*(PRX_PI/180.0));
                                quaternion_t true_rotation = incremental_rotation * rotated_normal;
                                config_t grasp_config(centroids[i], true_rotation);
                                PRX_DEBUG_COLOR("Computed grasping config: " << grasp_config, PRX_TEXT_MAGENTA);
                                grasps[ hash_string ].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode));
                            }
                        }  
                    }

                }


                if (save_grasps_to_file)
                {
                    PRX_DEBUG_COLOR("/n/n---Saving grasps to file----/n/n", PRX_TEXT_CYAN);
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string filename(w);
                    filename+="/prx_output/" + end_effector_name + "_"+ hash_string +".yaml";

                    std::ofstream fout;
                    fout.open(filename.c_str());

                    for (auto grasp : grasps[hash_string])
                    {
                        fout << grasp;
                    }

                    fout.close();
                }

                return grasps[hash_string];
            }


        }
    }
}