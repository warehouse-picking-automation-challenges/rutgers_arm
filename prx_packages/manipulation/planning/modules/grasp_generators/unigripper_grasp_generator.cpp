/**
 * @file unigripper_grasp_generator.cpp
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


#include "planning/modules/grasp_generators/unigripper_grasp_generator.hpp"

#include "simulation/workspace_trajectory.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"

#include "planning/manipulation_world_model.hpp"
#include "planning/queries/grasping_query.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx/planning/planner.hpp"


#include <algorithm>
#include <clipper.hpp>


PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::unigripper_grasp_generator_t, prx::packages::manipulation::grasp_generator_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace manipulation
        {
            unigripper_grasp_generator_t::unigripper_grasp_generator_t()
            {
            }

            unigripper_grasp_generator_t::~unigripper_grasp_generator_t()
            {
            }
            void unigripper_grasp_generator_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
            {
                grasp_generator_t::init(reader, template_reader);

                default_ee_normal.resize(3);
                default_ee_normal = parameters::get_attribute_as<vector_t>("default_ee_normal", reader, template_reader);
                manual_grasp_mode = parameters::get_attribute_as<unsigned>("manual_grasp_mode", reader, template_reader);
                manual_release_mode = parameters::get_attribute_as<unsigned>("manual_release_mode", reader, template_reader);
                manual_angle_increment = parameters::get_attribute_as<double>("manual_angle_increment", reader, template_reader, 0.0);
                nr_grasps_to_sample = parameters::get_attribute_as<int>("nr_grasps_to_sample", reader, template_reader, 100);
                total_nr_grasps = parameters::get_attribute_as<unsigned>("total_nr_grasps", reader, template_reader, 200);
                grasp_entire_mesh = parameters::get_attribute_as<bool>("grasp_entire_mesh", reader, template_reader, false);
                limiting_normal = parameters::get_attribute_as<bool>("limiting_normal", reader, template_reader, false);
                ee_dof_check = parameters::get_attribute_as<bool>("ee_dof_check", reader, template_reader, false);
                minimum_overlap_area = parameters::get_attribute_as<double>("minimum_overlap_area", reader, template_reader, -1);
                randomize_final_grasps = parameters::get_attribute_as<bool>("randomize_final_grasps", reader, template_reader, false);

                PRX_DEBUG_COLOR("ee_normal: " << default_ee_normal << " and increment: " << manual_angle_increment, PRX_TEXT_MAGENTA);


                // else
                // {
                //     PRX_FATAL_S("An end effector name in the grasp generator is required.");
                // }

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
                        vector_t pref_vec = parameters::get_attribute_as<vector_t>("normal", r, template_reader);
                        preferred_normals.push_back(pref_vec);
                    }
                }
                // else
                // {
                //     PRX_FATAL_S("Need to specify two preferred normals.");
                // }

                hand_long_axis = parameters::get_attribute_as<vector_t>("hand_long_axis", reader, template_reader, vector_t(1,0,0));
                limiting_axis = parameters::get_attribute_as<vector_t>("limiting_axis", reader, template_reader, vector_t(1,0,0));


            }
            const std::vector<grasp_t>&  unigripper_grasp_generator_t::compute_grasps(const util::geometry_t* ee_geom, const util::config_t& ee_local_config, movable_body_plant_t* object)
            {
                statistics = "";
                local_ee_config = ee_local_config;
                PRX_ASSERT(manipulation_model != NULL);
                if (grasp_entire_mesh)
                {
                    return compute_grasps_on_entire_mesh(ee_geom, object);
                }
                else
                {
                    return compute_grasps_on_descriptors(ee_geom, object);
                }
            }


            const std::vector<grasp_t>&  unigripper_grasp_generator_t::compute_grasps_on_descriptors(const util::geometry_t* ee_geom, movable_body_plant_t* object)
            {
                PRX_DEBUG_COLOR("Computing grasps over graph descriptors", PRX_TEXT_BLUE);

                config_t object_config;
                object->get_configuration(object_config);
                PRX_WARN_S ("Object configuration: " << object_config);
                // quaternion_t temp_quat = object_config.get_orientation();
                // temp_quat.normalize();
                // object_config.set_orientation(temp_quat);

                // double shelf_bottom [4] = { 0.6, 0.87, 1.1, 1.33 };

                // double obj_x, obj_y, obj_z;
                // object_config.get_position(obj_x, obj_y, obj_z);

                // for (int i = 0; i < 4; ++i)
                // {
                //     if (obj_z - shelf_bottom[i] >= 0.05 && obj_z - shelf_bottom[i] <= 0.1)
                //     {
                //         vector_t temp_vec = preferred_normals[0];
                //         preferred_normals[0] = preferred_normals[1];
                //         preferred_normals[1] = temp_vec;
                //         break;                             
                //     }     
                // } 

                for (int i=0;i<preferred_normals.size(); ++i)
                {
                    // PRX_PRINT("Preferred normals list: "<<preferred_normals[i], PRX_TEXT_MAGENTA);
                }

                std::string hash_string = object->get_object_type();
                PRX_PRINT("Object type: " << hash_string << "["<< nr_grasps_to_sample<<"]", PRX_TEXT_MAGENTA);

                // the vector that will store the grasps per preferred normal direction
                // there is one extra index for the non-preferred directions
                std::vector< std::vector<grasp_t> > normal_prioritized_grasps_list; 
                normal_prioritized_grasps_list.resize(preferred_normals.size()+1);
                foreach(std::vector<grasp_t> list, normal_prioritized_grasps_list)
                    list.clear();

                // Get end-effector specific grasp descriptors
                grasp_descriptor_t* descriptor = object->get_grasp_descriptor();
                const std::vector< grasp_surface_t* >& grasp_surfaces = descriptor->get_surfaces(end_effector_name);

                PRX_ASSERT(!grasp_surfaces.empty());


                unsigned rejected_normal_grasps = 0, rejected_area_grasps =0, rejected_collision_grasps = 0;;
                for (const auto surface : grasp_surfaces)
                {
                    // PRX_ERROR_S("----------------------------------------");
                    // Check if the normal in the global cooridnate frame is valid
                    bool valid_normal = true;
                    object->get_configuration(object_config);
                    vector_t rotated_test_normal = object_config.get_orientation().qv_rotation(surface->normal);
                    // PRX_PRINT("Surface normal:::"<<surface->normal, PRX_TEXT_CYAN);
                    // PRX_PRINT("Global Surface normal:::"<<rotated_test_normal, PRX_TEXT_CYAN);

                    //Check if the normal of the grasp surface is restricted. If so, it is not valid.
                    for (int normal_index = 0; normal_index < restricted_normals.size() && valid_normal; ++normal_index)
                    {

                        // PRX_WARN_S ("Restricted normal: " << restricted_normals[normal_index]);

                        // Check if dot product >= 0.95
                        if (restricted_normals[normal_index].dot_product(rotated_test_normal) >= 0.95)
                        {
                            // PRX_PRINT("Restricted: " << rotated_restricted_normal << " vs rotated test: " << rotated_test_normal, PRX_TEXT_RED);
                            PRX_PRINT(rotated_test_normal<<" is INVALID!", PRX_TEXT_RED);
                            valid_normal = false;
                        }
                        else
                        {
                            // PRX_PRINT("-", PRX_TEXT_CYAN);
                        }
                    }

                    //Determine preferred grasp index for the valid normal
                    int preferred_normal_index = preferred_normals.size();
                    for (int normal_index = 0; normal_index < preferred_normals.size() && valid_normal && preferred_normal_index == preferred_normals.size(); normal_index++)
                    {
                        // PRX_WARN_S ("Preferred normal: " << preferred_normals[normal_index]);

                        // Check if dot product >= 0.9
                        if (preferred_normals[normal_index].dot_product(rotated_test_normal) >= .8)
                        {
                            // PRX_PRINT("Preferred: " << rotated_preferred_normal << " vs rotated test: " << rotated_test_normal, PRX_TEXT_RED);
                            PRX_PRINT(rotated_test_normal<<" is PREFERRED at index "<<normal_index, PRX_TEXT_GREEN);
                            preferred_normal_index = normal_index;
                        }
                        else
                        {
                            // PRX_PRINT("-", PRX_TEXT_MAGENTA);
                        }
                    }

                    //If the surface normal is not restricted
                    if (valid_normal)
                    {
                        PRX_INFO_S("Valid normal");
                        PRX_DEBUG_COLOR("Default ee normal: " << default_ee_normal, PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("NR GRASPS: " << nr_grasps_to_sample, PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("Hand Long Axis: " << hand_long_axis, PRX_TEXT_MAGENTA);
                        PRX_DEBUG_COLOR("limiting_axis: " << limiting_axis, PRX_TEXT_MAGENTA);

                        quaternion_t rotated_normal;
                        rotated_normal.compute_rotation(default_ee_normal, surface->normal*-1.0);

                        vector_t sampled_point;
                        if (nr_grasps_to_sample > 0)
                        {

                            //Sample around centroid first
                            vector_t vertex1, vertex2, vertex3;
                            const std::vector<face_t>& object_faces = surface->mesh.get_faces();
                            vector_t centroid(0,0,0);
                            int v_count = 0;
                            for(const auto local_face : object_faces )
                            {

                                surface->mesh.get_vertex_at(local_face.get_index1(), &vertex1);
                                surface->mesh.get_vertex_at(local_face.get_index2(), &vertex2);
                                surface->mesh.get_vertex_at(local_face.get_index3(), &vertex3);

                                centroid[0]+=(vertex1[0]+vertex2[0]+vertex3[0]);
                                centroid[1]+=(vertex1[1]+vertex2[1]+vertex3[1]);
                                centroid[2]+=(vertex1[2]+vertex2[2]+vertex3[2]);

                                v_count+=3;
                            }
                            //Compute centroid of the surface from all the triangle vertices in the mesh
                            centroid[0]/=v_count;
                            centroid[1]/=v_count;
                            centroid[2]/=v_count;

                            int temp_nr_grasps = nr_grasps_to_sample;
                            sampled_point = centroid;
                            //Sample around the centroid at angular steps of 20 degrees
                            for(int angle_step=0; angle_step<360; angle_step+=30)
                            {
                                quaternion_t incremental_rotation(surface->normal, angle_step*(PRX_PI/180.0));
                                quaternion_t true_rotation = incremental_rotation * rotated_normal;
                                config_t grasp_config(sampled_point, true_rotation);

                                config_t global_grasp_config = grasp_config;
                                global_grasp_config.relative_to_global(object_config);

                                bool add_grasp = true;
                                //If the arm needs to be limited to a conical region around the limiting axis(+X by default)
                                if(limiting_normal)
                                {
                                    vector_t gripper_axis = hand_long_axis;
                                    //The axis of the hand in the global frame, for the specific grasping orientation
                                    vector_t rotated_axis = global_grasp_config.get_orientation().qv_rotation(gripper_axis);
                                    
                                    //If the axis of the hand in the global frame does not fall in a 45 degree conical region around the limiting axis(+X)
                                    if(rotated_axis.dot_product(limiting_axis)<0.5)
                                    {
                                        PRX_DEBUG_COLOR("Computed grasp config "<<grasp_config<<" with normal: "<<rotated_axis<<"."<<gripper_axis<<" was rejected by limiting normals.", PRX_TEXT_BROWN);
                                        add_grasp = false;
                                        rejected_normal_grasps++;
                                    }
                                }

                                //Grippy specific filter
                                if(ee_dof_check)
                                {
                                    //Local y and z axis are the constraints for grippy
                                    vector_t grippy_y_check =  global_grasp_config.get_orientation().qv_rotation(vector_t(0,1,0));
                                    vector_t grippy_z_check = global_grasp_config.get_orientation().qv_rotation(vector_t(0,0,1));
                                    //For cases where grippy is not pointing in front, the top of the hand, local Y, should be constrained to be facing forward (ie, the hand points outwards and not backwards)
                                    //For cases where grippy is pointing in front, the outward Z of the end effector should be constrained to be facing forward (ie. the hand points outwards)
                                    if( grippy_y_check.dot_product(limiting_axis) < 0.7071 && grippy_z_check.dot_product(limiting_axis) < 0.7071)
                                    {
                                        add_grasp = false;;
                                        rejected_normal_grasps++;
                                    }
                                }

                                //Add the grasp if it did not get invalidated
                                if(add_grasp)
                                {
                                    double overlapping_area = evaluate_overlapping_area(ee_geom, global_grasp_config, (*surface), object_config);
                                    if(overlapping_area > minimum_overlap_area)
                                    {
                                        config_t centroid_config(centroid, quaternion_t(0,0,0,1));
                                        centroid_config.relative_to_global(object_config);
                                        double distance_to_centroid = centroid_config.get_position().distance(global_grasp_config.get_position());
                                        // PRX_PRINT("Distance to centroid: "<<distance_to_centroid, PRX_TEXT_RED);
                                        // PRX_PRINT(overlapping_area<<" overlaps vs the total surface of "<<surface->total_area, PRX_TEXT_RED);
                                        // PRX_PRINT("Computed grasping config: " << grasp_config, PRX_TEXT_MAGENTA);
                                        normal_prioritized_grasps_list[preferred_normal_index].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode,overlapping_area, distance_to_centroid));
                                        temp_nr_grasps--;
                                    }
                                    else
                                    {
                                        PRX_DEBUG_COLOR ("Rejected grasp with overlapping area: " << overlapping_area << " vs min area: " << minimum_overlap_area, PRX_TEXT_CYAN);
                                        rejected_area_grasps++;
                                    }
                                }

                            }


                            object->get_configuration(object_config);

                            //Remaining samples will be randomly on the surface
                            int total_tries = 1000;
                            for (unsigned current = 0; current < temp_nr_grasps && --total_tries > 0 && temp_nr_grasps > 0; )
                            {
                                double angle = uniform_random(0.0,360.0);
                                descriptor->sample_surface(surface->index, sampled_point);
                                quaternion_t incremental_rotation(surface->normal, angle*(PRX_PI/180.0));
                                quaternion_t true_rotation = incremental_rotation * rotated_normal;
                                config_t grasp_config(sampled_point, true_rotation);

                                config_t global_grasp_config = grasp_config;
                                global_grasp_config.relative_to_global(object_config);

                                bool add_grasp = true;
                                if(limiting_normal)
                                {
                                    vector_t gripper_axis = hand_long_axis;
                                    vector_t rotated_axis = global_grasp_config.get_orientation().qv_rotation(gripper_axis);
                                    

                                    if(rotated_axis.dot_product(limiting_axis)<0.5)
                                    {
                                        PRX_DEBUG_COLOR("Computed grasp config "<<grasp_config<<" with normal: "<<rotated_axis<<"."<<gripper_axis<<" was rejected by limiting normals.", PRX_TEXT_BROWN);
                                        add_grasp = false;
                                        rejected_normal_grasps++;
                                    }
                                }

                                if(ee_dof_check)
                                {
                                    vector_t grippy_y_check =  global_grasp_config.get_orientation().qv_rotation(vector_t(0,1,0));
                                    vector_t grippy_z_check = global_grasp_config.get_orientation().qv_rotation(vector_t(0,0,1));
                                    if( grippy_y_check.dot_product(limiting_axis) < 0.7071 && grippy_z_check.dot_product(limiting_axis) < 0.7071)
                                    {
                                        add_grasp = false;
                                        rejected_normal_grasps++;
                                    }
                                }

                                
                                if(add_grasp)
                                {
                                    double overlapping_area = evaluate_overlapping_area(ee_geom, global_grasp_config, (*surface), object_config);
                                    if(overlapping_area > minimum_overlap_area)
                                    {   
                                        config_t centroid_config(centroid, quaternion_t(0,0,0,1));
                                        centroid_config.relative_to_global(object_config);
                                        double distance_to_centroid = centroid_config.get_position().distance(global_grasp_config.get_position());
                                        // PRX_PRINT("Distance to centroid: "<<distance_to_centroid, PRX_TEXT_RED);
                                        // PRX_PRINT("Computed grasping config: " << grasp_config, PRX_TEXT_MAGENTA);
                                        normal_prioritized_grasps_list[preferred_normal_index].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode,overlapping_area, distance_to_centroid));
                                        ++current;
                                    }
                                    else
                                    {
                                        rejected_area_grasps++;
                                    }
                                }

                                
                                
                            }      
                        }
                        else if (manual_angle_increment > PRX_ZERO_CHECK)
                        {
                            for (double angle = 0; angle < 360; angle+=manual_angle_increment)
                            {
                                descriptor->sample_surface(surface->index, sampled_point);
                                quaternion_t incremental_rotation(surface->normal, angle*(PRX_PI/180.0));
                                quaternion_t true_rotation = incremental_rotation * rotated_normal;
                                config_t grasp_config(sampled_point, true_rotation);
                                double overlapping_area = evaluate_overlapping_area(ee_geom, grasp_config, (*surface), object_config);
                                PRX_DEBUG_COLOR("Computed grasping config: " << grasp_config, PRX_TEXT_MAGENTA);
                                normal_prioritized_grasps_list[preferred_normal_index].push_back(grasp_t(grasp_config,manual_release_mode,manual_grasp_mode,overlapping_area));
                            }
                        }
                        else
                        {
                            PRX_FATAL_S ("Nr grasps to sample and manual angle increment are both 0!");
                        }
                    }
                    // PRX_PRINT("DONE!!!", PRX_TEXT_CYAN);  
                }

                unsigned actual_total = 0, actual_list_size = 0;

                object->get_configuration(object_config);

                std::vector<unsigned> actual_bin_total;
                std::vector< std::vector<grasp_t > > collision_free_grasps;

                unsigned total_grasp_counter = 0;
                for (auto grasp_list : normal_prioritized_grasps_list)
                {
                    total_grasp_counter += grasp_list.size();
                    std::vector<grasp_t> grasp_vector;
                    if (!grasp_list.empty())
                    {
                        // If this is set, we must collision check grasps
                        if (!ffee_context_name.empty())
                        {
                            unsigned collision_free_total = 0;

                            std::string old_context = manipulation_model->get_current_context();
                            state_t* old_state = manipulation_model->get_state_space()->alloc_point();

                            config_t test_ee_config;
                            manipulation_model->get_end_effector_local_config(test_ee_config);

                            manipulation_model->use_context(ffee_context_name);
                            state_t* ffee_state = manipulation_model->get_state_space()->alloc_point();
                            state_t* result_ffee_state = manipulation_model->get_state_space()->alloc_point();
                            std::vector<double> zero_state = {0.0,0.0,0.0,0.0,0.0,0.0, 1.0};

                            for (unsigned i = 0; i < grasp_list.size(); ++i)
                            {
                                // PRX_PRINT ("\n\nGrasp relative config: " << grasp_list[i].relative_config, PRX_TEXT_CYAN);
                                // PRX_PRINT (" EE config: " << test_ee_config, PRX_TEXT_CYAN);
                                // config_t unit_config = grasp_list[i].relative_config - test_ee_config;
                                // PRX_PRINT ("Resulting subtraction: " << unit_config, PRX_TEXT_CYAN);
                                config_t test_config = test_ee_config;
                                test_config.relative_to_global(grasp_list[i].relative_config);
                                test_config.relative_to_global(object_config);

                                manipulation_model->get_state_space()->set_from_vector(zero_state, ffee_state);
                                manipulation_model->IK(result_ffee_state, ffee_state, test_config);


                                PRX_DEBUG_COLOR("Checking ffee state: " << manipulation_model->get_state_space()->print_point(result_ffee_state, 5), PRX_TEXT_MAGENTA);
                                if (manipulation_model->valid_state(result_ffee_state))
                                {
                                    PRX_DEBUG_COLOR("Found a valid grasp!", PRX_TEXT_CYAN);
                                    PRX_DEBUG_COLOR("Relative config: " << grasp_list[i].relative_config, PRX_TEXT_MAGENTA);
                                    PRX_DEBUG_COLOR("ffee_state: " << manipulation_model->get_state_space()->print_point(ffee_state,5), PRX_TEXT_MAGENTA);
                                    grasp_vector.push_back(grasp_list[i]);
                                    actual_total++;
                                }
                                else
                                {
                                    PRX_DEBUG_COLOR("INVALID GRASP!", PRX_TEXT_RED);
                                    PRX_DEBUG_COLOR("Relative config: " << grasp_list[i].relative_config, PRX_TEXT_MAGENTA);
                                    PRX_DEBUG_COLOR("ffee_state: " << manipulation_model->get_state_space()->print_point(ffee_state,5), PRX_TEXT_MAGENTA);
                                    rejected_collision_grasps++;
                                }
                            }

                            // Restore old context and state
                            manipulation_model->get_state_space()->free_point(ffee_state);
                            manipulation_model->use_context(old_context);
                            manipulation_model->get_state_space()->copy_from_point(old_state);
                            manipulation_model->get_state_space()->free_point(old_state);

                            // Update collision free total
                            collision_free_grasps.push_back(grasp_vector);
                        }
                        else
                        {
                            grasp_vector.insert(grasp_vector.end(), grasp_list.begin(), grasp_list.end());
                            actual_total += grasp_list.size();
                            collision_free_grasps.push_back(grasp_vector);
                        }
                    }
                }

                unsigned DONTCHANGETHISTOTAL = total_nr_grasps;
                if (actual_total < DONTCHANGETHISTOTAL)
                    DONTCHANGETHISTOTAL = actual_total;

                PRX_PRINT ("Number of collision free grasps : " << actual_total, PRX_TEXT_GREEN);
                PRX_PRINT("\n++++ NUMBER OF GRASPS: " << total_grasp_counter, PRX_TEXT_RED);
                append_to_stat_string("\n>>>>>> REJECT NORMAL: " + boost::lexical_cast<std::string>(rejected_normal_grasps));
                append_to_stat_string("\n>>>>>> REJECT AREA  : " + boost::lexical_cast<std::string>(rejected_area_grasps));
                append_to_stat_string("\n++++ NUMBER OF GRASPS: " + boost::lexical_cast<std::string>(total_grasp_counter));
                append_to_stat_string("\n---- NUMBER OF COLLISION FREE GRASPS: " + boost::lexical_cast<std::string>(actual_total));
                PRX_PRINT("\n---- NUMBER OF COLLISION FREE GRASPS: " << actual_total, PRX_TEXT_RED);

                // Add a fraction of the grasp vector
                for (auto grasp_list : collision_free_grasps )
                {
                    if (!grasp_list.empty())
                    {
                        unsigned fractional_value = std::round(((double)grasp_list.size()/(double)actual_total) * (double)DONTCHANGETHISTOTAL);
                        if (fractional_value > grasp_list.size())
                            fractional_value = grasp_list.size();
                        PRX_PRINT ("Grasp list size: " << grasp_list.size(), PRX_TEXT_RED);
                        PRX_PRINT ("Total Nr Grasps: " << DONTCHANGETHISTOTAL, PRX_TEXT_RED);
                        PRX_PRINT ("Fractional value: " << fractional_value, PRX_TEXT_RED);
                        std::sort(grasp_list.begin(), grasp_list.end(), std::greater<grasp_t>());

                        auto vi = grasp_list.begin();
                        std::advance(vi, fractional_value);
                        grasps[hash_string].insert(grasps[hash_string].end(), grasp_list.begin(),vi);
                        actual_list_size++;
                    }
                }


                if (actual_list_size == 0)
                {
                    config_t dummy;
                    grasps[hash_string].push_back(grasp_t(dummy,manual_release_mode,manual_grasp_mode,0.0));
                }

// #ifndef NDEBUG //If we are in Debugging mode
                if (save_grasps_to_file)
                {
                    PRX_DEBUG_COLOR("\n\n---Saving grasps to file----\n\n", PRX_TEXT_CYAN);
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string filename(w);
                    filename+="/prx_output/" + end_effector_name + "_"+ hash_string +".yaml";

                    std::ofstream fout;
                    fout.open(filename.c_str());

                    for (auto check_grasp : grasps[hash_string])
                    {
                        fout << check_grasp;
                    }
                    fout.close();
                }

// #endif
                if (randomize_final_grasps)
                {
                    std::random_shuffle(grasps[hash_string].begin()+1, grasps[hash_string].end());
                }
                return grasps[hash_string];
            }

            const std::vector<grasp_t>&  unigripper_grasp_generator_t::compute_grasps_on_entire_mesh(const util::geometry_t* ee_geom, movable_body_plant_t* object)
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
                    filename+="/prx_output/" + end_effector_name + "_" + hash_string +".yaml";

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

            double unigripper_grasp_generator_t::evaluate_overlapping_area(const util::geometry_t* ee_geom, const config_t& ee_config, const grasp_surface_t& grasp_surface, const config_t& object_config)
            {
                // Convert the other geometry from local coordinates to global coordinates
                ClipperLib::Paths full_subj;
                // Get other vertices
                std::vector<vector_t> ee_normal_vectors;
                std::vector<vector_t> ee_centroids;
                std::vector<double> ee_surface_areas;
                ee_geom->get_trimesh()->get_all_normal_vectors(ee_normal_vectors, ee_centroids, ee_surface_areas);

                face_t* current_ee_face = new face_t();
                vector_t vertex1, vertex2, vertex3;
                vector_t rotated_vertex1, rotated_vertex2, rotated_vertex3;
                config_t holding_config1, holding_config2, holding_config3;
                // for(auto vec : ee_normal_vectors)
                // {
                //     PRX_DEBUG_COLOR("EE normal vector: " << vec, PRX_TEXT_BROWN);
                // }

                quaternion_t xy_plane_rotation(grasp_surface.normal, vector_t(0,0,1));
                std::vector< std::vector<double> > ee_triangles, surface_triangles;

                for(unsigned ee_index = 0; ee_index < ee_normal_vectors.size(); ++ee_index)
                {
                    ClipperLib::Path temp_subj;

                    // If the normal of the current triangle matches the ee_normal
                    if (default_ee_normal.dot_product(ee_normal_vectors[ee_index]) >= 0.8)
                    {
                        ee_geom->get_trimesh()->get_face_at(ee_index,current_ee_face);

                        ee_geom->get_trimesh()->get_vertex_at(current_ee_face->get_index1(), &vertex1);
                        ee_geom->get_trimesh()->get_vertex_at(current_ee_face->get_index2(), &vertex2);
                        ee_geom->get_trimesh()->get_vertex_at(current_ee_face->get_index3(), &vertex3);

                        holding_config1.set_orientation(0,0,0,1);
                        holding_config1.set_position(vertex1);
                        // holding_config1.relative_to_global(local_ee_config);
                        holding_config1.relative_to_global(ee_config);
                        holding_config1.global_to_relative(object_config);
                        rotated_vertex1 = holding_config1.get_position();

                        holding_config2.set_orientation(0,0,0,1);
                        holding_config2.set_position(vertex2);
                        // holding_config2.relative_to_global(local_ee_config);
                        holding_config2.relative_to_global(ee_config);
                        holding_config2.global_to_relative(object_config);
                        rotated_vertex2 = holding_config2.get_position();

                        holding_config3.set_orientation(0,0,0,1);
                        holding_config3.set_position(vertex3);
                        // holding_config3.relative_to_global(local_ee_config);
                        holding_config3.relative_to_global(ee_config);
                        holding_config3.global_to_relative(object_config);
                        rotated_vertex3 = holding_config3.get_position();

                        rotated_vertex1 = xy_plane_rotation.qv_rotation(rotated_vertex1);
                        rotated_vertex2 = xy_plane_rotation.qv_rotation(rotated_vertex2);
                        rotated_vertex3 = xy_plane_rotation.qv_rotation(rotated_vertex3);

                        temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex1[0]), ClipperLib::to_long(rotated_vertex1[1]), ClipperLib::to_long(rotated_vertex1[2]));
                        temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex2[0]), ClipperLib::to_long(rotated_vertex2[1]), ClipperLib::to_long(rotated_vertex2[2]));
                        temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex3[0]), ClipperLib::to_long(rotated_vertex3[1]), ClipperLib::to_long(rotated_vertex3[2]));

                        //PRX_DEBUG_COLOR ("Pushed back temp subj: " << temp_subj, PRX_TEXT_GREEN);
                        full_subj.push_back(temp_subj);
                    }
                }

                //PRX_DEBUG_COLOR ("Number of EE triangles to check: " << full_subj.size(), PRX_TEXT_MAGENTA);

                // Convert the other geometry from local coordinates to global coordinates
                ClipperLib::Paths full_clip;
                // Get other vertices
                std::vector<face_t> object_faces = grasp_surface.mesh.get_faces();

                for(const auto local_face : object_faces )
                {
                    ClipperLib::Path temp_clip;

                    grasp_surface.mesh.get_vertex_at(local_face.get_index1(), &vertex1);
                    grasp_surface.mesh.get_vertex_at(local_face.get_index2(), &vertex2);
                    grasp_surface.mesh.get_vertex_at(local_face.get_index3(), &vertex3);

                    rotated_vertex1 = xy_plane_rotation.qv_rotation(vertex1);
                    rotated_vertex2 = xy_plane_rotation.qv_rotation(vertex2);
                    rotated_vertex3 = xy_plane_rotation.qv_rotation(vertex3);

                    temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex1[0]), ClipperLib::to_long(rotated_vertex1[1]), ClipperLib::to_long(rotated_vertex1[2]));
                    temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex2[0]), ClipperLib::to_long(rotated_vertex2[1]), ClipperLib::to_long(rotated_vertex2[2]));
                    temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(rotated_vertex3[0]), ClipperLib::to_long(rotated_vertex3[1]), ClipperLib::to_long(rotated_vertex3[2]));

                    //PRX_DEBUG_COLOR ("Pushed back temp clip: " << temp_clip, PRX_TEXT_BLUE);
                    full_clip.push_back(temp_clip);
                }
                //PRX_DEBUG_COLOR ("Number of Grasp Surface triangles to check: " << full_clip.size(), PRX_TEXT_CYAN);

                //ClipperLib::Paths simple_subj, simple_clip;
                // ClipperLib::SimplifyPolygons(full_subj, simple_subj, ClipperLib::pftNonZero);
                // ClipperLib::SimplifyPolygons(full_clip, simple_clip, ClipperLib::pftNonZero);

                // Compute clipping
                ClipperLib::Paths solution;
                ClipperLib::Clipper clpr;
                clpr.AddPaths(full_subj, ClipperLib::ptSubject, true);
                clpr.AddPaths(full_clip, ClipperLib::ptClip, true);
                clpr.Execute( ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero );

                if (solution.size() >= 1)
                {
                    double total_area = 0.0;
                    for(auto i : solution)
                    {
                        //PRX_DEBUG_COLOR("Clipped solution: " << i, PRX_TEXT_GREEN);
                        total_area += (ClipperLib::Area(i) / (ClipperLib::scale*ClipperLib::scale));
                    }
                    // PRX_WARN_S ("Computed total_area: " << total_area);
                    return total_area;
                }
                else
                {
                    PRX_ERROR_S ("No overlap?");
                    return 0;
                }

            }

        }
    }
}