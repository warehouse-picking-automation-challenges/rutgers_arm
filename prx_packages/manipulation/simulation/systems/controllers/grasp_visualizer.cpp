/**
 * @file grasp_visualizer.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/controllers/grasp_visualizer.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream> 
PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_visualizer_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {
            grasp_visualizer_t::grasp_visualizer_t()
            {
                current_grasp_state=0;
                current_grasp_pose=0;

            }

            grasp_visualizer_t::~grasp_visualizer_t()
            {
            }

            void grasp_visualizer_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);
                if (parameters::has_attribute("grasp_path", reader, template_reader))
                {
                    grasp_path = parameters::get_attribute_as<std::string > ("grasp_path", reader, template_reader);
                }
                else if (parameters::has_attribute("grasp_file", reader, template_reader)) 
                {
                    grasp_file = parameters::get_attribute_as<std::string > ("grasp_file", reader, template_reader);
                }
                else
                {
                    PRX_FATAL_S ("Must specify a grasp file or grasp path!");
                }
                start_link = parameters::get_attribute_as<std::string > ("start_link", reader, template_reader,"");
                end_link = parameters::get_attribute_as<std::string > ("end_link", reader, template_reader,"");

                std::vector<const parameter_reader_t*> readers;
                bool get_poses_from_file = false;
                if(parameters::has_attribute("object_poses", reader, template_reader))
                {
                    readers = parameters::get_list("object_poses", reader, template_reader);
                }
                else
                {
                    get_poses_from_file = true;
                }

                std::string geom_name;

                foreach(const parameter_reader_t* r, readers)
                {
                    object_poses.push_back(r->get_attribute_as< std::vector<double> > ("pose"));
                }

                
                object=NULL;
                manipulator = NULL;

                // TODO: This code will only work on a single object right now.
                foreach(std::string name, subsystem_names)
                {
                    if(manipulator==NULL)
                        manipulator = dynamic_cast<manipulator_t*>(subsystems[name].get());
                    if(object==NULL)
                        object = dynamic_cast<movable_body_plant_t*>(subsystems[name].get());
                }
                if(manipulator==NULL || object == NULL)
                {
                    PRX_FATAL_S("Grasping visualizer requires an object and a floating manipulator");
                }


                if(get_poses_from_file)
                {
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string filename(w);
                    //Default case poses file location
                    std::string pose_file_path = parameters::get_attribute_as<std::string > ("pose_file_path", reader, template_reader,"/prx_packages/apc/input/case_poses");
                    filename+=(pose_file_path+"/"+object->get_object_type()+".yaml");
                    std::ifstream pose_file(filename.c_str());
                    PRX_PRINT("Case Pose Filename: "<<filename, PRX_TEXT_GREEN);
                    std::vector<std::vector<double> > case_poses;

                    if(pose_file)
                    {
                        double value;
                        int counter = 0;
                        std::vector<double> pose;
                        while ( pose_file >> value ) {
                            // PRX_PRINT(value<<" ", PRX_TEXT_BLUE);
                            pose.push_back(value);
                            if(++counter%7==0)
                            {   
                                case_poses.push_back(pose);
                                pose.clear();
                                // PRX_PRINT("\n", PRX_TEXT_BLUE);
                            }
                        }
                    }   

                    object_poses.clear();

                    foreach(std::vector<double> pose, case_poses)
                    {
                        if(pose.size()!=7)
                        {
                            PRX_WARN_S("Malformed pose file. Need 7 numbers to denote SE3 coordinated of the object.");
                        }
                        object_poses.push_back(pose);
                    }
                    if(case_poses.size()==0)
                    {
                        PRX_FATAL_S("No usable poses in the file.");
                    }
                }

                object->get_state_space()->set_from_vector(object_poses[0]);
                read_database_file(grasp_file);
            }

            void grasp_visualizer_t::compute_control()
            {
                if(grasping_states.size()>0)
                {
                    manipulator->get_state_space()->copy_from_point(grasping_states[current_grasp_state++%grasping_states.size()]);
                    //manipulator->get_control_space()->copy_from_point(grasping_states[current_grasp_state%grasping_states.size()]);
                    PRX_PRINT("Grasp ID: "<<(current_grasp_state-1)%grasping_states.size(),PRX_TEXT_BLUE);
                }

                if ( grasping_states.size()==0 || current_grasp_state%grasping_states.size() == 0)
                {
                    object->get_state_space()->set_from_vector(object_poses[++current_grasp_pose%object_poses.size()]);
                    read_database_file(grasp_file);
                    current_grasp_state = 0;
                }
            }

            void grasp_visualizer_t::read_database_file(std::string grasp_filename)
            {
                grasping_states.clear();

                config_t ee_config;
                config_t grasp_config;
                config_t object_config;
                config_t config;
                object->get_configuration(object_config);

                // Find out which file to load
                std::string top_front_face;
                std::vector<std::string> alternative_faces;
                determine_top_and_front_faces(object, top_front_face, alternative_faces);

                manipulator->get_end_effector_configuration(ee_config, 0);
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                if (grasp_filename.empty())
                    filename+="/"+grasp_path+"/"+object->get_object_type()+top_front_face+".yaml";
                else
                    filename+="/"+grasp_filename;
                PRX_PRINT("FILENAME: " << filename, PRX_TEXT_GREEN);

                if(boost::filesystem::exists( filename ))
                {
                    grasping_states.link_space(manipulator->get_state_space());
                    state_t* temp_state = manipulator->get_state_space()->alloc_point();
                    unsigned dimension = manipulator->get_state_space()->get_dimension();
                    std::vector<double> temp_vec(7);

                    YAML::Node doc = YAML::LoadFile(filename);
                    PRX_INFO_S(filename);
                    unsigned i;
                    for( i=0;i<doc.size();i++)
                    {
                        for( unsigned j=0; j < 7; ++j )
                        {
                            temp_vec[j] = doc[i]["relative_config"][j].as<double>();
                        }
                        grasp_config.set_position(temp_vec[0],temp_vec[1],temp_vec[2]);
                        grasp_config.set_xyzw_orientation(temp_vec[3],temp_vec[4],temp_vec[5],temp_vec[6]);
                        if(i==0)
                        {
                            PRX_INFO_S(ee_config);
                            PRX_INFO_S(grasp_config);
                            PRX_INFO_S(object_config);
                        }

                        config = ee_config;
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }
                        config.relative_to_global(grasp_config);
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }
                        config.relative_to_global(object_config);
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }

                        int grasp_mode = doc[i]["grasp_mode"].as<int>();
                        manipulator->IK_solver(temp_state,temp_state,config,start_link,end_link);
                        temp_state->at(dimension-1) = grasp_mode;

                        grasping_states.copy_onto_back(temp_state);
                    }
                    manipulator->get_state_space()->free_point(temp_state);
                }
                else
                {
                    PRX_FATAL_S("Grasping visualizer received incorrect filename: "<<filename);
                }
            }


            void grasp_visualizer_t::determine_top_and_front_faces(movable_body_plant_t* input_object, std::string& top_front_face, std::vector<std::string>& alternative_faces)
            {
                alternative_faces.clear();

                // if(original_object_state == NULL)
                //     original_object_state = object->get_state_space()->alloc_point();
                // else
                //     object->get_state_space()->copy_to_point(original_object_state);

                /** Get configuration of object **/
                config_t config;
                input_object->get_configuration(config);

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


            void grasp_visualizer_t::determine_face(const vector_t& axis, const quaternion_t& rotation, std::string& face)
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
