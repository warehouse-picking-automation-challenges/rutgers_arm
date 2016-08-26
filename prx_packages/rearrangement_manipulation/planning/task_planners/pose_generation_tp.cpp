/**
 * @file grasp_evaluation_tp.cpp
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

#include "planning/task_planners/pose_generation_tp.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/communication/visualization_comm.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::rearrangement_manipulation::pose_generation_tp_t, prx::plan::planner_t)

using namespace boost::algorithm;

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            pose_generation_tp_t::pose_generation_tp_t()
            {
                nr_object_poses = 0;

                char* w = std::getenv("PRACSYS_PATH");
                prx_dir = std::string(w) + "/prx_packages/rearrangement_manipulation/grasp_poses/";
            }

            pose_generation_tp_t::~pose_generation_tp_t()
            {

            }

            void pose_generation_tp_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_INFO_S("Initializing Pose Generator task planner ...");

                std::string template_name;
                planner_t::init(reader, template_reader);

                //Initializing the grasping planner.
                if(!parameters::has_attribute("grasping_planner",reader,template_reader))
                    PRX_FATAL_S("Manipulation task planner needs a grasping planner!");

                const parameter_reader_t* grasping_planner_template_reader = NULL;

                if( parameters::has_attribute("grasping_planner/template",reader,template_reader) )
                {
                    std::string template_name = parameters::get_attribute("grasping_planner/template",reader,template_reader);
                    grasping_planner_template_reader = new parameter_reader_t(ros::this_node::getName() + "/" + template_name, global_storage);
                }

                planner_t* tmp_grasping_planner = parameters::create_from_loader<planner_t > ("prx_planning", reader, "grasping_planner", grasping_planner_template_reader, "");
                grasping_planner = dynamic_cast<grasping_planner_t *>(tmp_grasping_planner);
                grasping_planner->set_pathname(path + "/grasping_planner");
                parameters::initialize(grasping_planner, reader, "grasping_planner", grasping_planner_template_reader, "");

                if( grasping_planner_template_reader != NULL )
                {
                    delete grasping_planner_template_reader;
                    grasping_planner_template_reader = NULL;
                }


                /** Entering in an empty string in the context name skips evaluate of that particular arm */
                context_name = parameters::get_attribute("context_name", reader, template_reader, "");
                
                /** Entering in an empty string for left/right filename skips saving the valid poses to a file */
                if (parameters::has_attribute("save_folder", reader, template_reader))
                {
                    save_folder = parameters::get_attribute("save_folder", reader, template_reader);
                }
                else
                    PRX_FATAL_S("Specify the file to store the poses");


                if(!reader->has_attribute("objects"))
                    PRX_FATAL_S("You need to specify some objects");

                foreach(const parameter_reader_t* object_reader, reader->get_list("objects"))
                {
                    std::string name = object_reader->get_attribute("name");
                    std::vector< std::pair<quaternion_t, std::vector<double> > > rotations;
                    foreach(const parameter_reader_t* rotation_reader, object_reader->get_list("orientations"))
                    {
                        std::vector<double> q = rotation_reader->get_attribute_as<std::vector<double> >("orientation");
                        std::vector<double> axis = rotation_reader->get_attribute_as<std::vector<double> >("axis");
                        PRX_ASSERT(q.size() == 4);
                        rotations.push_back( std::make_pair(quaternion_t(q[0], q[1], q[2], q[3]), axis));

                    }
                    initial_rotations[name] = rotations;
                }
                
                /** The number of poses to be sampled and evaluated per arm */
                nr_object_poses = reader->get_attribute_as<int>("nr_object_poses");

                min_bounds = reader->get_attribute_as<std::vector<double> >("min_bounds");
                max_bounds = reader->get_attribute_as<std::vector<double> >("max_bounds");

                if(reader->has_attribute("retract_config"))
                {
                    std::vector<double> conf = reader->get_attribute_as< std::vector<double> >("retract_config");
                    retract_config.set_position(conf[0], conf[1], conf[2]);
                    retract_config.set_orientation(conf[3], conf[4], conf[5], conf[6]);
                }
                else
                {
                    retract_config.set_position(0,0,-.05);
                    retract_config.set_orientation(0,0,0,1);
                }
                
            }

            void pose_generation_tp_t::setup()
            {
                PRX_INFO_S("Setup manipulation tp ...");
                grasping_planner->setup();
                grasping_query = new grasping_query_t();
            }

            void pose_generation_tp_t::reset(){}

            void pose_generation_tp_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
                grasping_planner->link_world_model(model);
            }

            const statistics_t* pose_generation_tp_t::get_statistics()
            {
                PRX_WARN_S("Get statistics for manipulation task planner is not implemented!");
                return new statistics_t();
            }

            void pose_generation_tp_t::link_specification(specification_t* new_spec)
            {
                task_planner_t::link_specification(new_spec);
            }

            void pose_generation_tp_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);               
            }        

            bool pose_generation_tp_t::succeeded() const
            {
                return true;
                //return false;
            }

            bool pose_generation_tp_t::execute()
            {
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);
                int grasps_size;

                manipulation_model->use_context(context_name);
                state_t* our_full_manipulator = manipulation_model->get_current_manipulation_info()->full_manipulator_state_space->alloc_point();

                std::string dir = prx_dir + "/" + save_folder;
                boost::filesystem::path output_dir(dir);
                if( !boost::filesystem::exists(output_dir) )
                {
                    boost::filesystem::create_directories(output_dir);
                }


                foreach(std::string type, initial_rotations | boost::adaptors::map_keys)
                {
                    PRX_DEBUG_COLOR("Computing grasps for type: " << type , PRX_TEXT_BROWN);
                    movable_body_plant_t* chosen_object = NULL;
                    std::vector< std::pair<quaternion_t, std::vector<double> > >* rotations = &initial_rotations[type];
                    int different_rotations = rotations->size();

                    unsigned i = 0;
                    do
                    {
                        chosen_object = objects[i];
                        i++;
                    }while(chosen_object->get_object_type() != type);

                    PRX_DEBUG_COLOR("Found the object " << chosen_object->get_pathname() <<  " of type " << type , PRX_TEXT_BROWN);
                    state_t* initial_object_state = chosen_object->get_state_space()->alloc_point();
                    state_t* object_state = chosen_object->get_state_space()->alloc_point();
                    std::vector<double> new_state(chosen_object->get_state_space()->get_dimension());
                    grasps_size = grasping_planner->nr_grasps(context_name, chosen_object);

                    int new_poses = 0; 
                    PRX_DEBUG_COLOR("Number of poses " << new_poses << "/" << nr_object_poses, PRX_TEXT_CYAN);
                    while(new_poses < nr_object_poses)
                    {
                        new_state[0] = uniform_random(min_bounds[0], max_bounds[0]);
                        new_state[1] = uniform_random(min_bounds[1], max_bounds[1]);
                        new_state[2] = uniform_random(min_bounds[2], max_bounds[2]);

                        int index = uniform_random(0, different_rotations-1);
                        quaternion_t q = rotations->at(index).first;
                        double theta = uniform_random(0,PRX_PI);
                        quaternion_t rot(rotations->at(index).second[0]*theta, rotations->at(index).second[1]*theta, rotations->at(index).second[2]*theta);
                        q = q*rot;
                        q.normalize();

                        new_state[3] = q.get_x();
                        new_state[4] = q.get_y();
                        new_state[5] = q.get_z();
                        new_state[6] = q.get_w();
                        chosen_object->get_state_space()->copy_vector_to_point(new_state, object_state);
                        chosen_object->get_state_space()->copy_from_point(object_state);

                        bool good_pose = false;
                        PRX_DEBUG_COLOR("New pose: " << chosen_object->get_state_space()->print_point(object_state, 5) << "   number of grasps: " << grasps_size,  PRX_TEXT_CYAN);
                        for(unsigned i = 0; i < grasps_size; ++i)
                        {
                            // grasping_query->setup(  manipulation_model->get_current_manipulation_info()->full_arm_state_space, 
                            //                         manipulation_model->get_current_manipulation_info()->full_arm_control_space, 
                            //                         chosen_object, 
                            //                         i, 
                            //                         object_state, 
                            //                         retract_config);
                            grasping_planner->link_query(grasping_query);
                            grasping_planner->resolve_query();
                            if(grasping_query->found_grasp)
                            {
                                if(!good_pose)
                                {
                                    std::vector<double> point_memory;
                                    chosen_object->get_state_space()->copy_point_to_vector(object_state, point_memory);
                                    valid_poses.push_back(grasp_pose_t(point_memory));
                                    new_poses++;
                                    good_pose = true;
                                }
                                // valid_poses.back().add_new_grasp(grasping_query->relative_grasp_config, grasping_query->grasping_mode, grasping_query->release_mode);
                            }
                            PRX_STATUS("For pose [ " << new_poses << "/" << nr_object_poses << "]:  "<< i << "/" << grasps_size << "]...", PRX_TEXT_BROWN);
                        }
                    }

                    chosen_object->get_state_space()->copy_from_point(initial_object_state);
                    chosen_object->get_state_space()->free_point(object_state);
                    chosen_object->get_state_space()->free_point(initial_object_state);

                    std::string file = dir + "/" + type + ".grasp_database";
                    PRX_DEBUG_COLOR("Open the file " << file << " to store the grasps for type: " << type, PRX_TEXT_GREEN);
                    std::ofstream fout;
                    fout.open(file.c_str());
                    PRX_ASSERT(fout.is_open());
                    for (unsigned i = 0; i < valid_poses.size(); ++i)
                    {
                        fout << "-\n";
                        fout << "  pose: [";
                        for (unsigned j = 0; j < 7; ++j)
                        {
                            fout << valid_poses[i].state[j];
                            if (j !=6)
                                fout << ',';
                        }
                        fout << "]\n";
                        fout << "  relative_configurations:";
                        for(unsigned j=0; j<valid_poses[i].valid_grasps.size(); ++j)
                        {
                            fout << "\n    -";
                            fout << "\n      relative_config: " << valid_poses[i].valid_grasps[j].serialize();
                            fout << "\n      grasp_mode: " << valid_poses[i].grasp_modes[j] << "";
                            fout << "\n      release_mode: " << valid_poses[i].release_modes[j];
                            // fout << "\n      grasping_index: " << valid_poses[i].grasp_indices[j];
                        }
                        fout<<"\n";
                    }
                    fout.close();
                    valid_poses.clear();

                }
                    
                return true;
            }

            void pose_generation_tp_t::resolve_query()
            {

            }
        }
    }
}
