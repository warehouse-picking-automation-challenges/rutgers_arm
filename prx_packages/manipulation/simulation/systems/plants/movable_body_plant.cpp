/**
 * @file rigid_body_plant.cpp
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

#include "simulation/systems/plants/movable_body_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::movable_body_plant_t, prx::sim::system_t)

namespace prx
{
    namespace packages
    {
        using namespace util;
        using namespace sim;

        namespace manipulation
        {
            movable_body_plant_t::movable_body_plant_t() : plant_t()
            {
                state_memory = {&_x,&_y,&_z,&_qx,&_qy,&_qz,&_qw};

                state_space = new space_t("SE3", state_memory);
                input_control_space = new space_t("EMPTY", control_memory);
                
                object_type = "";

                grasp_descriptor = NULL;
            }

            movable_body_plant_t::~movable_body_plant_t()
            {
                if (grasp_descriptor != NULL)
                {
                    delete grasp_descriptor;
                }
            }

            void movable_body_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                plant_t::init(reader, template_reader);
                if(parameters::has_attribute("object_type", reader , template_reader))
                    object_type = parameters::get_attribute("object_type", reader , template_reader);
                else
                    PRX_FATAL_S("You need to specify the object_type of your movable object");
                
                //Initialize the grasp descriptor for this object
                grasp_descriptor = new grasp_descriptor_t ();
                if( parameters::has_attribute("grasp_descriptor", reader, template_reader) )
                {
                    grasp_descriptor->init( (reader == NULL ? NULL : reader->get_subreader("grasp_descriptor")), (template_reader == NULL ? NULL : template_reader->get_subreader("grasp_descriptor")) );
                }
            }

            void movable_body_plant_t::push_state(const state_t * const state)
            {
                state_space->copy_from_point(state);
            }

            void movable_body_plant_t::move_object( const config_t& end_effector_config, const config_t& relative_config )
            {
                //Transform the configuration based on the manipulator configuration
                tmp_config = relative_config;
                tmp_config.relative_to_global( end_effector_config );
                //Store the result into the state
                tmp_config.get_position(_x, _y, _z);
                tmp_config.get_orientation().get(_qx, _qy, _qz, _qw);
                // root_config = relative_config;
                // root_config.relative_to_global( end_effector_config );
                // //Store the result into the state
                // root_config.get_position(_x, _y, _z);
                // root_config.get_orientation().get(_qx, _qy, _qz, _qw);

            }

            void movable_body_plant_t::relative_config_with_manipulator(const util::config_t& end_effector_config, util::config_t& relative_config )
            {
                relative_config.set_position(_x, _y, _z);
                relative_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
                relative_config.global_to_relative( end_effector_config );
            }


            void movable_body_plant_t::update_collision_info()
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
                plant_t::update_collision_info();
            }
            
            void movable_body_plant_t::propagate(const double simulation_step)
            {
            }

            void movable_body_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                root_config.set_position(_x, _y, _z);
                root_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
                plant_t::update_phys_configs(configs, index);
            }

            void movable_body_plant_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
            }

            void movable_body_plant_t::get_configuration( config_t& body_config ) const
            {
                body_config.set_position(_x, _y, _z);
                body_config.set_xyzw_orientation(_qx, _qy, _qz, _qw);
            }

            void movable_body_plant_t::set_configuration( const config_t& body_config )
            {
                root_config = body_config;
                root_config.get_position(_x,_y,_z);
                root_config.get_orientation().get(_qx, _qy, _qz, _qw);
            }

            void movable_body_plant_t::print_configuration( ) const
            {
                PRX_PRINT( "Object configuration: " << root_config, PRX_TEXT_GREEN );
            }

            grasp_descriptor_t* movable_body_plant_t::get_grasp_descriptor()
            {
                return grasp_descriptor;
            }

            std::string movable_body_plant_t::get_object_type() const
            {
                return object_type;
            }

            void movable_body_plant_t::update_vis_info() const
            {
                // if (grasp_descriptor != NULL)
                // {
                //     std::vector<geometry_info_t> geoms;
                //     std::vector<config_t> configs;
                //     std::vector< double > params;

                //     unsigned count = 0;
                //     //Visualize us some surfaces
                //     for( auto surf : grasp_descriptor->get_surfaces() )
                //     {
                //         std::string name = "surf_" + int_to_str(count);
                //         std::string o_name = "sam_surf_" + int_to_str(count);

                //         geoms.push_back(geometry_info_t(pathname, o_name, PRX_POLYGON, surf->sample_triangles, "magenta" ) );
                //         configs.push_back(root_config);
                //         geoms.push_back(geometry_info_t(pathname, name, PRX_POLYGON, surf->triangles, "viridian" ) );
                //         configs.push_back(root_config);
                //         count++;
                //     }
                    
                //     config_t temp_config;
                //     for( auto vol : grasp_descriptor->get_volumes() )
                //     {
                //         std::string name = "vol_" + int_to_str(count);

                //         params.clear();
                //         params.push_back( vol->radius );
                //         params.push_back( vol->height );

                //         geoms.push_back(geometry_info_t(pathname, name, PRX_CYLINDER, params, "magenta" ) );
                        
                //         temp_config = vol->relative_config;
                //         temp_config.relative_to_global( root_config );
                //         configs.push_back(temp_config);
                        
                //         count++;                    
                //     }
                    
                //     //Actually push this out to the visualization
                //     std::string name = pathname + "_info";
                    
                //     ((comm::visualization_comm_t*)comm::vis_comm)->visualization_geom_map[name] = geoms;
                //     ((comm::visualization_comm_t*)comm::vis_comm)->visualization_configs_map[name] = configs;
                //     geoms.clear();
                //     configs.clear();
                //     ((comm::visualization_comm_t*)comm::vis_comm)->send_geometries();
                // }
            }

        }
    }
}

