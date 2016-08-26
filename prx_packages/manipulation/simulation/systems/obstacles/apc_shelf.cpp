/**
 * @file apc_shelf.cpp
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


#include "simulation/systems/obstacles/apc_shelf.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::packages::manipulation::apc_shelf_t, prx::sim::system_t)

//Move to yaml file
//******************************
//bottom to top
//*******************************

#define LIP_HEIGHT 0.055
#define LIP_DEPTH 0.025
#define LIP_OFFSET 0.005
#define LIP_OFFSET_X 0.0025


#define LEG_WIDTH 0.05
#define LEG_DEPTH 0.01
#define LEG_OFFSET 0.005

#define DIVIDER_SIZE 0.004

#define TOP_HEIGHT 1.0
#define BOTTOM_HEIGHT 0.805

#define SHELF_BOTTOM_OFFSET -0.01

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace manipulation
        {
            apc_shelf_t::apc_shelf_t() { }

            apc_shelf_t::~apc_shelf_t() { }

            void apc_shelf_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                //read in dimensions of the shelf
                root_config.zero();
                if(parameters::has_attribute("root_configuration",reader,template_reader))
                {
                    parameters::initialize(&root_config,reader,"root_configuration",template_reader,"root_configuration");
                }

                FOURTH_SHELF_HEIGHT  = parameters::get_attribute_as<double>("FOURTH_SHELF_HEIGHT",reader,template_reader,0.27);
                THIRD_SHELF_HEIGHT = parameters::get_attribute_as<double>("THIRD_SHELF_HEIGHT",reader,template_reader,0.228);
                SECOND_SHELF_HEIGHT = parameters::get_attribute_as<double>("SECOND_SHELF_HEIGHT",reader,template_reader,0.228);
                FIRST_SHELF_HEIGHT  = parameters::get_attribute_as<double>("FIRST_SHELF_HEIGHT",reader,template_reader,0.265);
                LEFT_SHELF_WIDTH  = parameters::get_attribute_as<double>("LEFT_SHELF_WIDTH",reader,template_reader,0.275);
                MIDDLE_SHELF_WIDTH = parameters::get_attribute_as<double>("MIDDLE_SHELF_WIDTH",reader,template_reader,0.305);
                RIGHT_SHELF_WIDTH  = parameters::get_attribute_as<double>("RIGHT_SHELF_WIDTH",reader,template_reader,0.275);
                SHELF_DEPTH = parameters::get_attribute_as<double>("SHELF_DEPTH",reader,template_reader,.43);
                TOP_SHELF_OFFSET = parameters::get_attribute_as<double>("TOP_SHELF_OFFSET",reader,template_reader,0.023);
                HOR_SHELF_OFFSET = parameters::get_attribute_as<double>("HOR_SHELF_OFFSET",reader,template_reader,0.0075);
                
                shelf_dims.resize(3);
                shelf_dims[2] = FOURTH_SHELF_HEIGHT+THIRD_SHELF_HEIGHT+SECOND_SHELF_HEIGHT+FIRST_SHELF_HEIGHT;
                shelf_dims[1] = LEFT_SHELF_WIDTH+MIDDLE_SHELF_WIDTH+RIGHT_SHELF_WIDTH;
                shelf_dims[0] = SHELF_DEPTH;

                LEG_HEIGHT = BOTTOM_HEIGHT+shelf_dims[2]+TOP_SHELF_OFFSET;

                compute_shelf_geoms();
                compute_lips();
                compute_dividers();
                compute_large_regions();
            }

            void apc_shelf_t::compute_shelf_geoms()
            {
                // bottom_shelf, middle_shelf, top_shelf

                std::string geom_name;
                config_t rel_config;
                std::vector<double> rel_config_vec = {0,0,-99,0,0,0,1};
                double start_val;
                start_val = -shelf_dims[2]/2;

                geom_name = pathname + "/bottom_shelf" ;
                geometries[geom_name].set_box( shelf_dims[0], shelf_dims[1], DIVIDER_SIZE);
                rel_config_vec.assign( { 0.0, 0.0, start_val + FIRST_SHELF_HEIGHT + SHELF_BOTTOM_OFFSET, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);
                geom_name = pathname + "/middle_shelf" ;
                geometries[geom_name].set_box( shelf_dims[0], shelf_dims[1], DIVIDER_SIZE);
                rel_config_vec.assign( { 0.0, 0.0, start_val+FIRST_SHELF_HEIGHT+SECOND_SHELF_HEIGHT + SHELF_BOTTOM_OFFSET, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);
                geom_name = pathname + "/top_shelf" ;
                geometries[geom_name].set_box( shelf_dims[0], shelf_dims[1], DIVIDER_SIZE);
                rel_config_vec.assign( { 0.0, 0.0, start_val+FIRST_SHELF_HEIGHT+SECOND_SHELF_HEIGHT+THIRD_SHELF_HEIGHT + SHELF_BOTTOM_OFFSET, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

            }
            void apc_shelf_t::compute_lips()
            {
                //bottom_lip, second_lip, middle_lip, third_lip, top_lip
                std::string geom_name;
                config_t rel_config;
                std::vector<double> rel_config_vec = {0,0,-99,0,0,0,1};
                double start_val;
                start_val = -shelf_dims[2]/2;
                
                geom_name = pathname + "/bottom_lip" ;
                geometries[geom_name].set_box( LIP_DEPTH, shelf_dims[1], LIP_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2 - LIP_OFFSET_X, 0.0, start_val - LIP_OFFSET, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/second_lip" ;
                geometries[geom_name].set_box( LIP_DEPTH, shelf_dims[1], LIP_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2 - LIP_OFFSET_X, 0.0, start_val - LIP_OFFSET + FIRST_SHELF_HEIGHT, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/middle_lip" ;
                geometries[geom_name].set_box( LIP_DEPTH, shelf_dims[1], LIP_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2 - LIP_OFFSET_X, 0.0, start_val - LIP_OFFSET + FIRST_SHELF_HEIGHT+SECOND_SHELF_HEIGHT, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/third_lip" ;
                geometries[geom_name].set_box( LIP_DEPTH, shelf_dims[1], LIP_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2 - LIP_OFFSET_X, 0.0, start_val - LIP_OFFSET + FIRST_SHELF_HEIGHT+SECOND_SHELF_HEIGHT+THIRD_SHELF_HEIGHT, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/top_lip" ;
                geometries[geom_name].set_box( LIP_DEPTH, shelf_dims[1], LIP_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2 - LIP_OFFSET_X, 0.0, -start_val - LIP_OFFSET, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                //left_leg, right_leg
                geom_name = pathname + "/left_leg" ;
                geometries[geom_name].set_box( LEG_DEPTH, LEG_WIDTH, LEG_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2, shelf_dims[1]/2 + HOR_SHELF_OFFSET - LEG_WIDTH/2, -(shelf_dims[2]/2 - (LEG_HEIGHT/2-BOTTOM_HEIGHT)), 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/right_leg" ;
                geometries[geom_name].set_box( LEG_DEPTH, LEG_WIDTH, LEG_HEIGHT);
                rel_config_vec.assign( { -shelf_dims[0]/2, -(shelf_dims[1]/2 + HOR_SHELF_OFFSET - LEG_WIDTH/2), -(shelf_dims[2]/2 - (LEG_HEIGHT/2-BOTTOM_HEIGHT)), 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);
            }
            void apc_shelf_t::compute_dividers()
            {
                //left_side, left_divider, right_divider, right_side
                std::string geom_name;
                config_t rel_config;
                std::vector<double> rel_config_vec = {0,0,-99,0,0,0,1};

                double start_val;
                start_val = -shelf_dims[1]/2;
                
                geom_name = pathname + "/left_side" ;
                geometries[geom_name].set_box( shelf_dims[0], DIVIDER_SIZE, shelf_dims[2]);
                rel_config_vec.assign( { 0, shelf_dims[1]/2 + HOR_SHELF_OFFSET, 0.0, 0, 0, 0, 1} );//MIDDLE_SHELF_WIDTH/2 + LEFT_SHELF_WIDTH
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/left_divider" ;
                geometries[geom_name].set_box( shelf_dims[0], DIVIDER_SIZE, shelf_dims[2]);
                rel_config_vec.assign( { 0, start_val+RIGHT_SHELF_WIDTH+MIDDLE_SHELF_WIDTH, 0.0, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/right_divider" ;
                geometries[geom_name].set_box( shelf_dims[0], DIVIDER_SIZE, shelf_dims[2]);
                rel_config_vec.assign( { 0, start_val + RIGHT_SHELF_WIDTH, 0.0, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/right_side" ;
                geometries[geom_name].set_box( shelf_dims[0], DIVIDER_SIZE, shelf_dims[2]);
                rel_config_vec.assign( {0, -(shelf_dims[1]/2 + HOR_SHELF_OFFSET), 0.0, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

            }
            void apc_shelf_t::compute_large_regions()
            {
                //top, bottom, mid_divider
                std::string geom_name;
                config_t rel_config;
                std::vector<double> rel_config_vec = {0,0,-99,0,0,0,1};

                geom_name = pathname + "/top" ;
                geometries[geom_name].set_box( shelf_dims[0], shelf_dims[1] + 2*HOR_SHELF_OFFSET, TOP_HEIGHT);
                rel_config_vec.assign( { 0, 0, shelf_dims[2]/2 + TOP_HEIGHT/2, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/bottom" ;
                geometries[geom_name].set_box( shelf_dims[0], shelf_dims[1], BOTTOM_HEIGHT);
                rel_config_vec.assign( { 0.0, 0.0, -(shelf_dims[2]/2 + BOTTOM_HEIGHT/2), 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);

                geom_name = pathname + "/mid_divider" ;
                geometries[geom_name].set_box( DIVIDER_SIZE, shelf_dims[1], shelf_dims[2]);
                rel_config_vec.assign( { shelf_dims[0]/2, 0.0, 0.0, 0, 0, 0, 1} );
                rel_config.copy_from_vector(rel_config_vec);
                add_geom(geom_name,rel_config);
            }



            void apc_shelf_t::add_geom(std::string geom_name, util::config_t& rel_config)
            {
                geometries_names.push_back(geom_name);
                relative_configurations.push_back(config_list_element_t(geom_name,rel_config));                    
                configurations.push_back(relative_configurations.back());
                configurations.back().second.relative_to_global(root_config);
                collision_infos.push_back(std::pair<collision_checker_info_t*,config_t>(NULL,configurations.back().second));
                PRX_INFO_S("prx::sim::comm::vis_comm = "<<prx::sim::comm::vis_comm);
                if(prx::sim::comm::vis_comm!=NULL)
                {
                    PRX_INFO_S(relative_configurations.back().second.print());
                    prx::sim::comm::vis_comm->add_marker_to_array(geometries[geom_name],geom_name,relative_configurations.back().second);
                }
            }
        }
    }
}