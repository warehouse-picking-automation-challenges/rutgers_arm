/**
 * @file pushing_plant.cpp 
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

#include "simulation/pushing_plant.hpp"
#include "prx/utilities/math/2d_geometry/angle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/collision_checking/pqp_collision_checker.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::conformant::pushing_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        namespace conformant
        {
            pushing_plant_t::pushing_plant_t() : integration_plant_t()
            {
                state_memory = {&_x,&_y,&_yaw,&_r,&_angle_offset};
                control_memory = {&_vx,&_vy};

                state_space = new space_t("Pushing", state_memory);
                input_control_space = new space_t("XdYd", control_memory);
            }

            pushing_plant_t::~pushing_plant_t() { }

            void pushing_plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                _z = parameters::get_attribute_as<double>("z", reader, template_reader, 1.5);
                PRX_DEBUG_S("@@@Init pushing_plant_t : " << pathname);
                integration_plant_t::init(reader, template_reader);
            }

            void pushing_plant_t::propagate(const double simulation_step)
            {
                // PRX_INFO_S(state_space->print_memory());
                integration_plant_t::propagate(simulation_step);
                // PRX_INFO_S(state_space->print_memory());
                // exit(0);
                // PRX_INFO_S(_r);
            }

            void pushing_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                root_config.set_position(_x+_r*cos(_angle_offset+PRX_PI),_y+_r*sin(_angle_offset+PRX_PI),_z);
                root_config.set_orientation(0, 0, _yaw);

                augment_config_list(configs,index);
                configs[index].first = config_names[1];
                configs[index].second.set_position(_x,_y,_z);
                configs[index].second.set_orientation(0, 0, 0, 1);
                index++;
                plant_t::update_phys_configs(configs, index);
            }

            void pushing_plant_t::link_collision_info(collision_checker_t* collision_checker)
            {
                for(unsigned i=0;i<collision_infos.size();i++)
                    collision_infos[i].first = NULL;
                int i =0;
                foreach(std::string config_name, config_names)
                {
                    collision_infos[i].first = collision_checker->get_collision_info(config_name);
                    i++;
                }
            }
            void pushing_plant_t::update_collision_info()
            {
                root_config.set_position(_x+_r*cos(_angle_offset+PRX_PI),_y+_r*sin(_angle_offset+PRX_PI),_z);
                root_config.set_orientation(0, 0, _yaw);

                int index = 0;
                collision_infos[0].first->update_matrices(root_config);
                index++;
                collision_infos[index].second.set_position(_x,_y,_z);
                collision_infos[index].second.set_orientation(0, 0, 0, 1);
                collision_infos[index].first->update_matrices(collision_infos[index].second);
                index++;
                for( config_list_t::const_iterator config_iter = relative_config_map.begin(); config_iter != relative_config_map.end(); ++config_iter )
                {
                    collision_infos[index].second = config_iter->second;
                    collision_infos[index].second.relative_to_global(root_config);
                    collision_infos[index].first->update_matrices(collision_infos[index].second);
                    index++;
                }    


            }

            bool angle_test(double angle, double vx, double vy)
            {

                double ax = cos(angle);
                double ay = sin(angle);
                double norm = sqrt(vx*vx+vy*vy);
                vx /= norm;
                vy /= norm;
                if(ax*vx+ay*vy>.9)
                    return true;
                else
                    return false;

                // double ax,ay,lx,ly,ux,uy;
                // lx = cos(lower);
                // ly = sin(lower);
                // ux = cos(upper);
                // uy = sin(upper);
                // // PRX_INFO_S(lower<<" "<<angle<<" "<<upper);
                // // PRX_INFO_S(ax<<", "<<ay<<" "<<lx<<", "<<ly<<" "<<ux<<", "<<uy);
                // bool val =  (ax*ly-ay*lx) < 0 && (ux*ay-uy*ax) < 0;
                // // PRX_INFO_S(val);
                // return val;
            } 

            void pushing_plant_t::update_derivative(state_t * const result)
            {
                //yaw angle is easy
                if(fabs(_vx) < PRX_ZERO_CHECK)
                    _vx = 0;
                if(fabs(_vy) < PRX_ZERO_CHECK)
                    _vy = 0;
                result->memory[2] = (atan2(_vy,_vx)-_yaw)/simulation::simulation_step;

                double manip_x = _x+_r*cos(_angle_offset+PRX_PI);
                double manip_y = _y+_r*sin(_angle_offset+PRX_PI);
                double projected_x = manip_x + _vx*simulation::simulation_step;
                double projected_y = manip_y + _vy*simulation::simulation_step;
                double projected_r = sqrt((projected_x-_x)*(projected_x-_x)+(projected_y-_y)*(projected_y-_y));

                // PRX_INFO_S(state_space->print_memory());
                // PRX_INFO_S(input_control_space->print_memory());
                // PRX_INFO_S(manip_x<<" "<<manip_y<<" "<<_vx<<" "<<_vy<<" "<<projected_x<<" "<<projected_y);
                double diffy = (_y-projected_y);
                if(fabs(diffy) < PRX_ZERO_CHECK)
                    diffy = 0;
                double diffx = (_x-projected_x);
                if(fabs(diffx) < PRX_ZERO_CHECK)
                    diffx = 0;
                if( angle_test(_angle_offset, _vx, _vy) && projected_r < .8 )
                {
                    result->memory[0] = _vx;
                    result->memory[1] = _vy;
                    result->memory[3] = 0;
                    result->memory[4] = 0;
                }
                else
                {
                    result->memory[0] = 0;
                    result->memory[1] = 0;
                    result->memory[3] = (projected_r-_r)/simulation::simulation_step;
                    result->memory[4] = (atan2(diffy,diffx) - _angle_offset)/simulation::simulation_step;

                }
                // PRX_INFO_S(diffx<<" "<<diffy);
                // PRX_INFO_S(result->memory[0]<<" "<<result->memory[1]<<" "<<result->memory[2]<<" "<<result->memory[3]<<" "<<result->memory[4]);
                // if(_r >= state_space->get_bounds()[4]->get_upper_bound() && !angle_test(_vyaw, _angle_offset-PRX_PI/2, _angle_offset+PRX_PI/2) )
                // {
                //     _v = 0;
                // }
                // double projected_x = _v*cos(_vyaw);
                // double projected_y = _v*sin(_vyaw);
                // result->memory[0] = projected_x;
                // result->memory[1] = projected_y;
                // if( angle_test(_angle_offset, _vyaw-.05, _vyaw+.05)  && _r < .8 )
                // {
                //     result->memory[3] = 0;
                //     result->memory[4] = 0;
                // }
                // else
                // {
                //     double diffx = projected_x*simulation::simulation_step-_r*cos(_angle_offset);
                //     double diffy = projected_y*simulation::simulation_step-_r*sin(_angle_offset);
                //     if(fabs(diffx) < PRX_ZERO_CHECK)
                //         diffx = 0;
                //     if(fabs(diffy) < PRX_ZERO_CHECK)
                //         diffy = 0;
                //     result->memory[4] = atan2(diffy,diffx)/simulation::simulation_step;
                //     result->memory[3] = sqrt((diffy)*(diffy)+(diffx)*(diffx))/simulation::simulation_step;
                // }
                // PRX_WARN_S(result->memory[0]<<" "<<result->memory[1]<<" "<<result->memory[2]<<" "<<result->memory[3]<<" "<<result->memory[4]);
                // exit(0);
            }

        }



    }
}

