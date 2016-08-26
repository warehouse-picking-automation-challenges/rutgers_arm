/**
 * @file circle_collision_checker.cpp
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

#include "prx/simulation/collision_checking/circle_collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/system_ptr.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS( prx::sim::circle_collision_checker_t, prx::sim::collision_checker_t)

namespace prx
{
    using namespace util;
    
    namespace sim
    {        

        circle_collision_checker_t::circle_collision_checker_t()
        {
            collision_list = NULL;
        }

        circle_collision_checker_t::~circle_collision_checker_t() { }

        void circle_collision_checker_t::link_collision_list(collision_list_t* list)
        {
            //This is assuming that any changes to the environment (obstacles and added/removed systems) are reflected in 
            // new collision lists that are linked. 
            typedef std::pair<double, vector_t> vpair;

            std::set< vpair > holdset;
            collision_checker_t::link_collision_list(list);
            config_cache.clear();
            name_cache.clear();
            radii_cache.clear();
            non_circular_obstacle_cache.clear();

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                if( radii_map[pair.second] == -1 )
                {
                    PRX_DEBUG_COLOR("Found obstacle!: " << name_map[pair.second],PRX_TEXT_GREEN);
                    //            non_circular_obstacle_map[pair.second].get_minkowski(false, radii_map[pair.first]);
                    geometry_t* minkowski_sum = non_circular_obstacle_map[pair.second].get_minkowski(false, radii_map[pair.first]);
                    PRX_ASSERT(minkowski_sum != NULL);
                    config_t* other_conf = &config_map[pair.second];
                    double angle = 2 * asin(other_conf->get_orientation().get_z());
                    double x_offset = other_conf->get_position().at(0);
                    double y_offset = other_conf->get_position().at(1);

                    holdset.clear();

                    foreach(vector_t vec, minkowski_sum->get_trimesh()->get_vertices())
                    {

                        vector_t v((vec[0] * cos(angle) - vec[1] * sin(angle)) + x_offset, (vec[0] * sin(angle) + vec[1] * cos(angle)) + y_offset);
                        holdset.insert(std::pair<double, vector_t > (atan2(v[0] - x_offset, v[1] - y_offset), v));

                    }
                    non_circular_obstacle new_obst;

                    foreach(vpair v, holdset)
                    {
                        new_obst.minkowski_vertices.push_back(v.second);
                        PRX_DEBUG_COLOR("Vertex: " << v.second, PRX_TEXT_GREEN);
                        //                .print();
                    }
                    new_obst.obstacle_name = pair.second;
                    new_obst.other_name = pair.first;
                    new_obst.other_conf = &config_map[pair.first];
                    non_circular_obstacle_cache.push_back(new_obst);

                }
                else
                {
                    config_cache.push_back(std::make_pair(&config_map[pair.first], &config_map[pair.second]));
                    name_cache.push_back(std::make_pair(name_map[pair.first], name_map[pair.second]));
                    radii_cache.push_back(std::make_pair(radii_map[pair.first], radii_map[pair.second]));
                }
            }

        }

        void circle_collision_checker_t::set_configuration(const std::string& name, const config_t& config)
        {
            config_map[name].copy(config);
        }

        void circle_collision_checker_t::add_body(const std::string& name, const geometry_t& geometry, system_t* plant, bool is_obstacle)
        {
            if( is_obstacle )
            {

                if( body_map[name] == NULL )
                {
                    PRX_DEBUG_COLOR("Adding obstacle: " << name, PRX_TEXT_CYAN);
                    double radius;
                    geometry_type geo_type = geometry.get_type();
                    if( geo_type == PRX_CYLINDER || geo_type == PRX_OPEN_CYLINDER || geo_type == PRX_SPHERE )
                    {
                        radius = geometry.get_info()->at(0);
                        body_map[name] = plant;
                        radii_map[name] = radius;
                        name_map[name] = name;
                        //                obstacle_configs_and_radii.push_back(std::make_pair(&config_map[name], radii_map[name]));
                        //                obstacle_body_names.push_back(name);
                    }
                    else
                    {
                        body_map[name] = plant;
                        radii_map[name] = -1;
                        name_map[name] = name;
                        non_circular_obstacle_map[name] = geometry;
                    }
                }
            }

                // ALL systems are assumed to be cylindrical / spherical!
            else
            {

                if( body_map[name] == NULL )
                {
                    PRX_DEBUG_COLOR("Adding system: " << name, PRX_TEXT_CYAN);
                    //            double radius, height;
                    //            geometry.get_cylinder(radius,height);
                    body_map[name] = plant;
                    radii_map[name] = geometry.get_info()->at(0);
                    name_map[name] = name;
                    //            body_configs_and_radii.push_back(std::make_pair(&config_map[name], radii_map[name]));
                    //            body_names.push_back(name);
                }
            }
        }

        void circle_collision_checker_t::remove_body(const std::string& name)
        {
            PRX_FATAL_S(" Remove Body is Not supported by circle_collision_checker_t!");
        }

        collision_checker_info_t* circle_collision_checker_t::get_collision_info(const std::string& name)
        {
            return NULL;
        }

        bool circle_collision_checker_t::in_collision()
        {
            //    PRX_WARN_S ("CHeck da collisions");
            if( collision_list != NULL )
            {
                for( unsigned i = 0; i < config_cache.size(); i++ )
                {
                    if( config_cache[i].first->get_position().distance(config_cache[i].second->get_position()) <= radii_cache[i].first + radii_cache[i].second )
                        return true;
                }
                for( unsigned num = 0; num < non_circular_obstacle_cache.size(); num++ )
                {
                    bool c = false;

                    for( int i = 0, j = non_circular_obstacle_cache[num].minkowski_vertices.size() - 1; i < (int)non_circular_obstacle_cache[num].minkowski_vertices.size(); j = i++ )
                    {
                        const vector_t& point = non_circular_obstacle_cache[num].other_conf->get_position();
                        double p1 = point[1];
                        double p0 = point[0];
                        double i1 = non_circular_obstacle_cache[num].minkowski_vertices[i][1];
                        double i0 = non_circular_obstacle_cache[num].minkowski_vertices[i][0];
                        double j1 = non_circular_obstacle_cache[num].minkowski_vertices[j][1];
                        double j0 = non_circular_obstacle_cache[num].minkowski_vertices[j][0];
                        //                PRX_INFO_S ("Other position: " << point);
                        if( (((i1 <= p1) && (p1 < j1)) ||

                             ((j1 <= p1) && (p1 < i1))) &&

                            (p0 < (j0 - i0) * (p1 - i1) / (j1 - i1) + i0) )

                            c = !c;
                    }

                    if( c )
                        return true;
                }
            }

            return false;
        }

        collision_list_t* circle_collision_checker_t::colliding_bodies()
        {

            colliding_bodies_list->clear();
            //    PRX_DEBUG_S("Size : " << static_cast<vector_collision_list_t*>(collision_list)->size());
            if( collision_list != NULL )
            {
                //        foreach(collision_pair_t pair, collision_list->get_body_pairs())
                //        {
                ////            PRX_DEBUG_S("Pair to check : " << pair.first << " -> " << pair.second);
                //            if(in_collision(pair.first,pair.second))  
                //                colliding_bodies_list->add_new_pair(pair);
                //        }
                for( unsigned i = 0; i < config_cache.size(); i++ )
                {
                    if( config_cache[i].first->get_position().distance(config_cache[i].second->get_position()) <= radii_cache[i].first + radii_cache[i].second )
                        colliding_bodies_list->add_pair(name_cache[i].first, name_cache[i].second);
                }
                for( unsigned num = 0; num < non_circular_obstacle_cache.size(); num++ )
                {
                    bool c = false;

                    for( int i = 0, j = non_circular_obstacle_cache[num].minkowski_vertices.size() - 1; i < (int)non_circular_obstacle_cache[num].minkowski_vertices.size(); j = i++ )
                    {
                        vector_t point = non_circular_obstacle_cache[num].other_conf->get_position();
                        //                PRX_INFO_S ("Other position: " << point);
                        if( (((non_circular_obstacle_cache[num].minkowski_vertices[i][1] <= point[1]) && (point[1] < non_circular_obstacle_cache[num].minkowski_vertices[j][1])) ||

                             ((non_circular_obstacle_cache[num].minkowski_vertices[j][1] <= point[1]) && (point[1] < non_circular_obstacle_cache[num].minkowski_vertices[i][1]))) &&

                            (point[0] < (non_circular_obstacle_cache[num].minkowski_vertices[j][0] - non_circular_obstacle_cache[num].minkowski_vertices[i][0]) * (point[1] - non_circular_obstacle_cache[num].minkowski_vertices[i][1]) / (non_circular_obstacle_cache[num].minkowski_vertices[j][1] - non_circular_obstacle_cache[num].minkowski_vertices[i][1]) + non_circular_obstacle_cache[num].minkowski_vertices[i][0]) )

                            c = !c;
                    }

                    if( c )
                        colliding_bodies_list->add_pair(non_circular_obstacle_cache[num].other_name, non_circular_obstacle_cache[num].obstacle_name);
                }
            }

            return colliding_bodies_list;
        }

        //bool circle_collision_checker_t::in_collision(int index1, int index2)
        //{
        //    return (body_configs_and_radii[index1].first->get_position().distance(body_configs_and_radii[index2].first->get_position()) <= body_configs_and_radii[index1].second + body_configs_and_radii[index2].second);
        //}

        //bool circle_collision_checker_t::in_collision(const std::string& name1, const std::string& name2)
        //{       
        //    double radius1 = radii_map[name1];
        //    double radius2 = radii_map[name2];
        //    
        //    if(body_map[name1]->is_active() && body_map[name2]->is_active() )
        //    {
        //        return (config_map[name1].get_position().distance(config_map[name2].get_position()) <= radius1+radius2);
        //    }
        //    return false;
        //}

        std::ostream& operator<<(std::ostream& output, const circle_collision_checker_t& checker)
        {
            //    output << "Systems in the collision_checker" << std::endl;
            //    for(hash_t<std::string, pqp_info_t*>::const_iterator  iter= checker.models_map.begin(); iter != checker.models_map.end(); ++iter)
            //        output << iter->first << std::endl;        
            //
            //    output << "\nWhite list for collisions: " << std::endl;


            if( checker.collision_list )
                checker.collision_list->print();

            output << std::endl;

            return output;
        }

    }
}
