/**
 * @file bullet_obstacle.hpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#ifdef BULLET_FOUND
#pragma once

#ifndef PRX_ODE_OBSTACLE
#define PRX_ODE_OBSTACLE


#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/system.hpp"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
 
namespace prx 
 { 
 namespace sim 
 {

/**
 * Bullet version of obstacle.
 * 
 * @brief <b> Bullet version of obstacle </b>
 * 
 * @author Zakary Littlefield
 */
class bullet_obstacle_t : public system_t
{

  public:
    bullet_obstacle_t();
    virtual ~bullet_obstacle_t();

    /** @copydoc system_t::init() */
    void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

    /** @copydoc obstacle_t::get_state_space() */
    virtual const util::space_t* get_state_space() const;

    /** @copydoc obstacle_t::get_control_space() */
    virtual const util::space_t* get_control_space() const;

    
    /** @copydoc obstacle_t::compute_control() */
    virtual void compute_control();
    
    /** @copydoc obstacle_t::propagate() */
    virtual void propagate(const double simulation_step = 0);

    virtual void link_collision_info(collision_checker_t* collision_checker){}
    virtual void update_collision_info(){}
    
    /** @copydoc system_t::update_phys_configs() */
    virtual void update_phys_configs(util::config_list_t& configs,unsigned& index) const;

    /** @copydoc system_t::update_phys_geoms(util::geom_map_t&) */
    virtual void update_phys_geoms(util::geom_map_t& geoms) const;
    
    /** @copydoc system_t::get_sensed_geoms(util::geom_map_t&) */
    virtual void get_sensed_geoms(util::geom_map_t& geoms) const;
    
    /** @copydoc system_t::update_system_graph(system_graph_t&) */
    system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph);

    /** @copydoc system_t::set_param(const std::string&, const std::string&, const boost::any&) */
    virtual void set_param(const std::string& system_name, const std::string& parameter_name, const boost::any& value);
        
    /** @copydoc system_t::verify() */
    virtual void verify() const;
    
    /**
     * Returns the names of the geometries that the obstacle has.
     * 
     * @brief Returns the names of the geometries that the obstacle has.
     * 
     * @return A vector with all the names of the geometries.
     */
    virtual std::vector<std::string>* get_geometries_names();

    /**
     * Will be thrown if a subsystem path does not reference an existing subsystem.
     */
    class invalid_operation_exception : public std::runtime_error
    {

      public:

        invalid_operation_exception(const std::string& function) : std::runtime_error("Invalid function for this system: \"" + function + "\"."){ };
    };

    void add_bodies(std::vector<std::pair<btCollisionShape*,btRigidBody*> >& global_list);

    btTriangleIndexVertexArray* m;
    btTransform* transform;

  protected:

    /** @copydoc system_t::update_vis_info() */
    virtual void update_vis_info() const;
    
    /** @copydoc system_t::set_param() */
    void set_param(const std::string& parameter_name, const boost::any& value);

    /** @brief The map with the geometries of the obstacle.*/
    util::geom_map_t geometries;

    /** @brief The map with all the configurations for each different geometry of the obstacle.*/
    util::config_list_t configurations;
    
    /** @brief The names of each different geometry in this obstacle. Each obstacle can consist of a 
     combination of different geometries.*/
    std::vector<std::string> geometries_names;

    std::vector<std::pair<btCollisionShape*,btRigidBody*> > rigid_bodies;
};

} 
 }

#endif
#endif

