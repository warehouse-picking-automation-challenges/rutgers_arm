/**
 * @file bullet_obstacle.cpp 
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

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

#include "prx/simulation/systems/plants/bullet/bullet_obstacle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/systems/plants/bullet/bullet_body_info.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::bullet_obstacle_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {

        bullet_obstacle_t::bullet_obstacle_t() { }

        bullet_obstacle_t::~bullet_obstacle_t() { }

        void bullet_obstacle_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            std::vector<const parameter_reader_t*> readers = reader->get_list("geometries");
            std::string geom_name;

            config_t conf;
            vector_t pos(3);
            quaternion_t orient;

            foreach(const parameter_reader_t* r, readers)
            {
                geom_name = pathname + "/" + r->get_attribute_as< std::string > ("name");
                geometries_names.push_back(geom_name);
                geometries[geom_name] = *(r->initialize_new< geometry_t > (std::string("collision_geometry")));
                configurations.push_back(config_list_element_t(geom_name, *(r->initialize_new<config_t > (std::string("config")))));
                configurations.back().second.get(pos, orient);
                bullet_body_info_t* info = new bullet_body_info_t();
                PRX_INFO_S(geom_name);
                info->name = geom_name;
                info->geom = &geometries[geom_name];
                info->plant = this;
                info->config = conf;

                // config_names.push_back(geom_name);


                //create the rigid body in Bullet
                btCollisionShape* collision_shape;

                if( info->geom->get_type() == PRX_SPHERE )
                {
                    double radius;
                    info->geom->get_sphere(radius);
                    collision_shape = new btSphereShape(radius);
                }
                else if( info->geom->get_type() == PRX_BOX )
                {
                    double lx, ly, lz;
                    info->geom->get_box(lx, ly, lz);
                    collision_shape = new btBoxShape(btVector3(lx / 2, ly / 2, lz / 2));
                }
                else if( info->geom->get_type() == PRX_CYLINDER )
                {
                    double rad, height;
                    info->geom->get_cylinder(rad, height);
                    collision_shape = new btCylinderShapeZ(btVector3(rad, rad, height / 2));
                }
                else if( info->geom->get_type() == PRX_CAPSULE )
                {
                    double rad, height;
                    info->geom->get_capsule(rad, height);
                    collision_shape = new btCapsuleShapeZ(rad, height);
                }
                else if( info->geom->get_type() == PRX_HEIGHTMAP )
                {
                    // orient.set(vector_t(1,0,0),-PRX_PI/2);
                    // configurations.back().second.set_orientation(orient);
                    // numTriangles: number of triangles
                    // triangleIndexBase: the array of vertex indices that makes up the triangles
                    // triangleIndexStride: the number of bytes to skip in the vertex indices array to go from the start of one triangle to the start of the next triangle. Typically this is 3 times the sizeof the vertex index type.
                    // numVertices: number of vertices
                    // vertexBase: the array of vertex positions
                    // vertexStride: the number of bytes to skip in the vertex position array to go from the start of one vertex to the start of the next vertex. If the vertex position is composed of 3 floats for example, then this would be 3*sizeof(float). If it is 3 doubles with 1 double as padding, then 4*sizeof(double), for example.
                    const trimesh_t* mesh = info->geom->get_trimesh();
                    int num_faces = mesh->get_faces_size();
                    int num_vertices = mesh->get_vertices_size();
                    info->faces = new int[num_faces*3];
                    info->vertex_array = new double[num_vertices*3];

                    const std::vector<face_t>& faces = mesh->get_faces();
                    const std::vector<vector_t>& vertices = mesh->get_vertices();
                    for(int i=0;i<num_faces;i++)
                    {
                        info->faces[i*3+0] = faces[i].get_index1();
                        info->faces[i*3+1] = faces[i].get_index2();
                        info->faces[i*3+2] = faces[i].get_index3();
                    }
                    for(int i=0;i<num_vertices;i++)
                    {
                        info->vertex_array[i*3+0] = vertices[i][0];
                        info->vertex_array[i*3+1] = -vertices[i][1];
                        info->vertex_array[i*3+2] = vertices[i][2];
                    }
                    m = new btTriangleIndexVertexArray(num_faces,info->faces,3*sizeof(int),num_vertices,info->vertex_array,3*sizeof(double));
                    collision_shape = new btBvhTriangleMeshShape(m,true);
                    PRX_INFO_S("Created a heightmap!");
                }
                else
                {
                    PRX_FATAL_S("Unimplemented geometry type for bullet_obstacle");
                }
                transform = new btTransform(
                                                                                          btQuaternion(orient.get_x(), orient.get_y(), orient.get_z(), orient.get_w()),
                                                                                          btVector3(pos[0], pos[1], pos[2]));
                PRX_INFO_S(pos<<" "<<orient);
                btDefaultMotionState* motion_state = new btDefaultMotionState(*transform);
                collision_shape->setUserPointer(info);
                btRigidBody::btRigidBodyConstructionInfo construction_info(0, motion_state, collision_shape, btVector3(0, 0, 0));
                btRigidBody* rigid_body = new btRigidBody(construction_info);
                
                rigid_body->setFriction(1.0);
                rigid_body->setRollingFriction(.1);
                rigid_body->setRestitution(.2);
                rigid_body->setUserPointer(info);
                rigid_body->forceActivationState(DISABLE_DEACTIVATION);
                rigid_bodies.push_back(std::pair<btCollisionShape*, btRigidBody*>(collision_shape, rigid_body));

            }

        }

        const space_t* bullet_obstacle_t::get_state_space() const
        {
            throw invalid_operation_exception("bullet_obstacle_t::get_state_space()");
        }

        const space_t* bullet_obstacle_t::get_control_space() const
        {
            throw invalid_operation_exception("bullet_obstacle_t::get_control_space()");
        }

        void bullet_obstacle_t::compute_control()
        {
            throw invalid_operation_exception("bullet_obstacle_t::compute_control()");
        }

        void bullet_obstacle_t::propagate(const double simulation_step)
        {
            throw invalid_operation_exception("bullet_obstacle_t::propagate()");
        }

        void bullet_obstacle_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {

            foreach(config_list_element_t element, configurations)
            {
                if( configs.size() <= index )
                    configs.push_back(element);
                else
                    configs[index] = element;
                index++;
            }

            //    for( unsigned i=0; i<geom_ids.size(); ++i )
            //    {
            //        const dReal* pos = dGeomGetPosition( geom_ids[i] );
            //        dQuaternion ori;
            //        dGeomGetQuaternion( geom_ids[i], ori );
            //        
            //        PRX_LOG_INFO("ODE obstacle: ( %lf %lf %lf ) : %lf %lf %lf %lf", pos[0], pos[1], pos[2], ori[3], ori[0], ori[1], ori[2] );
            //        
            //        configs[ config_names[i] ].set_position( pos[0], pos[1], pos[2] );
            //        configs[ config_names[i] ].set_wxyz_orientation( ori[3], ori[0], ori[1], ori[2] );
            //    }
        }

        void bullet_obstacle_t::update_phys_geoms(geom_map_t& geoms) const { }
        
        void bullet_obstacle_t::get_sensed_geoms(geom_map_t& geoms) const { }

        system_graph_t::directed_vertex_t bullet_obstacle_t::update_system_graph(system_graph_t& graph)
        {
            //TODO: Think if we want the obstacles to be in the graph and if they are plants.
            return graph.add_node(pathname, true);
        }

        void bullet_obstacle_t::set_param(const std::string& system_name, const std::string& parameter_name, const boost::any& value)
        {
            if( system_name != "" )
                throw invalid_path_exception(system_name);
            set_param(parameter_name, value);
        }

        void bullet_obstacle_t::verify() const {
            // if( geometries.size() == 0 || configurations.size() == 0 )
            //     throw std::runtime_error("ode_obstacle_t::verify() failed because either geometries or configurations are not initialized.");
        }

        std::vector<std::string>* bullet_obstacle_t::get_geometries_names()
        {
            return &geometries_names;
        }

        void bullet_obstacle_t::update_vis_info() const { }

        void bullet_obstacle_t::set_param(const std::string& parameter_name, const boost::any& value) {
            // std::string name;
            // std::string member;
            // boost::tie(name, member) = split_path(parameter_name);

            // if( name == "configurations" )
            // {
            //     std::string param;
            //     boost::tie(member, param) = split_path(member);
            //     for( unsigned i = 0; i < configurations.size(); i++ )
            //     {
            //         if( member == configurations[i].first )
            //             configurations[i].second.set_param(param, boost::any_cast< double >(value));
            //     }
            // }
            // else if( name == "geometries" )
            // {
            //     geometries[ member ].set_params(boost::any_cast< std::vector<double>* >(value));
            // }
        }

        void bullet_obstacle_t::add_bodies(std::vector<std::pair<btCollisionShape*, btRigidBody*> >& global_list)
        {
            //add to the global list
            for( unsigned i = 0; i < rigid_bodies.size(); i++ )
            {
                global_list.push_back(rigid_bodies[i]);
            }
        }

    }
}
#endif