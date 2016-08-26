/**
 * @file fcl_collision_checker.cpp
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

#include "prx/simulation/collision_checking/fcl_collision_checker.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/system_ptr.hpp" 

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::fcl_collision_checker_t, prx::sim::collision_checker_t)

namespace prx
{
    using namespace util;

    namespace sim
    {

        fcl_collision_checker_t::fcl_collision_checker_t()
        {
            PRX_PRINT("FCL", PRX_TEXT_CYAN);
            collision_list = NULL;
        }

        fcl_collision_checker_t::~fcl_collision_checker_t()
        {
            for( models_map_iter = models_map.begin(); models_map_iter != models_map.end(); ++models_map_iter )
                delete models_map_iter->second;

            models_map.clear();

        }

        void fcl_collision_checker_t::link_collision_list(collision_list_t* list)
        {
            //This is assuming that any changes to the environment (obstacles and added/removed systems) are reflected in
            // new collision lists that are linked.

            collision_checker_t::link_collision_list(list);
            body_cache.clear();

            foreach(collision_pair_t pair, collision_list->get_body_pairs())
            {
                body_cache.push_back(std::pair<fcl_info_t*, fcl_info_t*>(models_map[pair.first], models_map[pair.second]));
            }

        }

        void fcl_collision_checker_t::set_configuration(const std::string& name, const config_t& config)
        {
            models_map[name]->update_matrices(config);
        }

        void fcl_collision_checker_t::add_body(const std::string& name, const geometry_t& geometry, system_t* plant, bool is_obstacle)
        {
            if( models_map[name] == NULL )
            {
                models_map[name] = new fcl_info_t(plant);
                create_model(name, geometry);
            }
        }

        void fcl_collision_checker_t::remove_body(const std::string& name)
        {
            if( models_map[name] != NULL )
            {
                models_map.erase(name);
                models_map[name] = NULL;
            }
        }

        bool fcl_collision_checker_t::in_collision()
        {
            if( collision_list != NULL )
            {
                unsigned size = body_cache.size();
                for( unsigned i = 0; i < size; i++ )
                    if( in_collision(body_cache[i].first, body_cache[i].second) )
                        return true;
            }

            return false;
            // if( collision_list != NULL )
            // {

            //     foreach(collision_pair_t pair, collision_list->get_body_pairs())
            //     {
            //         if( in_collision(pair.first, pair.second) )
            //         {
            //             return true;
            //         }
            //     }
            // }

            // return false;
        }

        collision_list_t* fcl_collision_checker_t::colliding_bodies()
        {

            colliding_bodies_list->clear();
            if( collision_list != NULL )
            {

                foreach(collision_pair_t pair, collision_list->get_body_pairs())
                {
                    if( in_collision(pair.first, pair.second) )
                        colliding_bodies_list->add_new_pair(pair);
                }
            }

            return colliding_bodies_list;
        }

#define PRIMITIVES 1
        void fcl_collision_checker_t::create_model(const std::string& name, const geometry_t& geometry)
        {

            geometry_type gt = geometry.get_type();
            if(PRIMITIVES && (gt <= 4 || gt ==6 ))
            {
                if(gt == PRX_SPHERE)
                {
                    double sphere;
                    geometry.get_sphere(sphere);
                    models_map[name]->model = new fcl::Sphere(sphere);
                }
                else if(gt == PRX_BOX)
                {
                    double x,y,z;
                    geometry.get_box(x,y,z);
                    models_map[name]->model = new fcl::Box(x,y,z);
                }
                else if(gt == PRX_CONE)
                {
                    double rad,z;
                    geometry.get_cone(rad,z);
                    models_map[name]->model = new fcl::Cone(rad,z);
                }
                else if(gt == PRX_CYLINDER)
                {
                    double rad,z;
                    geometry.get_cylinder(rad,z);
                    models_map[name]->model = new fcl::Cylinder(rad,z);                  
                }
                else if(gt == PRX_CAPSULE)
                {
                    double rad,z;
                    geometry.get_capsule(rad,z);
                    models_map[name]->model = new fcl::Capsule(rad,z);                
                }
                else
                {
                    PRX_FATAL_S("This should not happen! Trying to create FCL geometry ");
                }

                trimesh_t trimesh;
                geometry.get_trimesh(&trimesh);
                int nr_indices = trimesh.get_faces_size();
                vector_t vertex; // tmp variable
                face_t face; // tmp variable
                double radius=0;
                
                for( int i = 0; i < nr_indices; ++i )
                {
                    trimesh.get_face_at(i, &face);
                    trimesh.get_vertex_at(face.get_index1(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    trimesh.get_vertex_at(face.get_index2(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    trimesh.get_vertex_at(face.get_index3(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                }
                
                models_map[name]->broadphase_radius = radius;
            }
            else if(gt==PRX_CLOUD)
            {

            }
            else
            {
                trimesh_t trimesh;
                geometry.get_trimesh(&trimesh);
                PRX_DEBUG_S("create the model for : " << name);
                //    geometry.print();
                models_map[name]->model = new FCL_Mesh();
                FCL_Mesh* model = static_cast<FCL_Mesh*>(models_map[name]->model);

                model->beginModel();

                int nr_indices = trimesh.get_faces_size();

                // if( nr_indices % 3 != 0 )
                //     PRX_FATAL_S("There is a problem with the model. \nIt is probably not made out of triangles\n Number of indices : " << nr_indices);

                std::vector<fcl::Triangle> triangles;
                std::vector <fcl::Vec3f> points;

                vector_t vertex; // tmp variable
                face_t face; // tmp variable
                double radius=0;
                
                for( int i = 0; i < nr_indices; ++i )
                {
                    trimesh.get_face_at(i, &face);
                    trimesh.get_vertex_at(face.get_index1(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));

                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    trimesh.get_vertex_at(face.get_index2(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    trimesh.get_vertex_at(face.get_index3(), &vertex);
                    radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
                    points.push_back(fcl::Vec3f(vertex[0], vertex[1], vertex[2]));

                    triangles.push_back(fcl::Triangle(points.size() - 3, points.size() - 2, points.size() - 1));
                }
                
                models_map[name]->broadphase_radius = radius;
                
                
                model->addSubModel(points, triangles);
                model->endModel();
                model->computeLocalAABB();
            }
        }

        bool fcl_collision_checker_t::in_collision(const std::string& name1, const std::string& name2)
        {

#ifndef NDEBUG
            if( models_map[name1] == NULL )
                throw invalid_system_pair("The system " + name1 + " does not exist in the collision_checker");
            if( models_map[name2] == NULL )
                throw invalid_system_pair("The system " + name2 + " does not exist in the collision_checker");
#endif 
            fcl_info_t* info1 = models_map[name1];
            fcl_info_t* info2 = models_map[name2];

            return in_collision(info1,info2); 

            return false;
        }

        void fcl_collision_checker_t::update_model(const std::string& name, sensor_msgs::PointCloud2& cloud)
        {

            boost::shared_ptr<octomap::OcTree> tree(new octomap::OcTree(octomap_resolution));
            octomap::Pointcloud octomapCloud;
            octomap::point3d origin(0,0,0);
            sensor_msgs::PointCloud cloud1;
            sensor_msgs::convertPointCloud2ToPointCloud(cloud,cloud1);

            unsigned point_size = cloud1.points.size();
            PRX_INFO_S("NUMBER OF POINTS: "<<point_size);  

            for (unsigned i=0;i<point_size;i++)
            {
                // Check if the point is invalid
                if(!std::isnan(cloud1.points[i].x) && !std::isnan(cloud1.points[i].y) && !std::isnan(cloud1.points[i].y))
                    octomapCloud.push_back(cloud1.points[i].x,cloud1.points[i].y,cloud1.points[i].z);
            }
            tree->insertScan(octomapCloud,origin);

            if(models_map[name]->model!=NULL)
            {
                delete models_map[name]->model;
            }
            models_map[name]->model = new fcl::OcTree(tree);

            std::vector<boost::array<fcl::FCL_REAL, 6> > boxes_ = ((fcl::OcTree*)models_map[name]->model)->toBoxes();

            std::ofstream fout;
            fout.open("/Users/zlittlefield/prx/pracsys/prx_output/single_shot_output/point_cloud.txt");
            for(std::size_t i = 0; i < boxes_.size(); ++i)
            {
                fcl::FCL_REAL x = boxes_[i][0];
                fcl::FCL_REAL y = boxes_[i][1];
                fcl::FCL_REAL z = boxes_[i][2];
                fcl::FCL_REAL size = boxes_[i][3];
                fcl::FCL_REAL cost = boxes_[i][4];
                fcl::FCL_REAL threshold = boxes_[i][5];
                PRX_INFO_S(x<<" "<<y<<" "<<z<<" "<<size<<" "<<cost<<" "<<threshold);  
                fout<<std::endl<<"Box "<<x<<" "<<y<<" "<<z<<" ";
                fout<<octomap_resolution<<" "<<.5<<" "<<.5<<" "<<.5;
            }
            fout.close();
        }

        collision_checker_info_t* fcl_collision_checker_t::get_collision_info(const std::string& name)
        {
            return models_map[name];
        }

        bool fcl_collision_checker_t::in_collision(fcl_info_t* info1, fcl_info_t* info2)
        {

#ifndef NDEBUG
            if( info1 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");
            if( info2 == NULL )
                throw invalid_system_pair("A system does not exist in the collision_checker");

#endif
            if( info1->model!=NULL && info2->model!=NULL && info1->system->is_active() && info2->system->is_active() )
            {
                double dx = info1->translation[0] - info2->translation[0];
                double dy = info1->translation[1] - info2->translation[1];
                double dz = info1->translation[2] - info2->translation[2];
                double dr = info1->broadphase_radius + info2->broadphase_radius;
                // dr=PRX_INFINITY;
                // if(info1->broadphase_radius==PRX_INFINITY || info2->broadphase_radius==PRX_INFINITY )
                // {
                //     PRX_PRINT("Not created model",PRX_TEXT_RED);
                // }
                if( dx * dx + dy * dy + dz * dz < dr * dr )
                {
                    //PRX_PRINT("Not skipping collision checking.",PRX_TEXT_RED);

                    
                    //                contacts.clear();
                    //                bool valid;
                    //                valid = fcl::collide(info1->model, info1->transform, info2->model, info2->transform, 1, false, false, contacts);
                    //                fcl::CollisionObject c1(info1->model, info1->transform);
                    //                fcl::CollisionObject c2(info2->model, info2->transform);
                    fcl::CollisionRequest request;
                    fcl::CollisionResult result;
                    if( fcl::collide(info1->model, info1->transform, info2->model, info2->transform, request, result) > 0 )
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    //(fcl::collide(&c1,&c2,request,result)>0);
                    //return result.isCollision();
                }
                else
                {
                    //PRX_PRINT("Skipping collision checkinf.", PRX_TEXT_GREEN);
                }
            }
            return false;
        }


        std::ostream& operator<<(std::ostream& output, const fcl_collision_checker_t& checker)
        {
            output << "Systems in the collision_checker" << std::endl;
            for( hash_t<std::string, fcl_info_t*>::const_iterator iter = checker.models_map.begin(); iter != checker.models_map.end(); ++iter )
                output << iter->first << std::endl;

            output << "\nWhite list for collisions: " << std::endl;


            if( checker.collision_list )
                checker.collision_list->print();

            output << std::endl;

            return output;
        }

    }
}
