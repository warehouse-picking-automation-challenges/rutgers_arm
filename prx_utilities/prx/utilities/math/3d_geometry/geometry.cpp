/**
 * @file geometry.cpp
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


#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/math/3d_geometry/trimesh.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <clipper.hpp>

namespace prx
{
    namespace util
    {

        geometry_t::geometry_t() : type(PRX_NONE), scale(3), color(4)
        {
            radius_computed = false;
        }

        geometry_t::geometry_t(geometry_type intype, const std::vector<double>* params) : scale(3), color(4)
        {
            color[0] = 1.0;
            color[1] = 1.0;
            color[2] = 1.0;
            color[3] = 0.8;

            type = intype;

            set_params( params );
        }

        geometry_t::geometry_t(const geometry_t& geom)
        {
            type = geom.type;
            scale = geom.scale;
            color = geom.color;
            info = geom.info;
            trimesh = geom.trimesh;
            mesh_filename = geom.mesh_filename;
        }

        void geometry_t::init_geometry(std::string type, vector_t& dims)
        {
            if (type == "box")
            {
                if (dims.get_dim() != 3)
                    PRX_FATAL_S("Box must have three-dimensional dims attribute.")
                    set_box(dims[0], dims[1], dims[2]);
            }
            else if (type == "sphere")
            {
                if (dims.get_dim() != 1)
                    PRX_FATAL_S("Sphere must have one-dimensional dims attribute.")
                    set_sphere(dims[0]);
            }
            else if (type == "cone")
            {
                if (dims.get_dim() != 2)
                    PRX_FATAL_S("Sphere must have two-dimensional dims attribute.")
                    set_cone(dims[0], dims[1]);
            }
            else if (type == "cylinder")
            {
                if (dims.get_dim() != 2)
                    PRX_FATAL_S("Cylinder must have two-dimensional dims attribute.")
                    set_cylinder(dims[0], dims[1]);
            }
            else if (type == "bullet_cylinder")
            {
                if (dims.get_dim() != 2)
                    PRX_FATAL_S("Cylinder must have two-dimensional dims attribute.")
                    set_bullet_cylinder(dims[0], dims[1]);
            }
            else if (type == "open_cylinder")
            {
                if (dims.get_dim() != 2)
                    PRX_FATAL_S("Open cylinder must have two-dimensional dims attribute.")
                    set_open_cylinder(dims[0], dims[1]);
            }
            else if (type == "capsule")
            {
                if (dims.get_dim() != 2)
                    PRX_FATAL_S("Capsule must have two-dimensional dims attribute.")
                    set_capsule(dims[0], dims[1]);
            }
            else if (type == "linestrip")
            {
                if (dims.get_dim() <= 6)
                    PRX_FATAL_S("Linestrip needs at least 6-dimensional dims attribute for at least two points.");
                if(dims.get_dim()%3 != 0)
                    PRX_FATAL_S("Linestrip must have multiple 3-dimensional dims attribute.");

                std::vector<vector_t>* points = new std::vector<vector_t>();
                vector_t point;
                point.resize(3);
                for(unsigned int i=0; i<dims.get_dim()%3; ++i)
                {
                    point.set(dims[i],dims[i+1],dims[i+2]);
                    points->push_back(point);
                }
                set_linestrip(points);
            }
            else if (type == "point_cloud")
            {
                set_point_cloud();
            }
            else
            {
                PRX_FATAL_S("Unrecognized geometry type " << type.c_str());
            }
        }

        void geometry_t::set_scale(double x, double y, double z)
        {
            scale[0] = x;
            scale[1] = y;
            scale[2] = z;
        }

        void geometry_t::set_color(double r, double g, double b, double a)
        {
            color[0] = r;
            color[1] = g;
            color[2] = b;
            color[3] = a;
        }

        void geometry_t::set_sphere(double radius)
        {
            type = PRX_SPHERE;
            info.clear();
            info.push_back(radius);
            trimesh.create_sphere_trimesh(radius);
        }

        void geometry_t::set_box(double lx, double ly, double lz)
        {
            type = PRX_BOX;
            info.clear();
            info.push_back(lx);
            info.push_back(ly);
            info.push_back(lz);
            trimesh.create_box_trimesh(lx,ly,lz);
        }

        void geometry_t::set_cone(double radius, double height)
        {
            type = PRX_CONE;
            info.clear();
            info.push_back(radius);
            info.push_back(height);
            trimesh.create_cone_trimesh(radius,height);
        }

        void geometry_t::set_cylinder(double radius, double height)
        {
            type = PRX_CYLINDER;
            info.clear();
            info.push_back(radius);
            info.push_back(height);
            trimesh.create_cylinder_trimesh(radius,height);
        }

        void geometry_t::set_bullet_cylinder(double radius, double height)
        {
            type = PRX_BULLET_CYLINDER;
            info.clear();
            info.push_back(radius);
            info.push_back(height);
            trimesh.create_bullet_cylinder_trimesh(radius,height);
        }

        void geometry_t::set_open_cylinder( double radius, double height )
        {
            type = PRX_OPEN_CYLINDER;
            info.clear();
            info.push_back(radius);
            info.push_back(height);
            trimesh.create_open_cylinder_trimesh(radius,height);
        }

        void geometry_t::set_capsule( double radius, double height )
        {
            type = PRX_CAPSULE;
            info.clear();
            info.push_back(radius);
            info.push_back(height);
            trimesh.create_capsule_trimesh(radius,height);
        }

        void geometry_t::set_triangle( const vector_t* v1, const vector_t* v2, const vector_t* v3 )
        {
            type = PRX_TRIANGLE;
            trimesh.create_triangle_trimesh(v1,v2,v3);
        }

        void geometry_t::set_quad( const vector_t* v1, const vector_t* v2, const vector_t* v3, const vector_t* v4)
        {
            type = PRX_QUAD;
            trimesh.create_quad_trimesh(v1,v2,v3,v4);
        }

        void geometry_t::set_lines( const std::vector<std::pair<vector_t, vector_t> >* lines )
        {
            type = PRX_LINES;
            info.clear();
            typedef std::pair<vector_t, vector_t> line_t;
            foreach(const line_t line, *lines)
            {
                info.push_back(line.first[0]);
                info.push_back(line.first[1]);
                info.push_back(line.first[2]);
                info.push_back(line.second[0]);
                info.push_back(line.second[1]);
                info.push_back(line.second[2]);
            }
        }

        void geometry_t::set_linestrip( const std::vector<vector_t>* points )
        {
            type = PRX_LINESTRIP;
            info.clear();
            foreach(const vector_t point, *points)
            {
                info.push_back(point[0]);
                info.push_back(point[1]);
                info.push_back(point[2]);
            }
        }

        void geometry_t::set_mesh( const std::string& filename)
        {
            type = PRX_MESH;
            info.clear();
            mesh_filename = filename;
            trimesh.create_mesh_from_file(filename);
        }

        void geometry_t::set_heightmap( const std::string& filename)
        {
            type = PRX_HEIGHTMAP;
            info.clear();
            mesh_filename = filename;
            trimesh.create_heightmap_from_file(filename);
        }

        void geometry_t::set_point_cloud()
        {
            type = PRX_CLOUD;
            info.clear();
        }

        void geometry_t::set_polygon( const std::vector<double>& triangles )
        {
            type = PRX_POLYGON;
            info.clear();
            info = triangles;
            trimesh.create_from_vector( triangles );
        }

        geometry_type geometry_t::get_type() const
        {
            return type;
        }

        const std::vector<double>* geometry_t::get_info() const
        {
            return &info;
        }

        void geometry_t::get_sphere(double& radius) const
        {
            PRX_ASSERT(type == PRX_SPHERE);
            radius = info[0];
        }

        void geometry_t::get_box(double& lx, double& ly, double& lz) const
        {
            PRX_ASSERT(type == PRX_BOX);
            lx = info[0];
            ly = info[1];
            lz = info[2];

        }

        void geometry_t::get_cone(double& radius, double& height) const
        {
            PRX_ASSERT(type == PRX_CONE);
            radius = info[0];
            height = info[1];
        }

        void geometry_t::get_cylinder(double& radius, double& height) const
        {
            PRX_ASSERT(type == PRX_CYLINDER || type==PRX_BULLET_CYLINDER);
            radius = info[0];
            height = info[1];
        }

        void geometry_t::get_open_cylinder(double& radius, double& height) const
        {
            PRX_ASSERT(type == PRX_OPEN_CYLINDER);
            radius = info[0];
            height = info[1];
        }

        void geometry_t::get_capsule(double& radius, double& height) const
        {
            PRX_ASSERT(type == PRX_CAPSULE);
            radius = info[0];
            height = info[1];
        }

        void geometry_t::get_triangle(vector_t* v1, vector_t* v2, vector_t* v3) const
        {
            PRX_ASSERT(type == PRX_TRIANGLE);
            trimesh.get_vertex_at(0,v1);
            trimesh.get_vertex_at(1,v2);
            trimesh.get_vertex_at(2,v3);
        }

        void geometry_t::get_quad(vector_t* v1, vector_t* v2, vector_t* v3, vector_t* v4) const
        {
            PRX_ASSERT(type == PRX_QUAD);
            trimesh.get_vertex_at(0,v1);
            trimesh.get_vertex_at(1,v2);
            trimesh.get_vertex_at(2,v3);
            trimesh.get_vertex_at(3,v4);
        }

        const std::string& geometry_t::get_mesh() const
        {
            return mesh_filename;
        }
        const std::string& geometry_t::get_point_cloud_topic() const
        {
            return mesh_filename;
        }
        const std::string& geometry_t::get_heightmap() const
        {
            return mesh_filename;
        }

        vector_t geometry_t::get_color() const
        {
            return color;
        }
        vector_t geometry_t::get_scale() const
        {
            return scale;
        }

        void geometry_t::get_trimesh(trimesh_t* tri) const
        {
            tri->copy(&trimesh);
        }

        const trimesh_t* geometry_t::get_trimesh() const
        {
            return &trimesh;
        }

        void geometry_t::set_params( const std::vector<double>* params )
        {
            switch (type)
            {
                case PRX_SPHERE:
                    PRX_ASSERT(params->size() == 1);
                    set_sphere((*params)[0]);
                    break;

                case PRX_BOX:
                    PRX_ASSERT(params->size() == 3);
                    set_box((*params)[0], (*params)[1], (*params)[2]);
                    break;

                case PRX_CONE:
                    PRX_ASSERT(params->size() == 2);
                    set_cone((*params)[0], (*params)[1]);
                    break;

                case PRX_CYLINDER:
                    PRX_ASSERT(params->size() == 2);
                    set_cylinder((*params)[0], (*params)[1]);
                    break;
                    
                case PRX_BULLET_CYLINDER:
                    PRX_ASSERT(params->size() == 2);
                    set_bullet_cylinder((*params)[0], (*params)[1]);
                    break;

                case PRX_OPEN_CYLINDER:
                    PRX_ASSERT(params->size() == 2);
                    set_open_cylinder((*params)[0], (*params)[1]);
                    break;

                case PRX_CAPSULE:
                    PRX_ASSERT(params->size() == 2);
                    set_capsule((*params)[0], (*params)[1]);
                    break;

                case PRX_LINES:
                    // Must have at least two points.
                    PRX_ASSERT(params->size() >= 6);
                    // Must have three params per point.
                    PRX_ASSERT(params->size() % 3 == 0);
                    // Must have two points per line.
                    PRX_ASSERT(params->size() % 6 == 0);
                    this->type = type;
                    info = *params; // Copy the info directly
                    break;

                case PRX_LINESTRIP:
                    // Must have at least two points.
                    PRX_ASSERT(params->size() >= 6);
                    // Must have three params per point.
                    PRX_ASSERT(params->size() % 3 == 0);
                    this->type = type;
                    info = *params; // Copy the info directly
                    break;
                case PRX_CLOUD:
                    // Must have three params per point.
                    PRX_ASSERT(params->size() % 3 == 0);
                    info = *params; // Copy the info directly
                    break;
                    
                case PRX_POLYGON:
                    PRX_ASSERT(params->size() % 9 == 0);
                    set_polygon( *params );
                    break;

                default:
                    PRX_FATAL_S("Invalid geometry type: " << type);
                    break;
            }
        }

        double geometry_t::get_bounding_radius() const
        {
            if( radius_computed )
            {
                return bounding_radius;
            }
            switch (type)
            {
                case PRX_SPHERE:
                    bounding_radius = info[0];
                    break;

                case PRX_BOX:
                    bounding_radius = sqrt((info[0]*info[0]) + (info[1]*info[1]));
                    break;

                case PRX_CONE:
                    bounding_radius = info[0];
                    break;

                case PRX_CYLINDER:
                    bounding_radius = info[0];
                    break;

                case PRX_OPEN_CYLINDER:
                    bounding_radius = info[0];
                    break;

                case PRX_CAPSULE:
                    bounding_radius = info[0];
                    break;

                case PRX_LINESTRIP:
                    bounding_radius = 0;
                    break;

                case PRX_LINES:
                    bounding_radius = 0;
                    break;

                case PRX_POLYGON:
                    bounding_radius = 0;
                    break;

                default:
                    bounding_radius = 0;
                    // PRX_ERROR_S("Requested bounding radius for an invalid geometry type: " << type);
                    break;
            }
            radius_computed = true;
            return bounding_radius;
        }

        /** Calculates the minkowski sum for a circle in 2D
         *
         * TODO: Generalize to 3D
         *
         * @return Returns a new geometry containing the minkowski sum
         */
        geometry_t* geometry_t::get_minkowski(bool is_3D, double object_radius, double epsilon) const
        {
            //    double epsilon = 0;//object_radius*0.1;
            //    PRX_LOG_DEBUG ("Geometry type is %i", type);

            if (!is_3D)
            {
                switch (type)
                {
                        //            case PRX_LINES:
                        //                PRX_LOG_DEBUG("Calculating minkowski sum for a line");
                        //                return NULL;
                        //            case PRX_LINESTRIP:
                        //                PRX_LOG_DEBUG("Calculating minkowski sum for a linestrip");
                        //                return NULL;
                    case PRX_BOX:
                    {
                        //                PRX_LOG_DEBUG("2D: Calculating minkowski sum for a box (linestrip)");

                        geometry_t* minkowski_sum;
                        minkowski_sum = new geometry_t;
                        minkowski_sum->set_box(info[0] + 2*object_radius + epsilon, info[1] + 2*object_radius + epsilon, info[2] + 2*object_radius + epsilon);

                        return minkowski_sum;
                    }
                    case PRX_CYLINDER:
                    {
                        //                PRX_LOG_DEBUG("2D: Calculating minkowski sum for a cylinder (circle)");

                        geometry_t* minkowski_sum;
                        minkowski_sum = new geometry_t;
                        minkowski_sum->set_box(2*info[0] + 2*object_radius + epsilon, 2*info[0] + 2*object_radius + epsilon, info[1] + 2*object_radius + epsilon);

                        return minkowski_sum;
                    }
                    case PRX_SPHERE:
                    {
                        //                PRX_LOG_DEBUG("2D: Calculating minkowski sum for a cylinder (circle)");

                        geometry_t* minkowski_sum;
                        minkowski_sum = new geometry_t;
                        minkowski_sum->set_box(2*info[0] + 2*object_radius + epsilon, 2*info[0] + 2*object_radius + epsilon, info[1] + 2*object_radius + epsilon);

                        return minkowski_sum;
                    }
                    default:
                        // PRX_WARN_S("Unsupported minkowski sum type. Returning NULL");
                        return NULL;
                }
            }
            else
            {
                // PRX_WARN_S("3D Minkowski sums are not currently supported");
                return NULL;
            }

        }

        void geometry_t::init(const parameter_reader_t* const reader)
        {
            const std::string geom_type = reader->get_attribute("type");

            // PRX_DEBUG_COLOR("MAKING GEOMETRY " << geom_type.c_str(), PRX_TEXT_CYAN);
            if (geom_type == "box")
            {
                const vector_t dims = reader->get_attribute_as<vector_t>("dims");
                if (dims.get_dim() != 3)
                    PRX_FATAL_S("Box must have three-dimensional dims attribute.")
                    set_box(dims[0], dims[1], dims[2]);
            }
            else if (geom_type == "sphere")
                set_sphere(reader->get_attribute_as<double>("radius"));
            else if (geom_type == "cone")
                set_cone(reader->get_attribute_as<double>("radius"),
                         reader->get_attribute_as<double>("height"));
            else if (geom_type == "cylinder")
                set_cylinder(reader->get_attribute_as<double>("radius"),
                             reader->get_attribute_as<double>("height"));
            else if (geom_type == "bullet_cylinder")
                set_bullet_cylinder(reader->get_attribute_as<double>("radius"),
                             reader->get_attribute_as<double>("height"));
            else if (geom_type == "open_cylinder")
                set_open_cylinder(reader->get_attribute_as<double>("radius"),
                                  reader->get_attribute_as<double>("height"));
            else if (geom_type == "capsule")
                set_capsule(reader->get_attribute_as<double>("radius"),
                            reader->get_attribute_as<double>("height"));
            else if (geom_type == "mesh")
                set_mesh(reader->get_attribute_as<std::string>("filename"));
            else if (geom_type == "point_cloud")
                set_point_cloud();
            else if (geom_type == "heightmap")
                set_heightmap(reader->get_attribute_as<std::string>("filename"));
            else if (geom_type == "polygon")
            {
                const std::vector< double > triangles = reader->get_attribute_as< std::vector<double> >("triangles");
                set_polygon( triangles );
            }
            else
            {
                const std::string trace = reader->trace();
                PRX_FATAL_S("Unrecognized geometry type (" << geom_type.c_str() << ")  in " << trace.c_str());
            }

            if ( reader->has_attribute("scale"))
            {
                scale = reader->get_attribute_as<vector_t>("scale");
                if (scale.get_dim() != 3)
                    PRX_FATAL_S("Scale must have 3 elements at " << reader->trace() );
            }
            else
                set_scale(1.0,1.0,1.0);

            if( reader->has_attribute("material" ) )
            {
                std::string color_string = reader->get_attribute("material");
                if( color_string == "yellow" )
                {    color[0] = 1; color[1] = 1; color[2] = 0; color[3] = 1.0;   }
                else if( color_string == "blue" )
                {    color[0] = 0; color[1] = 0; color[2] = 1; color[3] = 1.0;   }
                else if( color_string == "dark_grey" )
                {    color[0] = 0.25; color[1] = 0.25; color[2] = 0.25; color[3] = 1.0;   }
                else if( color_string == "black" )
                {    color[0] = 0; color[1] = 0; color[2] = 0; color[3] = 1.0;   }
                else if( color_string == "green" )
                {    color[0] = 0; color[1] = 1; color[2] = 0; color[3] = 1.0;   }
                else if( color_string == "red" )
                {    color[0] = 1; color[1] = 0; color[2] = 0; color[3] = 1.0;   } 
                else if( color_string == "silver" )
                {    color[0] = 0.75; color[1] = 0.75; color[2] = 0.75; color[3] = 1.0; }
                else if( color_string == "cyan" )
                {    color[0] = 0.7; color[1] = 1; color[2] = 1; color[3] = 1.0;   }
                else if( color_string == "orange" )
                {    color[0] = 1; color[1] = 0.6; color[2] = 0.05; color[3] = 1.0;   }
                else if( color_string == "brown" )
                {    color[0] = 0.4; color[1] = 0.25; color[2] = 0.0; color[3] = 1.0;}
                else if( color_string == "glass" )
                {    color[0] = 0.5; color[1] = 0.5; color[2] = 0.55; color[3] = 0.18;}
                else
                {    color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.0;   }
            }
            else
            {    color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.0;   }

        }

        std::ostream& operator<<( std::ostream& out, const geometry_t& geom )
        {
            std::string name = "";
            switch( geom.type )
            {
            case PRX_NONE: 
                name = "PRX_NONE";
                break;
            case PRX_SPHERE: 
                name = "PRX_SPHERE";
                break;
            case PRX_BOX: 
                name = "PRX_BOX";
                break;
            case PRX_CONE: 
                name = "PRX_CONE";
                break;
            case PRX_CYLINDER: 
                name = "PRX_CYLINDER";
                break;
            case PRX_OPEN_CYLINDER: 
                name = "PRX_OPEN_CYLINDER";
                break;
            case PRX_CAPSULE: 
                name = "PRX_CAPSULE";
                break;
            case PRX_TRIANGLE: 
                name = "PRX_TRIANGLE";
                break;
            case PRX_QUAD: 
                name = "PRX_QUAD";
                break;
            case PRX_LINES: 
                name = "PRX_LINES";
                break;
            case PRX_LINESTRIP: 
                name = "PRX_LINESTRIP";
                break;
            case PRX_MESH: 
                name = "PRX_MESH";
                break;
            case PRX_CLOUD: 
                name = "PRX_CLOUD";
                break;
            case PRX_HEIGHTMAP: 
                name = "PRX_HEIGHTMAP";
                break;
            case PRX_POLYGON: 
                name = "PRX_POLYGON";
                break;
            default:
                name = "GARBAGE!!!";
                break;
            }
            
            out << "Geometry type is: " << name << "\n";
            
            if (name == "GARBAGE!!!")
                return out;
            
            for(size_t i=0; i<geom.info.size(); ++i)
                out << "info[" << i << "] : " << geom.info[i] << "\n";
            // out << geom.trimesh;
            return out;
        }

        void MyZFillCallback(ClipperLib::IntPoint& bot1, ClipperLib::IntPoint& top1, 
                            ClipperLib::IntPoint& bot2, ClipperLib::IntPoint& top2, ClipperLib::IntPoint& pt)
        {

            if (bot1.X == top1.X || bot1.Y == top1.Y)
                pt.Z = bot1.Z;
            else
            {
                // parametric equations
                // x=x0+t(x1-x0) 
                // y=y0+t(y1-y0)
                // z=z0+t(z1-z0)
                double t1 = (pt.X - bot1.X) / (top1.X - bot1.X);
                double t2 = (pt.Y - bot1.Y) / (top1.Y - bot1.Y);
                pt.Z = (long long)(bot1.Z + t1 * (top1.Z - bot1.Z));
            }

        }


        double geometry_t::intersect_and_compute_area(geometry_t* other_geom, const config_t& other_global_config, const config_t& this_global_config)
        {
            PRX_DEBUG_COLOR ("Other global: "  << other_global_config, PRX_TEXT_RED);
            PRX_DEBUG_COLOR("This global: " << this_global_config, PRX_TEXT_BLUE);

            config_t holding_config1(0,0,0,0,0,0,1), holding_config2(0,0,0,0,0,0,1), holding_config3(0,0,0,0,0,0,1);
            vector_t vertex1(0,0,0), vertex2(0,1,0), vertex3(1,1,0);
            vector_t vertex21(1,1,0), vertex22(1,0,0), vertex23(0,0,0);

            // Convert the other geometry from local coordinates to global coordinates
            ClipperLib::Paths full_subj;
            // Get other vertices
            std::vector<face_t> other_faces = other_geom->get_trimesh()->get_faces();

            //for(auto local_face : other_faces )
            {
                ClipperLib::Path temp_subj;

                // other_geom->get_trimesh()->get_vertex_at(local_face.index1, &vertex1);
                // other_geom->get_trimesh()->get_vertex_at(local_face.index2, &vertex2);
                // other_geom->get_trimesh()->get_vertex_at(local_face.index3, &vertex3);

                // holding_config1.set_orientation(0,0,0,1);
                // holding_config1.set_position(vertex1);
                // holding_config1.relative_to_global(other_global_config);

                // holding_config2.set_orientation(0,0,0,1);
                // holding_config2.set_position(vertex2);
                // holding_config2.relative_to_global(other_global_config);

                // holding_config3.set_orientation(0,0,0,1);
                // holding_config3.set_position(vertex3);
                // holding_config3.relative_to_global(other_global_config);

                // vector_t new_position1 = holding_config1.get_position();
                // vector_t new_position2 = holding_config2.get_position();
                // vector_t new_position3 = holding_config3.get_position();
                // PRX_WARN_S ("Pos 1 : " << new_position1);
                // PRX_WARN_S ("Pos 2 : " << new_position2);
                // PRX_WARN_S ("Pos 3 : " << new_position3);
                // PRX_WARN_S ("Temp subj : " << temp_subj);

                temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(vertex1[0]), ClipperLib::to_long(vertex1[1]), ClipperLib::to_long(vertex1[2]));
                temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(vertex2[0]), ClipperLib::to_long(vertex2[1]), ClipperLib::to_long(vertex2[2]));
                temp_subj << ClipperLib::IntPoint(ClipperLib::to_long(vertex3[0]), ClipperLib::to_long(vertex3[1]), ClipperLib::to_long(vertex3[2]));

                PRX_WARN_S ("Temp subj : " << temp_subj);
                full_subj.push_back(temp_subj);
                ClipperLib::Path temp_subj2;
                temp_subj2 << ClipperLib::IntPoint(ClipperLib::to_long(vertex21[0]), ClipperLib::to_long(vertex21[1]), ClipperLib::to_long(vertex21[2]));
                temp_subj2 << ClipperLib::IntPoint(ClipperLib::to_long(vertex22[0]), ClipperLib::to_long(vertex22[1]), ClipperLib::to_long(vertex22[2]));
                temp_subj2 << ClipperLib::IntPoint(ClipperLib::to_long(vertex23[0]), ClipperLib::to_long(vertex23[1]), ClipperLib::to_long(vertex23[2]));

                full_subj.push_back(temp_subj2);
            }

            PRX_DEBUG_COLOR ("Full subj size: " << full_subj.size(), PRX_TEXT_MAGENTA);

            // Convert the other geometry from local coordinates to global coordinates
            ClipperLib::Paths full_clip;
            // Get other vertices
            std::vector<face_t> this_faces = this->get_trimesh()->get_faces();
            // vector_t vertexx1(0.5,0.5,0), vertexx2(0.5,2,0), vertexx3(2,2,0);
            // vector_t vertexx21(2,2,0), vertexx22(2,0.5,0), vertexx23(0.5,0.5,0);
            vector_t vertexx1(0,0,1), vertexx2(0,1,1), vertexx3(1,1,-1);
            vector_t vertexx21(1,1,-1), vertexx22(1,0,-1), vertexx23(0,0,1);

            //for(auto local_face : this_faces )
            {
                ClipperLib::Path temp_clip;

                // this->get_trimesh()->get_vertex_at(local_face.index1, &vertex1);
                // this->get_trimesh()->get_vertex_at(local_face.index2, &vertex2);
                // this->get_trimesh()->get_vertex_at(local_face.index3, &vertex3);

                // holding_config1.set_orientation(0,0,0,1);
                // holding_config1.set_position(vertex1);
                // holding_config1.relative_to_global(this_global_config);

                // holding_config2.set_orientation(0,0,0,1);
                // holding_config2.set_position(vertex2);
                // holding_config2.relative_to_global(this_global_config);

                // holding_config3.set_orientation(0,0,0,1);
                // holding_config3.set_position(vertex3);
                // holding_config3.relative_to_global(this_global_config);

                // vector_t new_position1 = holding_config1.get_position();
                // vector_t new_position2 = holding_config2.get_position();
                // vector_t new_position3 = holding_config3.get_position();

                temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(vertexx1[0]), ClipperLib::to_long(vertexx1[1]), ClipperLib::to_long(vertexx1[2]));
                temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(vertexx2[0]), ClipperLib::to_long(vertexx2[1]), ClipperLib::to_long(vertexx2[2]));
                temp_clip << ClipperLib::IntPoint(ClipperLib::to_long(vertexx3[0]), ClipperLib::to_long(vertexx3[1]), ClipperLib::to_long(vertexx3[2]));
                PRX_WARN_S ("Temp temp_clip : " << temp_clip);
                full_clip.push_back(temp_clip);


                ClipperLib::Path temp_clip2;

                temp_clip2 << ClipperLib::IntPoint(ClipperLib::to_long(vertexx21[0]), ClipperLib::to_long(vertexx21[1]), ClipperLib::to_long(vertexx21[2]));
                temp_clip2 << ClipperLib::IntPoint(ClipperLib::to_long(vertexx22[0]), ClipperLib::to_long(vertexx22[1]), ClipperLib::to_long(vertexx22[2]));
                temp_clip2 << ClipperLib::IntPoint(ClipperLib::to_long(vertexx23[0]), ClipperLib::to_long(vertexx23[1]), ClipperLib::to_long(vertexx23[2]));

                full_clip.push_back(temp_clip2);

            }
            PRX_DEBUG_COLOR ("Full subj size: " << full_clip.size(), PRX_TEXT_MAGENTA);

            //ClipperLib::Paths simple_subj, simple_clip;
            // ClipperLib::SimplifyPolygons(full_subj, simple_subj, ClipperLib::pftNonZero);
            // ClipperLib::SimplifyPolygons(full_clip, simple_clip, ClipperLib::pftNonZero);

            // Compute clipping
            ClipperLib::Paths solution;
            ClipperLib::Clipper clpr;
            clpr.ZFillFunction(MyZFillCallback);
            clpr.AddPaths(full_clip, ClipperLib::ptSubject, true);
            clpr.AddPaths(full_subj, ClipperLib::ptClip, true);
            clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero );

            PRX_WARN_S ("Solution : " << solution[0]);

            if (solution.size() >= 1)
            {
                PRX_WARN_S ("Intersection causes multiple intersections?");
                for(auto i : solution)
                {
                    PRX_WARN_S ("Area :: " << (ClipperLib::Area(i) / (ClipperLib::scale*ClipperLib::scale)));
                }
                return (ClipperLib::Area(solution[0]) / (ClipperLib::scale*ClipperLib::scale));
            }
            else
            {
                return 0;
            }

        }


    



        bool geometry_t::contains_point(const vector_t& v, double buffer)
        {
            double squared_rad;
            vector_t n;
            switch(type)
            {
                case PRX_SPHERE://sphere
                    squared_rad = info[0]*info[0];
                    if(v.squared_norm() < squared_rad+buffer)
                        return true;
                    break;
                case PRX_BOX://box
                    if(v[0]>-.5*info[0]-buffer && v[0]<.5*info[0]+buffer &&
                       v[1]>-.5*info[1]-buffer && v[1]<.5*info[1]+buffer &&
                       v[2]>-.5*info[2]-buffer && v[2]<.5*info[2]+buffer)
                    {
                        return true;
                    }
                    break;
                case PRX_CONE:
                    PRX_WARN_S("Cannot check point containment in cone");
                    break;
                case PRX_CYLINDER://cylinder
                case PRX_OPEN_CYLINDER:
                    n = v;
                    n[2] = 0;
                    squared_rad = info[0]*info[0]+buffer;
                    if(v.squared_norm() < squared_rad &&
                       v[2]>-.5*info[1]-buffer && v[2]<.5*info[1]+buffer )
                        return true;
                    break;
                case PRX_CAPSULE:
                    PRX_WARN_S("Cannot check point containment in capsule");
                    break;
                case PRX_MESH:
                    // PRX_WARN_S(trimesh.min_x<<" "<<trimesh.max_x<<" "<<trimesh.min_y<<" "<<trimesh.max_y<<" "<<trimesh.min_z<<" "<<trimesh.max_z);
                    if(  trimesh.min_x - buffer < v[0] && trimesh.max_x + buffer > v[0] 
                        && trimesh.min_y - buffer < v[1] && trimesh.max_y + buffer > v[1] 
                        && trimesh.min_z - buffer < v[2] && trimesh.max_z + buffer > v[2])
                        return true;
                    break;
                default:
                    PRX_WARN_S("Cannot check point containment in geometry");
            }
            return false;
        }

    }
}
