/**
 * @file grasp_descriptor.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zacharias Psarakis, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "utilities/math/grasp_descriptor.hpp"
#include "prx/utilities/definitions/random.hpp"

namespace prx 
{ 
    using namespace util;
    
    namespace packages 
    {
        namespace manipulation
        {
            grasp_descriptor_t::grasp_descriptor_t()
            {
                
            }
            
            grasp_descriptor_t::~grasp_descriptor_t()
            {
                foreach( grasp_surface_t* surf, surfaces )
                {
                    delete surf;
                }
                foreach( grasp_volume_t* vol, volumes )
                {
                    delete vol;
                }
            }
            
            void grasp_descriptor_t::init( const parameter_reader_t* reader, const parameter_reader_t* template_reader )
            {
                surface_scaling_factor = parameters::get_attribute_as<double>("surface_scaling_factor", reader, template_reader, 0.85);

                //Read in some surfaces
                if( parameters::has_attribute("surfaces", reader, template_reader) )
                {
                    foreach( const parameter_reader_t* r, parameters::get_list("surfaces", reader, template_reader) )
                    {
                        //We have a new surface
                        surfaces.push_back( new grasp_surface_t() );
                        grasp_surface_t& surf = *(surfaces.back());
                        surf.index = surfaces.size()-1;
                        surf.mesh.mesh_file_loaded = true;
                        
                        //Get the triangle information
                        surf.triangles = r->get_attribute_as< std::vector< double > >("triangles");
                        unsigned nr_faces = surf.triangles.size()/9;
                        
                        //Alright now, I have to make a mini-mesh
                        trimesh_t temp_mesh;
                        temp_mesh.create_from_vector( surf.triangles );

                        std::vector<vector_t> normal_vectors;
                        std::vector<vector_t> centroids;

                        //Get the mesh information
                        temp_mesh.get_all_normal_vectors( normal_vectors, centroids, surf.face_areas );
                        vector_t surf_centroid(3);
                        foreach( vector_t& v, centroids )
                        {
                            surf_centroid += v;
                        }
                        surf_centroid /= nr_faces;

                        vector_t temp_normal;
                        for( unsigned i=0; i<normal_vectors.size(); ++i )
                        {
                            temp_normal += normal_vectors[i];
                        }
                        temp_normal /= (double)nr_faces;
                        temp_normal.normalize();
                        
                        //Alright, let's create the transformed points
                        for( unsigned i=0; i<surf.triangles.size(); i+=3 )
                        {
                            //Get the vertex
                            vector_t tmppt( surf.triangles[i], surf.triangles[i+1], surf.triangles[i+2] );
                            //move it relative to the object center
                            tmppt -= surf_centroid;
                            //scale it down
                            tmppt *= surface_scaling_factor;
                            //move it back to its original origin
                            tmppt += surf_centroid;
                            //And put it into the triangles vector
                            if (1 - std::fabs(temp_normal[0]) <= PRX_ZERO_CHECK)
                                surf.sample_triangles.push_back( surf.triangles[i] );
                            else
                                surf.sample_triangles.push_back( tmppt[0] );

                            if (1 - std::fabs(temp_normal[1]) <= PRX_ZERO_CHECK)
                                surf.sample_triangles.push_back( surf.triangles[i+1] );
                            else
                                surf.sample_triangles.push_back( tmppt[1] );

                            if (1 - std::fabs(temp_normal[2]) <= PRX_ZERO_CHECK)
                                surf.sample_triangles.push_back( surf.triangles[i+2] );
                            else
                                surf.sample_triangles.push_back( tmppt[2] );
                        }
                        
                        //Create the sampling mesh
                        surf.mesh.create_from_vector( surf.sample_triangles );
                        
                        normal_vectors.clear();
                        centroids.clear();
                        
                        //Get normal and surface area information
                        surf.mesh.get_all_normal_vectors( normal_vectors, centroids, surf.face_areas );
                        
                        //TODO: Need to make sure to handle bad normals
                        
                        //Then, we need to compute the normal for the actual surface
                        surf.normal.zero();
                        for( unsigned i=0; i<normal_vectors.size(); ++i )
                        {
                            surf.normal += normal_vectors[i];
                            surf.total_area += surf.face_areas[i];
                        }
                        surf.normal /= (double)nr_faces;
                        surf.normal.normalize();
                        

                        // PRX_PRINT("\n\n\nInit of the grasp descriptor.\n\n\n", PRX_TEXT_RED);
                        //Now, we also have to figure out what end-effectors this is good for
                        std::vector< std::string > valid_end_effectors = r->get_attribute_as< std::vector< std::string > >("valid_end_effectors");//parameters::get_attribute_as< std::vector< std::string > >( "valid_end_effectors", reader, template_reader );
                        foreach( std::string name, valid_end_effectors )
                        {
                            // PRX_PRINT("Loading grasp descriptors for the end effector: "<<name, PRX_TEXT_RED);
                            surface_map[name].push_back( surfaces.back() );
                        }
                        
                        surf.sample_mesh = surf.mesh;
                        surf.mesh = temp_mesh;
                    }
                }
                //Read in the volumes
                if( parameters::has_attribute("volumes", reader, template_reader) )
                {
                    foreach( const parameter_reader_t* r, parameters::get_list("volumes", reader, template_reader) )
                    {
                        //We have a new volume
                        volumes.push_back( new grasp_volume_t() );
                        grasp_volume_t& vol = *(volumes.back());
                        vol.index = volumes.size()-1;
                        
                        //get some of the basic parameters
                        vol.height = r->get_attribute_as< double >("height");
                        vol.radius = r->get_attribute_as< double >("radius");
                        
                        vol.relative_config.set_position( r->get_attribute_as<vector_t>("position") );
                        vol.relative_config.set_orientation( r->get_attribute_as<quaternion_t>("orientation") );

                        vol.volume = PRX_PI * vol.radius * vol.height;

                        //Now, we also have to figure out what end-effectors this is good for
                        std::vector< std::string > valid_end_effectors = parameters::get_attribute_as< std::vector< std::string > >( "valid_end_effectors", reader, template_reader );
                        foreach( std::string name, valid_end_effectors )
                        {
                           volume_map[name].push_back( volumes.back() );
                        }
                    }
                }
                    
            }
            
            void grasp_descriptor_t::sample_volume( unsigned index, vector_t& sampled_point ) const
            {
                //Alright, get the volume we are sampling from
                grasp_volume_t* vp = volumes[index];
                
                //Naive approach: Just need to sample a height, angle, and radius
                double h = uniform_random() * vp->height;
                double r = uniform_random() * vp->radius;
                double t = 2 * PRX_PI * uniform_random();
                
                //And transform that into an (x,y,z) position
                vector_t pos( r * cos(t), r * sin(t) , h );
                
                sampled_point = vp->relative_config.get_orientation().qv_rotation( pos );
                sampled_point += vp->relative_config.get_position();
            }
            
            void grasp_descriptor_t::sample_surface( unsigned index, vector_t& sampled_point ) const
            {
                PRX_ASSERT( index < surfaces.size() );
                
                //Get the surface we are interested in
                grasp_surface_t* sp = surfaces[index];
                
                //Alright, now, let's pick a triangle face
                double r = uniform_random() * sp->total_area;
                
                int i = -1;
                while( r > 0 )
                {
                    r -= sp->face_areas[i+1];
                    ++i;
                }
                
                //Get the face in question
                face_t f;
                sp->sample_mesh.get_face_at( i, &f );
                
                //Get the point vectors
                const vector_t& a = *(sp->sample_mesh.get_vertex_at( f.get_index1() ));
                const vector_t& b = *(sp->sample_mesh.get_vertex_at( f.get_index2() ));
                const vector_t& c = *(sp->sample_mesh.get_vertex_at( f.get_index3() ));

                //Get our sample point
                double x = uniform_random();
                double y = uniform_random();
                if( x + y > 1 )
                {
                    x = 1-x;
                    y = 1-y;
                }
                
                sampled_point = a + (b-a)*x + (c-a)*y;

                // PRX_PRINT ("Sampled point: " << sampled_point, PRX_TEXT_MAGENTA);
            }
            
            const std::vector< grasp_volume_t* >& grasp_descriptor_t::get_volumes( std::string end_effector ) const
            {
                if( end_effector == "" )
                {
                    return volumes;
                }
                else
                {
                    if (volume_map.find(end_effector) == volume_map.end())
                    {
                        PRX_WARN_S ("Volumes not generated for end effector: " << end_effector);
                        return empty_volumes;
                    }
                    return volume_map[end_effector];
                }
            }
            
            const std::vector< grasp_surface_t* >& grasp_descriptor_t::get_surfaces( std::string end_effector ) const
            {
                if( end_effector == "" )
                {
                    return surfaces;
                }
                else
                {
                    if (surface_map.find(end_effector) == surface_map.end())
                    {
                        PRX_WARN_S ("Surface map not generated for end effector: " << end_effector);
                        return empty_surfaces;
                    }
                    return surface_map[end_effector];
                }
            }
            
        }
    }
}
