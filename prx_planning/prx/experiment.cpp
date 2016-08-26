/**
 * @file main.cpp 
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

#include <stdlib.h>

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/planning/applications/planning_application.hpp"


 #include "prx/utilities/math/3d_geometry/geometry.hpp"

#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/simulation/collision_checking/fcl_collision_checker.hpp"
 #include <boost/progress.hpp>

#include <boost/function.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <ros/callback_queue.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

 
#ifdef __APPLE__
#include <PQP.h>
#else
#include <PQP.h>
#endif


#define MAX_CALLS 100000
typedef fcl::CollisionGeometry FCL_Model;

using namespace prx;
using namespace boost::program_options;
using namespace util;
using namespace util::parameters;
using namespace sim;
using namespace plan;

void rotation_matrix_from_quat(quaternion_t q, PQP_REAL (&rotation_matrix)[3][3]);

int main( int ac, char* av[] )

{   
	double PQP_total_time = 0;
	double FCL_total_time = 0;
	boost::timer timer;

	srand (time(NULL));
 	std::string node_name;
    std::string filename="";
    if (ac == 1)
    {
        node_name = "planning";
    }
    else
    {
        node_name = av[1];
        // This skips the first 8 characters, which by default in ros is: __name:=
        std::string test = node_name.substr(0,8);
        if (test == "__name:=")
        {
            node_name = node_name.substr(8);
        }
    }

    ros::init(ac, av, node_name);
    PRX_PRINT("Starting Planning node with name: " << node_name, PRX_TEXT_BLUE);
    ros::NodeHandle main_node_handle;
    if(ros::param::has("yaml_input"))
    {
        ros::param::get("yaml_input",filename);
    }

	global_storage = new parameter_storage_t( "" );

	parameter_reader_t reader(node_name, global_storage);

	planning_application_t* app  = parameters::create_from_loader<planning_application_t>("prx_planning",&reader,"",NULL,"");

	geometry_t* sphere = new geometry_t();
	geometry_t* box = new geometry_t();

	vector_t sphere_size(1);
	sphere_size.set_at(0,2.0);
	vector_t box_size(2,2,2);
	PRX_PRINT(sphere_size.get_dim(),PRX_TEXT_GREEN);
	sphere->init_geometry("sphere",sphere_size);
	box->init_geometry("box",box_size);

	// //Create Model
	vector_t vertex; // tmp variable
    face_t face; // tmp variable

    trimesh_t trimesh;
    sphere->get_trimesh(&trimesh);
    // PRX_DEBUG_COLOR("create the model for : " << name, PRX_TEXT_GREEN);
    //    geometry.print();
    PQP_Model* PQP_sphere_model = new PQP_Model();

    PQP_sphere_model->BeginModel();
    double radius = 0;
    for( int i = 0; i < trimesh.get_faces_size(); ++i )
    {
        trimesh.get_face_at(i, &face);
        PQP_REAL first[3], second[3], third[3];

        trimesh.get_vertex_at(face.get_index1(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        first[0] = vertex[0];
        first[1] = vertex[1];
        first[2] = vertex[2];

        trimesh.get_vertex_at(face.get_index2(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        second[0] = vertex[0];
        second[1] = vertex[1];
        second[2] = vertex[2];

        trimesh.get_vertex_at(face.get_index3(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        third[0] = vertex[0];
        third[1] = vertex[1];
        third[2] = vertex[2];

        PQP_sphere_model->AddTri(first, second, third, i);
    }
    //models_map[name]->broadphase_radius = radius;

    PQP_sphere_model->EndModel();

    PQP_Model* PQP_box_model = new PQP_Model();

    PQP_box_model->BeginModel();

    radius = 0;
    for( int i = 0; i < trimesh.get_faces_size(); ++i )
    {
        trimesh.get_face_at(i, &face);
        PQP_REAL first[3], second[3], third[3];

        trimesh.get_vertex_at(face.get_index1(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        first[0] = vertex[0];
        first[1] = vertex[1];
        first[2] = vertex[2];

        trimesh.get_vertex_at(face.get_index2(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        second[0] = vertex[0];
        second[1] = vertex[1];
        second[2] = vertex[2];

        trimesh.get_vertex_at(face.get_index3(), &vertex);
        radius = PRX_MAXIMUM(radius, sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]));
        third[0] = vertex[0];
        third[1] = vertex[1];
        third[2] = vertex[2];

        PQP_box_model->AddTri(first, second, third, i);
    }
    //models_map[name]->broadphase_radius = radius;

    PQP_box_model->EndModel();

    FCL_Model* FCL_sphere_model = new fcl::Sphere(2.0);
    FCL_Model* FCL_box_model = new fcl::Box(2,2,2);


    int i=0;
    int gain=5;
    
    while(i<MAX_CALLS){
        //GET SOME RANDOM VALUES
        double qxs=(double)rand() / RAND_MAX , qys=(double)rand() / RAND_MAX , qzs=(double)rand() / RAND_MAX , qws=(double)rand() / RAND_MAX , qxb=(double)rand() / RAND_MAX , qyb=(double)rand() / RAND_MAX , qzb=(double)rand() / RAND_MAX , qwb=(double)rand() / RAND_MAX , txs=gain*((double)rand() / RAND_MAX), tys=gain*((double)rand() / RAND_MAX), tzs=gain*((double)rand() / RAND_MAX), txb=gain*((double)rand() / RAND_MAX), tyb=gain*((double)rand() / RAND_MAX), tzb=gain*((double)rand() / RAND_MAX);
		//DEFINE SOME MATRICES
	    PQP_REAL sphere_translation_matrix[3]={txs,tys,tzs};
		quaternion_t sphere_quat(qxs,qys,qzs,qws);
		sphere_quat.normalize();
		PQP_REAL sphere_rotation_matrix[3][3];
	    rotation_matrix_from_quat(sphere_quat, sphere_rotation_matrix);
	    // for (int i = 0; i<3; i++){
	    // 	for(int j=0; j<3; j++){
	    // 		std::cout<<sphere_rotation_matrix[i][j]<<std::endl;
	    // 	}
	    // }
	    PQP_REAL box_translation_matrix[3]={txb,tyb,tzb};
		quaternion_t box_quat(qxb,qyb,qzb,qwb);
		box_quat.normalize();
		PQP_REAL box_rotation_matrix[3][3];
	    rotation_matrix_from_quat(box_quat, box_rotation_matrix);
	    //call Collision Checker

	    PQP_CollideResult collision_result;
	    timer.restart();
                
	    PQP_Collide(&collision_result,
	                sphere_rotation_matrix, sphere_translation_matrix, PQP_sphere_model,
	                box_rotation_matrix, box_translation_matrix, PQP_box_model,
	                PQP_FIRST_CONTACT);

	    PQP_total_time += timer.elapsed();
	    //return collision_result.Colliding();
	    PRX_PRINT("The PQP Collision result is: "<<collision_result.Colliding(),PRX_TEXT_GREEN);

        fcl::Quaternion3f quat_s, quat_b;
        fcl::Vec3f translation_s, translation_b;
        fcl::Transform3f transform_s, transform_b;
        translation_s.setValue(txs, tys, tzs);
        translation_b.setValue(txb, tyb, tzb);
        quat_s.getW() = qxs;
        quat_s.getX() = qys;
        quat_s.getY() = qzs;
        quat_s.getZ() = qws;
        quat_b.getW() = qxb;
        quat_b.getX() = qyb;
        quat_b.getY() = qzb;
        quat_b.getZ() = qwb;
        transform_s.setTransform(quat_s, translation_s);
        transform_b.setTransform(quat_b, translation_b);
        fcl::CollisionRequest request;
        fcl::CollisionResult result;    
        bool res;
        timer.restart();
        if( fcl::collide(FCL_sphere_model, transform_s, FCL_box_model, translation_b, request, result) > 0 )
        {
            res= true;
        }
        else
        {
            res= false;
        }
        FCL_total_time+=timer.elapsed();
        PRX_PRINT("The FCL Collision result is: "<<res,PRX_TEXT_CYAN);

	    i++;
    }
    PRX_PRINT("PQP Total Time: "<<PQP_total_time,PRX_TEXT_CYAN);
    PRX_PRINT("FCL Total Time: "<<FCL_total_time,PRX_TEXT_CYAN);


//     global_reader = NULL;
//     std::string node_name;
//     if (ac == 1)
//     {
//         node_name = "planning";
//     }
//     else
//     {
//         node_name = av[1];
//         // This skips the first 8 characters, which by default in ros is: __name:=
//         std::string test = node_name.substr(0,8);
//         if (test == "__name:=")
//         {
//             node_name = node_name.substr(8);
//         }
//     }
    // ros::init(ac, av, node_name);
//     PRX_INFO_S ("Initializing prx_planning node with name: " << node_name);
//     ros::NodeHandle main_node_handle;

//     // Wait for parameter setting scripts to finish.
//     while (ros::param::has("prx/parameter_mutex")) {}
    
    
//     std::string init_name = "prx/initialization/";
//     init_name+=node_name;
//     ros::param::set(init_name,true);    
//     if (ros::param::has("prx/initialization/order"))
//     {
//         parameter_reader_t init_reader("prx/initialization/");
//         //make sure nothing comes before this
//         std::vector<std::string> node_list = init_reader.get_attribute_as<std::vector<std::string> >("order");
//         unsigned pos=node_list.size();
//         for(unsigned i=0;i<pos;i++)
//         {
//             if(node_list[i]==node_name)
//                 pos = i;
//         }
//         for(unsigned i=0;i<pos;i++)
//         {
//             while (ros::param::has("prx/initialization/"+node_list[i])) 
//             {
//                 sleep(1);
//             }
//         }
//     }
//     else
//     {
//         //assume waiting on a node called simulation        
//         while (ros::param::has("prx/initialization/simulation")) 
//         {
//             sleep(1);
//         }
//     }
    
    // parameter_reader_t* reader;
//     int random_seed;
//     string_hash s;
//     try
//     {
//         random_seed = s(node_name);
//         init_random(random_seed);
//         PRX_INFO_S ("Found a random seed in input: "<< random_seed);

        
// //         if(reader.has_attribute("random_seed"))
// //         {
// //             random_seed = reader.get_attribute_as<int>("random_seed");
// //             init_random(random_seed);
// //         }
// //         else
// //         {
            
// // //            std::srand(time(NULL));
// //             pid_t pid = getpid();
// //             unsigned int id = pid;
// //             random_seed = rand_r(&id);
// //             PRX_WARN_S ("No random seed found. Setting a truly random seed: " << random_seed);

// //             init_random(random_seed);
// //         }
//     }
//     catch(...)
//     {
//         PRX_ERROR_S ("Exception caught trying to read random seed for prx_planning.");
//     }
    
//     if (reader.has_attribute("print_random_seed"))
//     {
       
//         if (reader.get_attribute_as<bool>("print_random_seed") == true)
//         {
//             std::ofstream fout;
//             std::string filename = ros::this_node::getName() + "_random_seed.txt";
//             PRX_PRINT ("Saving random seed: " << random_seed << " to file: " << filename, PRX_TEXT_CYAN);

//             fout.open(filename.substr(1).c_str(), std::fstream::app);
//             fout << random_seed << std::endl;
//             fout.close();
//         }
//     }
    
//     PRX_DEBUG_S(" Planning application type to initialize: " << reader.get_attribute_as<std::string>("type"));

    // planning_application_t* app;

//     // std::string type_name = reader.get_attribute_as<std::string>("type");
//     // pluginlib::ClassLoader<planning_application_t>& loader = planning_application_t::get_loader(); 
//     // planning_application_t* app = loader.createUnmanagedInstance("prx_planning/" + type_name);
//     PRX_ASSERT(app != NULL);
//     app->init(&reader);
    
//     ros::param::del(init_name); 
//     // Run until the planner or ROS wants to stop.
//     ros::getGlobalCallbackQueue()->callAvailable();
    
//     app->execute();
//     bool persistent = true;
    
//     if(reader.has_attribute("persistent"))
//     {
//         PRX_ASSERT(false);
//         persistent = reader.get_attribute_as<bool>("persistent");
//         if(persistent)
//             PRX_INFO_S("prx_planning node "<<node_name<<" will continue to run...");
//     }
//     // This timer calls the frame function from task planner every 1/10th of a second

//     if (persistent)
//     {
//         ros::MultiThreadedSpinner spinner(2);
//         spinner.spin();
//     }
// //    while( persistent && ros::ok() )
// //    {
// //        ros::getGlobalCallbackQueue()->callAvailable();
// ////        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
// ////        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(simulation::simulation_step));
// //    }

    return 0;
}


void rotation_matrix_from_quat(quaternion_t q, PQP_REAL (&rotation_matrix)[3][3])
{
	PQP_REAL qx=q.get_x();
	PQP_REAL qy=q.get_y();
	PQP_REAL qz=q.get_z();
	PQP_REAL qw=q.get_w();
    rotation_matrix[0][0] = 1 - 2 * qy * qy - 2 * qz*qz;
    rotation_matrix[0][1] = 2 * qx * qy - 2 * qz*qw;
    rotation_matrix[0][2] = 2 * qx * qz + 2 * qy*qw;
    rotation_matrix[1][0] = 2 * qx * qy + 2 * qz*qw;
    rotation_matrix[1][1] = 1 - 2 * qx * qx - 2 * qz*qz;
    rotation_matrix[1][2] = 2 * qy * qz - 2 * qx*qw;
    rotation_matrix[2][0] = 2 * qx * qz - 2 * qy*qw;
    rotation_matrix[2][1] = 2 * qy * qz + 2 * qx*qw;
    rotation_matrix[2][2] = 1 - 2 * qx * qx - 2 * qy*qy;
}

