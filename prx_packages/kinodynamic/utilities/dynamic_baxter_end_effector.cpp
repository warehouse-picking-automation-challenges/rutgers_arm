/**
 * @file dynamic_baxter_end_effector.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "utilities/dynamic_baxter_end_effector.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS( prx::packages::mapping_functions::dynamic_baxter_end_effector_t, prx::util::mapping_function_t)

namespace prx
{
#ifdef RBDL_FOUND
    using namespace RigidBodyDynamics::Addons;
    using namespace RigidBodyDynamics::Math;
    using namespace RigidBodyDynamics;
#endif
    using namespace util;
    namespace packages
    {        
        namespace mapping_functions
        {
            void dynamic_baxter_end_effector_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                mapping_name = parameters::get_attribute_as<std::string>("type",reader,template_reader);
                
    #ifdef RBDL_FOUND
                std::string filename = parameters::get_attribute_as<std::string>("urdf_file",reader,template_reader);

                URDFReadFromFile(filename.c_str(), &model, false);
                model.gravity = Vector3d (0., 0., 0);
                Q = VectorNd::Zero (model.dof_count);
                QDot = VectorNd::Zero (model.dof_count);
                QDDot = VectorNd::Zero (model.dof_count);
                id = model.GetBodyId("left_paddle");

                for(unsigned i=0;i<range;i++)
                {
                    memory.push_back(new double);
                }
                if(range!=0)
                {
                    subspace = new space_t(output_space_name,memory);
                    if(template_reader!=NULL)
                        subspace->init(template_reader);
                    subspace->init(reader);
                }
    #endif           
            }
            void dynamic_baxter_end_effector_t::embed() const
            {
    #ifdef RBDL_FOUND
                unsigned start_index = preimage_interval.first;
                unsigned image_start_index = image_interval.first;
                for(unsigned i=0;i<model.dof_count;i++)
                {
                    Q[i] = get_preimage_index(start_index + i);
                }
                for(unsigned i=0;i<model.dof_count;i++)
                {
                    QDot[i] = get_preimage_index(start_index + model.dof_count + i);
                }
                Vector3d zero = VectorNd::Zero(3);
                Vector3d pos = CalcBodyToBaseCoordinates(model,Q,id,VectorNd::Zero(3));
                Vector3d vel = CalcPointVelocity(model,Q,QDot,id,VectorNd::Zero(3),false);
                Vector3d offset = VectorNd::Zero(3);
                offset[0] = 1;
                Vector3d pos2 = CalcBodyToBaseCoordinates(model,Q,id,offset,false);
                pos2 = pos2 - pos;
                zero = offset - zero;
                double closeness = pos2.adjoint()*zero;
                get_image_index(image_start_index) = pos[0];
                get_image_index(image_start_index+1) = pos[1];
                get_image_index(image_start_index+2) = pos[2];
                get_image_index(image_start_index+3) = vel[0];
                get_image_index(image_start_index+4) = vel[1];
                get_image_index(image_start_index+5) = vel[2];
                get_image_index(image_start_index+6) = closeness;
    #endif    
            }

            void dynamic_baxter_end_effector_t::invert() const
            {
    #ifdef RBDL_FOUND
                // unsigned start_index = preimage_interval.first;
                // unsigned image_start_index = image_interval.first;
                // std::vector< unsigned int > ids;
                // ids.push_back(id);
                // std::vector< Vector3d > body_points;
                // std::vector< Vector3d > target_pos;
                // VectorNd result = VectorNd::Zero(model.dof_count);
                // body_points.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos[0][0] = get_image_index(image_start_index);
                // target_pos[0][1] = get_image_index(image_start_index+1);
                // target_pos[0][2] = get_image_index(image_start_index+2);
                // // PRX_INFO_S("--- "<<target_pos[0].transpose());

                // InverseKinematics(model,Q,ids,body_points,target_pos,result);

                // // PRX_INFO_S("--- "<<result.transpose());
                // for(unsigned i=0;i<model.dof_count;i++)
                // {
                //     get_preimage_index(start_index + i) = result[i];
                // }
    #endif    
            }

            void dynamic_baxter_end_effector_t::init_spaces()
            {
                //subspace and preimage should be set
                if( subspace == NULL || preimage_space == NULL )
                    PRX_FATAL_S("dynamic_baxter_end_effector_t doesn't have subspace or preimage_space");
                
                // These variables are not used.
    //            unsigned start_index = preimage_interval.first;
    //            //make the image space have the correct bounds information
    //            double low, high;
                subspace->get_bounds()[0]->set_bounds(-.6,1);
                subspace->get_bounds()[1]->set_bounds(-.4,1);
                subspace->get_bounds()[2]->set_bounds(-1,1);
                subspace->get_bounds()[3]->set_bounds(-2,2);
                subspace->get_bounds()[4]->set_bounds(-2,2);
                subspace->get_bounds()[5]->set_bounds(-2,2);
                subspace->get_bounds()[5]->set_bounds(-1,1);

            }
        }
    }
}