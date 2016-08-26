/**
 * @file baxter_end_effector_cost.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/simulation/cost_functions/baxter_end_effector_cost.hpp"
#include "prx/simulation/systems/system.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::baxter_end_effector_cost_t, prx::sim::cost_function_t)

namespace prx
{
#ifdef RBDL_FOUND
    using namespace RigidBodyDynamics::Addons;
    using namespace RigidBodyDynamics::Math;
    using namespace RigidBodyDynamics;
#endif
	using namespace util;
    namespace sim
    {       
        baxter_end_effector_cost_t::baxter_end_effector_cost_t()
        {
#ifdef RBDL_FOUND
            char* w = std::getenv("PRACSYS_PATH");
            std::string dir(w);
            std::string filename = dir+"/prx_packages/baxter/input/urdf/baxter.urdf";
            URDFReadFromFile(filename.c_str(), &model, false);
            model.gravity = Vector3d (0., 0., 0);
            Q = VectorNd::Zero (model.dof_count);
            QDot = VectorNd::Zero (model.dof_count);
            QDDot = VectorNd::Zero (model.dof_count);
            id = model.GetBodyId("left_paddle");
            update = true;
#endif 
        }
        baxter_end_effector_cost_t::~baxter_end_effector_cost_t()
        {
        }

        double baxter_end_effector_cost_t::state_cost(const space_point_t* s)
        {

#ifdef RBDL_FOUND
            for(unsigned i=0;i<model.dof_count;i++)
            {
                Q[i] = s->memory[i];
            }
            for(unsigned i=0;i<model.dof_count;i++)
            {
                QDot[i] = s->memory[i+model.dof_count];
            }
            Vector3d zero = VectorNd::Zero(3);
            Vector3d vel = CalcPointVelocity(model,Q,QDot,id,zero,update);
            Vector3d vel2 = vel;
            double velSquared = vel.adjoint()*vel2;
            // update = false;
            return velSquared;
#endif 
        }

        double baxter_end_effector_cost_t::trajectory_cost(const trajectory_t& t)
        {
            double total = 0;
            update = true;
            foreach(state_t* state, t)
            {
                total+=state_cost(state)*simulation::simulation_step;
            }
            return total;
        }

        double baxter_end_effector_cost_t::heuristic_cost(const util::space_point_t* s,const util::space_point_t* t)
        {
            return .1*dist(s,t)/5;
        }


    }
}
