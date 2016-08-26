/**
 * @file parallel_propagation_application.hpp 
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


#ifndef PRX_PARALLEL_PROPAGATION_APPLICATION_HPP
#define	PRX_PARALLEL_PROPAGATION_APPLICATION_HPP


#include "prx/planning/applications/planning_application.hpp"

#include <actionlib/server/simple_action_server.h>
#include <prx_planning/parallel_propAction.h>

namespace prx
{
    namespace plan
    {
        class parallel_propagation_application_t : public planning_application_t
        {

          public:
            parallel_propagation_application_t();
            
            virtual ~parallel_propagation_application_t(){}

            /**
             * @copydoc planning_application_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader);

            /**
             * @copydoc planning_application_t::execute()
             *
             * This execute function will actually terminate once the planning is
             * completed.
             */
            virtual void execute();

            void executeCB(const prx_planning::parallel_propGoalConstPtr &goal);

            ros::NodeHandle nh_;

            actionlib::SimpleActionServer<prx_planning::parallel_propAction> as_;

            prx_planning::parallel_propResult result_;


        };

    }
}

#endif

