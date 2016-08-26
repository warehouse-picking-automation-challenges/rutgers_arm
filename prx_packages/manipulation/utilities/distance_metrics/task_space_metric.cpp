/**
 * @file distance_metric.cpp
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

#include "utilities/distance_metrics/task_space_metric.hpp"
#include "utilities/distance_metrics/task_space_node.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::util::task_space_metric_t, prx::util::distance_metric_t)

namespace prx
{
    namespace util
    {

        task_space_metric_t::task_space_metric_t( )
        {
            cspace_metric = NULL;
            workspace_metric = NULL;

            std::vector<double*> state_dim = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);
            end_effector_space = new space_t("SE3", state_dim);

            use_workspace_metric = false;
        }

        task_space_metric_t::~task_space_metric_t( )
        {
            delete end_effector_space;
            delete cspace_metric;
            delete workspace_metric;
        }

        void task_space_metric_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            workspace_metric = parameters::initialize_from_loader<distance_metric_t>("prx_utilities", reader, "workspace_metric", template_reader, "workspace_metric");
            cspace_metric = parameters::initialize_from_loader<distance_metric_t>("prx_utilities", reader, "cspace_metric", template_reader, "cspace_metric");
        }

        void task_space_metric_t::link_space( const space_t* inspace )
        {
            workspace_metric->link_space(end_effector_space);
            cspace_metric->link_space(inspace);
            distance_function = cspace_metric->distance_function;
        }

        unsigned task_space_metric_t::add_point( const abstract_node_t* embed )
        {
            const task_space_node_t* check = dynamic_cast<const task_space_node_t*>(embed);

            PRX_ASSERT(check != NULL);

            task_space_node_t* ee_node = new task_space_node_t(check);
            task_space_node_t* arm_node = new task_space_node_t(check);
            ee_node->point = check->end_effector_point;
            ee_node->other_metric_node = arm_node;
            arm_node->point = check->arm_point;
            arm_node->other_metric_node = ee_node;
            workspace_metric->add_point(ee_node);
            cspace_metric->add_point(arm_node);


        }

        unsigned task_space_metric_t::add_points( const std::vector< const abstract_node_t* >& embeds )
        {
            foreach(const abstract_node_t* node, embeds)
            {
                add_point(node);
            }
        }
        
        void task_space_metric_t::remove_point( const abstract_node_t* node )
        {

            const abstract_node_t* arm_node = cspace_metric->single_query(node);

            if (arm_node != NULL)
            {
                const task_space_node_t* arm_check = dynamic_cast<const task_space_node_t*>(arm_node);
                const task_space_node_t* ee_check = arm_check->other_metric_node;

                PRX_ASSERT(arm_check != NULL);
                PRX_ASSERT(ee_check != NULL);

                cspace_metric->remove_point(arm_check);
                delete arm_check;

                workspace_metric->remove_point(ee_check);
                delete ee_check;
            }
        }

        const std::vector< const abstract_node_t* > task_space_metric_t::multi_query( const space_point_t* query_point, unsigned ink ) const
        {
            if (use_workspace_metric)
            {
                return workspace_metric->multi_query(query_point, ink);
            }
            else
            {
                return cspace_metric->multi_query(query_point, ink);
            }

        }

        const std::vector< const abstract_node_t* > task_space_metric_t::radius_query( const space_point_t* query_point, double rad )const
        {
            if (use_workspace_metric)
            {
                return workspace_metric->radius_query(query_point, rad);
            }
            else
            {
                return cspace_metric->radius_query(query_point, rad);
            }

        }

        const abstract_node_t* task_space_metric_t::single_query( const space_point_t* query_point, double* dist ) const
        {
            if (use_workspace_metric)
            {
                return workspace_metric->single_query(query_point, dist);
            }
            else
            {
                return cspace_metric->single_query(query_point, dist);
            }

        }

        const std::vector< const abstract_node_t* > task_space_metric_t::multi_query( const abstract_node_t* query_point, unsigned ink ) const
        {
            const task_space_node_t* ts_query_point = dynamic_cast<const task_space_node_t*>(query_point);
            if(ts_query_point!=NULL)
            {
                if (use_workspace_metric)
                {
                    return workspace_metric->multi_query(ts_query_point->end_effector_point, ink);
                }
                else
                {
                    return cspace_metric->multi_query(ts_query_point->arm_point, ink);
                }
            }
            else
            {
                if(query_point->point!=NULL)
                {
                    return cspace_metric->multi_query(query_point->point, ink); 
                }
                else
                {
                    std::vector<const abstract_node_t* > empty;
                    PRX_ERROR_S("Task space cast failed and point in query point is NULL.");
                    return empty;
                }
            }
        }

        const std::vector< const abstract_node_t* > task_space_metric_t::radius_query( const abstract_node_t* query_point, const double rad ) const
        {
            const task_space_node_t* ts_query_point = dynamic_cast<const task_space_node_t*>(query_point);
            if(ts_query_point!=NULL)
            {
                if (use_workspace_metric)
                {
                    return workspace_metric->radius_query(ts_query_point->end_effector_point, rad);
                }
                else
                {
                    return cspace_metric->radius_query(ts_query_point->arm_point, rad);
                }
            }
            else
            {
                if(query_point->point!=NULL)
                {
                    return cspace_metric->radius_query(query_point->point, rad); 
                }
                else
                {
                    std::vector<const abstract_node_t* > empty;
                    PRX_ERROR_S("Task space cast failed and point in query point is NULL.");
                    return empty;
                }
            }

        }

        const abstract_node_t* task_space_metric_t::single_query( const abstract_node_t* query_point, double* dist ) const
        {
            const task_space_node_t* ts_query_point = dynamic_cast<const task_space_node_t*>(query_point);
            if(ts_query_point!=NULL)
            {
                if (use_workspace_metric)
                {
                    return workspace_metric->single_query(ts_query_point->end_effector_point, dist);
                }
                else
                {
                    return cspace_metric->single_query(ts_query_point->arm_point, dist);
                }
            }
            else
            {
                if(query_point->point!=NULL)
                {
                    return cspace_metric->single_query(query_point->point, dist); 
                }
                else
                {
                    PRX_ERROR_S("Task space cast failed and point in query point is NULL.");
                    return NULL;
                }
            }

        }

        unsigned task_space_metric_t::radius_and_closest_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const 
        { 
            if (use_workspace_metric)
            {
                return workspace_metric->radius_and_closest_query(query_point, rad, closest);
            }
            else
            {
                return cspace_metric->radius_and_closest_query(query_point, rad, closest);
            }
        }

        unsigned task_space_metric_t::radius_query( const space_point_t* query_point, double rad, std::vector<const abstract_node_t*>& closest ) const 
        {
            if (use_workspace_metric)
            {
                return workspace_metric->radius_query(query_point, rad, closest);
            }
            else
            {
                return cspace_metric->radius_query(query_point, rad, closest);
            }
        }

        void task_space_metric_t::clear( )
        {
            workspace_metric->clear();
            cspace_metric->clear();
        }

        void task_space_metric_t::rebuild_data_structure( )
        {
            workspace_metric->rebuild_data_structure();
            cspace_metric->rebuild_data_structure();
        }
        
        void task_space_metric_t::print()
        {
            PRX_PRINT("Workspace Metric: ", PRX_TEXT_CYAN);
            workspace_metric->print();
            PRX_PRINT("CSpace Metric: ", PRX_TEXT_MAGENTA);
            cspace_metric->print();
        }

        bool task_space_metric_t::has_point( const abstract_node_t* embed ) 
        { 
            if (use_workspace_metric)
            {
                return workspace_metric->has_point(embed);
            }
            else
            {
                return cspace_metric->has_point(embed);
            }
        }

        unsigned task_space_metric_t::get_nr_points( )
        {
            PRX_ASSERT(cspace_metric->get_nr_points() == workspace_metric->get_nr_points());
            return cspace_metric->get_nr_points();
        }
    }
}
