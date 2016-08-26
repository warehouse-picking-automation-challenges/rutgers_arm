/**
 * @file gta_planner.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield,  Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace packages
    {
        namespace gta
        {
            struct action_t
            { 
                action_t()
                {
                    previous_action_cost = original_cost = action_cost =  0;
                    w_i = 1; l_i = 0;
                    action = NULL;
                    action_plan = NULL;
                    agent_id = -1;
                }
                action_t(double new_cost, sim::trajectory_t* new_traj, sim::plan_t* new_plan, int new_id)
                {
                    previous_action_cost = original_cost = action_cost =  0;
                    w_i = 1; l_i = 0;
                    action_cost = new_cost;
                    action = new_traj;
                    action_plan = new_plan;
                    agent_id = new_id;
                }
                double previous_action_cost;
                double action_cost;
                
                double original_cost;
                double w_i, l_i;
                
                sim::trajectory_t* action;
                sim::plan_t* action_plan;
                int agent_id;


            };
            inline bool operator < (const action_t& x, const action_t& y)
            {
                return x.action_cost < y.action_cost;
            }
            inline bool operator > (const action_t& x, const action_t& y)
            {
                return x.action_cost > y.action_cost;
            }
            //inline bool operator == (const action_t& x, const action_t& y)
            //{
            //    return (x.action== y.action);
            //}
            //inline bool operator != (const action_t& x, const action_t& y)
            //{
            //    return !(x == y);
            //}
            struct action_pair_t
            {
                action_pair_t(action_t* action1, action_t* action2, double payoff)
                {
                    my_action = action1;
                    other_action = action2;
                    pairwise_payoff = payoff;
                }
                action_t *my_action;
                action_t *other_action;
                double pairwise_payoff;
            };
            inline bool operator < (const action_pair_t& x, const action_pair_t& y)
            {
                return x.pairwise_payoff < y.pairwise_payoff;
            }
            inline bool operator > (const action_pair_t& x, const action_pair_t& y)
            {
                return x.pairwise_payoff > y.pairwise_payoff;
            }

            enum compute_action_type
            {
                PRX_PRM_ACTIONS = 0,
                PRX_RRT_ACTIONS = 1,
                PRX_H_GRAPH_ACTIONS = 2,
                PRX_IRS_ACTIONS = 3
            };
        }
    }
}
