/**
 * @file trajectory.cpp
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

#include "simulation/workspace_trajectory.hpp"

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {
            
            workspace_trajectory_t::workspace_trajectory_t()
            {
                // PRX_PRINT("Allocating a workspace trajectory", PRX_TEXT_MAGENTA);
                for( unsigned i=0; i<7; ++i )
                {
                    space_memory.push_back( new double );
                }
                
                state_space = new space_t("SE3", space_memory);
                num_states = 0;
                max_num_states = 0;
                end_state = states.begin();
                const_end_state = states.begin();
            }

            workspace_trajectory_t::workspace_trajectory_t(const workspace_trajectory_t& t)
            {
                // PRX_PRINT("Allocating a workspace trajectory", PRX_TEXT_BLUE);
                for( unsigned i=0; i<7; ++i )
                {
                    space_memory.push_back( new double );
                }
                
                state_space = new space_t("SE3", space_memory);
                num_states = 0;
                max_num_states = 0;
                end_state = states.begin();
                const_end_state = states.begin();

                (*this) = t;
            }

            workspace_trajectory_t::~workspace_trajectory_t()
            {
                for( unsigned i=0; i<space_memory.size(); ++i )
                {
                    delete space_memory[i];
                }
                delete state_space;
                state_space = NULL;
            }

            void workspace_trajectory_t::copy_onto_front( const util::config_t& config )
            {
                PRX_ASSERT(state_space != NULL);
                if( (num_states + 1) >= max_num_states )
                {
                    increase_buffer();
                }

                state_t* new_state = states.back();
                states.pop_back();
                states.push_front(new_state);
                
                state_t* st = *states.begin();
                config.get_position().get( st->memory[0], st->memory[1], st->memory[2] );
                config.get_orientation().get( st->memory[3], st->memory[4], st->memory[5], st->memory[6] );

                ++num_states;
                end_state = states.begin();
                const_end_state = states.begin();
                std::advance(end_state, num_states);
                std::advance(const_end_state, num_states);
            }

            void workspace_trajectory_t::copy_onto_back( const util::config_t& config )
            {
                PRX_ASSERT(state_space != NULL);
                if( (num_states + 1) >= max_num_states )
                {
                    increase_buffer();

                    end_state = states.begin();
                    const_end_state = states.begin();
                    std::advance(end_state, num_states);
                    std::advance(const_end_state, num_states);

                }

                state_t* st = *end_state;
                config.get_position().get( st->memory[0], st->memory[1], st->memory[2] );
                config.get_orientation().get( st->memory[3], st->memory[4], st->memory[5], st->memory[6] );
                
                ++end_state;
                ++const_end_state;
                ++num_states;
            }

            
            double workspace_trajectory_t::distance( unsigned index_a, unsigned index_b ) const
            {
                PRX_ASSERT(index_a < num_states);
                PRX_ASSERT(index_b < num_states);
                
                return state_space->distance( states[index_a], states[index_b] );
            }

        }
    }
}
