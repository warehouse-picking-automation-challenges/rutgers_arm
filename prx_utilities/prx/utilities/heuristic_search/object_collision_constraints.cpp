/**
 * @file object_collision_constraints.cpp 
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

#include "prx/utilities/heuristic_search/object_collision_constraints.hpp"

#include <fstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::util::object_collision_constraints_t, prx::util::constraints_t)


namespace prx
{    
    namespace util
    {        

        object_collision_constraints_t::object_collision_constraints_t()
        {
        }

        object_collision_constraints_t::~object_collision_constraints_t(){}


        void object_collision_constraints_t::init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader)
        {
            constraints_t::init(reader,template_reader);
        }

        std::string object_collision_constraints_t::get_type()
        {
            return "object_collision_constraints";
        }

        void object_collision_constraints_t::clear()
        {
            constraint_sets.clear();
            constraints.clear();
        }

        constraints_t& object_collision_constraints_t::operator=(const constraints_t& c)
        {
            const object_collision_constraints_t& rsc = dynamic_cast<const object_collision_constraints_t&>(c);
            PRX_ASSERT( &rsc != NULL );
            constraints = rsc.constraints;
            return *this;
        }

        bool object_collision_constraints_t::operator == ( const constraints_t& c) const
        {
            const object_collision_constraints_t& rsc = dynamic_cast<const object_collision_constraints_t&>(c);
            PRX_ASSERT( &rsc != NULL );            
            return constraints == rsc.constraints;
        }            

        bool object_collision_constraints_t::operator<(const constraints_t& c) const
        {
            const object_collision_constraints_t& rsc = dynamic_cast<const object_collision_constraints_t&>(c);
            PRX_ASSERT( &rsc != NULL );
            return constraints.size() < rsc.constraints.size();
        }

        void object_collision_constraints_t::merge(const constraints_t* c)
        {
            const object_collision_constraints_t* rsc = dynamic_cast<const object_collision_constraints_t*>(c);
            PRX_ASSERT( rsc != NULL );
            //Merge the constraints
            constraints.insert( rsc->constraints.begin(), rsc->constraints.end() );
        }

        bool object_collision_constraints_t::intersect(constraints_t* out_constraints, const constraints_t* valid_constraints)
        {
            object_collision_constraints_t* out_c = dynamic_cast< object_collision_constraints_t* >(out_constraints);
            const object_collision_constraints_t* v_c = dynamic_cast<const object_collision_constraints_t* >(valid_constraints);
            PRX_ASSERT( v_c != NULL );
            PRX_ASSERT( out_c != NULL );
            std::vector<unsigned> common;
            
            out_c->constraints.clear();
            std::set_intersection( constraints.begin(), constraints.end(), v_c->constraints.begin(), v_c->constraints.end(), std::back_inserter(common) );
            out_c->constraints.insert( common.begin(), common.end() );
            
            return common.size() > 0;
        }

        bool object_collision_constraints_t::has_intersection(const constraints_t* valid_constraints) const
        {
            const object_collision_constraints_t* v_c = dynamic_cast<const object_collision_constraints_t*>(valid_constraints);
            PRX_ASSERT( v_c != NULL );            
            foreach( unsigned c, constraints )
            {
                if( v_c->constraints.count( c ) > 0 )
                {
                    return true;
                }
            }
            return false;
        }

        bool object_collision_constraints_t::has_hard_constraints(const constraints_t* valid_constraints) const
        {
            // const object_collision_constraints_t* v_c = dynamic_cast<const object_collision_constraints_t*>(valid_constraints);
            // //Interested in only for level 0 constraints because for this problem the most important hard constraints are in the level 0
            // if(v_c->constraints.size() > 0 && v_c->constraints[0].size() > 0)
            //     foreach(unsigned c, constraints)
            //         if(v_c->constraints[0].count(c) > 0)
            //             return true; 
            return false;
        }

        void object_collision_constraints_t::add_to_constraint_sets( const constraints_t* c, double d )
        {
            const object_collision_constraints_t* occ = dynamic_cast< const object_collision_constraints_t* >( c );
            PRX_ASSERT( occ != NULL );
            constraint_sets.push_back( std::make_pair( occ->constraints, d ) );
        }


        bool object_collision_constraints_t::exact_constraint_comparison(const constraints_t* new_constraints, double new_distance)
        {                
            std::vector<unsigned> indices;

            //If we do NOT dominate the incoming constraints
            if( !dominates(new_constraints, new_distance) )
            {
                const object_collision_constraints_t* c = dynamic_cast<const object_collision_constraints_t*>(new_constraints);
                PRX_ASSERT( c != NULL );
                //Look at all of our constraint sets
                for( unsigned i = 0; i < constraint_sets.size(); ++i )
                {
                    //if the constraint set is a super set of the incoming constraints AND its distance is larger than the incoming distance
                    if( std::includes(constraint_sets[i].first.begin(), constraint_sets[i].first.end(), c->constraints.begin(), c->constraints.end()) && constraint_sets[i].second - new_distance >= -PRX_ZERO_CHECK )
                    {
                        //Mark this set for deletion
                        indices.push_back(i);
                    }
                }
                
                //For each constraint set marked for deletion
                for( int i = indices.size() - 1; i >= 0; --i )
                {
                    //Remove it from the graph's constraint sets
                    constraint_sets.erase(constraint_sets.begin() + indices[i]);
                }
                
                //Then, add a new constraint set to the graph node with the incoming constraints and distance
                constraint_sets.push_back(std::make_pair(c->constraints,new_distance));
                return true;
            }
            //We do dominate the incoming constriants, so let's not accept the incoming
            return false;
        }

        bool object_collision_constraints_t::dominates(const constraints_t* new_constraints, double dist)
        {
            const object_collision_constraints_t* c = dynamic_cast<const object_collision_constraints_t*>(new_constraints);
            PRX_ASSERT( c != NULL );
            //For each of the constraint sets (paths) in this constraint class
            for( unsigned i = 0; i < constraint_sets.size(); ++i )
            {
                //If this constraint set is a subset of the incoming constraints
                if( std::includes(c->constraints.begin(), c->constraints.end(), constraint_sets[i].first.begin(), constraint_sets[i].first.end()) )
                {
                    // If the constraint set cost is better than the incoming distance.
                    if( fabs(constraint_sets[i].second - dist) > PRX_ZERO_CHECK && constraint_sets[i].second < dist )
                    {
                        //The incoming set is dominated
                        return true;
                    }
                    //If the constraint sets are identical
                    else if( constraint_sets[i].first.size() == c->constraints.size() )
                    {
                        // PRX_DEBUG_COLOR(" The new node will dominate for sure :  dist:" << constraint_sets[i].second <<  "   C:" << print_constraints(constraint_sets[i].first) , PRX_TEXT_GREEN);
                        //We don't dominate because it is the same constraints, but we have a higher cost
                        return false;
                    }
                }
            }
            // PRX_DEBUG_COLOR("No one could dominate the new node. It is safe!", PRX_TEXT_GREEN);
            //No one dominates the new node so it will be added in the lists.
            return false;
        }

        void object_collision_constraints_t::serialize(std::ofstream& output_stream)
        {
            int size = constraints.size();
            output_stream << size << " ";
            foreach(unsigned c, constraints)
            {
                output_stream << c << " ";
            }
            // if(size > 0)
            //     output_stream << std::endl;
        }

        void object_collision_constraints_t::deserialize(std::ifstream& input_stream)
        {
            unsigned size;
            unsigned constraint;
            input_stream >> size;
            PRX_DEBUG_COLOR("Size: " << size, PRX_TEXT_CYAN);
            for( unsigned i = 0; i < size; ++i )
            {
                input_stream >> constraint;
                constraints.insert(constraint);
            }
            PRX_DEBUG_COLOR(print(), PRX_TEXT_CYAN);
        }

        std::string object_collision_constraints_t::print() const
        {
            std::stringstream output(std::stringstream::out);
            output << "Constraints: " ;
            foreach(unsigned c, constraints)
            {
                output << c << ", ";
            }
            return output.str();
        }
    }   
}
