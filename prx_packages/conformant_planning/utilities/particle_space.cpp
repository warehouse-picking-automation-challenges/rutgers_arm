/**
 * @file particle_space.cpp
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

#include "utilities/particle_space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"


namespace prx
{
    namespace util
    {

        particle_space_t::particle_space_t(const space_t* in_space, unsigned num_particles)
        {
            point_space = in_space;
            particles = num_particles;
            stored_memory = NULL;

            dimension = particles*point_space->get_dimension();

            //create new memory for the new space
#ifndef NDEBUG
            stored_memory = new particle_point_t(this, particles);
#else
            stored_memory = new particle_point_t(particles);
#endif

            stored_memory->link_size = point_space->get_dimension();
            for(unsigned i=0;i<particles;i++)
            {
                stored_memory->links[i] = point_space->alloc_point();
            }

            space_name = in_space->get_space_name() + int_to_str(num_particles);
            for( unsigned i = 0; i < dimension; i++ )
            {
                bounds.push_back(new bounds_t());
                scale.push_back(new double);
            }
            set_default_bounds();
            set_default_scale();

        }
        particle_space_t::~particle_space_t()
        {

        }


        std::string particle_space_t::print_point(const space_point_t * const point, unsigned prec) const
        {

            CHILD_CHECK(point)

            std::stringstream out(std::stringstream::out);
            if( dimension > 0 )
            {
                out << std::fixed << std::setprecision(prec) << '<';
                for( unsigned int i = 0; i < dimension - 1; ++i )
                    out << point->at(i) << ',';
                out << point->at(dimension - 1) << '>';
            }
            else
            {
                PRX_WARN_S("Trying to print an empty space!");
            }

            return out.str();
        }

        space_point_t* particle_space_t::alloc_point() const
        {
#ifndef NDEBUG
            particle_point_t* point = new particle_point_t(this, particles);
#else
            particle_point_t* point = new particle_point_t(particles);
#endif

            point->link_size = point_space->get_dimension();
            for(unsigned i=0;i<particles;i++)
            {
                point->links[i] = point_space->alloc_point();
            }
            copy_point(point,stored_memory);

            return point;
        }
        void particle_space_t::free_point(space_point_t* point) const
        {
            particle_point_t* p_point = dynamic_cast<particle_point_t*>(point);

            for(unsigned i=0;i<particles;i++)
            {
                point_space->free_point(p_point->links[i]);
            }
            delete point;
        }


        void particle_space_t::copy_to_point(space_point_t * const point) const
        {
            CHILD_CHECK(point)

            particle_point_t* const p_point = dynamic_cast<particle_point_t* const>(point);
            for(unsigned i=0;i<particles;i++)
            {
                point_space->copy_point(p_point->links[i],stored_memory->links[i]);
            }

        }

        void particle_space_t::copy_from_point(const space_point_t * const point, bool enforces_bounds) const
        {
            CHILD_CHECK(point)

            const particle_point_t* const p_point = dynamic_cast<const particle_point_t* const>(point);
            for(unsigned i=0;i<particles;i++)
            {
                point_space->copy_point(stored_memory->links[i],p_point->links[i]);
            }
            // if( enforces_bounds )
            //     enforce_bounds();
        }

        void particle_space_t::copy_point(space_point_t * const destination, const space_point_t * const source) const
        {
            CHILD_CHECK(destination)
            CHILD_CHECK(source)

            particle_point_t* const d_point = dynamic_cast<particle_point_t* const>(destination);
            const particle_point_t* const s_point = dynamic_cast<const particle_point_t* const>(source);
            for(unsigned i=0;i<particles;i++)
            {
                point_space->copy_point(d_point->links[i],s_point->links[i]);
            }
        }

        void particle_space_t::copy_to_particle(space_point_t * const point, unsigned particle_num) const
        {
            point_space->copy_point(point,stored_memory->links[particle_num]);
        }


        void particle_space_t::copy_from_particle(const space_point_t * const point, unsigned particle_num) const
        {
            point_space->copy_point(stored_memory->links[particle_num],point);
        }


        void particle_space_t::copy_to_particle(space_point_t * const point, unsigned particle_num, const space_point_t * const particle_set) const
        {
            const particle_point_t* const d_point = dynamic_cast<const particle_point_t* const>(particle_set);
            point_space->copy_point(point,d_point->links[particle_num]);
        }


        void particle_space_t::copy_from_particle(const space_point_t * const point, unsigned particle_num, space_point_t * const particle_set) const
        {
            particle_point_t* const d_point = dynamic_cast<particle_point_t* const>(particle_set);
            point_space->copy_point(d_point->links[particle_num],point);
        }


        void particle_space_t::copy_particle(unsigned to, unsigned from) const
        {
            point_space->copy_point(stored_memory->links[to],stored_memory->links[from]);
        }

        void particle_space_t::copy_particle(unsigned to, unsigned from,space_point_t * const particle_set) const
        {
            particle_point_t* const d_point = dynamic_cast<particle_point_t* const>(particle_set);
            point_space->copy_point(d_point->links[to],d_point->links[from]);
        }

        void particle_space_t::zero(space_point_t * const point) const
        {
            CHILD_CHECK(point)
            particle_point_t* const d_point = dynamic_cast<particle_point_t* const>(point);

            for( unsigned int i = 0; i < particles; ++i )
            {
                point_space->zero(d_point->links[i]);
            }

        }

    }
}
