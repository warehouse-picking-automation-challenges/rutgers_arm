/**
 * @file region.cpp
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

#include "simulation/structures/OD_info.hpp"
#include "simulation/structures/origin.hpp"
#include "simulation/structures/world_structure.hpp"

#include "prx/utilities/parameters/parameter_reader.hpp"


#include <sstream>

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace crowd
        {
            OD_info_t::OD_info_t(origin_t* in_entrance, origin_t* in_exit, unsigned in_agent_type, int in_arrival_frame, int in_duration_frames, int in_departure_frame)
            {
                entrance = in_entrance;
                exit_origin = in_exit;
                agent_type = in_agent_type;
                arrival_frame = in_arrival_frame;
                duration_frames = in_duration_frames;
                departure_frame = in_departure_frame;
            }

            OD_info_t::OD_info_t(world_structure_t* world_structure, unsigned in_agent_type, const hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, const parameter_reader_t * const reader)
            {
                agent_type = in_agent_type;
                entrance = world_structure->get_origin(reader->get_attribute("entrance"));
                exit_origin = world_structure->get_origin(reader->get_attribute("exit"));
                arrival_frame = MSEC_2_FRAMES * reader->get_attribute_as<double>("arrival_time");
                departure_frame = MSEC_2_FRAMES * reader->get_attribute_as<double>("departure_time");
                duration_frames = departure_frame - arrival_frame;// MSEC_2_FRAMES * reader->get_attribute_as<double>("duration");
                has_luggage = reader->get_attribute_as<bool>("has_luggage");
                has_disability = reader->get_attribute_as<bool>("has_disability");
                max_speed = reader->get_attribute_as<double>("walking_speed");
                desire_to_go = reader->get_attribute_as<double>("departure_desire", 0.5);
                desires.resize(attractor_index.size());
                foreach( const parameter_reader_t* r, reader->get_list("attraction_response") )
                {
                    desires[attractor_index[r->get_attribute("name")]]= r->get_attribute_as<double>("responce");
                }
                //Corrections for disabilities and luggage
                if( has_luggage )
                {
                    max_speed *= 0.85;
                }
                if( has_disability )
                {
                    max_speed *= 0.5;
                }
            }

            OD_info_t::OD_info_t(world_structure_t* world_structure, unsigned in_agent_type, const hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, const YAML::Node& node)
            {
                agent_type = in_agent_type;
                std::string str;
                
                entrance = world_structure->get_origin(node["entrance"].as<std::string>());                
                exit_origin = world_structure->get_origin(node["exit"].as<std::string>());
                arrival_frame = MSEC_2_FRAMES * node["arrival_time"].as<double>();
                departure_frame = MSEC_2_FRAMES * node["departure_time"].as<double>();                
                duration_frames = departure_frame - arrival_frame;// MSEC_2_FRAMES * reader->get_attribute_as<double>("duration");
                has_luggage = node["has_luggage"].as<bool>();
                has_disability = node["has_disability"].as<bool>();
                max_speed = node["walking_speed"].as<double>();
                desire_to_go = node["departure_desire"].as<double>();
                desires.resize(attractor_index.size());
                
                YAML::Node influences = node["attraction_influences"];
                for(unsigned i=0; i<influences.size(); i++)
                {
                    desires[attractor_index[influences[i]["name"].as<std::string>()]] = influences[i]["desire"].as<double>();
                }
                //Corrections for disabilities and luggage
                if( has_luggage )
                {
                    max_speed *= 0.85;
                }
                if( has_disability )
                {
                    max_speed *= 0.5;
                }
            }

            OD_info_t::OD_info_t(world_structure_t* world_structure, const hash_t< std::string, unsigned >& agent_index, const hash_t<std::string, unsigned>& attractor_index, double MSEC_2_FRAMES, std::ifstream& fin)
            {
                //Local stuff for the reading
                char read_string[255];
                char read_value[255];
                char junk;
                
                //Get the routine info: entrance, exit, agent_type, arrival_time, departure_time, duration, luggage, disability, walking_speed, departure_desire
                fin.getline( read_string, 255, ',' );//Entrance
                entrance = world_structure->get_origin( read_string );
                fin.getline( read_string, 255, ',' );//Exit
                exit_origin = world_structure->get_origin( read_string );
                fin.getline( read_string, 255, ',' );//Agent Type
                agent_type = agent_index[ read_string ];
                fin.getline( read_string, 255, ',' );//Arrival Frame
                arrival_frame = MSEC_2_FRAMES * std::atoi( read_string );
                fin.getline( read_string, 255, ',' );//Departure Frame
                departure_frame = MSEC_2_FRAMES * std::atoi( read_string );
                fin.getline( read_string, 255, ',' );//Duration
                duration_frames = MSEC_2_FRAMES * std::atoi( read_string );
                fin.getline( read_string, 255, ',' );//Has Luggage
                has_luggage = (strncmp( read_string, "true", 255 ) == 0);
                fin.getline( read_string, 255, ',' );//Has Disability
                has_disability = (strncmp( read_string, "true", 255 ) == 0);
                fin.getline( read_string, 255, ',' );//Max Speed
                max_speed = std::atof( read_string );
                fin.getline( read_string, 255, ',' );//Departure desire
                desire_to_go = std::atof( read_string );

                desires.resize(attractor_index.size());

                //Now, have to read the attraction strengths
                for( unsigned i=0; i<attractor_index.size(); ++i )
                {
                    fin.getline( read_string, 255, ',' );//Attractor Name
                    fin.getline( read_value, 255, ',' );//Attraction Strength
                    unsigned a_index = attractor_index[ std::string( read_string ) ];
                    desires[ a_index ] = std::atof( read_value );
                }

                //Corrections for disabilities and luggage
                if( has_luggage )
                {
                    max_speed *= 0.85;
                }
                if( has_disability )
                {
                    max_speed *= 0.5;
                }
                
                //Strip out the newline for the next agent
                fin.getline( read_string, 255, '\n' );
            }

            std::string OD_info_t::print()
            {
                std::stringstream output(std::stringstream::out);
                output << entrance->name << " -> " << exit_origin->name << ") type:" << agent_type << "(" << arrival_frame << "," << departure_frame << ")";
                return output.str();
            }

            bool operator<(const OD_info_t &od1, const OD_info_t &od2)
            {
                if(od1.arrival_frame < od2.arrival_frame)
                    return true;
                return false;
            }
        }
    }
}
