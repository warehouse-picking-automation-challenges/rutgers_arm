/**
 * @file planner.cpp
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
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/distance_metrics/distance_metric.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include "prx/planning/planner.hpp"
#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/planning/modules/validity_checkers/validity_checker.hpp"
#include "prx/planning/modules/local_planners/local_planner.hpp"
#include <ctime>

namespace prx
{
    using namespace util;
    using namespace util::parameters;
    using namespace sim;

    namespace plan
    {

        pluginlib::ClassLoader<planner_t> planner_t::loader("prx_planning", "prx::plan::planner_t");

        pluginlib::ClassLoader<planner_t>& planner_t::get_loader()
        {
            return loader;
        }

        std::ofstream planner_t::planning_stat_file;
        std::string planner_t::stat_filename;

        planner_t::planner_t()
        {
            validity_checker = NULL;
            sampler = NULL;
            local_planner = NULL;
            metric = NULL;
            path = "";
        }

        planner_t::~planner_t()
        {
            if( validity_checker != NULL )
                delete validity_checker;
            if( sampler != NULL )
                delete sampler;
            if( local_planner != NULL )
                delete local_planner;
            if( metric != NULL )
                delete metric;
        }

        void planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {
            visualize = get_attribute_as<bool> ("visualize", reader, template_reader, true);

            //The first planner to update the statistics filename writes the current datetime to the file.
            if(parameters::has_attribute("stat_file", reader, NULL) && stat_filename.empty())
            {
                stat_filename = parameters::get_attribute_as<std::string > ("stat_file", reader, template_reader);
                char* w = std::getenv("PRACSYS_PATH");
                std::string file_prefix(w);
                file_prefix+="/prx_output/";
                stat_filename = file_prefix + stat_filename;
                std::time_t _time = std::time(NULL); 
                std::string current_time(std::ctime(&_time));
                PRX_INFO_S("Writing to planning stats file "<<stat_filename);
                append_to_stat_file("\n\n\n********************************************************************\nPlanning statistics for execution on "+current_time+"\n********************************************************************\n");
            }
        }

        void planner_t::set_pathname(std::string in_path)
        {
            path = in_path;
        }

        void planner_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( parameter_name == "visualize" )
            {
                visualize = boost::any_cast<bool>(value);
            }
        }

        void planner_t::update_visualization() const
        {
            //    PRX_WARN_S("Planner update_visualization: visualize: "<<visualize);
            if( visualize )
            {
                PRX_PRINT(path << ": update visualization.", PRX_TEXT_LIGHTGRAY);
                update_vis_info();
            }
        }

        bool planner_t::serialize()
        {
            PRX_WARN_S("Serialize not implemented in base-class planner_t!");
        }

        void planner_t::update_vis_info() const
        {
            PRX_ERROR_S("Abstract planner does not visualize any geometries.");
        }

        std::string planner_t::get_name() const
        {
            return reverse_split_path(path).second;
        }


        void planner_t::append_to_stat_file(std::string output_text)
        {
            //If the filename is empty that means none of the planners had the parameter stat_file
            if(!stat_filename.empty())
            {
                planning_stat_file.open (stat_filename.c_str(), std::ios::out | std::ios::app);
                planning_stat_file << output_text;
                planning_stat_file.close();
            }
            else
            {
                //The file where to write the statistics is unknown
                PRX_WARN_S("No stat file defined in "<<get_name());
            }
        }


    }
}
