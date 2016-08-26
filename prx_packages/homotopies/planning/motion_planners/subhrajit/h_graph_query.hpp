///**
// * @file h_graph_query.hpp
// *
// * @copyright Software License Agreement (BSD License)
// * Copyright (c) 2013, Rutgers
// * All Rights Reserved.
// * For a full description see the file named LICENSE.
// *
// * @authors Andrew Kimmel, Andrew Dobson, Athanasios Krontiris, Zakary Littlefield,  Kostas Bekris
// *
// * Email: pracsys@googlegroups.com
// */
//
//#pragma once
//#ifndef PRX_H_GRAPH_QUERY_HPP
//#define	PRX_H_GRAPH_QUERY_HPP
//
//#include "prx/utilities/definitions/defs.hpp"
//#include "prx/planning/queries/motion_planning_query.hpp"
//#include "prx/utilities/parameters/parameter_reader.hpp"
//#include "prx/utilities/boost/hash.hpp"
//#include <complex>
//
//PRX_START
//typedef std::complex<double> complex_t;
//
//class radial_goal_region_t;
//
//class h_graph_query_t : public plan::motion_planning_query_t
//{
//
//    public:
//        h_graph_query_t();
//        h_graph_query_t(space_t* state_space, space_t* control_space, const std::string& my_plant);
//        ~h_graph_query_t();
//        
//        void print();
//        virtual void link_spaces(space_t* state_space, space_t* control_space);
//        
//        
//        
//        
//    private:
//            
//};
//        
//PRX_FINISH
//        
//#endif