/**
 * @file preprocess_coordination_mp.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/motion_planners/coordination_graph/coordination_constraint.hpp"
#include "prx/utilities/statistics/image.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"

#include <fstream>


namespace prx
{
    namespace packages
    {
        namespace coordination_manipulation
        {
            coordination_constraints_t::coordination_constraints_t()
            {
                max_right_size = 0;
                max_left_size = 0;
            }
            coordination_constraints_t::~coordination_constraints_t()
            {
                
            }
            bool coordination_constraints_t::is_valid_state(unsigned right_start_index, unsigned left_start_index, unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object)
            {
//                PRX_PRINT ("Checking rob: " << right_object_index << " and lob: " << left_object_index, PRX_TEXT_CYAN);
//                PRX_PRINT ("At rstep: " << right_start_index << " and lstep: " << left_start_index, PRX_TEXT_MAGENTA);
                if (right_object_index > object_limits[right_object] || left_object_index > object_limits[left_object])
                {
                    PRX_FATAL_S("Invalid object indices!");
                }
                current_constraint = &all_constraints[right_object+left_object][right_object_index][left_object_index];
                bool valid_bounds = (right_start_index < current_constraint->size()) && (left_start_index < current_constraint->at(0).size());
                return ( valid_bounds && (*current_constraint)[right_start_index][left_start_index] );
            }
            
            
            coord_constraint* coordination_constraints_t::get_constraint(unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object)
            {
                return &all_constraints[right_object+left_object][right_object_index][left_object_index];
            }
            
            unsigned coordination_constraints_t::estimate_conflict_area(unsigned right_object_index, unsigned left_object_index, std::string right_object, std::string left_object)
            {
                return all_constraint_areas[right_object+left_object][right_object_index][left_object_index];
            }


            bool coordination_constraints_t::deserialize_constraints(const std::vector<std::string>& object_types, const std::vector<int>& num_objects, const std::string& directory, const std::string& experiment)
            {
                /** Check if directory exists */
                if( !boost::filesystem::exists(directory) )
                {
                    PRX_ERROR_S ("Directory: " << directory << " does not exist!");
                    return false;
                }
                
                
                PRX_PRINT ("Deserializing coordination graphs!", PRX_TEXT_GREEN);
                for (unsigned i = 0; i < object_types.size(); ++i)
                {
                    std::string right_object = object_types[i];
                    int right_max = num_objects[i];
                    object_limits[right_object] = right_max;
                    
                    for(unsigned j = 0; j < object_types.size(); ++j)
                    {
                        std::string left_object = object_types[j];
                        int left_max = num_objects[j];
                        
                        constraint_matrix coordination_constraints;
                        std::vector< std::vector< unsigned > > constraint_conflict_area;
                        
                        coordination_constraints.resize(right_max);
                        constraint_conflict_area.resize(right_max);
                        all_constraint_areas[right_object+left_object].resize(right_max);
                        for(unsigned x_index = 0; x_index < right_max; x_index++)
                        {
                            coordination_constraints[x_index].resize(left_max);
                            constraint_conflict_area[x_index].resize(left_max, 0);
                            all_constraint_areas[right_object+left_object][x_index].resize(left_max);
                        }
                        
                        for(unsigned right_index = 0; right_index < right_max; right_index++)
                        {
                            for (unsigned left_index = 0; left_index < left_max; left_index++)
                            {
                                std::stringstream filename;
                                filename << directory << "[COORDINATION_CONSTRAINTS][" << experiment <<"]_R" << right_object << right_index
                                         << "_L" << left_object << left_index << ".txt";
                                current_constraint = &coordination_constraints[right_index][left_index];
                                if(deserialize_helper(right_index, left_index, right_object, left_object, filename.str(), current_constraint))
                                {
                                    //print_constraint(right_index, left_index, right_object, left_object, current_constraint);
                                }
                                else
                                {
                                    PRX_FATAL_S ("Could not load coordination graph. OBJECT ID Right: " << right_index << ", left: " << left_index);
                                }

                            }
                        }
                        
                        all_constraints[right_object+left_object] = coordination_constraints;
                        all_constraint_areas[right_object+left_object] = constraint_conflict_area;
                    }
                }
                PRX_PRINT ("FINISHED Deserializing coordination graphs!", PRX_TEXT_GREEN);
                
                return true;

            }
            
            bool coordination_constraints_t::deserialize_helper(unsigned rob_index, unsigned lob_index, std::string right_object, std::string left_object, const std::string& filename, coord_constraint* constraint_to_fill)
            {
                std::ifstream fin;
                PRX_DEBUG_COLOR ("LOADING FILE: " << filename, PRX_TEXT_MAGENTA);
                fin.open(filename.c_str());
                if(!fin.is_open())
                {
                    PRX_ERROR_S ("Could not open file");
                    return false;
                }
                
                // PRX_ERROR_S("Inside graph deserialize");
                
                std::string experiment;
                fin >> experiment;
                
                std::string robject, lobject;
                fin >> robject >> lobject;
                
                if (right_object != robject || left_object != lobject)
                {
                    PRX_ERROR_S ("Right and left object types are incorrect!");
                    return false;
                }
                
                unsigned check_right, check_left;
                fin >> check_right >> check_left;
                
                if (check_right != rob_index || check_left != lob_index)
                {
                    PRX_ERROR_S ("Wrong object identifier in file!");
                    return false;
                }
                
                unsigned right_arm_size, left_arm_size;
                fin >> right_arm_size >> left_arm_size;
                
                if (right_arm_size > max_right_size)
                {
                    max_right_size = right_arm_size;
                }
                
                if (left_arm_size > max_left_size)
                {
                    max_left_size = left_arm_size;
                }
                
                PRX_DEBUG_COLOR("Right plan size: " << right_arm_size << " and left plan size: " << left_arm_size, PRX_TEXT_CYAN);
                
                constraint_to_fill->resize(right_arm_size);
                
                for(unsigned x_index = 0; x_index < right_arm_size; x_index++)
                {
                    (*constraint_to_fill)[x_index].resize(left_arm_size, false);
                }
                
                unsigned num_vertices;
                fin >> num_vertices;
                
                // PRX_WARN_S ("Read in num vertices: " << num_vertices);
                if (num_vertices > 0)
                {
                    int right_step, left_step;
                    for (unsigned current_index = 0; current_index <= num_vertices; current_index++)
                    {
                        fin >> right_step >> left_step;
                        (*constraint_to_fill)[right_step][left_step] = true;

                    }
                }
                //PRX_ASSERT(false);
                fin.close();
                
                unsigned conflict_area_counter = (right_arm_size*left_arm_size) - num_vertices;
                all_constraint_areas[right_object+left_object][rob_index][lob_index] = conflict_area_counter;
                PRX_DEBUG_COLOR("Conflict area calculated: " << conflict_area_counter, PRX_TEXT_RED);
                
//                 PRX_INFO_S(" Inside PRM serialization now, saving to file: ");
//                 char* w = std::getenv("PRACSYS_PATH");
//                 std::string dir(w);
//                 dir += ("/prx_output/");
//                 boost::filesystem::path output_dir(dir);
//                 if( !boost::filesystem::exists(output_dir) )
//                 {
//                     boost::filesystem::create_directories(output_dir);
//                 }
//                 std::stringstream blahstream;
//                 blahstream << dir << "test_" << rob_index << lob_index << ".txt";
//                 std::string file = blahstream.str();
//                 PRX_INFO_S("Filename is: " << filename);
//                 std::ofstream fout;
//                 fout.open(file.c_str());
//                 PRX_ASSERT(fout.is_open());
                
//                 PRX_ERROR_S("Inside graph serialize");
                
// //                if (save_constraints)
//                 {
//                     fout << num_vertices << std::endl;
//                     for(int right_step = 0; right_step < right_arm_size; right_step++)
//                     {

//                         for(int left_step = 0; left_step < left_arm_size; left_step++)
//                         {
//                             if ((*constraint_to_fill)[right_step][left_step])
//                             {
//                                 fout << right_step << " " << left_step << std::endl;

//                             }
//                         }
//                     }
//                 }
//                 fout.close();
                
                return true;
                
//                if (save_constraints)
//                {
//                    fout << preprocess_query->right_object_index << " " << preprocess_query->left_object_index << std::endl;
//                    fout << num_vertices << std::endl;
//                    for(int right_step = 0; right_step < right_arm_size; right_step++)
//                    {
//
//                        for(int left_step = 0; left_step < left_arm_size; left_step++)
//                        {
//                            if (valid_vertex[right_step*max_plan_size + left_step])
//                            {
//                                fout << right_step << " " << left_step << std::endl;
//
//                            }
//                        }
//                    }
//                }
                
                
            }
            
            unsigned coordination_constraints_t::get_max_left_size()
            {
                return max_left_size;
            }
            
            unsigned coordination_constraints_t::get_max_right_size()
            {
                return max_right_size;
            }
            
            void coordination_constraints_t::print_constraint(unsigned rob_index, unsigned lob_index, std::string rob_type, std::string lob_type, coord_constraint* constraint_to_print)
            {
                
                char* w = std::getenv("PRACSYS_PATH");
                std::string dir(w);
                std::stringstream s1;
                s1<<"/prx_output/";
                dir += (s1.str());
                boost::filesystem::path output_dir (dir);
                if (!boost::filesystem::exists(output_dir))
                {
                    boost::filesystem::create_directories( output_dir );
                }
                util::image_t image(400,400);
                
                unsigned right_arm_size = constraint_to_print->size();
                unsigned left_arm_size = (*constraint_to_print)[0].size();
                PRX_DEBUG_COLOR("Printing constraint with right size: " << right_arm_size << " and left size: " << left_arm_size, PRX_TEXT_MAGENTA);
                for(unsigned right_index = 0; right_index < right_arm_size; right_index++)
                {
                    for (unsigned left_index = 0; left_index < left_arm_size; left_index++)
                    {
                        double x,y;
                        unsigned char color = 0;
                        x = ((double)right_index / (double)right_arm_size)*400 - 200;
                        y = -1*((double)left_index / (double)left_arm_size)*400 + 200;
                        if((*constraint_to_print)[right_index][left_index])
                        {
                            color = 255;
                        }
                        for(double i=x-2;i<=x+2;i++)
                        {
                            for(double j=y-2;j<=y+2;j++)
                            {
                                int i_pixel = floor(i+.5);
                                int j_pixel = floor(j+.5);
                                image.color_pixel(i_pixel,j_pixel,color,color,color);
                            }            
                        }
                    }
                }
//                foreach(state_t* state, input_query->path)
//                {
//                    double x,y;
//                    x = ((state->at(0)) / state_space->get_bounds()[0]->get_upper_bound())*200;
//                    y = -1*(state->at(1) / state_space->get_bounds()[1]->get_upper_bound())*200;
//                    unsigned char color;
//                    for(double i=x-1;i<=x+1;i++)
//                    {
//                        for(double j=y-1;j<=y+1;j++)
//                        {
//                            int i_pixel = floor(i+.5);
//                            int j_pixel = floor(j+.5);
//                            image.color_pixel(i_pixel,j_pixel,0,255,0);
//                        }            
//                    }
//                }
                std::stringstream s;
                s<<"R" << rob_index << rob_type << "_" << "_L" << lob_index << lob_type << ".png";
                dir += s.str();
                image.encode(dir.c_str());
            }
            
            
        }
    }
}