/**
 * @file parameter_reader.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

// #include <ros/param.h>
// #include <XmlRpc.h>
// #include <deque>
// #include <boost/tuple/tuple.hpp> // boost::tie

namespace prx
{
    namespace util
    {
        parameter_storage_t* global_storage;
        
        bool parameter_reader_t::has_attribute(const std::string& path) const
        {
            return param_store->has_namespace( path, root );
        }

        const std::string parameter_reader_t::get_attribute(const std::string& path) const
        {
            return param_store->get_as_string( path, root );
        }

        const std::string parameter_reader_t::get_attribute(const std::string& name, const std::string& default_val) const
        {
            if( has_attribute(name) )
                return get_attribute(name);

            return default_val;
        }

        std::unique_ptr<const parameter_reader_t> parameter_reader_t::get_child(const std::string& path) const
        {
            return std::unique_ptr< const parameter_reader_t >(get_subreader(path));
        }

        std::vector<const parameter_reader_t*> parameter_reader_t::get_list(const std::string& path) const
        {
            return get_subreaders(path);
        }

        std::map<const std::string, const parameter_reader_t*> parameter_reader_t::get_map(const std::string& path) const
        {
            return get_subreaders_map( path );
        }

        parameter_reader_t::parameter_reader_t(const std::string& input_path, parameter_storage_t* storage) : path(input_path)
        {
            //First, let's clean up the path we have been given.
            char last = path[path.length() - 1];
            if( last == '/' )
            {
                path = path.substr(0,path.length()-1);
            }
            last = path[0];
            if( last == '/' )
            {
                path = path.substr(1,path.length()-1);
            }

            param_store = storage;
            root = param_store->get_namespace( path );
        }
 
        parameter_reader_t::parameter_reader_t(const std::string& input_path, parameter_storage_t* storage, const XmlRpc::XmlRpcValue* input_root)
        : root(input_root), path(input_path), param_store(storage)
        {
        }


        const std::string parameter_reader_t::trace() const
        {
            return path;
        }

        const parameter_reader_t* parameter_reader_t::get_subreader(const std::string& key) const
        {
            const XmlRpc::XmlRpcValue* new_namespace = param_store->get_namespace( key, root );
            
            std::string full_path = path + "/" + key;
            
            return new parameter_reader_t( full_path, param_store, new_namespace );
        }

        std::vector<const parameter_reader_t*> parameter_reader_t::get_subreaders(const std::string& key) const
        {
            std::vector<const parameter_reader_t*> sub_readers;
            
            std::string full_path = path + "/" + key;
            parameter_storage_t::parameter_list namespaces = param_store->get_list( key, root );
            
            for( unsigned i=0; i<namespaces.size(); ++i )
            {
                sub_readers.push_back( new parameter_reader_t( full_path, param_store, namespaces[i] ) );
            }
            
            return sub_readers;
        }

        std::map<const std::string, const parameter_reader_t*> parameter_reader_t::get_subreaders_map(const std::string& key) const
        {
            std::map<const std::string, const parameter_reader_t*> sub_readers_map;
            
            std::string map_path = path + "/" + key;
            parameter_storage_t::parameter_map namespaces = param_store->get_map( key, root );
            
            foreach( parameter_storage_t::parameter_map::value_type element, namespaces )
            {
                std::string full_path = map_path + "/" + element.first;
                sub_readers_map[ element.first ] = new parameter_reader_t( full_path, param_store, element.second );
            }
            
            return sub_readers_map;
        }

        std::istream& operator>>(std::istream& input, std::vector<double>& v)
        {
            while( input )
            {
                double r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        std::istream& operator>>(std::istream& input, std::vector<unsigned>& v)
        {
            while( input )
            {
                unsigned r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        std::istream& operator>>(std::istream& input, std::vector<int>& v)
        {
            while( input )
            {
                int r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        std::istream& operator>>(std::istream& input, std::vector<std::string>& v)
        {
            while( input )
            {
                std::string r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        namespace parameters
        {

            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader, const std::string& default_val)
            {
                return get_attribute_as<std::string > (name, reader, template_reader, default_val);
            }

            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                return get_attribute_as<std::string > (name, reader, template_reader);
            }

            bool has_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                bool has = false;
                if( reader != NULL && reader->has_attribute(name) )
                    has = true;
                else if( template_reader != NULL && template_reader->has_attribute(name) )
                    has = true;
                return has;
            }

            std::vector<const parameter_reader_t*> get_list(const std::string list_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                if( reader != NULL && reader->has_attribute(list_name) )
                    return reader->get_list(list_name);
                else if( template_reader != NULL && template_reader->has_attribute(list_name) )
                    return template_reader->get_list(list_name);

                return std::vector<const parameter_reader_t*>();

            }

            parameter_reader_t::reader_map_t get_map(const std::string map_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                if( reader != NULL && reader->has_attribute(map_name) )
                    return reader->get_map(map_name);
                else if( template_reader != NULL && template_reader->has_attribute(map_name) )
                    return template_reader->get_map(map_name);

                return parameter_reader_t::reader_map_t();

            }
        }
    }
}
