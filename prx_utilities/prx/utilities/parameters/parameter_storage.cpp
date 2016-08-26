/**
 * @file parameter_storage.cpp
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

#include "prx/utilities/parameters/parameter_storage.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <XmlRpc.h>
 
namespace prx
{
    namespace util
    {

        parameter_storage_t::parameter_storage_t(std::string ns)
        {
            if( !ros::param::get(ns, memory)  )
            {
                std::cerr<<"Unexpected namespace"<<std::endl;
            }
        }
        
        parameter_storage_t::~parameter_storage_t()
        {

        }

        bool parameter_storage_t::has_namespace(std::string ns)
        {
            return has_namespace(ns,&memory);
        }

        parameter_storage_t::parameter_map parameter_storage_t::get_map(const XmlRpc::XmlRpcValue* input)
        {
            parameter_storage_t::parameter_map sub_readers_map;
            if( input->getType() != XmlRpc::XmlRpcValue::TypeStruct )
                std::cerr<<"Element is not a map."<<std::endl;


            const MyXmlRpcValue* my_rpc_value = static_cast<const MyXmlRpcValue*>(input);
            const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value->get_map();

            foreach(const key_value_t& key_value, value_map)
            {
                const std::string key = key_value.first;
                sub_readers_map[key] = &key_value.second;
            }
            return sub_readers_map;
        }

        parameter_storage_t::parameter_map parameter_storage_t::get_map(std::string ns, const XmlRpc::XmlRpcValue* input)
        {
            return get_map( get_namespace( ns, input ) );
        }

        parameter_storage_t::parameter_list parameter_storage_t::get_list(const XmlRpc::XmlRpcValue* input)
        {
            parameter_storage_t::parameter_list sub_readers_list;
            if( input->getType() != XmlRpc::XmlRpcValue::TypeArray )
                std::cerr<<"Element is not a list."<<std::endl;
            for( int i = 0; i < input->size(); ++i )
            {
                sub_readers_list.push_back(&(*input)[i]);
            }
            return sub_readers_list;
        }

        parameter_storage_t::parameter_list parameter_storage_t::get_list(std::string ns, const XmlRpc::XmlRpcValue* input)
        {
            return get_list( get_namespace( ns, input ) );
        }


        XmlRpc::XmlRpcValue* parameter_storage_t::get_namespace(std::string ns)
        {
            return const_cast<XmlRpc::XmlRpcValue*>(get_namespace(ns,&memory));
        }

        std::string parameter_storage_t::get_as_string(const XmlRpc::XmlRpcValue* input_first)
        {
            XmlRpc::XmlRpcValue* input = const_cast<XmlRpc::XmlRpcValue*>(input_first);
            std::string value;
            switch( input->getType() )
            {
                case XmlRpc::XmlRpcValue::TypeBoolean:
                    value = boost::lexical_cast<std::string > (static_cast<bool>(*input));
                    break;
                case XmlRpc::XmlRpcValue::TypeInt:
                    value = boost::lexical_cast<std::string > (static_cast<int>(*input));
                    break;
                case XmlRpc::XmlRpcValue::TypeDouble:
                    value = boost::lexical_cast<std::string > (static_cast<double>(*input));
                    break;
                case XmlRpc::XmlRpcValue::TypeString:
                    value = (std::string)(*input);
                    break;
                case XmlRpc::XmlRpcValue::TypeDateTime:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeDateTime");
                case XmlRpc::XmlRpcValue::TypeBase64:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeBase64");
                case XmlRpc::XmlRpcValue::TypeArray:
                    value.append(get_as_string(&(*input)[0]));
                    for( int i = 1; i < input->size(); ++i )
                    {
                        value.push_back(' ');
                        value.append(get_as_string(&(*input)[i]));
                    }
                    break;
                case XmlRpc::XmlRpcValue::TypeStruct:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeStruct");
                case XmlRpc::XmlRpcValue::TypeInvalid:
                    throw XmlRpc::XmlRpcException("Invalid XmlRpcValue type.");
            }

            return value;
        }

        std::string parameter_storage_t::get_as_string(std::string ns, const XmlRpc::XmlRpcValue* input_first)
        {
            return get_as_string( get_namespace( ns, input_first ) );
        }

        const XmlRpc::XmlRpcValue* parameter_storage_t::get_namespace(std::string ns, const XmlRpc::XmlRpcValue* sub_memory )
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(ns);

            if( name.empty() )
            {
                return sub_memory;    
            }
            
            if( subpath.empty() ) 
            {
                const MyXmlRpcValue* my_rpc_value = static_cast<const MyXmlRpcValue*>(sub_memory);
                const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value->get_map();


                foreach(const key_value_t& key_value, value_map)
                {
                    const std::string& key = key_value.first;
                    if(name==key)
                    {
                        return &key_value.second;
                    }
                }

            }
            else
            {
                const MyXmlRpcValue* my_rpc_value = static_cast<const MyXmlRpcValue*>(sub_memory);
                const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value->get_map();

                foreach(const key_value_t& key_value, value_map)
                {
                    const std::string& key = key_value.first;
                    if(name==key)
                        return get_namespace(subpath,&key_value.second);
                }
            }

        }

        bool parameter_storage_t::has_namespace(std::string ns, const XmlRpc::XmlRpcValue* sub_memory )
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(ns);

            if( subpath.empty() ) 
            {
                const MyXmlRpcValue* my_rpc_value = static_cast<const MyXmlRpcValue*>(sub_memory);
                const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value->get_map();


                foreach(const key_value_t& key_value, value_map)
                {
                    const std::string& key = key_value.first;
                    if(name==key)
                    {
                        return true;
                    }
                }
            }
            else
            {
                const MyXmlRpcValue* my_rpc_value = static_cast<const MyXmlRpcValue*>(sub_memory);
                const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value->get_map();

                foreach(const key_value_t& key_value, value_map)
                {
                    const std::string& key = key_value.first;
                    if(name==key)
                        return has_namespace(subpath,&key_value.second);
                }
            }
            return false;

        }

    }
}
