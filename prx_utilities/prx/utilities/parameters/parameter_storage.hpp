/**
 * @file parameter_storage.hpp
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

#pragma once

#ifndef PRX_PARAMETER_STORAGE_HPP
#define PRX_PARAMETER_STORAGE_HPP

#include "prx/utilities/definitions/defs.hpp"

namespace prx
{
    namespace util
    {

        class parameter_storage_t
        {
        public:
            typedef std::map<const std::string, const XmlRpc::XmlRpcValue*> parameter_map;
            typedef std::vector<const XmlRpc::XmlRpcValue*> parameter_list;

            parameter_storage_t(std::string ns);
            virtual ~parameter_storage_t();

            XmlRpc::XmlRpcValue* get_namespace(std::string ns);
            bool has_namespace(std::string ns);
            const XmlRpc::XmlRpcValue* get_namespace(std::string ns, const XmlRpc::XmlRpcValue* sub_memory );
            bool has_namespace(std::string ns, const XmlRpc::XmlRpcValue* sub_memory );

            parameter_map get_map(const XmlRpc::XmlRpcValue* input);
            parameter_map get_map(std::string ns, const XmlRpc::XmlRpcValue* input);
            parameter_list get_list(const XmlRpc::XmlRpcValue* input);
            parameter_list get_list(std::string ns, const XmlRpc::XmlRpcValue* input);
            std::string get_as_string(const XmlRpc::XmlRpcValue* input_first);
            std::string get_as_string(std::string ns, const XmlRpc::XmlRpcValue* input_first);

        protected:
            typedef XmlRpc::XmlRpcValue::ValueStruct::value_type key_value_t;

            class MyXmlRpcValue : public XmlRpc::XmlRpcValue
            {
            public:
                const ValueStruct& get_map() const
                {
                    return *_value.asStruct;
                }
            };

            XmlRpc::XmlRpcValue memory;

        };
        
    }
}

#endif //PRX_PARAMETER_STORAGE_HPP
