/**
 * @file osg_camera.hpp 
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
#pragma once


#ifndef PRX_OSG_CAMERA_HPP
#define	PRX_OSG_CAMERA_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/visualization/camera.hpp"
#include "prx/visualization/handler.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
    }

    namespace vis
    {
        /** @brief Computes world coordinates of a particular node **/
        class getWorldCoordOfNodeVisitor : public osg::NodeVisitor 
        {
            public:
               getWorldCoordOfNodeVisitor():
                  osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
                  {
                     wcMatrix= new osg::Matrixd();
                  }
                  virtual void apply(osg::Node &node)
                  {
                     if (!done)
                     {
                        if ( 0 == node.getNumParents() ) // no parents
                        {
                           wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
                           done = true;
                        }
                        traverse(node);
                     }
                  }
                  osg::Matrixd* giveUpDaMat() 
                  {
                     return wcMatrix;
                  }
            private:
               bool done;
               osg::Matrix* wcMatrix;
        };

        /**
         * OSG implementation of the abstract camera_t class.
         * 
         * @brief <b> OSG implementation of the abstract camera_t class. </b>
         *
         * @authors Andrew Kimmel
         */
        class osg_camera_t : public camera_t
        {

          public:

            osg_camera_t();

            ~osg_camera_t();
            
            /**
             * @copydoc camera_t::init()
             */
            void init(const util::parameter_reader_t* reader);


            /**
             * Updates the camera based on the current view
             * 
             * @brief Updates the camera based on the current view
             * @param view The view the camera will use
             * @param node The node the camera will follow
             * @param ratio The aspect ratio of the view
             */
            void set_view(osg::View* view, osg::Group* node, double ratio);

            /**
             * Resets the camera parameters to hard-coded values
             * 
             * @brief Resets the camera parameters to hard-coded values
             */
            void reset_view();

            /**
             * @copydoc camera_t::get_camera_vector()
             */
            void get_camera_vector(int index, util::vector_t& vec) const;

            /**
             * @copydoc camera_t::set_camera_vector()
             */
            void set_camera_vector(int index, const util::vector_t& new_vec);

            /**
             * @copydoc camera_t::horizontal_rotation()
             */
            void horizontal_rotation(double rot);

            /**
             * @copydoc camera_t::vertical_rotation()
             */
            void vertical_rotation(double rot);

            /**
             * @copydoc camera_t::move_up()
             */
            void move_up();

            /**
             * @copydoc camera_t::move_down()
             */
            void move_down();

            /**
             * @copydoc camera_t::set_move_pos_in()
             */
            void set_move_pos_in(bool forward);

            /**
             * @copydoc camera_t::set_move_pos_side()
             */
            void set_move_pos_side(bool forward);

            /**
             * @copydoc camera_t::reset()
             */
            void reset();

            /**
             * @copydoc camera_t::speed_up()
             */
            void speed_up();

            /**
             * @copydoc camera_t::speed_down()
             */
            void speed_down();

            /**
             * @copydoc camera_t::is_ortho()
             */
            bool is_ortho() const;

            /**
             * @copydoc camera_t::set_ortho()
             */
            void set_ortho(bool orthographic);

            /**
             * @copydoc camera_t::is_follower()
             */
            bool is_follower() const;

            /**
             * @copydoc camera_t::toggle_follow()
             */
            void toggle_follow();

            /**
             * @copydoc camera_t::is_initialized()
             */
            bool is_initialized() const;

            /**
             * @copydoc camera_t::set_initialized()
             */
            void set_initialized(bool init);

            /**
             * @copydoc camera_t::print()
             */
            std::string print();

            /**
             * @copydoc camera_t::get_type()
             */
            int get_type() const;

            /**
             * Updates the camera's position. For type 2 cameras,
             * this update is gradual and smooth.
             * 
             * @brief Updates the camera's position 
             */
            void camera_frame();

            /**
             * Retrieves a pointer to the camera
             * 
             * @brief Retrieves a pointer to the camera
             * @return A pointer to the camera
             */
            osg::ref_ptr<osg::Camera> get_wrapped_camera() const;

            /** @brief For type 2 cameras, determines which direction the camera is moving */
            bool direction[8];
            
            void save_camera_image(osg::View* current_view);

          protected:

            /** @brief Internal storage of the OSG camera */
            osg::ref_ptr<osg::Camera> camera;
            
            /** @brief Computes the world coordinates of a particular node */
            osg::Matrixd* getWorldCoords( osg::Node* node);
            
            std::string image_dir;

        };

    }
}

#endif	/* PRX_OSG_CAMERA_HPP */

