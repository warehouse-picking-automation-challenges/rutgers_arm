/**
 * @file osg_camera.cpp 
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

#include <osgGA/UFOManipulator>

#include "prx/visualization/PLUGINS/OSG/osg_camera.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_helpers.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <osgGA/UFOManipulator>
#include <osgGA/TrackballManipulator>

namespace prx 
{ 
    using namespace util;
    namespace vis 
    {

osg_camera_t::osg_camera_t()
{
    reset_view();
    
    char* w = std::getenv("PRACSYS_PATH");
    std::string dir(w);
    dir += ("/prx_output/");
    boost::filesystem::path output_dir (dir);
    if (!boost::filesystem::exists(output_dir))
    {
        PRX_ERROR_S("Output folder does not exist, creating directory");
        // If not create it
        boost::filesystem::create_directory( output_dir );

    }
    dir+="images/";
    //PRX_DEBUG_S ("Dir: " << dir);
//    for (size_t i=0; i<p.we_wordc;i++ ) 
//        std::cout << w[i] << std::endl;

    // Check if images directory exists in current directory
    boost::filesystem::path images_dir (dir);
    //PRX_ERROR_S("Path: " << images_dir.system_complete(images_dir).string());
//    boost::filesystem::create_directory( images_dir );
    if (!boost::filesystem::exists(images_dir))
    {
        PRX_ERROR_S("Image folder does not exist, creating directory");
        // If not create it
        boost::filesystem::create_directory( images_dir );

    }
    
    image_dir = dir;
}

osg_camera_t::~osg_camera_t()
{
    
}
void osg_camera_t::reset_view()
{
    if (ortho)
    {
        eye =  vector_t(0.0, 0.0, 20.0);;
        center = vector_t (0.0,0.0,0.0);
    }
    else
    {
        eye = vector_t(0.0,0.0,-130.0);
        center = vector_t(0.0,.0,0.0);
    }

    vector_t diff = center - eye;
    vector_t x_vec (1.0,0.0,0.0);

    up.cross_product(diff, x_vec);
    //up = vector_t (0.0,1.0,0.0);

    camera_speed  = 3.0;
    camera_rotate = 0.001;
}

osg::Matrixd* osg_camera_t::getWorldCoords( osg::Node* node) 
{
   getWorldCoordOfNodeVisitor* ncv = new getWorldCoordOfNodeVisitor();
   if (node && ncv)
   {
      node->accept(*ncv);
      return ncv->giveUpDaMat();
   }
   else
   {
      return NULL;
   }
} 

void osg_camera_t::set_view ( osg::View* current_view, osg::Group* selectedNode, double ratio )
{
    //PRX_WARN_S ("SET VIEW!");

    if( follower && !ortho && selectedNode )
    {
        //PRX_ERROR_S ("FOLLOWING!");
        
        // Cast node to transform first
        osg::Transform* transform_node = selectedNode->asTransform();
        osg::PositionAttitudeTransform* pat_node = NULL;
        if (transform_node != NULL)
        {
            pat_node = transform_node->asPositionAttitudeTransform();
        }
        
        osg::Vec3 new_eye, node_position;
        double new_angle;
        if (pat_node != NULL)
        {
//            PRX_ERROR_S("Found Pat node");
//            PRX_PRINT("Position: " << pat_node->getPosition().x() << " , "<< pat_node->getPosition().y() << " , "<< pat_node->getPosition().z() << " , ", PRX_TEXT_MAGENTA);
//            PRX_PRINT("Attitude: " << pat_node->getAttitude().x() << ","<< pat_node->getAttitude().y() << ","<< pat_node->getAttitude().z() << ","<< pat_node->getAttitude().w() << ",", PRX_TEXT_CYAN);
            
            /** Attaches the following camera to the position-attitude of the picked node*/
            osg::Matrix* matrix = getWorldCoords(selectedNode);
            osg::Matrix _inverseMatrix = *matrix;
            matrix->invert( _inverseMatrix );
            node_position.set( _inverseMatrix(3,0), _inverseMatrix(3,1), _inverseMatrix(3,2 ));
            osg::Matrix R(_inverseMatrix);
            R(3,0) = R(3,1) = R(3,2) = 0.0;
            new_eye = osg::Vec3(1,0,0) * R; // camera up is +Z, regardless of CoordinateFrame
            current_view->getCamera()->setViewMatrixAsLookAt ( node_position, node_position+new_eye, (osg::Vec3(0,0,1) *R));
            
            
        }
        else
        {
            node_position = selectedNode->getBound().center();
            osg::Vec3 diff_vec = toVec3(eye - center);
            new_eye = node_position + diff_vec;
            current_view->getCamera()->setViewMatrixAsLookAt ( new_eye, node_position, toVec3(up));
            eye = fromVec3(new_eye);
            center = fromVec3(node_position);

        }


//        printf ("Current camera values during follow mode\n");
//        eye.print();
//        center.print();
//        up.print();
    }
    else 
    {
        if ( ortho )
            current_view->getCamera()->setProjectionMatrixAsOrtho( right, left, top, bottom, zNear, zFar);
        else
            current_view->getCamera()->setProjectionMatrixAsPerspective( 75.0, ratio, 0.1, 10000 );

//        printf ("Current camera values\n");
//        eye.print();
//        center.print();
//        up.print();
        current_view->getCamera()->setViewMatrixAsLookAt( toVec3(eye), toVec3(center), toVec3(up) );
    }
}


std::string osg_camera_t::print()
{
    std::stringstream out(std::stringstream::out);
    out << "Camera::" << std::endl << "Eye: " << eye << " Center: " << center << " Up: " << up << std::endl;
    return out.str();
}

void osg_camera_t::save_camera_image(osg::View* current_view)
{
    int x,y,width,height;
    x = current_view->getCamera()->getViewport()->x();
    y = current_view->getCamera()->getViewport()->y();
    width = current_view->getCamera()->getViewport()->width();
    height = current_view->getCamera()->getViewport()->height();

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);
    std::string _filename = "test.jpg";
    if (osgDB::writeImageFile(*image,image_dir+_filename))
    {
        std::cout << "Saved screen image to `"<<image_dir+_filename<<"`"<< std::endl;
    } 
}

void osg_camera_t::get_camera_vector( int index, vector_t& vec ) const
{
    if( index == PRX_CAMERA_EYE )
	vec = eye;
    else if( index == PRX_CAMERA_CENTER )
	vec = center;
    else
	vec = up;
}

// stub
void osg_camera_t::set_camera_vector ( int index, const vector_t& new_vec )
{
    if( index == PRX_CAMERA_EYE )
	eye = new_vec;
    else if( index == PRX_CAMERA_CENTER )
	center = new_vec;
    else
	up = new_vec;
}


void osg_camera_t::horizontal_rotation( double rotation )
{
    rotation *= camera_rotate;

    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
    double roll  = atan2( center[2]-eye[2], D );
    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );

    theta -= rotation;

    center[2] =  eye[2] + dist * sin(roll) ;
    D = dist * cos(roll);
    center[0] =  eye[0] + D*sin(theta);
    center[1] =  eye[1] + D*cos(theta) ;
}

void osg_camera_t::vertical_rotation(double rotation)
{
//    if (camera_type != 2)
    {
        rotation *= camera_rotate;

        double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
        double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
        double roll  = atan2( center[2]-eye[2], D );
        double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );

        roll -= rotation;

        center[2] = eye[2] + dist * sin(roll);
        D = dist * cos(roll);
        center[0] = eye[0] + D*sin(theta);
        center[1] = eye[1] + D*cos(theta);
    }
//    else
//    {
////        if (rotation < 0)
////            direction.push_back(3);
////        else
////            direction.push_back(4);
//    }
}


void osg_camera_t::move_down( )
{
    if (ortho)
    {
        top -= camera_speed;
        bottom -= camera_speed;
    }
    else
    {
        eye[2] -= camera_speed;
        center[2] -= camera_speed;
    }
}


void osg_camera_t::move_up( )
{
    if (ortho)
    {
        top += camera_speed;
        bottom += camera_speed;
    }
    else
    {
        eye[2] += camera_speed;
        center[2] += camera_speed;
    }
}

void osg_camera_t::set_move_pos_in( bool forward )
{
    in = center - eye;
    in.normalize();
    in *= camera_speed ;

    if( forward )
    {
        if (camera_type == 2)
        {
 //           direction.push_back(1);
           // in[1] = 0.0;
           // eye+= in;
           // center+=in;


        }
        else if (ortho)
        {
            left   += camera_speed;
            right  -= camera_speed;
            top    -= camera_speed;
            bottom += camera_speed;
        }
        else
        {
            double hold = eye[2];
            eye += in;
            eye[2] = hold;
            if( !follower || eye.distance( center ) < 0.1 )
            {
                hold = center[2];
                center += in;
                center[2] = hold;
            }
        }
    }
    else
    {
        
        if (camera_type == 2)
        {
 //           direction.push_back(2);
//            in[1] = 0.0;
//            eye-= in;
//            center-=in;


        }
        else if (ortho)
        {
            left   -= camera_speed;
            right  += camera_speed;
            top    += camera_speed;
            bottom -= camera_speed;
        }
        else
        {
            double hold = eye[2];
            eye -= in;
            eye[2] = hold;
            if ( !follower )
            {
                hold = center[2];
                center -= in;
                center[2] = hold;
            }
        }
    }
}


void osg_camera_t::set_move_pos_side( bool forward )
{
    in = center - eye;
    cross.cross_product( in, up );
    cross.normalize();
    cross *= camera_speed;

    if( forward )
    {
        if (ortho)
        {
            left += camera_speed;
            right += camera_speed;

        }
        else
        {
            center += cross;
            eye += cross;
        }
    }
    else
    {
        if (ortho)
        {
            left -= camera_speed;
            right -= camera_speed;
        }
        else
        {
            center -= cross;
            eye -= cross;
        }
    }
}


void osg_camera_t::reset()
{

    eye = init_eye;
    center = init_center;
    up = init_up;
    camera_speed  = init_cam_speed;
    camera_rotate = init_cam_rotate;
}


void osg_camera_t::speed_up()
{
    camera_speed+=0.2;
}

void osg_camera_t::speed_down()
{
    camera_speed-=0.2;
    if (camera_speed < 0.02)
        camera_speed = 0.02;
}


bool osg_camera_t::is_ortho() const
{
    return ortho;
}


void osg_camera_t::set_ortho( bool orthographic)
{
    ortho = orthographic;
}

bool osg_camera_t::is_follower() const
{
    return follower;
}

void osg_camera_t::toggle_follow()
{
    follower = !follower;
    
    PRX_DEBUG_COLOR("Camera now has follow value: " << follower, PRX_TEXT_CYAN);
}


bool osg_camera_t::is_initialized() const
{
    return initialized;
}

void osg_camera_t::set_initialized( bool init)
{
    initialized = init;
}

int osg_camera_t::get_type() const
{
    return camera_type;
}

void osg_camera_t::camera_frame()
{
    in = center - eye;
    cross.cross_product( in, up );
    cross.normalize();
    in.normalize();
    cross *= 3;
    in *= 3 ;

    if (camera_type == 2)
    {
        for (int i = 0; i < 8; ++i)
        {
            if (direction[i])
            {
//                PRX_ERROR_S ("Direction: " << i);
                if (i == 0)
                {
                    center -= cross;
                    eye -= cross;
                }
                else if (i == 1)
                {
//                    in[2] = 0.0;
                    eye-= in;
                    center-=in;
                }
                else if (i == 2)
                {
                    center += cross;
                    eye += cross;
                }
                else if (i == 3)
                {
                    double rotation = 42;
                    rotation *= .42;
                    
                    horizontal_rotation(rotation);

//                    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
//                    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
//                    double roll  = atan2( center[2]-eye[2], D );
//                    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );
//
//                    theta -= rotation;
//
//                    center[2] =  eye[2] + dist * sin(roll) ;
//                    D = dist * cos(roll);
//                    center[0] =  eye[0] + D*cos(theta);
//                    center[1] =  eye[1] + D*sin(theta) ;
                }
                else if (i == 4)
                {
//                    in[2] = 0.0;
                    eye+= in;
                    center+=in;
                }

                else if (i == 5)
                {
                    double rotation = -42;
                    rotation *= .42;
                    
                    horizontal_rotation(rotation);

//                    double D     = sqrt(  pow(center[0]-eye[0],2) + pow(center[1]-eye[1],2) );
//                    double theta = atan2( center[0]-eye[0], center[1]-eye[1] );
//                    double roll  = atan2( center[2]-eye[2], D );
//                    double dist  = sqrt(  pow(center[0]-eye[0],2) + pow(center[2]-eye[2],2) + pow(center[1]-eye[1],2) );
//
//                    theta -= rotation;
//
//                    center[2] =  eye[2] + dist * sin(roll) ;
//                    D = dist * cos(roll);
//                    center[0] =  eye[0] + D*cos(theta);
//                    center[1] =  eye[1] + D*sin(theta) ;
                }
                else if (i == 6)
                {
                    eye[2] += camera_speed;
                    center[2] += camera_speed;
                }
                else if (i == 7)
                {
                    eye[2] -= camera_speed;
 
                    center[2] -= camera_speed;
     
                }
            }
        }

        if (eye[2] < 1.0)
        {
            eye[2] = 1.0;
        }
        if (center[2] < 1.0)
        {
            center[2] = 1.0;
        }
    }
}

osg::ref_ptr<osg::Camera> osg_camera_t::get_wrapped_camera() const
{
    return camera;
}

void osg_camera_t::init(const parameter_reader_t* reader)
{
    reset();
    initialized = false;
    follower = false;
    ortho = false;
    camera_speed = INIT_CAM_SPEED;
    camera_rotate = INIT_CAM_ROTATE;
    for (int i = 0; i < 8; ++i)
            direction[i] = false;

    camera = new osg::Camera();

    ortho = reader->get_attribute_as<bool>("ortho");
    camera_type = reader->get_attribute_as<int>("camera_type", 0);


    if (ortho)
    {
        reset_view();
        left = reader->get_attribute_as<double>("ortho_param/left");
        right = reader->get_attribute_as<double>("ortho_param/right");
        bottom = reader->get_attribute_as<double>("ortho_param/bottom");
        top = reader->get_attribute_as<double>("ortho_param/top");
        zNear = reader->get_attribute_as<double>("ortho_param/zNear");
        zFar = reader->get_attribute_as<double>("ortho_param/zFar");

    }
    else
    {
        eye = reader->get_attribute_as<vector_t>("eye");
        center = reader->get_attribute_as<vector_t>("center");

        up.set(0.0,0.0,1.0);
        init_eye = eye;
        init_center = center;
        init_up = up;
        set_initialized(true);
    }

    camera_speed = reader->get_attribute_as<double>("speed/move");
    camera_rotate = reader->get_attribute_as<double>("speed/rotate");
    init_cam_speed = camera_speed;
    init_cam_rotate = camera_rotate;

    PRX_DEBUG_S("Created osg_camera_t.");
}

    }
 }
