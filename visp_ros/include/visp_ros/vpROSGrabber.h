/****************************************************************************
 *
 * $Id: vpROSGrabber.h 3803 2012-06-22 10:22:59Z fpasteau $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Camera video capture for ROS image_transort_compressed.
 *
 * Authors:
 * Francois Pasteau
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpROSGrabber.h
  \brief class for Camera video capture for ROS middleware.
*/

#ifndef vpROSGrabber_h
#define vpROSGrabber_h

#include <visp3/core/vpConfig.h>

#if defined( VISP_HAVE_OPENCV )

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <visp_bridge/camera.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#include <opencv2/highgui/highgui.hpp>
#else
#include <highgui.h>
#endif

/*!
  \class vpROSGrabber

  \ingroup Framegrabber CameraDriver

  \brief Class for cameras video capture for ROS cameras.

  Needs OpenCV available on http://opencv.willowgarage.com/wiki/.
  Needs pthread

  The code below shows how to use this class.
  \code
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp_ros/vpROSGrabber.h>

int main()
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<unsigned char> I; // Create a gray level image container
  vpROSGrabber g;        // Create a grabber for ROS
  g.setCameraInfoTopic("/camera/camera_info");
  g.setImageTopic("/camera/image_raw");
  g.setRectify(true);

  g.open(I);                           // Open the framegrabber
  g.acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

 */
class VISP_EXPORT vpROSGrabber : public vpFrameGrabber
{
protected:
  ros::NodeHandle *m_n;
  ros::Subscriber m_img_sub;
  ros::Subscriber m_cam_info_sub;
  ros::AsyncSpinner *m_spinner;
  volatile bool m_isInitialized;
  volatile unsigned int m_width;
  volatile unsigned int m_height;
  image_geometry::PinholeCameraModel m_p;
  cv::Mat m_img;
  bool m_flip;
  volatile bool m_rectify;
  volatile bool m_mutex_image, m_mutex_param;
  void imageCallbackRaw( const sensor_msgs::Image::ConstPtr &msg );
  void imageCallback( const sensor_msgs::CompressedImage::ConstPtr &msg );
  void paramCallback( const sensor_msgs::CameraInfo::ConstPtr &msg );
  volatile bool m_first_img_received;
  volatile bool m_first_param_received;
  volatile uint32_t m_sec, m_nsec;
  std::string m_master_uri;
  std::string m_topic_image;
  std::string m_topic_cam_info;
  std::string m_nodespace;
  std::string m_image_transport;
  vpCameraParameters m_cam;

public:
  vpROSGrabber();
  virtual ~vpROSGrabber();

  void open( int argc, char **argv );
  void open();
  void open( vpImage< unsigned char > &I );
  void open( vpImage< vpRGBa > &I );

  void acquire( vpImage< unsigned char > &I );
  void acquire( vpImage< vpRGBa > &I );
  cv::Mat acquire();
  bool acquireNoWait( vpImage< unsigned char > &I );
  bool acquireNoWait( vpImage< vpRGBa > &I );

  void acquire( vpImage< unsigned char > &I, struct timespec &timestamp );
  void acquire( vpImage< vpRGBa > &I, struct timespec &timestamp );
  void acquire( vpImage< unsigned char > &I, double &timestamp_second );
  void acquire( vpImage< vpRGBa > &I, double &timestamp_second );
  cv::Mat acquire( struct timespec &timestamp );
  bool acquireNoWait( vpImage< unsigned char > &I, struct timespec &timestamp );
  bool acquireNoWait( vpImage< vpRGBa > &I, struct timespec &timestamp );

  void close();

  void setCameraInfoTopic( const std::string &topic_name );
  void setImageTopic( const std::string &topic_name );
  void setMasterURI( const std::string &master_uri );
  void setNodespace( const std::string &nodespace );
  void setImageTransport( const std::string &image_transport );
  void setFlip( bool flipType );
  void setRectify( bool rectify );

  bool getCameraInfo( vpCameraParameters &cam );
  void getWidth( unsigned int width ) const;
  void getHeight( unsigned int height ) const;
  unsigned int getWidth() const;
  unsigned int getHeight() const;
};

#endif
#endif
