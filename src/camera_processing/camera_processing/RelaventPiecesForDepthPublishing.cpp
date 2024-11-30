


NOTES from zed_wrapper_nodelet.cpp

Need to produce a sensor_msgs Image

Relavent bits from zed_wrapper_nodelet.cpp:

sl::Mat mat_depth;

    if (!mOpenniDepthMode)
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, mMatResol);
    }
    else
    {
      mZed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU, mMatResol);
    }


    ros::Time stamp = sl_tools::slTime2Ros(grab_ts);
  if (mSvoMode)
  {
    stamp = ros::Time::now();
  }


    // Publish the depth image if someone has subscribed to
  if (depthSubnumber > 0)
  {
    sensor_msgs::ImagePtr depthImgMsg = boost::make_shared<sensor_msgs::Image>();
    publishDepth(depthImgMsg, mat_depth, stamp);
  }



  void ZEDWrapperNodelet::publishDepth(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat depth, ros::Time t)
{
  mDepthCamInfoMsg->header.stamp = t;

  // NODELET_DEBUG_STREAM("mOpenniDepthMode: " << mOpenniDepthMode);

  if (!mOpenniDepthMode)
  {
    // NODELET_INFO("Using float32");
    sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
    mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);

    return;
  }

  // NODELET_INFO("Using depth16");
  sl_tools::imageToROSmsg(imgMsgPtr, depth, mDepthOptFrameId, t);
  mPubDepth.publish(imgMsgPtr, mDepthCamInfoMsg);
}



ros::Time slTime2Ros(sl::Timestamp t)
{
  uint32_t sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
  return ros::Time(sec, nsec);
}







point cloud stuff


mZed.retrieveMeasure(mCloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU, mMatResol);




void ZEDWrapperNodelet::publishPointCloud()
{
  sensor_msgs::PointCloud2Ptr pointcloudMsg = boost::make_shared<sensor_msgs::PointCloud2>();

  // Publish freq calculation
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
  last_time = now;

  mPcPeriodMean_usec->addValue(elapsed_usec);

  // Initialize Point Cloud message
  // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

  int ptsCount = mMatResol.width * mMatResol.height;

  pointcloudMsg->header.stamp = mPointCloudTime;

  if (pointcloudMsg->width != mMatResol.width || pointcloudMsg->height != mMatResol.height)
  {
    pointcloudMsg->header.frame_id = mPointCloudFrameId;  // Set the header values of the ROS message

    pointcloudMsg->is_bigendian = false;
    pointcloudMsg->is_dense = false;

    pointcloudMsg->width = mMatResol.width;
    pointcloudMsg->height = mMatResol.height;

    sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);
  }

  // Data copy
  sl::Vector4<float>* cpu_cloud = mCloud.getPtr<sl::float4>();
  float* ptCloudPtr = (float*)(&pointcloudMsg->data[0]);

  // We can do a direct memcpy since data organization is the same
  memcpy(ptCloudPtr, (float*)cpu_cloud, 4 * ptsCount * sizeof(float));

  // Pointcloud publishing
  mPubCloud.publish(pointcloudMsg);
}