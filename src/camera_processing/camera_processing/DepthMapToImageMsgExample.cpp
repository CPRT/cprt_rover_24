 std::unique_ptr<sensor_msgs::msg::Image> imageToROSmsg(
   const sl::Mat & img, const std::string & frameId, const rclcpp::Time & t)
 {
   std::unique_ptr<sensor_msgs::msg::Image> imgMessage = std::make_unique<sensor_msgs::msg::Image>();

   imgMessage->header.stamp = t;
   imgMessage->header.frame_id = frameId;
   imgMessage->height = img.getHeight();
   imgMessage->width = img.getWidth();

   int num = 1;  // for endianness detection
   imgMessage->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

   imgMessage->step = img.getStepBytes();

   size_t size = imgMessage->step * imgMessage->height;

   uint8_t * data_ptr = nullptr;

   sl::MAT_TYPE dataType = img.getDataType();

   switch (dataType) {
     case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
       imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
       imgMessage->encoding = sensor_msgs::image_encodings::MONO8;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::BGR8;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;

     case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
       imgMessage->encoding = sensor_msgs::image_encodings::BGRA8;
       data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
       imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
       break;
   }

   return imgMessage;
 }