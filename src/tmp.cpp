// // --- A. 预取当前 World 到 Body (IMU) 的变换 ---
      // M3D R_w2i = _state.rot_end.transpose();
      // V3D t_w2i = -R_w2i * _state.pos_end;
      
      // 遍历点云中的每一个点
      // for (size_t i = 0; i < size; i++)
      // {
      //   PointTypeRGB pointRGB;
      //   pointRGB.x = pcl_wait_pub->points[i].x;
      //   pointRGB.y = pcl_wait_pub->points[i].y;
      //   pointRGB.z = pcl_wait_pub->points[i].z;
      //   V3D p_w(pointRGB.x, pointRGB.y, pointRGB.z);

      //   // --- B. 转换到 Body (IMU) 系并计算球面坐标 ---
      //   V3D p_body = R_w2i * p_w + t_w2i;
      //   Sphere s;
      //   if (!s.xyz2tg(p_body)) continue; // 极小值保护

      //   bool colored = false;

      //   // --- C. 遍历所有相机进行快速区间筛选 ---
      //   for (int c = 0; c < camera_num_; ++c) {
      //     auto& cu = cameras_[c];
      //     auto& vm = cu.vio_manager;
      //     if (!vm || !vm->new_frame_) continue;

      //     // 1. 检查 Theta 范围
      //     if (s.theta < cu.min_theta || s.theta > cu.max_theta) continue;

      //     // 2. 检查 Gamma 范围 (处理跨越 0/2PI 的边界情况)
      //     bool in_gamma = false;
      //     if (!cu.gamma_wraps) {
      //       in_gamma = (s.gamma >= cu.min_gamma && s.gamma <= cu.max_gamma);
      //     } else {
      //       in_gamma = (s.gamma >= cu.min_gamma || s.gamma <= cu.max_gamma);
      //     }
      //     if (!in_gamma) continue;

      //     // --- 只有区间命中的点，才执行昂贵的投影校验 ---

      //     // 3. 将世界点投影到像素坐标 (w2c 内部处理了所有外参链)
      //     V2D pc = vm->new_frame_->w2c(p_w);

      //     // 4. 判断是否在图像范围内
      //     if (vm->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) 
      //     {
      //       cv::Mat& img_rgb = vm->img_rgb;
      //       V3F pixel;

      //       // 根据模型选择插值方式
      //       if(vm->cam_model_type == "MEICamera") {
      //         // MEI 模型需要 pf 辅助计算球面上采样
      //         V3D pf = vm->new_frame_->w2f(p_w);
      //         pixel = vm->getInterpolatedPixelFromSphere(img_rgb, pc, pf);
      //       } else {
      //         pixel = vm->getInterpolatedPixel(img_rgb, pc);
      //       }

      //       if(pixel[0] >= 0) {
      //         pointRGB.r = pixel[2];
      //         pointRGB.g = pixel[1];
      //         pointRGB.b = pixel[0];
              
      //         // 距离检查
      //         if (p_body.norm() > blind_rgb_points) {
      //           laserCloudWorldRGB->push_back(pointRGB);
      //           colored = true;
      //         }
      //       }
      //     }
          
      //     // 如果该相机已经成功着色，跳出相机循环
      //     if (colored) break; 
      //   }
      // }
      // for (size_t i = 0; i < size; i++)
      // {
      //   // auto& p_lidar = pcl_wait_pub->points[i];
      //   PointTypeRGB pointRGB;
      //   pointRGB.x = pcl_wait_pub->points[i].x; 
      //   pointRGB.y = pcl_wait_pub->points[i].y; 
      //   pointRGB.z = pcl_wait_pub->points[i].z;
      
      //   V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);

      //   // bool colored = false;

      //   // 暴力遍历：不经过 Sphere 筛选，直接尝试投影
      //   // for (int c = 0; c < camera_num_; ++c) {
      //     auto& vm = cameras_[0].vio_manager;
      //     if (!vm || !vm->new_frame_) {
      //       printf("\033[1;31m!! WRONG CAMERA !!\033[0m\n");
      //       continue;
      //     }

      //     // 1. 投影到像素
      //     V3D pf = vm->new_frame_->w2f(p_w); 
      //     if (vm->cam_model_type == "Pinhole" && pf[2] < 0) continue;
      //     V2D pc = vm->new_frame_->w2c(p_w);
      //     // 2. 严格判断是否在画面内
      //     if (vm->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) 
      //     {
            
      //       // if (pf[2] < 0) continue;

      //       cv::Mat& img_rgb = vm->img_rgb;
      //       V3F pixel;

      //       if(vm->cam_model_type == "MEICamera") {
      //         pixel = vm->getInterpolatedPixelFromSphere(img_rgb, pc, pf);
      //         if(pixel[0] >= 0) {
      //           pointRGB.r = pixel[2];
      //           pointRGB.g = pixel[1];
      //           pointRGB.b = pixel[0];
      //           // colored = true;
      //           laserCloudWorldRGB->push_back(pointRGB);
      //           // break; 
      //         }
      //       } else {
      //         pixel = vm->getInterpolatedPixel(img_rgb, pc);
      //         if(pixel[0] >= 0) {
      //           pointRGB.r = pixel[2];
      //           pointRGB.g = pixel[1];
      //           pointRGB.b = pixel[0];
      //           // colored = true;
      //           laserCloudWorldRGB->push_back(pointRGB);
      //           // break; 
      //         }
      //       }

      //       // if(pixel[0] >= 0) {
      //       //   pointRGB.r = pixel[2];
      //       //   pointRGB.g = pixel[1];
      //       //   pointRGB.b = pixel[0];
      //       //   // colored = true;
      //       //   laserCloudWorldRGB->push_back(pointRGB);
      //       //   // break; 
      //       // }
      //     }
      //   // }
 
      //   // if (colored) {
      //   //   laserCloudWorldRGB->push_back(pointRGB);
      //   // }
      // }


      void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      
      double t1 = omp_get_wtime();

      VIOManagerPtr vm = cameras_[0].vio_manager;
      if (!vm || !vm->new_frame_) {
        printf("\033[1;31m!! WRONG CAMERA !!\033[0m\n");
        return;
      }
      cv::Mat img_rgb = vm->img_rgb;

      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;
      
        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vm->new_frame_->w2f(p_w)); 
        if (vm->cam_model_type == "Pinhole" && pf[2] < 0) continue;
        V2D pc(vm->new_frame_->w2c(p_w));
        // 获取插值后的像素
        if (vm->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          if(vm->cam_model_type == "MEICamera") {
            // V3F pixel = vm->getInterpolatedPixel(img_rgb, pc);
            V3F pixel = vm->getInterpolatedPixelFromSphere(img_rgb, pc, pf);
            if(pixel[0] >= 0) {
              pointRGB.r = pixel[2];
              pointRGB.g = pixel[1];  
              pointRGB.b = pixel[0];
              // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
              // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
              // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
              // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
              if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
            }
          } 
          else {
            V3F pixel = vm->getInterpolatedPixel(img_rgb, pc);
            pointRGB.r = pixel[2];
            pointRGB.g = pixel[1];
            pointRGB.b = pixel[0];
            // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
            // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
            // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
            // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
            if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
          }
        }
      }
      double t2 = omp_get_wtime();
      printf("\033[1;32m[Sphere-Accelerated Render Time]: %-27f s\033[0m\n", t2 - t1);
    }
  }

  /*** 发布和保存逻辑（保持原样，但注意变量引用） ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO) {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  } else {
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  
  laserCloudmsg.header.stamp = this->node->get_clock()->now();
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);
  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          // LIVO模式：优先保存VIO生成的彩色点云
          if (LidarMeasures.lio_vio_flg == VIO)
          {
            if (laserCloudWorldRGB->size() > 0)
            {
              // 有彩色点云时，累积彩色点云
              *pcl_wait_save += *laserCloudWorldRGB;
              scan_wait_num++;
              std::cout << "[PCD Save] Accumulated " << laserCloudWorldRGB->size() 
                        << " RGB points. Total: " << pcl_wait_save->size() << std::endl;
            }
            else
            {
              // 如果VIO时没有生成彩色点云（pub_num < pub_scan_num），累积灰度点云作为备用
              // 这样即使程序被中断，也能保存部分点云
              if (pcl_w_wait_pub->size() > 0)
              {
                *pcl_wait_save_intensity += *pcl_w_wait_pub;
                scan_wait_num++;
                std::cout << "[PCD Save] VIO mode but no RGB points, accumulated " 
                          << pcl_w_wait_pub->size() << " intensity points. Total: " 
                          << pcl_wait_save_intensity->size() << std::endl;
              }
            }
          }
          else if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
          {
            // LIO模式下不累积点云，因为它们是中间状态，会在下一次VIO时被着色
            // 如果用户想要保存所有点云，可以取消下面的注释
            // *pcl_wait_save_intensity += *pcl_w_wait_pub;
            // scan_wait_num++;
          }
        }
        else
        {
          // ONLY_LIO或ONLY_LO模式：保存灰度点云
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
          if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) 
          {
            scan_wait_num++;
            std::cout << "[PCD Save] LIO/LO mode, accumulated " << pcl_w_wait_pub->size() 
                      << " intensity points. Total: " << pcl_wait_save_intensity->size() << std::endl;
          }
        }
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }

  // --- 修改 PCD 保存和图片保存中的单例引用 ---
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO && !cameras_.empty()) {
      // 默认保存 cam0 的图
      static int img_wait_num = 0;
      if (++img_wait_num >= img_save_interval) {
          double update_time = LidarMeasures.measures.back().vio_time;
          std::stringstream ss_time;
          ss_time << std::fixed << std::setprecision(6) << update_time;
          cv::imwrite(std::string(ROOT_DIR) + "Log/image/" + ss_time.str() + ".png", cameras_[0].vio_manager->img_rgb);
          img_wait_num = 0;
      }
  }

  if(laserCloudWorldRGB->size() > 0) PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO) PointCloudXYZI().swap(*pcl_w_wait_pub);
}
