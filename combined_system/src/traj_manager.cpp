#include "plan_manage/traj_manager.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>
#include <cmath>
#include <chrono>

using namespace chrono;

void TrajPlanner::init(const ros::NodeHandle& nh)
{
  nh_ = nh;

  nh_.param("planning/car_id", car_id_, 0);
  nh_.param("planning/traj_piece_duration", traj_piece_duration_, 1.0);
  nh_.param("planning/traj_res", traj_res_, 8);
  nh_.param("planning/dense_traj_res", dense_traj_res_, 20);

  nh_.param("vehicle/cars_num", cars_num_, 1);
    nh_.param("vehicle/car_id", car_id_, 0);
    nh_.param("vehicle/car_length", car_length_, 0.248);
    nh_.param("vehicle/car_width", car_width_, 0.192);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 0.1);
    nh_.param("vehicle/wheelbase", car_wheelbase_, 0.148);
  kino_path_finder_.reset(new path_searching::KinoAstar);
  kino_path_finder_->setMap(map_ptr_);
  kino_path_finder_->init(nh_);

  ploy_traj_opt_.reset(new plan_manage::PolyTrajOptimizer);
  ploy_traj_opt_->init(nh_);

  KinopathPub_ = nh_.advertise<visualization_msgs::Marker>("car_kino_trajs", 10);
  wholebody_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("car_wholebodyTraj", 2);

}

void TrajPlanner::setMap(const MappingProcess::Ptr& ptr)
{
  this->map_ptr_ = ptr;
}


// use kino a* to generate a path
bool TrajPlanner::getKinoPath(Eigen::Vector4d &end_state, bool first_search)
{

  Eigen::Vector4d start_state;
  Eigen::Vector2d init_ctrl;

  start_state = start_state_;
  init_ctrl << start_ctrl_;
  //steer and acc


  Eigen::Vector2d start_pos = start_state.head(2);
  kino_path_finder_->findNearestNode(start_pos, first_search);
  kino_path_finder_->reset();
  auto searcht1 = high_resolution_clock::now();

  std::cout<<"start state: "<<start_state.transpose()<<" end_state: "<<end_state.transpose()<<std::endl;
  std::cout<<"init ctrl: "<<init_ctrl.transpose()<<std::endl;
  int status = kino_path_finder_->search(start_state, init_ctrl, end_state, true);
  auto searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;
  std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;
  if (status == path_searching::KinoAstar::NO_PATH)
  {
    std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_state, init_ctrl, end_state, false);
    if (status == path_searching::KinoAstar::NO_PATH)
    {
      std::cout << "[kino replan]: Can't find path." << std::endl;
      return false;
    }
    else
    {
      std::cout << "[kino replan]: retry search success." << std::endl;
    }
  }
  else
  {
    std::cout << "[kino replan]: kinodynamic search success." << std::endl;
  }

  auto time_searcht1 = high_resolution_clock::now();
  kino_path_finder_->getTruncatedposLists();
  kino_path_finder_->getSingulNodes();
  bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_);
  auto time_searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
  cout << "search time: " << time_searchms.count() << " ms" << endl;
  if(!middle_node_success)
    return false;

  return true;
}

void TrajPlanner::setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time)
{
  has_init_state_ = true;
  start_state_ = state;
  if(start_state_(3) < 0.05)
    start_state_(3) = 0.05;
  start_time_ = start_time;

  Eigen::Vector2d init_ctrl(0.0, 0.0);
  start_ctrl_ = init_ctrl;
}

void TrajPlanner::displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs)
{
  visualization_msgs::Marker sphere, line_strip, carMarkers;
  sphere.header.frame_id = line_strip.header.frame_id = carMarkers.header.frame_id = "map";
  sphere.header.stamp = line_strip.header.stamp = carMarkers.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  sphere.action = visualization_msgs::Marker::DELETE;
  line_strip.action = visualization_msgs::Marker::DELETE;

  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);

  sphere.action = line_strip.action = carMarkers.action = visualization_msgs::Marker::ADD;
  sphere.id = 0;
  line_strip.id = 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.a = line_strip.color.a = 0.5;
  sphere.scale.x = 0.5;
  sphere.scale.y = 0.5;
  sphere.scale.z = 0.5;
  line_strip.scale.x = 0.25;

  geometry_msgs::Point pt;
  unsigned int size = kino_trajs->size();
  for (unsigned int i = 0; i < size; ++i){
    sphere.color.r = line_strip.color.r = i*1.0/(size*1.0);
    sphere.color.g = line_strip.color.g = 0.0;
    sphere.color.b = line_strip.color.b = i*1.0/(size*1.0);

    for (int k = 0; k < kino_trajs->at(i).traj_pts.size(); k++)
    {
      Eigen::Vector3d trajpt = kino_trajs->at(i).traj_pts[k];
      double yaw = kino_trajs->at(i).thetas[k];
      pt.x = trajpt(0);
      pt.y = trajpt(1);
      pt.z = 0.1;
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

    }
  }

  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);
}

void TrajPlanner::displayTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj)
{
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "map";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub_.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.1){
      Eigen::Vector2d pos = display_traj->at(i).traj.getPos(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp4 = pos+offset4;
      point4.x = tmp4[0]; 
      point4.y = tmp4[1];
      point4.z = 0;

      carMarkers.points.push_back(point1);
      carMarkers.points.push_back(point2);

      carMarkers.points.push_back(point2);
      carMarkers.points.push_back(point3);

      carMarkers.points.push_back(point3);
      carMarkers.points.push_back(point4);

      carMarkers.points.push_back(point4);
      carMarkers.points.push_back(point1);

    }

  }

  wholebody_traj_pub_.publish(carMarkers);



}

bool TrajPlanner::checkCollisionWithObs(const double& t_now)
{

    bool collision = false;

    int segmentId = traj_container_.locateSingulId(t_now);
    double segment_i_start_time = traj_container_.singul_traj[segmentId].start_time;
    double segment_i_end_time = traj_container_.singul_traj[segmentId].end_time;
    int segment_i_singul = traj_container_.singul_traj[segmentId].traj.getDirection();
    if(t_now < segment_i_start_time || t_now > segment_i_end_time)
    {
        return collision;
    }
    else
    {
        double segment_i_duration = segment_i_end_time - segment_i_start_time;
        for(double t = t_now; t < segment_i_end_time - segment_i_duration / 3.0; t += 0.05)
        {
            Eigen::Vector2d pos = traj_container_.singul_traj[segmentId].traj.getPos(t - segment_i_start_time);
            Eigen::Vector2d vel = traj_container_.singul_traj[segmentId].traj.getdSigma(t - segment_i_start_time);
            double yaw = atan2(segment_i_singul * vel(1), segment_i_singul * vel(0));
            Eigen::Vector3d state; state << pos, yaw;
            kino_path_finder_->checkCollisionUsingPosAndYaw(state, collision);
            if(collision)
            {
                return collision;
            }
        }
    }

    return collision;
}

bool TrajPlanner::RunAvoidance()
{

  Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  ROS_WARN("begin to run minco");
  nav_msgs::Path debug_msg0,debug_msg1;
  display_hPolys_.clear();

  double worldtime = start_time_;
  double basetime = 0.0;

  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container, finState_container;
  duration_container.resize(kino_trajs_.size());

  for(unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    double timePerPiece = traj_piece_duration_;
    int segment_idx = i;
    plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
    singul_container.push_back(kino_traj.singul);
    std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
    plan_utils::MinJerkOpt initMJO;
    plan_utils::Trajectory initTraj;
    int piece_nums;
    double initTotalduration = 0.0;
    for(const auto pt : pts)
    {
      initTotalduration += pt[2];
    }
    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
    timePerPiece = initTotalduration / piece_nums;
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums /** 1.2*/;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    double res_time = 0;
    for(int i = 0; i < piece_nums; i++ )
    {
      int resolution;
      if(i==0||i==piece_nums-1)
      {
        resolution = dense_traj_res_;
      }
      else
      {
        resolution = traj_res_;
      }
      for(int k = 0; k <= resolution; k++)
      {
        double t = res_time + 1.0 * k / resolution * ego_piece_dur_vec[i];
        Eigen::Vector3d pos = kino_path_finder_->CalculateInitPos(t, kino_traj.singul);
        statelist.push_back(pos);
        if(k==resolution && i!=piece_nums-1)
        {
          ego_innerPs.col(i) = pos.head(2);
        }
      }
      res_time += ego_piece_dur_vec[i];
    }
    std::cout<<"s: "<<kino_traj.singul<<"\n";
    
    double tm1 = ros::Time::now().toSec();
    auto corridor_t1 = std::chrono::high_resolution_clock::now();

    getRectangleConst(statelist);
    sfc_container.push_back(hPolys_);
    display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    auto corridor_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> corridor_ms = corridor_t2 - corridor_t1;
    cout << "corridor time: " << corridor_ms.count() << " ms" << endl;
    
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(kino_traj.start_state);
    finState_container.push_back(kino_traj.final_state);
    basetime += initTotalduration;

  }
  std::cout<<"try to optimize!\n";

  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container, duration_container,
                                                        sfc_container, singul_container, worldtime, 0.0);
  std::cout<<"optimize ended!\n";

  if (flag_success)
  {
    traj_container_.clearSingul();
    std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
    for(unsigned int i = 0; i < kino_trajs_.size(); i++)
    {
      traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, car_id_); // todo time
      std::cout<<"init duration: "<<duration_container[i]<<std::endl;
      std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
      std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
      std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
      worldtime = traj_container_.singul_traj.back().end_time;
    }
    
    return true;
  }
  else
  {
    ROS_ERROR("[PolyTrajManager] Planning fails! ");
    return false;
  }

}


void TrajPlanner::getRectangleConst(std::vector<Eigen::Vector3d> statelist)
{
    hPolys_.clear();
    double resolution = map_ptr_->getResolution();
    double step = resolution * 1.0;
    double limitBound = 10.0;

    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        distance2center << car_width_ / 2.0, car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0, car_length_ / 2.0 - car_d_cr_;
        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        Eigen::MatrixXd hPoly;
        hPoly.resize(4, 4);
        while(have_stopped_expanding.norm() != 0)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                bool isocc = false;               
                switch(i)
                {
                case 0: // dy
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0) + step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 1: // dx
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, -distance2center(2));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 - car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 2: // -dy
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 3: // -dx
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, distance2center(0));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 + car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                }
            }
        }
        Eigen::Vector2d point1, norm1;
        point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        Eigen::Vector2d point2, norm2;
        point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;

        Eigen::Vector2d point3, norm3;
        point3 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;

        Eigen::Vector2d point4, norm4;
        point4 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;   

        hPolys_.push_back(hPoly);     

    }
}

bool TrajPlanner::computeVel(double& linear_vel, double& angular_vel){
  double time_now = ros::Time::now().toSec();

    int segment_id = traj_container_.locateSingulId(time_now);
    if (segment_id < 0 || segment_id >= traj_container_.singul_traj.size())
    {
        ROS_WARN("No valid trajectory segment found!");
        linear_vel = 0.0;
        angular_vel = 0.0;
        return true;
    }

    auto& segment = traj_container_.singul_traj[segment_id];
    double segment_start_time = segment.start_time;
    double segment_end_time = segment.end_time;
    if (time_now < segment_start_time || time_now > segment_end_time) {
      ROS_WARN("No valid trajectory segment found!");
      linear_vel = 0.0;
      angular_vel = 0.0;
      return true;
    }
           
    linear_vel = segment.traj.getVel(time_now - segment_start_time);
    angular_vel = segment.traj.getCurv(time_now - segment_start_time) * linear_vel;  
    
    //cout << "linear vel: " << linear_vel << "angular_vel" << angular_vel;
    return false;

}
