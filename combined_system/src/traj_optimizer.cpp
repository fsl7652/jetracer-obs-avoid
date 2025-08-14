#include "plan_manage/traj_optimizer.h"
// using namespace std;

namespace plan_manage
{
  bool PolyTrajOptimizer::OptimizeTrajectory(
      const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
      std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
      std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now, double help_eps)
  {
    trajnum = initInnerPts.size();
    epis = help_eps;
    cfgHs_container = hPoly_container;
    iniState_container = iniStates;
    finState_container = finStates;
    singul_container = singuls;
    variable_num_ = 0;
    jerkOpt_container.clear();
    piece_num_container.clear();
    jerkOpt_container.resize(trajnum);
    piece_num_container.resize(trajnum);
    double final_cost;

    if(initTs.size()!=trajnum){
      ROS_ERROR("initTs.size()!=trajnum");
      return false;
    }

    

    for(int i = 0; i < trajnum; i++){
      //check
      if(initInnerPts[i].cols()==0){
        ROS_ERROR("There is only a piece?");
        return false;
      }
      int piece_num_ = initInnerPts[i].cols() + 1;
      piece_num_container[i] = piece_num_;

      if(cfgHs_container[i].size()!=(piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1)){
        std::cout<<"cfgHs size: "<<cfgHs_container[i].size()<<std::endl;
        ROS_ERROR("cfgHs size error!");
        return false;
      }
      for (int k = 0; k < (piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1); k++)
      {
        cfgHs_container[i][k].topRows<2>().colwise().normalize(); // norm vector outside
      }

      //reset the start end max_vel_
      if(iniState_container[i].col(1).norm()>=max_vel_){
        iniState_container[i].col(1) = iniState_container[i].col(1).normalized()*(max_vel_-1.0e-2);
      }
      if(iniState_container[i].col(2).norm()>=max_acc_){
        iniState_container[i].col(2) = iniState_container[i].col(2).normalized()*(max_acc_-1.0e-2);
      }
      if(finState_container[i].col(1).norm()>=max_vel_){
        finState_container[i].col(1) = finState_container[i].col(1).normalized()*(max_vel_-1.0e-2);
      }
      if(finState_container[i].col(2).norm()>=max_acc_){
        finState_container[i].col(2) = finState_container[i].col(2).normalized()*(max_acc_-1.0e-2);
      }
      jerkOpt_container[i].reset(iniState_container[i], finState_container[i], piece_num_);
      variable_num_ += 2 * (piece_num_ - 1);


    }  
    variable_num_ += trajnum;
    

    // ros::Time t0 = ros::Time::now();
    auto t0 = std::chrono::high_resolution_clock::now();
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ, flag_success;
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    for(int i = 0; i<trajnum; i++){
      memcpy(x.data()+offset,initInnerPts[i].data(), initInnerPts[i].size() * sizeof(x[0]));
      offset += initInnerPts[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, initTs.size());
    RealT2VirtualT(initTs, Vt);



    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 64;//128
    lbfgs_params.past = 3; //3 
    lbfgs_params.g_epsilon = 0.0;
    // lbfgs_params.max_linesearch = 200;
    lbfgs_params.min_step = 1.0e-12;
    lbfgs_params.delta = 1.0e-4;
    lbfgs_params.max_iterations = 1000;
    t_now_ = now;



    /* ---------- prepare ---------- */
    iter_num_ = 0;
    flag_force_return = false;
    force_stop_type_ = DONT_STOP;
    flag_still_occ = false;
    flag_success = false;
    /* ---------- optimize ---------- */
    // t1 = ros::Time::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout<<"begin to optimize!\n";
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        NULL,
        this,
        lbfgs_params);
    // t2 = ros::Time::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_ms = t2 - t1;
    std::chrono::duration<double, std::milli> total_time_ms = t2 - t0;
    // double time_ms = (t2 - t1).toSec() * 1000;
    // double total_time_ms = (t2 - t0).toSec() * 1000;
    /* ---------- get result and check collision ---------- */
    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGS_CANCELED ||
        result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
      flag_force_return = false;
      flag_success = true;
      printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms.count(), total_time_ms.count(), final_cost);

    } 
    else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms.count(), total_time_ms.count(), final_cost);
      ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      flag_force_return = false;
      flag_success = true;
    }
    else
    {
      printf("\033[31m[PolyTrajOptimizer]iter=%d,time(ms)=%5.3f, error.\n\033[0m", iter_num_, time_ms.count());
      ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
    }
    return flag_success;
  }


  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
  { 

    double total_smcost = 0.0, total_timecost = 0.0, penalty_cost = 0.0;
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    int offset = 0;
    
    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
    std::vector<Eigen::VectorXd> arrayt_container;
    std::vector<Eigen::VectorXd> arraygradt_container;
    std::vector<Eigen::VectorXd> arraygradT_container;
    std::vector<Eigen::VectorXd> arrayT_container;
    std::vector<double> trajtimes; trajtimes.push_back(0.0);
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(grad.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      offset += 2 * (opt->piece_num_container[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, opt->trajnum);
    Eigen::Map<Eigen::VectorXd>gradt(grad.data()+offset, opt->trajnum);
    Eigen::VectorXd T(opt->trajnum);
    opt->VirtualT2RealT(t, T);
    //T is sum time

    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      Eigen::VectorXd arraygradT(opt->piece_num_container[trajid]);
      Eigen::VectorXd arrayT(opt->piece_num_container[trajid]);
      arrayT.setConstant(T[trajid]/opt->piece_num_container[trajid]);
      arraygradT.setZero();
      arrayT_container.push_back(arrayT);
      arraygradT_container.push_back(arraygradT);
      trajtimes.push_back(T[trajid]);
    }


    if(T.sum() > 1000 || T.sum() < 0.1)
    {
      for(int trajid = 0; trajid < opt->trajnum; trajid++)
      {
        gradP_container[trajid].setZero();
      }
      // gradP.setZero();
      gradt.setZero();
      return 999999.0;
    }
          

    double smoothness_cost, time_of_cost, collision_penalty, dynamic_penalty, feasibility_penalty;

    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double smoo_cost = 0;
      Eigen::VectorXd obs_surround_feas_qvar_costs(3);
      obs_surround_feas_qvar_costs.setZero();
      opt->jerkOpt_container[trajid].generate(P_container[trajid], arrayT_container[trajid]);
      opt->initAndGetSmoothnessGradCost2PT(arraygradT_container[trajid], smoo_cost, trajid); // Smoothness cost   
      opt->addPVAGradCost2CT(arraygradT_container, obs_surround_feas_qvar_costs, trajid, trajtimes[trajid]); 
      total_smcost += smoo_cost;
      penalty_cost += obs_surround_feas_qvar_costs.sum();
      
      
      smoothness_cost = total_smcost;
      collision_penalty = obs_surround_feas_qvar_costs(0);
      dynamic_penalty = obs_surround_feas_qvar_costs(1);
      feasibility_penalty = obs_surround_feas_qvar_costs(2);
    }
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double time_cost = 0.0;
      opt->jerkOpt_container[trajid].getGrad2TP(arraygradT_container[trajid], gradP_container[trajid]); 
      double gradsumT,gradsumt;
      gradsumT = arraygradT_container[trajid].sum() / arraygradT_container[trajid].size();
      opt->VirtualTGradCost(T[trajid],t[trajid],gradsumT,gradsumt,time_cost);
      gradt[trajid] = gradsumt;
      total_timecost += time_cost;

      time_of_cost = total_timecost;
    }

    opt->iter_num_ += 1;
    return total_smcost + total_timecost + penalty_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }
  void PolyTrajOptimizer::VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double& costT){
    double gdVT2Rt;
    if (VT > 0)
    {
      gdVT2Rt = VT + 1.0;
    }
    else
    {
      double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
      gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
    }

    gdVT = (gdRT + wei_time_) * gdVT2Rt;
    costT = RT * wei_time_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost, int trajid)
  {
    jerkOpt_container[trajid].initGradCost(gdT, cost);
  }

  void PolyTrajOptimizer::addPVAGradCost2CT(std::vector<Eigen::VectorXd>  &gdTs, Eigen::VectorXd &costs,  const int trajid, const double trajtime)
  {
    int M = gdTs[trajid].size();                         // number of pieces
    double T = jerkOpt_container[trajid].get_T1().sum(); // total duration of the trajectory
    double delta_T = T / M;                              // time duration of one piece
    int K = std::floor(T / delta_t_);                    // number of constrain points
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
    double z_h0, z_h1, z_h2, z_h3, z_h4;    
    double vel2_reci, vel2_reci_e, vel3_2_reci_e, acc2, cur2, cur;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;

    Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaKc, gradViolaKLc, gradViolaKRc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaKt, gradViolaKLt, gradViolaKRt;
    double violaPos, violaVel, violaAcc, violaCur, violaCurL, violaCurR, violaDynamicObs;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaCurPenaD, violaCurPenaDL, violaCurPenaDR, violaDynamicObsPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaCurPena, violaCurPenaL, violaCurPenaR, violaDynamicObsPena;

    // std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];
    int singul_ = singul_container[trajid];

    double omg_j;
    costs.setZero();
    Eigen::Matrix2d ego_R, help_R;


    for(int j = 0; j <= K + 1; j++) // iterate over all constrain points
    {
      double constrain_pt_t = (j == K + 1) ? T : j * delta_t_;

      //--- locate piece id: i ---
      int i = 0;
      for(i = 0; i < M; i++)
      {
        if(constrain_pt_t > delta_T * i - 1e-3 && constrain_pt_t < delta_T * (i + 1) + 1e-3)
          break;
      }
      int time_int_pena = (j == K + 1) ? 1 : (-i);
      // int time_int_pena = -i;

      double t_bar = constrain_pt_t - i * delta_T;
      const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].get_b().block<6, 2>(i * 6, 0);
      
      s1 = t_bar;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;

      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;

      sigma = c.transpose() * beta0;
      dsigma = c.transpose() * beta1;
      ddsigma = c.transpose() * beta2;
      dddsigma = c.transpose() * beta3;
      ddddsigma = c.transpose() * beta4;
      
      if(j == 0)
        omg_j = 0.5;
      else if(j == K)
      {
        omg_j = 0.5 * (T / delta_t_ - K + 1);
      }
      else if(j == K + 1)
      {
        omg_j = 0.5 * (T / delta_t_ - K);
      }
      else
        omg_j = 1.0;

      z_h0 = dsigma.norm();
      z_h1 = ddsigma.transpose() * dsigma;
      z_h2 = dddsigma.transpose() * dsigma;
      z_h3 = ddsigma.transpose() * B_h * dsigma;

      if ( z_h0 < 1e-4 )
      {
        continue;
      }

      vel2_reci = 1.0 / (z_h0 * z_h0);
      vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
      vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
      z_h0 = 1.0 / z_h0;
      z_h4 = z_h1 * vel2_reci;
      
      acc2 = z_h1 * z_h1 * vel2_reci;
      cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
      cur = z_h3 * vel3_2_reci_e;

      violaCur = cur2 - max_cur_ * max_cur_;
      violaCurL = cur-max_cur_;
      violaCurR = -cur-max_cur_;

      ego_R << dsigma(0), -dsigma(1),
               dsigma(1), dsigma(0);
      ego_R = singul_ * ego_R * z_h0;

      Eigen::Matrix2d temp_a, temp_v;
      temp_a << ddsigma(0), -ddsigma(1),
                ddsigma(1), ddsigma(0);
      temp_v << dsigma(0), -dsigma(1),
                dsigma(1), dsigma(0);
      Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

      violaVel = 1.0 / vel2_reci - max_vel_ * max_vel_;
      violaAcc = acc2 - max_acc_ * max_acc_;

      if (violaVel > 0.0)
      {
        positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);

        gradViolaVc = 2.0 * beta1 * dsigma.transpose(); // 6*2
        gradViolaVt = 2.0 * z_h1;                       // 1*1
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * violaVelPenaD * gradViolaVc;
        gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * violaVelPenaD * gradViolaVt * /*(-i)*/time_int_pena;
        costs(2) += delta_t_ * omg_j * wei_feas_ * violaVelPena;

      }
        
      if (violaAcc > 0.0)
      {
        positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);

        gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() - z_h4 * z_h4 * dsigma.transpose()) +
                      2.0 * beta2 * z_h4 * dsigma.transpose(); // 6*2
        gradViolaAt = 2.0 * (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += delta_t_ * omg_j * wei_feas_ * violaAccPenaD * gradViolaAc;
        gdTs[trajid](i) += delta_t_ * omg_j * wei_feas_ * violaAccPenaD * gradViolaAt * /*(-i)*/time_int_pena;
        costs(2) += delta_t_ * omg_j * wei_feas_ * violaAccPena;
      }              


    }
    int N = gdTs[trajid].size();
    Eigen::Vector2d outerNormal;

    double step, alpha;

    double approxcur2, approxviolaCur,approxviolaCurPenaD,approxviolaCurPena;
    Eigen::Matrix<double, 6, 2> gradapproxViolaKc;
    double gradapproxViolaKt;
    
    std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];

    double omg;
    int i_dp = 0;
    double t = 0;

    int pointid = -1;


    for (int i = 0; i < N; ++i)
    {
      int K;
      if(i==0 || i==N-1){
        K = destraj_resolution_;
      }
      else{
        K = traj_resolution_;
      }
      const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].get_b().block<6, 2>(i * 6, 0);
      step = jerkOpt_container[trajid].get_T1()(i) / K; // T_i /k
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
        alpha = 1.0 / K * j;
        
        //update s1 for the next iteration
        s1 += step;
        pointid++;

        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        ddddsigma = c.transpose() * beta4;
         
        // ctrl_points_.col(i_dp) = sigma;
        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        // some help values
        
        z_h0 = dsigma.norm();
        z_h1 = ddsigma.transpose() * dsigma;
        z_h2 = dddsigma.transpose() * dsigma;
        z_h3 = ddsigma.transpose() * B_h * dsigma;
       
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }


        // add cost z_h0 = ||v||
        if ( z_h0 < 1e-4 || (j==0&&i==0) || (i==N-1&&j==K))
        {
          continue;
        }
        //avoid siguality

        vel2_reci = 1.0 / (z_h0 * z_h0);
        vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
        vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
        z_h0 = 1.0 / z_h0;

        z_h4 = z_h1 * vel2_reci;
        violaVel = 1.0 / vel2_reci - max_vel_ * max_vel_;
        acc2 = z_h1 * z_h1 * vel2_reci;
        cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
        cur = z_h3 * vel3_2_reci_e;
        violaAcc = acc2 - max_acc_ * max_acc_;

        violaCur = cur2 - max_cur_ * max_cur_;
        violaCurL = cur-max_cur_;
        violaCurR = -cur-max_cur_;

        ego_R << dsigma(0), -dsigma(1),
                 dsigma(1), dsigma(0);
        ego_R = singul_ * ego_R * z_h0;

        Eigen::Matrix2d temp_a, temp_v;
        temp_a << ddsigma(0), -ddsigma(1),
                  ddsigma(1), ddsigma(0);
        temp_v << dsigma(0), -dsigma(1),
                  dsigma(1), dsigma(0);
        Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

        for(auto le : vec_le_)
        {
          Eigen::Vector2d bpt = sigma + ego_R * le;

          Eigen::Matrix2d temp_l_Bl;
          temp_l_Bl << le(0), -le(1),
                       le(1), le(0);          

          int corr_k = cfgHs[pointid].cols();

          for(int k = 0; k < corr_k; k++)
          {
            outerNormal = cfgHs[pointid].col(k).head<2>();
            violaPos = outerNormal.dot(bpt - cfgHs[pointid].col(k).tail<2>());

            if(violaPos > 0)
            {
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
              
              gradViolaPc = beta0 * outerNormal.transpose() + 
                            beta1 * outerNormal.transpose() * (singul_ * temp_l_Bl * z_h0 - ego_R * le * dsigma.transpose() * vel2_reci);
              
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + R_dot * le);

              jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_obs_ * violaPosPenaD * gradViolaPc;
              gdTs[trajid](i) += omg * wei_obs_ * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

              costs(0) += omg * step * wei_obs_ * violaPosPena; 
            }
          }
        }
        if(violaCurL > 0.0){
          positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
          //@hzc
          gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
          gradViolaKLt  = alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDL * gradViolaKLc;
          gdTs[trajid](i) += omg * wei_feas_ * 10.0 * (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaL;
        }
        if(violaCurR > 0.0){
          positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
          //@hzc
          gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
          gradViolaKRt  = -(alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1));
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDR * gradViolaKRc;
          gdTs[trajid](i) += omg * wei_feas_ * 10.0 * (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaR;
        }
      }
      t += jerkOpt_container[trajid].get_T1()(i);
    }

  }

  void PolyTrajOptimizer::positiveSmoothedL1(const double &x, double &f, double &df)
  {
        const double pe = 1.0e-4;
        const double half = 0.5 * pe;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;
        const double d2c = 3.0 * f3c;
        const double d3c = 4.0 * f4c;

        if (x < pe)
        {
            f = (f4c * x + f3c) * x * x * x;
            df = (d3c * x + d2c) * x * x;
        }
        else
        {
            f = x - half;
            df = 1.0;
        }
        return;
  }
  void PolyTrajOptimizer::positiveSmoothedL3(const double &x, double &f, double &df){
    df = x * x;
    f = df *x;
    df *= 3.0;
   

    return ;
  }

  void PolyTrajOptimizer::init(ros::NodeHandle& nh)
  {
    nh_ = nh;
    nh_.param("optimizing/traj_resolution", traj_resolution_, 8);
    nh_.param("optimizing/des_traj_resolution", destraj_resolution_, 20);
    nh_.param("optimizing/wei_sta_obs", wei_obs_, 10000.0);
    nh_.param("optimizing/wei_feas", wei_feas_, 2000.0);
    nh_.param("optimizing/wei_sqrvar", wei_sqrvar_, 500.0);
    nh_.param("optimizing/wei_time", wei_time_, 1000.0);
    nh_.param("optimizing/max_vel", max_vel_, 0.25);
    nh_.param("optimizing/min_vel", min_vel_,  -0.25);
    nh_.param("optimizing/max_acc", max_acc_, 0.15);
    nh_.param("optimizing/min_acc", min_acc_, -0.15);
    nh_.param("optimizing/max_cur", max_cur_, 0.2);
    nh_.param("optimizing/half_margin", half_margin, 0.15);

    nh_.param("vehicle/cars_num", cars_num_, 1);
    nh_.param("vehicle/car_id", car_id_, 0);
    nh_.param("vehicle/car_length", car_length_, 0.248);
    nh_.param("vehicle/car_width", car_width_, 0.192);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 0.1);
    nh_.param("vehicle/wheelbase", car_wheelbase_, 0.148);

      B_h << 0, -1,
             1, 0;
      delta_t_ = 0.2;
      ifdynamic_ = false;

      car_width_ += 2 * half_margin;
      car_length_ += 2 * half_margin;

      double half_wid = 0.5 * car_width_;
      double half_len = 0.5 * car_length_;

      lz_set_.push_back(Eigen::Vector2d(  half_len,  half_wid));
      lz_set_.push_back(Eigen::Vector2d(- half_len,  half_wid));
      lz_set_.push_back(Eigen::Vector2d(  half_len, -half_wid));
      lz_set_.push_back(Eigen::Vector2d(- half_len, -half_wid));
    
      Eigen::Vector2d le_1, le_2, le_3, le_4;        // vertexs of the ego car in the body frame
      vec_le_.clear(); vec_lo_.clear();
      le_1 << car_d_cr_ + car_length_ / 2.0, car_width_ / 2.0;
      le_2 << car_d_cr_ + car_length_ / 2.0, -car_width_ / 2.0;
      le_3 << car_d_cr_ - car_length_ / 2.0, -car_width_ / 2.0;
      le_4 << car_d_cr_ - car_length_ / 2.0, car_width_ / 2.0;
      vec_le_.push_back(le_1); vec_le_.push_back(le_2); vec_le_.push_back(le_3); vec_le_.push_back(le_4); 
      vec_le_.push_back(le_1); 
 


  }
}