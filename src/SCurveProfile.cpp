#include <cart_opt_ctrl/SCurveProfile.hpp>
#include <iostream>

SCurveProfile::SCurveProfile ( double s_init, double vi_init, double a_init, double s_final, double v_final, double a_final, double v_max, double a_max, double j_max ) {
  si_ = s_init;
  sf_ = s_final;
  vi_ = vi_init;
  vf_ = v_final;
  ai_ = a_init;
  af_ = a_final;
  v_max_ = v_max;
  a_max_ = a_max;
  j_max_ = j_max;
  period_ = 0.001;
}

SCurveProfile::SCurveProfile() {
  SCurveProfile(0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 2.0, 5.0);
}

void SCurveProfile::config (double s_init, double vi_init, double a_init, double s_final, double v_final, double a_final, double v_max, double a_max, double j_max) {
  si_ = s_init;
  sf_ = s_final;
  vi_ = vi_init;
  vf_ = v_final;
  ai_ = a_init;
  af_ = a_final;
  
  // Max vel acc and jerk need to be positive
  v_max_ = std::abs(v_max);
  a_max_ = std::abs(a_max);
  j_max_ = std::abs(j_max);
  
  // vf cannot be over vmax or below -vmax
  if (vf_>0)
    vf_ = std::min(vf_, v_max_);
  else
    vf_ = std::max(vf_, -v_max_);
  
  // af cannot be over amax or below -amax
  if (af_>0){
    if (vf_>0)
      af_ = std::min(af_, a_max_);
    else
      af_ = std::min(af_, std::min(a_max_, std::sqrt((v_max_+vf_)*2*j_max_)));
  }
  else
    af_ = std::max(af_, -a_max_);
}
void SCurveProfile::set_s_init ( double s_init ) {
  si_ = s_init;
}

void SCurveProfile::set_v_init ( double v_init ) {
  vi_ = v_init;
}

void SCurveProfile::set_a_init ( double a_init ) {
  ai_ = a_init;
}

void SCurveProfile::set_s_final ( double s_final ) {
  // S final cannot be below S init
  sf_ = std::max(si_, s_final);
}

void SCurveProfile::set_v_final ( double v_final ) {
  // vf cannot be over vmax or below -vmax
  if (v_final>0)
    vf_ = std::min(v_final, v_max_);
  else
    vf_ = std::max(v_final, -v_max_);
  if (af_>0){
    if (vf_>0)
      af_ = std::min(af_, a_max_);
    else
      af_ = std::min(af_, std::min(a_max_, std::sqrt((v_max_+vf_)*2*j_max_) ) );
  }
  else
    af_ = std::max(af_, std::max(-std::sqrt((v_max_-vf_)*2*j_max_), -a_max_));
}

void SCurveProfile::set_a_final ( double a_final ) {
  // af cannot be over amax or below -amax
  if (a_final>0){
    if (vf_>0)
      af_ = std::min(a_final, a_max_);
    else
      af_ = std::min(a_final, std::min(a_max_, std::sqrt((v_max_+vf_)*2*j_max_) ) );
  }
  else
    af_ = std::max(a_final, std::max(-std::sqrt((v_max_-vf_)*2*j_max_), -a_max_));
}

void SCurveProfile::set_v_max ( double v_max ) {
  v_max_ = std::abs(v_max);
  if (vf_>0)
    vf_ = std::min(vf_, v_max_);
  else
    vf_ = std::max(vf_, -v_max_);
  if (af_>0){
    if (vf_>0)
      af_ = std::min(af_, a_max_);
    else
      af_ = std::min(af_, std::min(a_max_, std::sqrt((v_max_+vf_)*2*j_max_) ) );
  }
  else
    af_ = std::max(af_, std::max(-std::sqrt((v_max_-vf_)*2*j_max_), -a_max_));
}

void SCurveProfile::set_a_max ( double a_max ) {
  a_max_ = std::abs(a_max);
  if (af_>0){
    if (vf_>0)
      af_ = std::min(af_, a_max_);
    else
      af_ = std::min(af_, std::min(a_max_, std::sqrt((v_max_+vf_)*2*j_max_)));
  }
  else
    af_ = std::max(af_, std::max(-std::sqrt((v_max_-vf_)*2*j_max_), -a_max_));
}

void SCurveProfile::set_j_max ( double j_max ) {
  j_max_ = std::abs(j_max);
}

void SCurveProfile::set_period ( double period ) {
  period_ = period;
}

void SCurveProfile::compute_curves(){
  // Reset vector with initial conditions
  j_vect_.clear();
  j_vect_.push_back(0.0);
  a_vect_.clear();
  a_vect_.push_back(ai_);
  v_vect_.clear();
  v_vect_.push_back(vi_);
  s_vect_.clear();
  s_vect_.push_back(si_);
  t_vect_.clear();
  t_vect_.push_back(0.0);
  

  
  if(ai_ > a_max_)
    while (a_vect_[a_vect_.size()-1]-j_max_*period_>a_max_)
//     if(a_vect_[a_vect_.size()-1]-j_max_*period_>a_max_)
      compute_next_step(-j_max_);
    
  if(ai_ < -a_max_)
    while (a_vect_[a_vect_.size()-1]+j_max_*period_<-a_max_)
//     if(a_vect_[a_vect_.size()-1]+j_max_*period_<-a_max_)
      compute_next_step(j_max_);
    

  
  double distance_left = sf_-si_;
  compute_breaking();
  
  // If target cannot be reach, break hard
  if(break_dist_ > distance_left){
    vf_ = 0;
    af_ = 0;
    compute_breaking();
  }
  
  bool started_breaking = false;
  bool too_fast_on_start = ((vi_ + ai_*ai_/(2*j_max_) > v_max_) && (ai_>=0)) ||((vi_ - ai_*ai_/(2*j_max_) >v_max_) && (ai_<0));
  bool on_cruise = (vi_ == v_max_) && (ai_ == 0);
  
  while((break_dist_ <= distance_left) && (break_time_>=0)){
//   if((break_dist_ <= distance_left) && (break_time_>=0)){
    // If v=vmax and a=0, keep going
    if (on_cruise)
      compute_cruise_step();
    else{
      // if vi > vmax
      if(too_fast_on_start){
        if(started_breaking){
          // a almost 0
          if(a_vect_[a_vect_.size()-1]+j_max_*period_>0){
            compute_next_step(0);
            on_cruise = true;
          }
          else
            compute_next_step(j_max_);
        }else{
          if((v_vect_[v_vect_.size()-1] - a_vect_[a_vect_.size()-1]*a_vect_[a_vect_.size()-1]/(2*j_max_) <= v_max_ ) && (a_vect_[a_vect_.size()-1]<0) ){
            compute_next_step(j_max_);
            started_breaking = true;
          }
          else{
            if(a_vect_[a_vect_.size()-1] > -a_max_)
              compute_next_step(-j_max_);
            else
              compute_next_step(0);
          }
        }
      }
      // vi <vmax
      else{
        // breaking to reach vmax
        if(started_breaking){
          if (a_vect_[a_vect_.size()-1]-j_max_*period_ < 0){
            compute_next_step(0);
            on_cruise = true;
          }
          else
            compute_next_step(-j_max_);
        }
        else{
          if((v_vect_[v_vect_.size()-1] + a_vect_[a_vect_.size()-1]*a_vect_[a_vect_.size()-1]/(2*j_max_) >= v_max_) && (a_vect_[a_vect_.size()-1]>0) ){
            compute_next_step(-j_max_);
            started_breaking = true;
          }
          else{
            if (a_vect_[a_vect_.size()-1] < a_max_)
              compute_next_step(j_max_);
            else
              compute_next_step(0);
          }
        }
      }
    }
    compute_breaking();
    distance_left = sf_ - s_vect_[s_vect_.size()-1];
  }
  
  compute_breaking();
  double t_final = t_vect_[t_vect_.size()-1] + break_time_;
  
  // true when starting last concave phase
  started_breaking = false;
  
  while(t_vect_[t_vect_.size()-1] <=t_final){
//   if(t_vect_[t_vect_.size()-1] <=t_final){
    // Stop when goal is reached
    if (started_breaking && ((a_vect_[a_vect_.size()-1] < af_ && a_vect_[a_vect_.size()-2] > af_) ||(a_vect_[a_vect_.size()-1] > af_ && a_vect_[a_vect_.size()-2] < af_))){
      compute_next_step(0);
//       break;
    }
    if ( (vf_ >= v_vect_[v_vect_.size()-1] + (af_*af_-a_vect_[a_vect_.size()-1]*a_vect_[a_vect_.size()-1])/(2*j_max_) && (a_vect_[a_vect_.size()-1]<0)) 
      || (vf_ >= v_vect_[v_vect_.size()-1] + (-af_*af_+a_vect_[a_vect_.size()-1]*a_vect_[a_vect_.size()-1])/(2*j_max_) && (a_vect_[a_vect_.size()-1]>0))){
      compute_next_step(j_max_);
      started_breaking = true;
    }
    else{
      if (a_vect_[a_vect_.size()-1] > -a_max_)
        compute_next_step(-j_max_);
      else
        compute_next_step(0);
    }
  }
}

void SCurveProfile::compute_breaking(){
  double ac = a_vect_[a_vect_.size()-1];
  double vc = v_vect_[v_vect_.size()-1];
  
  double t_ramp_fall = compute_ramp_fall_time(vc, vf_, ac, -a_max_, af_);
  double t_convexe_fall = compute_convexe_time(ac, -a_max_);
  double t_concave_fall = compute_concave_time(-a_max_,af_);
  
  if (t_ramp_fall <0){
    t_ramp_fall = 0;
    
    if(4*j_max_*(vc-vf_)+2*af_*af_+2*ac*ac >0){
      t_convexe_fall = ac/j_max_ + sqrt(4*j_max_*(vc-vf_)+2*af_*af_+2*ac*ac)/(2*j_max_);
      t_concave_fall = (af_ -ac)/j_max_ + t_convexe_fall;
    }
    else{
      t_concave_fall = 0;
      t_convexe_fall = 0;
    }
    
    if(t_concave_fall<=0)
      t_concave_fall = 0;
  
    if(t_convexe_fall<0)
      t_convexe_fall = 0;
    
    break_time_ = t_convexe_fall+t_concave_fall;
    break_dist_ = compute_phase_distance(t_convexe_fall, -j_max_, ac, vc) + compute_phase_distance(t_concave_fall, j_max_, ac - j_max_*t_convexe_fall, vc + ac*t_convexe_fall -j_max_/2 *t_convexe_fall*t_convexe_fall); 
  }
  else{
    break_time_ = t_convexe_fall+t_ramp_fall+t_concave_fall;
    break_dist_ = compute_convexe_distance(ac, vc, -a_max_) + compute_ramp_fall_distance(vc+ac*t_convexe_fall-j_max_/2*t_convexe_fall*t_convexe_fall,vc,vf_,ac,-a_max_,af_) 
      + compute_concave_distance(-a_max_,vc+ac*t_convexe_fall-j_max_/2*t_convexe_fall*t_convexe_fall-t_ramp_fall*a_max_, af_) ;
  }
}

void SCurveProfile::compute_next_step(double j){
  double last_a = a_vect_[a_vect_.size()-1];
  double last_v = v_vect_[v_vect_.size()-1];
  double last_s = s_vect_[s_vect_.size()-1];
  double last_t = t_vect_[t_vect_.size()-1];
  
  j_vect_.push_back(j);
  a_vect_.push_back(last_a + j*period_);
  v_vect_.push_back(last_v + last_a*period_ + j/2.0 * period_*period_);
  s_vect_.push_back(last_s + last_v*period_ + last_a/2.0*period_*period_ + j/6.0*period_*period_*period_);
  t_vect_.push_back(last_t + period_);
}

void SCurveProfile::compute_cruise_step(){
  double last_a = a_vect_[a_vect_.size()-1];
  double last_v = v_vect_[v_vect_.size()-1];
  double last_s = s_vect_[s_vect_.size()-1];
  double last_t = t_vect_[t_vect_.size()-1];
  
  j_vect_.push_back(0);
  a_vect_.push_back(0);
  v_vect_.push_back(last_v);
  s_vect_.push_back(last_s + last_v*period_);
  t_vect_.push_back(last_t + period_);
}

double SCurveProfile::compute_phase_distance(double time_in_phase, double j_phase, double phase_acc_start, double phase_vel_start){
  return phase_vel_start*time_in_phase+phase_acc_start/2.0*time_in_phase*time_in_phase+j_phase/6.0*time_in_phase*time_in_phase*time_in_phase;
}

double SCurveProfile::compute_concave_distance(double phase_acc_start, double phase_vel_start, double phase_acc_final){
  return compute_phase_distance(compute_concave_time(phase_acc_start, phase_acc_final), j_max_, phase_acc_start, phase_vel_start);
}

double SCurveProfile::compute_concave_time(double phase_acc_start, double phase_acc_final){
  return (phase_acc_final-phase_acc_start)/j_max_;
}

double SCurveProfile::compute_convexe_distance(double phase_acc_start, double phase_vel_start, double phase_acc_final){
  return compute_phase_distance(compute_convexe_time(phase_acc_start, phase_acc_final), -j_max_, phase_acc_start, phase_vel_start);
}

double SCurveProfile::compute_convexe_time(double phase_acc_start, double phase_acc_final){
  return (phase_acc_start-phase_acc_final)/j_max_;
}

double SCurveProfile::compute_ramp_rise_distance(double phase_vel_start, double rise_vel_start, double rise_vel_final, double rise_acc_start, double rise_acc, double rise_acc_final){
  return compute_phase_distance(compute_ramp_rise_time(rise_vel_start, rise_vel_final, rise_acc_start, rise_acc, rise_acc_final), 0 , rise_acc, phase_vel_start);
}

double SCurveProfile::compute_ramp_rise_time(double rise_vel_start, double rise_vel_final, double rise_acc_start, double rise_acc, double rise_acc_final ){
  return ((rise_vel_final-rise_vel_start)-(rise_acc*rise_acc-rise_acc_start*rise_acc_start/2-rise_acc_final*rise_acc_final/2.0)/j_max_)/rise_acc;
}

double SCurveProfile::compute_ramp_fall_distance(double phase_vel_start, double fall_vel_start, double fall_vel_final, double fall_acc_start, double fall_acc, double fall_acc_final){
  return compute_phase_distance(compute_ramp_fall_time(fall_vel_start, fall_vel_final, fall_acc_start, fall_acc, fall_acc_final), 0 , fall_acc, phase_vel_start);
}

double SCurveProfile::compute_ramp_fall_time(double fall_vel_start, double fall_vel_final ,double fall_acc_start, double fall_acc, double fall_acc_final ){
  return ((fall_vel_start-fall_vel_final)-(fall_acc*fall_acc-fall_acc_start*fall_acc_start/2-fall_acc_final*fall_acc_final/2)/j_max_)/(-fall_acc);
}

double SCurveProfile::compute_cruise_distance(double cruise_vel, double phase_pos_start, double phase_pos_final){
  return compute_phase_distance(compute_cruise_time(cruise_vel, phase_pos_start, phase_pos_final), 0, 0, cruise_vel);
}

double SCurveProfile::compute_cruise_time(double cruise_vel, double phase_pos_start, double phase_pos_final){
  return (phase_pos_final-phase_pos_start)/cruise_vel;
}
