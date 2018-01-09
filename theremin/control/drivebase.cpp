#include "drivebase.h"
#include <cmath>

//
// Constants assocaited with this physical drive base
//

//
// The circumference of the wheel in inches
//
const double Drivebase::WHEEL_CIRCUMFERENCE=Robot_constants::DRIVE_WHEEL_DIAMETER*PI;

//
// The number of inches per encoder tick
//
const double Drivebase::INCHES_PER_TICK=WHEEL_CIRCUMFERENCE/(double)Drivebase::TICKS_PER_REVOLUTION;

//
// The motors for the robot
//
const std::vector<int> Drivebase::LeftMotors = { 0, 1 } ;
const std::vector<int> Drivebase::RightMotors = { 2, 3 } ;

//
// The PDB locations for the robot
//
const std::vector<int> Drivebase::LeftPDPChannels = { 0, 1 } ;
const std::vector<int> Drivebase::RightPDPChannels = { 2, 13 } ;

//
// The maximum output voltage while doing automated driving
//
const double Drivebase::MAX_OUT = 0.6 ;

//
// The maximum we will change the motor voltage in one step
//
const double Drivebase::MAX_STEP = 0.2 ;

//
// The time to ramp from stopped to MAX_OUT
//
const double Drivebase::RAMP_UP_TIME_MS = 2000 ;

//
// The time to ramp from MAX_OUT to stopped
//
const double Drivebase::SLOW_WITHIN_DISTANCE = 60 ;

//
// For a given motor index, and its side, return the PDB current
// channel associated with the motor.
//
size_t Drivebase::pdb_location(Drivebase::Side s, size_t which)
{
  size_t ret = 0xffffffff ;
  
  if (s == Drivebase::Side::Left) {
    if (which < LeftPDPChannels.size())
      ret = LeftPDPChannels[which] ;
  }
  else if (s == Drivebase::Side::Right) {
    if (which < RightPDPChannels.size())
      ret = RightPDPChannels[which] ;
  }

  return ret ;
}

size_t Drivebase::motor_location(Drivebase::Side s, size_t which)
{
  size_t ret = 0xffffffff ;
  
  if (s == Drivebase::Side::Left) {
    if (which < LeftMotors.size())
      ret = LeftMotors[which] ;
  }
  else if (s == Drivebase::Side::Right) {
    if (which < RightMotors.size())
      ret = RightMotors[which] ;
  }

  return ret ;
}

int Drivebase::encoderconv(Maybe_inline<Encoder_output> encoder)
{
  if (encoder)
    return *encoder ;

  return 0 ;
}

double Drivebase::ticks_to_inches(int ticks)
{
  return ticks * INCHES_PER_TICK ;
}

Drivebase::Distances Drivebase::ticks_to_inches(const Drivebase::Encoder_ticks &ticks)
{
  double l = ticks_to_inches(ticks.l) ;
  double r = ticks_to_inches(ticks.r) ;

  Distances d(l, r) ;
  return d ;
}

int Drivebase::inches_to_ticks(double inches)
{
  return (int)(inches / INCHES_PER_TICK) ;
}

Robot_inputs Drivebase::Input_reader::operator()(Robot_inputs all, Input in) const
{
  for(size_t i = 0 ; i < LeftMotors.size() ; i++) {
    size_t pdbloc = pdb_location(Side::Left, i) ;
    all.current[pdbloc] = in.current[i] ;
  }

  for(size_t i = 0 ; i < RightMotors.size() ; i++) {
    size_t pdbloc = pdb_location(Side::Right, i) ;
    all.current[pdbloc] = in.current[i + NUMBER_MOTORS] ;
  }

  all.digital_io.in[LEFT_ENCODER_FIRST] = in.left.first ;
  all.digital_io.in[LEFT_ENCODER_SECOND] = in.left.second ;
  all.digital_io.in[RIGHT_ENCODER_FIRST] = in.right.first ;
  all.digital_io.in[RIGHT_ENCODER_SECOND] = in.right.second ;
  all.digital_io.encoder[LEFT_ENCODER] = -inches_to_ticks(in.distances.l) ;
  all.digital_io.encoder[RIGHT_ENCODER] = inches_to_ticks(in.distances.r) ;

  all.navx.angle = in.angle ;

  return all ;
}

Drivebase::Input Drivebase::Input_reader::operator()(const Robot_inputs &in) const
{
  Input ret ;

  for(size_t i = 0 ; i < LeftMotors.size() ; i++) {
    size_t pdbloc = pdb_location(Side::Left, i) ;
    ret.current[i] = in.current[pdbloc] ;
  }

  for(size_t i = 0 ; i < RightMotors.size() ; i++) {
    size_t pdbloc = pdb_location(Side::Right, i) ;
    ret.current[i + NUMBER_MOTORS] = in.current[pdbloc] ;
  }

  ret.left = std::make_pair(in.digital_io.in[LEFT_ENCODER_FIRST], in.digital_io.in[LEFT_ENCODER_SECOND]) ;
  ret.right = std::make_pair(in.digital_io.in[RIGHT_ENCODER_FIRST], in.digital_io.in[RIGHT_ENCODER_SECOND]) ;

  ret.distances.l = -ticks_to_inches(encoderconv(in.digital_io.encoder[LEFT_ENCODER])) ;
  ret.distances.r = ticks_to_inches(encoderconv(in.digital_io.encoder[RIGHT_ENCODER])) ;

  ret.angle = in.navx.angle ;

  return ret ;
}

Drivebase::Goal Drivebase::Goal::distances(const Distances &dist)
{
  Goal a ;
  a.mode_ = Goal::Mode::DISTANCES ;
  a.distances_ = dist ;
  return a ;
}

Drivebase::Goal Drivebase::Goal::absolute(double l, double r)
{
  Goal a ;
  a.mode_ = Goal::Mode::ABSOLUTE ;
  a.left_ = l ;
  a.right_ = r ;
  return a ;
}

Drivebase::Goal Drivebase::Goal::drive_straight(const Distances &target, double init_angle, double init_angle_i)
{
  Goal a ;
  a.mode_ = Goal::Mode::DRIVE_STRAIGHT ;
  a.distances_ = target ;
  a.angle_ = init_angle ;
  a.angle_i_ = init_angle_i ;
  return a ;
}

Drivebase::Goal Drivebase::Goal::rotate(Rad angle)
{
  Goal a ;
  a.mode_ = Goal::Mode::ROTATE ;
  a.angle_ = angle ;
  return a ;
}

void Drivebase::Estimator::update(Time now, Drivebase::Input in, Drivebase::Output out)
{
  last.dt = now - last.now ;
  last.now = now ;
  last.last_output = out ;
  last.prev_angle = last.angle ;
  last.angle = in.angle ;
  last.distances = in.distances ;

  //
  // TODO: see how the speed measurements are being used.  This does not looks like it
  //       may have problems.
  //

  for(size_t i = 0 ; i < LeftMotors.size() ; i++) {
    double current = in.current[i] ;
    motor_check[i].update(now, current, out.l) ;
  }

  for(size_t i = 0 ; i < RightMotors.size() ; i++) {
    double current = in.current[i + NUMBER_MOTORS] ;
    motor_check[i].update(now, current, out.r) ;
  }

  double average_speed = (std::fabs(last.speeds.l) + std::fabs(last.speeds.r)) / 2.0 ;
  double average_current = mean(in.current) ;
  stall_monitor.update(average_current, average_speed) ;
  
  if (stall_monitor.get())
    last.stall = *(stall_monitor.get()) ;
}

Robot_outputs Drivebase::Output_applicator::operator()(Robot_outputs robot, Drivebase::Output b) const
{
  for(size_t i = 0 ; i < LeftMotors.size() ; i++) {
    size_t loc = motor_location(Side::Left, i) ;
    robot.talon_srx[loc].power_level = b.l ;
  }
  
  for(size_t i = 0 ; i < RightMotors.size() ; i++) {
    size_t loc = motor_location(Side::Right, i) ;
    robot.talon_srx[loc].power_level = b.l ;
  }

  //
  // TODO: The original code set the digital outputs associated with the encoders
  //       which makes no sense to me.  What am I missing?
  //

  return robot ;
}

Drivebase::Output Drivebase::Output_applicator::operator()(Robot_outputs robot) const
{
  size_t loc ;
  Drivebase::Output ret ;

  loc = motor_location(Side::Left, 0) ;
  ret.l = robot.talon_srx[loc].power_level ;

  loc = motor_location(Side::Right, 0) ;
  ret.r = robot.talon_srx[loc].power_level ;

  return ret ;
}

Drivebase::Output Drivebase::control(Drivebase::Status status, Drivebase::Goal goal)
{
  switch(goal.mode()){
  case Drivebase::Goal::Mode::DISTANCES:
    return trapezoidal_speed_control(status,goal);
  case Drivebase::Goal::Mode::ABSOLUTE:
    return Drivebase::Output(goal.left(),goal.right());
  case Drivebase::Goal::Mode::DRIVE_STRAIGHT:
    return drive_straight(status,goal);
  case Drivebase::Goal::Mode::ROTATE:
    return rotation_control(status,goal);
  default:
    nyi ;
  }
}

Drivebase::Output Drivebase::trapezoidal_speed_control(Drivebase::Status status, Drivebase::Goal goal)
{
  Drivebase::Output out ;
  const double MS_PER_SEC = 1000 ;
  double avg_goal = goal.distances().avg() ;
  double avg_dist = status.distances.avg() ;
  double avg_last = status.last_output.avg() ;
  double remaining = avg_goal - avg_dist ;
  double new_out ;
  
  if (remaining > SLOW_WITHIN_DISTANCE) {
    //
    // The acceleration to ramp up the robot speed
    //
    const double accel = MAX_OUT / RAMP_UP_TIME_MS ;

    //
    // TODO: Note, we clamp the step at MAX_STEP.  However, this is an absolute number and not
    //       a function of the time since the last call.  This will work but is not exactly
    //       what we want.
    //
    double step = clamp(status.dt * accel * MS_PER_SEC, 0.0, MAX_STEP) ;
    new_out = clamp(avg_last + step, 0.0, MAX_OUT) ;
  }
  else {
    //
    // We are getting close to the desired distance, use special calculations
    // basing the voltage on the distance remaining
    //
    const double decel = MAX_OUT / SLOW_WITHIN_DISTANCE ;
    new_out = clamp(remaining * decel, 0.0, avg_last) ;
  }
  
  out = Output(new_out, new_out) ;
  return out ;
}

//
// Normalize the angle to be between 180 and -180
//
double Drivebase::normalize_angle(const double angle)
{
  static const double DEGS_PER_REV = 360.0 ;
  
  double netangle = fmod(angle, DEGS_PER_REV) ;
  if (netangle > DEGS_PER_REV / 2)
    netangle = DEGS_PER_REV / 2 - netangle ;

  return netangle ;
}

Drivebase::Output Drivebase::rotation_control(Drivebase::Status status, Drivebase::Goal goal)
{
  double status_angle = normalize_angle(status.angle) ;
  double goal_angle = normalize_angle(goal.angle()) ;
  double P = 0.005 ;
  double FLOOR = 0.15 ;
  double ZEROVALUE = 0.0001 ;

  double delta = goal_angle - status_angle ;
  double power = clamp(delta * P, -MAX_OUT, MAX_OUT) ;

  if (power < -ZEROVALUE && power > -FLOOR)
    power = -FLOOR ;
  else if (power > ZEROVALUE && power < FLOOR)
    power = FLOOR ;
  
  Output out  = Output(power, -power) ;
  return out ;
}

Drivebase::Output Drivebase::drive_straight(Drivebase::Status status, Drivebase::Goal goal)
{
  Drivebase::Output out = trapezoidal_speed_control(status, goal) ;
  
  static const double FLOOR = .08; 
  static const double P = .05, I = .1, D = .005;
  static const double ZEROVALUE = 0.0001 ;
  
  double error = normalize_angle(goal.angle()) - normalize_angle(status.angle) ;
  double preverror = normalize_angle(goal.angle()) - normalize_angle(status.prev_angle) ;
  double error_d = (error - preverror) / status.dt ;
  double change = P*error + I*goal.angle_i() + D*error_d;
  out.l = clamp(out.l + change, -MAX_OUT, MAX_OUT);
  out.r = clamp(out.r - change, -MAX_OUT, MAX_OUT);
  
  if(fabs(out.l) > ZEROVALUE && fabs(out.l) < FLOOR)
    out.l = copysign(FLOOR, out.l);
  
  if(fabs(out.r) > ZEROVALUE && fabs(out.r) < FLOOR)
    out.r = copysign(FLOOR, out.r);
  
  return out; 
}

bool Drivebase::ready(Drivebase::Status status, Drivebase::Goal goal)
{
  const double TOLERANCE_INCHES = 1 ;
  const double TOLERANCE_DEGREES = 3 ;
  bool ret = false ;
  
  switch(goal.mode()){
  case Drivebase::Goal::Mode::ABSOLUTE:
    ret = true ;
    break ;
    
  case Drivebase::Goal::Mode::DISTANCES:
    ret = fabs(goal.distances().l - status.distances.l) < TOLERANCE_INCHES ;
    break ;
      
  case Drivebase::Goal::Mode::DRIVE_STRAIGHT:
    ret = (goal.distances() - status.distances).avg() < TOLERANCE_INCHES ;
    break ;
    
  case Drivebase::Goal::Mode::ROTATE:
    ret = fabs(normalize_angle(goal.angle()) - normalize_angle(status.angle)) < TOLERANCE_DEGREES ;
    break ;
    
  default:
    nyi ;
  }

  return ret ;
}

std::set<Drivebase::Output> examples(Drivebase::Output*)
{
  std::set<Drivebase::Output> result ;
  return result ;
}

std::set<Drivebase::Input> examples(Drivebase::Input*)
{
  std::set<Drivebase::Input> result ;
  return result ;
}

std::set<Drivebase::Status> examples(Drivebase::Status*)
{
  std::set<Drivebase::Status> result ;
  return result ;
}

std::set<Drivebase::Goal> examples(Drivebase::Goal*)
{
  std::set<Drivebase::Goal> result ;
  return result ;
}
