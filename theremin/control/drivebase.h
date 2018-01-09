#include "../util/interface.h"
#include "../util/countdown_timer.h"
#include "../util/stall_monitor.h"
#include "../util/robot_constants.h"
#include "motor_check.h"
#include <vector>
#include <cmath>
#include <set>

//
// Overview
//
// This is the class that represents the drivebase.  The drivebase uses hardware
// resources and as such specific resources must be identified.
//
// Motors:
// - There is assumed to be an equal number of motors (N) on each side of the robot
// - The motors on the left side of the robot are controlled by motor controllers
//   numbered zero through N - 1.
// - The motors on the right side of the robot are controlled by motor controllers
//   numbered N throught 2N - 1
//
// Encoders:
// - A single encoder uses two digital inputs and are mapped to a single encoder number
// - The left encoder number is given by the constant below LEFT_ENCODER
// - The two digital inputs for the left encoder is given by LEFT_ENCODER_FIRST and
//   LEFT_ENCODER_SECOND
// - The right encoder number is given by the constant below RIGHT_ENCODER
// - The two digital inputs for the right encoder is given by RIGHT_ENCODER_FIRST and
//   RIGHT_ENCODER_SECOND
//
// Hardawre Interface
// The hardware interface consists of the information stored on the Robot_inputs and the
// Robot_outputs structures.  These structures provide an abstraction to the robot.  To read
// the state of the robot, the drivebase examines information in the Robot_inputs structure.
// To set the state of the robot, the drivebase sets values in the Robot_outputs structure.
//
// Modes:
// This class operates in one of three modes
//
// TODO:
// - The physical characteristics of the drive base and the wiring of the drivebase
//   are all littered throughout the code.  These should be broken out into a seperate
//   class and this class should be supplied during initialization.
//

class Drivebase
{
 public:
  //
  // Constant defining the number of encoder ticks per revolution of a wheel
  //
  static const unsigned int TICKS_PER_REVOLUTION = 200 ;

  //
  // Constant defining the circumference of the wheel in inches
  //
  static const double WHEEL_CIRCUMFERENCE ;

  //
  // Calculated constant return the number of inches per encoder tick
  //
  static const double INCHES_PER_TICK ;

  //
  // The total number of motors, per side, for the robot
  //
  static const size_t NUMBER_MOTORS = 2 ;

  //
  // The maximum output voltage when doing automated driving
  //
  static const double MAX_OUT ;

  //
  // The maximum to change the output voltage in one iteration of
  // the robot loop
  //
  static const double MAX_STEP ;

  //
  // The time to ramp from stopped to MAX_OUT when automated
  //
  static const double RAMP_UP_TIME_MS ;

  //
  // The distance to ramp from MAX_OUT to stopped
  //
  static const double SLOW_WITHIN_DISTANCE ;

  //
  // The encoders for the robot
  //
  static const int LEFT_ENCODER = 0 ;			// Left encoder number
  static const int LEFT_ENCODER_FIRST = 0 ;		// First digital input for left encoder
  static const int LEFT_ENCODER_SECOND = 1 ;		// Second digital input for left encoder
  static const int RIGHT_ENCODER = 1 ;			// Right encoder number
  static const int RIGHT_ENCODER_FIRST = 2 ;		// First digital input for right encoder
  static const int RIGHT_ENCODER_SECOND = 3 ;		// Second digital input for right encoder

  //
  // The motors for the robot
  //
  static const std::vector<int> LeftMotors ;
  static const std::vector<int> RightMotors ;

  //
  // The PDB locations for the robot
  //
  static const std::vector<int> LeftPDPChannels ;
  static const std::vector<int> RightPDPChannels ;

  
 public:
  //
  // Identifies the side of the robot for the motors
  //
  enum class Side {
    Left,
      Right
      } ;

  //
  // The info associated with an encoder, assocaited with two digital inputs
  //
  typedef std::pair<Digital_in, Digital_in> Encoder_info ;

  //
  // The structure holds the number of encoder ticks seen since the robot
  // was last powered on
  //
  struct Encoder_ticks
  {
    double l ;
    double r ;

    Encoder_ticks()
    {
      l = 0 ;
      r = 0 ;
    }

    Encoder_ticks(double vl, double vr)
    {
      l = vl ;
      r = vr ;
    }
  } ;

  //
  // The output voltage applied to the left and right side of the robot
  //
  struct Output
  {
    Output()
    {
      l = 0 ;
      r = 0 ;
    }

    Output(double lval, double rval)
    {
      l = lval ;
      r = rval ;
    }

    Output(double value)
    {
      l = value ;
      r = value ;
    }
    
    double l ;
    double r ;

    double avg() const
    {
      return (l + r) / 2.0 ;
    }
  } ;

  //
  // The speed of the left wheels and right wheels of the robot in inches
  // per second.
  //
  struct Speeds
  {
    double l ;
    double r ;

    Speeds()
    {
      l = 0 ;
      r = 0 ;
    }
  } ;

  //
  // The distances traveled by the wheels on each side of the robot
  //
  struct Distances
  {
    double l ;
    double r ;

    Distances()
    {
      l = 0 ;
      r = 0 ;
    } ;

    Distances(double d)
    {
      l = d ;
      r = d ;
    }

    Distances(double lval, double rval)
    {
      l = lval ;
      r = rval ;
    }

    double avg() const
    {
      return (l + r) / 2.0 ;
    } ;
  } ;

  //
  // This structure holds the state of the robot inputs for the drivebase
  // in units of the subsystem.  For instance, distances are stored in inches
  // which are specific to this drivebase.
  //
  struct Input
  {
    //
    // This stores the current drawn by the motors.  There are
    // 2 * NUMBER_MOTORS entries in this vector.  The first NUMBER_MOTORS
    // entries are the left motors.  The next NUMBER_MOTORS entries
    // are the right motors
    //
    std::vector<double> current ;

    //
    // The state of the two digital inputs associated with each encoder
    //
    Encoder_info left ;
    Encoder_info right ;

    //
    // The total distance, in inches, the drivebase has traveled
    //
    Distances distances ;

    //
    // The current angle of the drive base in degrees
    //
    double angle ;
  } ;

  //
  // The robot reader is a functional object that converts
  // to and from the Robot_inputs structure and the Input
  // structure.
  //
  struct Input_reader
  {
    Input operator()(Robot_inputs const &) const ;
    Robot_inputs operator()(Robot_inputs, Input) const ;
  } ;

  //
  // This is the goal the drivebase is trying to achieve
  //
  struct Goal
  {
    enum class Mode {
      ABSOLUTE,				// Drive an absolute voltage
      DISTANCES,			// Drive a distance using a trapezoid voltage function
      DRIVE_STRAIGHT,			// Same as distances, but use NAVX to drive straight
      ROTATE				// Rotate to a fixed angle
    } ;
    
  private:
    Mode mode_ ;
    Distances distances_ ;
    double angle_ ;
    double angle_i_ ;
    double left_ ;
    double right_ ;

  public:
    Goal()
    {
      mode_ = Mode::ABSOLUTE ;
      angle_ = 0 ;
      angle_i_ = 0 ;
      left_ = 0 ;
      right_ = 0 ;
    }
    
    Mode mode() const
    {
      return mode_ ;
    }
    
    Distances distances() const
    {
      assert(mode_ == Drivebase::Goal::Mode::DISTANCES || mode_ == Drivebase::Goal::Mode::DRIVE_STRAIGHT);
      Distances d(left_, right_) ;
      return d ;
    }
    
    Rad angle() const
    {
      assert(mode_ == Drivebase::Goal::Mode::ROTATE || mode_ == Drivebase::Goal::Mode::DRIVE_STRAIGHT);
      return angle_;
    }
    
    double angle_i() const
    {
      assert(mode_ == Drivebase::Goal::Mode::DRIVE_STRAIGHT);
      return angle_i_;
    }
      
    double right() const
    {
      assert(mode_ == Drivebase::Goal::Mode::ABSOLUTE);
      return right_;
    }
    
    double left() const
    {
      assert(mode_ == Drivebase::Goal::Mode::ABSOLUTE);
      return left_;
    }

    static Goal distances(const Distances &) ;
    static Goal absolute(double, double) ;
    static Goal drive_straight(const Distances &, double, double) ;
    static Goal rotate(Rad) ;
  } ;

  struct Status
  {
    std::vector<Motor_check::Status> motor ;
    bool stall ;
    Speeds speeds ;
    Distances distances ;
    Output last_output ;
    Time dt ;
    Time now ;
    double angle ;
    double prev_angle ;
  } ;
  typedef Status Status_detail ;

  struct Estimator
  {
    std::vector<Motor_check> motor_check ;
    Status_detail last ;
    Countdown_timer speed_timer ;
    Stall_monitor stall_monitor ;

    void update(Time, Input, Output) ;
    
    Status_detail get() const
    {
      return last ;
    }
    
    Estimator()
    {
    }
  } ;

  struct Output_applicator
  {
    Robot_outputs operator()(Robot_outputs, Output) const ;
    Output operator()(Robot_outputs) const ;
  } ;

 public:
  Drivebase()
    {
    }

  Estimator estimator ;
  Input_reader input_reader ;
  std::vector<Motor_check> motor_check ;

  //
  // Static functions associated with the drive base class
  //

  //
  // Return the PDP location of a motor given the side it is on and
  // which motor it is on that given side.
  //
  static size_t pdb_location(Side s, size_t motor) ;

  //
  // Return the motor controller index of a motor given the side it is on
  // and which motor is of interest on that side
  //
  static size_t motor_location(Side s, size_t motor) ;

  //
  // Return the encoder value for a given encoder.  The value may not be valid
  // in which case return zero.
  //
  static int encoderconv(Maybe_inline<Encoder_output> encoder) ;

  //
  // Return the number of inches given a number of encoder ticks
  //
  static double ticks_to_inches(int ticks) ;

  //
  // Return the number of inches for each of the left and right encoders given
  // the number of encoder ticks for each side.
  //
  static Distances ticks_to_inches(const Encoder_ticks &ticks) ;

  //
  // Return the number of ticks, given a distance in inches
  //
  static int inches_to_ticks(double inches) ;

  //
  // Create the drivebase outputs based on the current status and goal when the goal
  // is a distance goal
  //
  static Drivebase::Output trapezoidal_speed_control(Drivebase::Status status, Drivebase::Goal goal) ;

  //
  // Create the drivebase outputs based on the current status and goal when the goal
  // is a rotational goal
  //
  static Drivebase::Output rotation_control(Drivebase::Status status, Drivebase::Goal goal) ;

  //
  // Create the drivebase outputs based on the current status and goal when the goal
  // is a drive straight goal
  //
  static Drivebase::Output drive_straight(Drivebase::Status status, Drivebase::Goal goal) ;

  //
  // The main control function for the drive base.  It provides the output required to meet the designated
  // goal given the current status
  //
  static Drivebase::Output control(Drivebase::Status status, Drivebase::Goal goal) ;

  //
  // Calculate the displacement needed
  //
  static double normalize_angle(const double angle) ;

  //
  // Determine if we have met our goal
  //
  static bool ready(Drivebase::Status status, Drivebase::Goal goal) ;
} ;

inline Drivebase::Output control(Drivebase::Status status, Drivebase::Goal goal)
{
  return Drivebase::control(status, goal) ;
}

inline bool ready(Drivebase::Status status, Drivebase::Goal goal)
{
  return Drivebase::ready(status,goal) ;
}

inline Drivebase::Status status(const Drivebase::Status_detail &detail)
{
  return detail ;
}

inline Drivebase::Distances operator-(const Drivebase::Distances &a, const Drivebase::Distances &b)
{
  return Drivebase::Distances(a.l - b.l, a.r - b.r) ;
}

inline Drivebase::Distances operator+(const Drivebase::Distances &a, const Drivebase::Distances &b)
{
  return Drivebase::Distances(a.l + b.l, a.r + b.r) ;
}

inline bool operator==(const Drivebase::Distances &a, const Drivebase::Distances &b)
{
  static double TOLERANCE = 0.1 ;
  return fabs(a.l - b.l) < TOLERANCE && fabs(a.r - b.r) < TOLERANCE ;
}

//
// TODO - why do we want these?  They do not seem to make sense or be useful
//
inline bool operator<(const Drivebase::Output &a, const Drivebase::Output &b)
{
  return true ;
}

inline bool operator!=(const Drivebase::Output &a, const Drivebase::Output &b)
{
  return true ;
}

inline bool operator<(const Drivebase::Status &a, const Drivebase::Status &b)
{
  return true ;
}

inline bool operator!=(const Drivebase::Status &a, const Drivebase::Status &b)
{
  return true ;
}

inline bool operator!=(const Drivebase::Estimator &a, const Drivebase::Estimator &b)
{
  return true ;
}

inline bool operator<(const Drivebase::Input &a, const Drivebase::Input &b)
{
  return true ;
}

inline bool operator!=(const Drivebase::Input &a, const Drivebase::Input &b)
{
  return true ;
}

inline bool operator<(const Drivebase::Goal &a, const Drivebase::Goal &b)
{
  return true ;
}




//
// Stream operators, finish these
//
inline std::ostream& operator<<(std::ostream &strm, const Drivebase::Output &obj)
{
  //
  // TODO - print the output structure
  //
  return strm ;
}

inline std::ostream& operator<<(std::ostream &strm, const Drivebase::Input &obj)
{
  //
  // TODO - print the output structure
  //
  return strm ;
}

inline std::ostream& operator<<(std::ostream &strm, const Drivebase::Goal &obj)
{
  //
  // TODO - print the output structure
  //
  return strm ;
}

inline std::ostream& operator<<(std::ostream &strm, const Drivebase::Status &obj)
{
  //
  // TODO - print the output structure
  //
  return strm ;
}

std::set<Drivebase::Input> examples(Drivebase::Input *) ;
std::set<Drivebase::Output> examples(Drivebase::Output*);
std::set<Drivebase::Status> examples(Drivebase::Status*);
std::set<Drivebase::Goal> examples(Drivebase::Goal*);
