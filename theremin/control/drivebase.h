#include "../util/interface.h"
#include "../util/countdown_timer.h"
#include "../util/stall_monitor.h"
#include "motor_check.h"
#include <vector>

class Drivebase
{
 public:
  enum class Side {
    Left,
    Right
  } ;
      
  typedef std::pair<Digital_in, Digital_in> Encoder_info ;
  
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

  struct Output
  {
    double l ;
    double r ;
  } ;

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
  } ;

  
  struct Input
  {
    std::vector<double> current ;
    Encoder_info left ;
    Encoder_info right ;
    Distances distances ;
    double angle ;
  } ;

  struct Input_reader
  {
    Input operator()(Robot_inputs const &) const ;
    Robot_inputs operator()(Robot_inputs, Input) const ;
  } ;

  struct Goal
  {
    enum class Mode {
        ABSOLUTE,
	DISTANCES,
	DRIVE_STRAIGHT,
	ROTATE
      } ;
  private:
    Mode mode_ ;
    Distances distances_ ;
    double angle_ ;
    double angle_i_ ;
    double left_ ;
    double right_ ;

  public:
    Goal() ;
    Mode mode() const ;
    Distances distances() const ;
    Rad angle() const ;
    double angle_i() const ;
    double right() const ;
    double left() const ;

    static Goal distances(Distances) ;
    static Goal absolute(double, double) ;
    static Goal drive_straight(Distances, double, double) ;
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
    Status_detail get() const ;
    Estimator() ;
  } ;

  struct Output_applicator
  {
    Robot_outputs operator()(Robot_outputs, Output) const ;
    Output operator()(Robot_outputs) const ;
  } ;

 public:
  Drivebase(int count)
  {
    motor_check.resize(count) ;
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
  static unsigned pdb_location(Side s, size_t motor) ;

  //
  // Return the encoder value for a given encoder.  The value may not be valid
  // in which case return zero.
  //
  static int encoderconv(Maybe_inline<Encoder_output> encoder) ;

} ;
