#include "motor_check.h"
#include <vector>

struct Drivebase
{
  typedef std::pair<Digital_in, Digital_in> Encoder_info ;
  
  Drivebase(int count)
  {
    motor_check.resize(count) ;
  }

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
  } ;

  struct Output
  {
  } ;

  struct Goal
  {
  } ;

  struct Status_detail
  {
  } ;

  struct Estimator
  {
  } ;

  struct Output_applicator
  {
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

  struct Status DRIVEBASE_STATUS ;

  std::vector<Motor_check> motor_check ;
} ;
