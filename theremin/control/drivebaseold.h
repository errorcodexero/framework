#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include <iosfwd>
#include <set>
#include "../util/interface.h"
#include "motor_check.h"
#include "../util/quick.h"
#include "../util/countdown_timer.h"
#include "../util/stall_monitor.h"

struct Drivebase{
};
bool operator==(Drivebase::Encoder_ticks const&,Drivebase::Encoder_ticks const&);
bool operator!=(Drivebase::Encoder_ticks const&,Drivebase::Encoder_ticks const&);
bool operator<(Drivebase::Encoder_ticks const&,Drivebase::Encoder_ticks const&);
Drivebase::Encoder_ticks operator+(Drivebase::Encoder_ticks const&,Drivebase::Encoder_ticks const&);
Drivebase::Encoder_ticks operator-(Drivebase::Encoder_ticks const&);
Drivebase::Encoder_ticks operator-(Drivebase::Encoder_ticks const&,Drivebase::Encoder_ticks const&);
std::ostream& operator<<(std::ostream&,Drivebase::Encoder_ticks const&);

bool operator==(Drivebase::Distances const&,Drivebase::Distances const&);
bool operator!=(Drivebase::Distances const&,Drivebase::Distances const&);
bool operator<(Drivebase::Distances const&,Drivebase::Distances const&);
std::ostream& operator<<(std::ostream&,Drivebase::Distances const&);
Drivebase::Distances operator+(Drivebase::Distances const&,Drivebase::Distances const&);
Drivebase::Distances& operator+=(Drivebase::Distances&,Drivebase::Distances const&);
Drivebase::Distances operator-(Drivebase::Distances const&);
Drivebase::Distances operator-(Drivebase::Distances const&,Drivebase::Distances const&);
Drivebase::Distances fabs(Drivebase::Distances const&);

double ticks_to_inches(const int);
int inches_to_ticks(const double);
Drivebase::Distances ticks_to_inches(const Drivebase::Encoder_ticks);
Drivebase::Encoder_ticks inches_to_ticks(const Drivebase::Distances);

int encoderconv(Maybe_inline<Encoder_output>);

double total_angle_to_displacement(double);

CMP1(Drivebase::Encoder_ticks)
CMP1(Drivebase::Speeds)

std::ostream& operator<<(std::ostream&,Drivebase::Input const&);
bool operator<(Drivebase::Input const&,Drivebase::Input const&);
bool operator==(Drivebase::Input const&,Drivebase::Input const&);
bool operator!=(Drivebase::Input const&,Drivebase::Input const&);
std::set<Drivebase::Input> examples(Drivebase::Input*);

std::ostream& operator<<(std::ostream&,Drivebase::Output const&);
bool operator<(Drivebase::Output const&,Drivebase::Output const&);
bool operator==(Drivebase::Output const&,Drivebase::Output const&);
bool operator!=(Drivebase::Output const&,Drivebase::Output const&);
std::set<Drivebase::Output> examples(Drivebase::Output*);

CMP1(Drivebase::Status)
std::set<Drivebase::Status> examples(Drivebase::Status*);

std::ostream& operator<<(std::ostream&,Drivebase::Goal const&);
std::set<Drivebase::Goal> examples(Drivebase::Goal*);
bool operator<(Drivebase::Goal const&,Drivebase::Goal const&);
bool operator==(Drivebase::Goal const&,Drivebase::Goal const&);

Drivebase::Status status(Drivebase::Status_detail);
Drivebase::Output control(Drivebase::Status_detail,Drivebase::Goal);
bool ready(Drivebase::Status,Drivebase::Goal);

bool operator==(Drivebase::Estimator const&,Drivebase::Estimator const&);
bool operator!=(Drivebase::Estimator const&,Drivebase::Estimator const&);

bool operator!=(Drivebase const&,Drivebase const&);
std::ostream& operator<<(std::ostream&,Drivebase const&);

float range(const Robot_inputs); //To be used for wall following NOT DONE 

#endif
