#include "prehover_gain_schedule.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using lab4_prehover::Gains;
static Gains at(double t) {
  return lab4_prehover::scheduledGains(t, 5.0, .3, 10.0, 12.0, 1.0, 60.0, 4.0);
}
static bool close(double x, double y, double epsilon=1e-10) {return std::abs(x-y)<epsilon;}
static void expectsInvalid(double t, double release, double transition, double end,
                           double hover_kr=12, double hover_kw=1, double race_kr=60, double race_kw=4) {
  bool rejected=false;
  try {lab4_prehover::scheduledGains(t,release,transition,end,hover_kr,hover_kw,race_kr,race_kw);}
  catch (const std::invalid_argument&) {rejected=true;}
  assert(rejected);
}
int main() {
  // Frozen plan time at 5 s cannot raise the gains before actual-state release.
  for(double t : {0.0, 3.0, 4.99, 5.0}) {
    assert(close(at(t).kr,12)); assert(close(at(t).komega,1)); assert(close(at(t).race_weight,0));
  }
  assert(close(at(5.15).kr,36)); assert(close(at(5.15).komega,2.5));
  for(double t : {5.3, 6., 8., 9.7}) {
    assert(close(at(t).kr,60)); assert(close(at(t).komega,4)); assert(close(at(t).race_weight,1));
  }
  assert(close(at(9.85).kr,36)); assert(close(at(9.85).komega,2.5));
  for(double t : {10., 11., 60.}) {
    assert(close(at(t).kr,12)); assert(close(at(t).komega,1)); assert(close(at(t).race_weight,0));
  }
  for(double t : {0.,3.,5.,10.,11.}) {
    const auto g=lab4_prehover::scheduledPositionGains(at(t).race_weight,4,3,36,12);
    assert(close(g.kx,4)); assert(close(g.kv,3));
  }
  for(double t : {5.3,6.,8.,9.7}) {
    const auto g=lab4_prehover::scheduledPositionGains(at(t).race_weight,4,3,36,12);
    assert(close(g.kx,36)); assert(close(g.kv,12));
  }
  const auto middle=lab4_prehover::scheduledPositionGains(at(5.15).race_weight,4,3,36,12);
  assert(close(middle.kx,20)); assert(close(middle.kv,7.5));
  // No overshoot and no single 20 ms feedback step across the whole gain range.
  double previous=at(0).race_weight;
  for(int i=1;i<=600;++i) {
    Gains g=at(i*.02);
    assert(g.kr>=12-1e-10 && g.kr<=60+1e-10);
    assert(g.komega>=1-1e-10 && g.komega<=4+1e-10);
    const auto p=lab4_prehover::scheduledPositionGains(g.race_weight,4,3,36,12);
    assert(p.kx>=4-1e-10 && p.kx<=36+1e-10);
    assert(p.kv>=3-1e-10 && p.kv<=12+1e-10);
    assert(close((p.kx-4)/32,g.race_weight));
    assert(close((p.kv-3)/9,g.race_weight));
    assert(std::abs(g.race_weight-previous)<.126);
    previous=g.race_weight;
  }
  double rising=-1,falling=2;
  for(int i=0;i<=300;++i) {
    const double rise=at(5.+i*.001).race_weight;
    const double fall=at(9.7+i*.001).race_weight;
    assert(rise>=rising-1e-12 && fall<=falling+1e-12);
    rising=rise;falling=fall;
  }
  // Smoothstep gives zero first and second derivatives at all four boundaries.
  const double h=1e-5;
  for(double boundary : {5.,5.3,9.7,10.}) {
    const double centre=at(boundary).race_weight;
    const double first=(at(boundary+h).race_weight-at(boundary-h).race_weight)/(2*h);
    const double second=(at(boundary+h).race_weight-2*centre+at(boundary-h).race_weight)/(h*h);
    assert(std::abs(first)<1e-6); assert(std::abs(second)<.02);
  }
  expectsInvalid(std::numeric_limits<double>::quiet_NaN(),5,.3,10);
  expectsInvalid(-1,5,.3,10);
  expectsInvalid(0,5,0,10);
  expectsInvalid(0,5,.3,5.5);
  expectsInvalid(0,5,.3,10,-12);
  expectsInvalid(0,5,.3,10,12,1,60,std::numeric_limits<double>::infinity());
  for(double invalid_weight : {-1.,1.01,std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;
    try {lab4_prehover::scheduledPositionGains(invalid_weight,4,3,36,12);}
    catch(const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }
  // Regression: a plan time a few nanoseconds below the plan end (ROS Duration
  // quantisation) produced a weight of -2e-16 and aborted the controller at the
  // end of front03 (end 9.643234760796052, time_from_start 9.64323476).
  for(double end : {9.643234760796052, 9.694443577862277, 10.0}) {
    for(double below : {1e-9, 7.96052e-10, 2.7e-10, 1e-12, 0.0}) {
      const double t=end-below;
      const Gains g=lab4_prehover::scheduledGains(t,5.0,.3,end,12.0,1.0,60.0,4.0);
      assert(g.race_weight>=0 && g.race_weight<=1);
      const auto p=lab4_prehover::scheduledPositionGains(g.race_weight,4,3,36,12);
      assert(p.kx>=4 && p.kx<=4+1e-6);
    }
    const double start=5.0+.3-1e-10;
    assert(lab4_prehover::scheduledGains(start,5.0,.3,end,12.0,1.0,60.0,4.0).race_weight<=1);
  }
  assert(lab4_prehover::smoothWeight(1.0-2.65e-9)<=1.0);
  assert(lab4_prehover::smoothWeight(2.65e-9)>=0.0);
  std::cout<<"PASS: coordinated position/attitude hover, measured-release hold, C2 ramps, race, terminal hold, sampled steps and invalid-input checks\n";
}
