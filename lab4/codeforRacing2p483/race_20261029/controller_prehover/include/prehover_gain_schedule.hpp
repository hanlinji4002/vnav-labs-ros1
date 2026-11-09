#ifndef LAB4_PREHOVER_GAIN_SCHEDULE_HPP
#define LAB4_PREHOVER_GAIN_SCHEDULE_HPP
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lab4_prehover {
struct Gains { double kr; double komega; double race_weight; };
struct PositionGains { double kx; double kv; };
inline double smoothWeight(double value) {
  const double u = std::max(0.0, std::min(1.0, value));
  // Rounding can push the quintic a few ulp above 1 just below u=1; the
  // downstream range checks treat that as an invalid schedule, so clamp.
  return std::max(0.0, std::min(1.0, u*u*u*(10.0 + u*(-15.0 + 6.0*u))));
}
inline Gains scheduledGains(double plan_time, double release, double transition,
                            double end, double hover_kr, double hover_kw,
                            double race_kr, double race_kw) {
  const double values[] = {plan_time, release, transition, end, hover_kr, hover_kw, race_kr, race_kw};
  for (double v : values) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite gain schedule");
  if (plan_time < 0 || release < 0 || transition <= 0 || end < release+2*transition ||
      hover_kr <= 0 || hover_kw <= 0 || race_kr <= 0 || race_kw <= 0)
    throw std::invalid_argument("invalid gain schedule range");
  const double rising = smoothWeight((plan_time-release)/transition);
  const double falling = 1.0-smoothWeight((plan_time-(end-transition))/transition);
  const double weight = std::min(rising, falling);
  return {hover_kr+weight*(race_kr-hover_kr), hover_kw+weight*(race_kw-hover_kw), weight};
}
inline PositionGains scheduledPositionGains(double weight, double hover_kx, double hover_kv,
                                            double race_kx, double race_kv) {
  const double values[] = {weight, hover_kx, hover_kv, race_kx, race_kv};
  for (double v : values) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite position schedule");
  if (weight < 0 || weight > 1 || hover_kx <= 0 || hover_kv <= 0 || race_kx <= 0 || race_kv <= 0)
    throw std::invalid_argument("invalid position schedule range");
  return {hover_kx+weight*(race_kx-hover_kx), hover_kv+weight*(race_kv-hover_kv)};
}
}
#endif
