#pragma once
#include <chrono>
#include <algorithm>
namespace IPTAMMobile {
inline double MappingSteadySeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double& MappingDeadlineSeconds() {
  static thread_local double deadline=0.;
  return deadline;
}
inline bool MappingBudgetExpired() {
  return MappingDeadlineSeconds()>0. && MappingSteadySeconds()>=MappingDeadlineSeconds();
}
class ScopedMappingDeadline {
 public:
  explicit ScopedMappingDeadline(double deadline):previous_(MappingDeadlineSeconds()) {
    MappingDeadlineSeconds()=previous_>0.?std::min(previous_,deadline):deadline;
  }
  ~ScopedMappingDeadline() { MappingDeadlineSeconds()=previous_; }
 private:
  double previous_;
};
} // namespace IPTAMMobile
