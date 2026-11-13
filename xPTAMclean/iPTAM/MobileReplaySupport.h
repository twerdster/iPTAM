#pragma once
#include "MobileConfig.h"
#include "MapMaker.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace IPTAMMobile {
inline void ApplyProfile(const std::string& profile) {
  ResetConfig();
  if(profile=="paper" || profile=="auto") {
    Config().automaticInitialization=profile=="auto";
    return;
  }
  if(profile!="diagnostics" && profile!="serial" && profile!="coarse" && profile!="partial")
    throw std::runtime_error("mobile profile must be diagnostics, serial, coarse, partial, paper or auto");
  Config().coarseTracking=profile=="coarse" || profile=="partial";
  Config().repairDepthIntervals=profile!="diagnostics";
  Config().partialPoints=profile=="partial";
  Config().cullMeasurements=false;
  Config().cullKeyframes=false;
  Config().idleMapping=profile!="diagnostics";
  if(!Config().coarseTracking) {Config().maxActivePoints=5000;Config().maxTrackedPoints=1000;}
}
inline void ApplyOption(const std::string& name,const std::string& value) {
  if(name=="--mobile-profile") {ApplyProfile(value);return;}
  auto integer=[&] {size_t used=0;int n=std::stoi(value,&used);if(used!=value.size()) throw std::runtime_error("Invalid integer option");return n;};
  if(name=="--mobile-depth-repair") {
    if(value!="0" && value!="1") throw std::runtime_error("mobile depth repair must be 0 or 1");
    Config().repairDepthIntervals=value=="1";
  } else if(name=="--mobile-idle-mapping") {
    if(value!="0" && value!="1") throw std::runtime_error("mobile idle mapping must be 0 or 1");
    Config().idleMapping=value=="1";
  } else if(name=="--mobile-idle-ms") {
    size_t used=0;double n=std::stod(value,&used);
    if(used!=value.size() || !std::isfinite(n) || n<=0 || n>20) throw std::runtime_error("mobile idle budget must be positive and at most20 ms; disable scheduling with --mobile-idle-mapping 0");
    Config().idleBudgetMs=n;
  } else if(name=="--mobile-max-keyframes") {
    int n=integer();if(n<3 || n>96) throw std::runtime_error("mobile active keyframes must be 3..96");Config().maxActiveKeyframes=n;
  } else if(name=="--mobile-max-points") {
    int n=integer();if(n<50 || n>5000) throw std::runtime_error("mobile active points must be 50..5000");Config().maxActivePoints=n;
  } else if(name=="--mobile-max-tracked") {
    int n=integer();if(n<10 || n>1000) throw std::runtime_error("mobile tracked points must be 10..1000");Config().maxTrackedPoints=n;
  } else throw std::runtime_error("Unknown mobile option: "+name);
}
inline std::string ConfigJSON() {
  const auto& c=Config();std::ostringstream out;
  out<<std::setprecision(15)<<"{\"variant\":\"paper-inspired modern core\",\"coarse_tracking\":"<<c.coarseTracking
     <<",\"repair_depth_intervals\":"<<c.repairDepthIntervals
     <<",\"partial_points\":"<<c.partialPoints<<",\"automatic_initialization\":"<<c.automaticInitialization
     <<",\"cull_measurements\":"<<c.cullMeasurements<<",\"cull_keyframes\":"<<c.cullKeyframes
     <<",\"idle_mapping\":"<<c.idleMapping<<",\"idle_budget_ms\":"<<c.idleBudgetMs
     <<",\"max_observations\":"<<c.maxObservations<<",\"max_active_keyframes\":"<<c.maxActiveKeyframes
     <<",\"max_active_points\":"<<c.maxActivePoints<<",\"max_tracked_points\":"<<c.maxTrackedPoints
     <<",\"max_new_candidates\":"<<c.maxNewCandidates<<",\"rolling_shutter_enabled\":"<<c.rollingShutter
     <<",\"readout_seconds\":"<<c.readoutSeconds<<",\"readout_axis\":"<<c.readoutAxis
     <<",\"readout_direction\":"<<c.readoutDirection<<",\"timestamp_fraction\":"<<c.exposureTimestampFraction<<"}";return out.str();
}
inline std::string DiagnosticsJSON(const MapMaker& mapper) {
  const auto& d=mapper.Diagnostics();std::ostringstream out;out<<std::setprecision(15);
  out<<"{\"epipolar_attempts\":"<<d.epipolarAttempts<<",\"epipolar_accepted\":"<<d.epipolarAccepted
     <<",\"repaired_depth_intervals\":"<<d.repairedDepthIntervals<<",\"budget_stops\":"<<d.budgetStops
     <<",\"invalid_depth_intervals\":"<<d.invalidDepthIntervals<<",\"rejected\":[";
  for(size_t i=0;i<d.rejected.size();++i) {if(i)out<<',';out<<d.rejected[i];}
  out<<"],\"stage_calls\":[";
  for(size_t i=0;i<d.stageCalls.size();++i) {if(i)out<<',';out<<d.stageCalls[i];}
  out<<"],\"stage_ms\":[";
  for(size_t i=0;i<d.stageMs.size();++i) {if(i)out<<',';out<<d.stageMs[i];}
  out<<"],\"max_stage_ms\":[";
  for(size_t i=0;i<d.maxStageMs.size();++i) {if(i)out<<',';out<<d.maxStageMs[i];}
  auto finite=[](double x){return std::isfinite(x)?x:0.;};
  out<<"],\"last_depth_mean\":"<<finite(d.lastDepthMean)<<",\"last_depth_sigma\":"<<finite(d.lastDepthSigma)
     <<",\"last_depth_start\":"<<finite(d.lastDepthStart)<<",\"last_depth_end\":"<<finite(d.lastDepthEnd)
     <<",\"culled_measurements\":"<<d.culledMeasurements<<",\"culled_keyframes\":"<<d.culledKeyframes
     <<",\"observation_bridge_retentions\":"<<d.observationBridgeRetentions<<",\"observation_support_retentions\":"<<d.observationSupportRetentions
     <<",\"fixed_low_support_views\":"<<d.fixedLowSupportViews<<",\"bundle_rs_corrections\":"<<d.bundleRSCorrections
     <<",\"partial_seeded\":"<<d.partialSeeded<<",\"partial_promoted\":"<<d.partialPromoted
     <<",\"partial_updates\":"<<d.partialUpdates<<",\"partial_rejected\":"<<d.partialRejected
     <<",\"partial_count\":"<<mapper.PartialPointCount()<<",\"idle_stages\":"<<d.idleStages
     <<",\"pending_partial_keyframes\":"<<mapper.PendingPartialPromotionCount()
     <<",\"deferred_partial_queued\":"<<d.deferredPartialQueued<<",\"deferred_partial_completed\":"<<d.deferredPartialCompleted
     <<",\"deferred_partial_cancelled\":"<<d.deferredPartialCancelled
     <<",\"idle_overruns\":"<<d.idleOverruns<<",\"idle_ms\":"<<d.idleMs
     <<",\"resource_limit\":"<<mapper.ResourceLimitReached()<<"}";return out.str();
}
} // namespace IPTAMMobile
