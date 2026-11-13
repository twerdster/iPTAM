#pragma once
// Experimental implementation of ideas from Klein/Murray ISMAR 2009.
// This configuration belongs to the single estimator worker. Configure before
// constructing an engine; never change it concurrently with tracking/mapping.
namespace IPTAMMobile {
struct Settings {
  bool repairDepthIntervals = true;
  bool coarseTracking = true;
  bool partialPoints = true;
  bool automaticInitialization = false; // Explicit profile; fixed-pair control default.
  bool cullMeasurements = true;
  bool cullKeyframes = true;
  bool idleMapping = true;
  int maxObservations = 8;
  int maxActiveKeyframes = 35;
  int maxActivePoints = 600;
  int maxTrackedPoints = 120;
  int maxNewCandidates = 25;
  double idleBudgetMs = 4.0;
  bool rollingShutter = false;
  double readoutSeconds = 0.0;
  int readoutAxis = 1; // 0 columns, 1 rows, in original sensor orientation.
  int readoutDirection = 1;
  double exposureTimestampFraction = 0.5;
};
inline Settings& Config() { static Settings settings; return settings; }
inline void ResetConfig() { Config() = Settings(); }
}
