#include "Tracker.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>
#ifdef IPTAM_MOBILE_PAPER
#include "MobileReplaySupport.h"
#include "RollingShutter.h"
#endif
struct Frame {double timestamp; std::string path;};
std::string token(std::istream& in) {
 std::string t; while(in >> t) { if(t[0]!='#') return t; std::getline(in,t); }
 throw std::runtime_error("Truncated PGM header");
}
CVD::Image<CVD::byte> loadPGM(const std::string& path) {
 std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("Cannot open "+path);
 if(token(in)!="P5") throw std::runtime_error("Expected binary 8-bit PGM: "+path);
 int w=std::stoi(token(in)),h=std::stoi(token(in));
 if(w<64||h<64||w>8192||h>8192||token(in)!="255") throw std::runtime_error("Unsupported PGM dimensions/range");
 char whitespace; in.get(whitespace); if(whitespace=='\r'&&in.peek()=='\n') in.get();
 CVD::Image<CVD::byte> im(CVD::ImageRef(w,h)); in.read(reinterpret_cast<char*>(im.data()),w*h);
 if(in.gcount()!=w*h) throw std::runtime_error("Truncated PGM pixels: "+path); return im;
}
int main(int argc,char** argv) {
 try {
  std::map<std::string,std::string> args;
  for(int i=1;i<argc;i+=2) { if(i+1>=argc) throw std::runtime_error("Arguments require values"); args[argv[i]]=argv[i+1]; }
  for(auto a:{"--frames","--fx","--fy","--cx","--cy","--output"}) if(!args.count(a)) throw std::runtime_error(std::string("Missing ")+a);
#ifdef IPTAM_MOBILE_PAPER
  IPTAMMobile::ApplyProfile(args.count("--mobile-profile")?args["--mobile-profile"]:"auto");
  for(const auto& a:args) if(a.first.find("--mobile-")==0 && a.first!="--mobile-profile") IPTAMMobile::ApplyOption(a.first,a.second);
#endif
  const std::string lensModel=args.count("--lens-model")?args["--lens-model"]:"pinhole";
  if(lensModel!="pinhole" && lensModel!="equidistant") throw std::runtime_error("--lens-model must be pinhole or equidistant");
  std::ifstream manifest(args["--frames"]); if(!manifest) throw std::runtime_error("Cannot open frame manifest");
  std::string line; std::getline(manifest,line); if(line!="timestamp,path"&&line!="timestamp,path\r") throw std::runtime_error("Expected timestamp,path CSV header");
  std::vector<Frame> frames; double previous=-std::numeric_limits<double>::infinity(); std::string base=args["--frames"].substr(0,args["--frames"].find_last_of('/')+1);
  while(std::getline(manifest,line)) {if(line.empty()) continue; if(line.back()=='\r') line.pop_back(); auto comma=line.find(','); if(comma==std::string::npos) throw std::runtime_error("Invalid manifest row"); Frame f{std::stod(line.substr(0,comma)),line.substr(comma+1)}; if(!std::isfinite(f.timestamp)||f.timestamp<=previous) throw std::runtime_error("Timestamps must increase"); previous=f.timestamp; if(f.path.empty()||f.path[0]!='/') f.path=base+f.path; frames.push_back(f);}
  if(frames.empty()) throw std::runtime_error("Empty frame manifest");
  int first=args.count("--init-first")?std::stoi(args["--init-first"]):0;
  int second=args.count("--init-second")?std::stoi(args["--init-second"]):20;
  int limit=args.count("--max-frames")?std::stoi(args["--max-frames"]):frames.size();
  if(first<0||second<=first||limit<=0) throw std::runtime_error("Invalid frame indices");
  auto im=loadPGM(frames[0].path); auto size=im.size();
#ifdef IPTAM_MOBILE_PAPER
  if(args.count("--rolling-verified")) {
    for(const auto& arg:args) if(arg.first.find("--rolling-")==0 &&
      arg.first!="--rolling-verified" && arg.first!="--rolling-readout-seconds" && arg.first!="--rolling-axis" &&
      arg.first!="--rolling-direction" && arg.first!="--rolling-timestamp-fraction" && arg.first!="--rolling-timing-source")
      throw std::runtime_error("Unknown rolling shutter option: "+arg.first);
    for(const char* name:{"--rolling-readout-seconds","--rolling-axis","--rolling-direction","--rolling-timestamp-fraction","--rolling-timing-source"})
      if(!args.count(name)) throw std::runtime_error(std::string("Missing verified timing field ")+name);
    if(args["--rolling-verified"]!="1" || args["--rolling-timing-source"].empty()) throw std::runtime_error("Rolling shutter requires verified prepared-image timing");
    IPTAMMobile::RollingShutterCalibration timing;timing.known=true;timing.width=size.x;timing.height=size.y;
    if(args["--rolling-axis"]=="rows") timing.axis=1;
    else if(args["--rolling-axis"]=="columns") timing.axis=0;
    else throw std::runtime_error("rolling axis must be rows or columns in prepared-image coordinates");
    auto number=[&](const char* name) {size_t used=0;double value=std::stod(args[name],&used);if(used!=args[name].size()||!std::isfinite(value)) throw std::runtime_error(std::string("Invalid timing number: ")+name);return value;};
    double direction=number("--rolling-direction");if(direction!=1 && direction!=-1) throw std::runtime_error("rolling direction must be +1 or -1");
    timing.direction=int(direction);timing.readoutSeconds=number("--rolling-readout-seconds");
    timing.timestampFraction=number("--rolling-timestamp-fraction");
    std::string reason;if(!timing.Validate(reason)) throw std::runtime_error(reason);
    auto& config=IPTAMMobile::Config();config.rollingShutter=true;config.readoutSeconds=timing.readoutSeconds;
    config.readoutAxis=timing.axis;config.readoutDirection=timing.direction;config.exposureTimestampFraction=timing.timestampFraction;
  } else for(const auto& arg:args) if(arg.first.find("--rolling-")==0) throw std::runtime_error("Rolling timing supplied without --rolling-verified 1");
#endif
  ATANCamera c("replay");
  c.SetProjectionModel(lensModel=="equidistant"?ATANCamera::ProjectionModel::Equidistant:ATANCamera::ProjectionModel::FOV);
  const char* actualLensModel=c.GetProjectionModel()==ATANCamera::ProjectionModel::Equidistant?"equidistant":"pinhole";
  c.SetImageSize(size);
  c.SetParameters(TooN::makeVector(std::stod(args["--fx"])/size.x,std::stod(args["--fy"])/size.y,(std::stod(args["--cx"])+.5)/size.x,(std::stod(args["--cy"])+.5)/size.y,0.));
  Map map; MapMaker mapper(map,c); Tracker tracker(size,c,map,mapper);
#ifdef IPTAM_MOBILE_PAPER
  std::ofstream diagnostics(args["--output"]+".diagnostics.jsonl");
  std::ofstream config(args["--output"]+".config.json");config<<IPTAMMobile::ConfigJSON()<<'\n';
  if(!diagnostics || !config) throw std::runtime_error("Cannot create mobile diagnostics");
#endif
  std::ofstream out(args["--output"]); if(!out) throw std::runtime_error("Cannot create output");
  out<<"timestamp,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,quality,found,attempted,points,keyframes,elapsed_ms,visible,warp_rejected,template_rejected,initial_trails,solver_failures,solver_retained_stops,pose_inliers\n"<<std::setprecision(12);
  size_t good=0,bad=0; double total=0;
  const bool paced=args.count("--source-paced") && args["--source-paced"]=="1";
  const auto wallStart=std::chrono::steady_clock::now();
  for(size_t i=0;i<frames.size()&&i<size_t(limit);++i) {
   if(paced) {
    auto target=wallStart+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(frames[i].timestamp-frames[0].timestamp));
#ifdef IPTAM_MOBILE_PAPER
    if(IPTAMMobile::Config().idleMapping && target>std::chrono::steady_clock::now())
      mapper.ProcessIdleWork(std::chrono::duration<double>(target.time_since_epoch()).count(),IPTAMMobile::Config().idleBudgetMs);
#endif
    std::this_thread::sleep_until(target);
   }
   bool secondCommand=true;
#ifdef IPTAM_MOBILE_PAPER
   secondCommand=!IPTAMMobile::Config().automaticInitialization;
#endif
   im=loadPGM(frames[i].path); if(i==size_t(first)||(secondCommand && i==size_t(second))) tracker.StartTracking();
   auto start=std::chrono::steady_clock::now(); tracker.SetFrameTimestamp(frames[i].timestamp); tracker.TrackFrame(im,0,false);
   double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(); total+=ms;
   auto p=tracker.GetCurrentPose().inverse(); auto t=p.get_translation(); auto r=p.get_rotation().get_matrix();
   out<<frames[i].timestamp; for(int j=0;j<3;++j) {if(!std::isfinite(t[j])) throw std::runtime_error("Nonfinite pose");out<<','<<t[j];}
   for(int j=0;j<3;++j) for(int k=0;k<3;++k) {if(!std::isfinite(r[j][k])) throw std::runtime_error("Nonfinite rotation");out<<','<<r[j][k];}
   int q=tracker.TrackingQuality(); good+=q==2 && tracker.MeasurementsFound()>=10 && tracker.PoseInliers()>=10 && !map.vpPoints.empty(); bad+=q==0;
   out<<','<<q<<','<<tracker.MeasurementsFound()<<','<<tracker.MeasurementsAttempted()<<','<<map.vpPoints.size()<<','<<map.vpKeyFrames.size()<<','<<ms<<','<<tracker.LastVisibleCount()<<','<<tracker.LastWarpRejectedCount()<<','<<tracker.LastTemplateRejectedCount()<<','<<tracker.InitialTrailCount()<<','<<mapper.NumericalFailureCount()<<','<<mapper.RetainedSolveCount()<<','<<tracker.PoseInliers()<<'\n';
   out.flush(); if(!out) throw std::runtime_error("Trajectory write failed");
#ifdef IPTAM_MOBILE_PAPER
   double delivery=std::chrono::duration<double>(start-wallStart).count();
   diagnostics<<"{\"frame\":"<<i<<",\"timestamp\":"<<std::setprecision(15)<<frames[i].timestamp
              <<",\"delivery_wall_seconds\":"<<delivery<<",\"schedule_lag_ms\":"<<(paced?std::max(0.,delivery-(frames[i].timestamp-frames[0].timestamp))*1000.:0.)
              <<",\"blur_fallback\":"<<tracker.MobileBlurFallback()<<",\"coarse_matches\":"<<tracker.MobileCoarseMatches()
              <<",\"initialization_stage\":\""<<tracker.MobileInitializationStage()<<"\",\"rs_observations\":"<<tracker.MobileRSObservations()
              <<",\"rs_corrections\":"<<tracker.MobileRSCorrections()<<",\"mapping\":"<<IPTAMMobile::DiagnosticsJSON(mapper)<<"}\n";
   diagnostics.flush();if(!diagnostics) throw std::runtime_error("Mobile diagnostics write failed");
   if(mapper.ResourceLimitReached()) throw std::runtime_error("Mobile map resource limit: "+mapper.ResourceLimitReason());
#endif
   if(i%50==0) std::cerr<<"frame "<<i<<" "<<tracker.GetMessageForUser()<<'\n';
  }
  std::cout<<"map_good="<<map.IsGood()<<" good_frames="<<good<<" bad_frames="<<bad<<" points="<<map.vpPoints.size()<<" keyframes="<<map.vpKeyFrames.size()<<" solver_failures="<<mapper.NumericalFailureCount()<<" solver_retained_stops="<<mapper.RetainedSolveCount()<<" lens_model="<<actualLensModel<<" total_ms="<<total<<'\n';
  return map.IsGood()?0:2;
 } catch(const std::exception& e) {std::cerr<<"Replay error: "<<e.what()<<'\n'; return 1;}
}
