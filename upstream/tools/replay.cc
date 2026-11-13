#include "OriginalPTAM.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
std::string token(std::istream& input) {
  std::string s;
  while(input>>s) {if(s[0]=='#'){std::getline(input,s);continue;}return s;}
  throw std::runtime_error("Truncated PGM header");
}
std::vector<std::uint8_t> pgm(const std::string& path,int width,int height) {
  std::ifstream input(path,std::ios::binary);
  if(!input || token(input)!="P5") throw std::runtime_error("Cannot read binary grayscale PGM: "+path);
  const int w=std::stoi(token(input)),h=std::stoi(token(input)),max=std::stoi(token(input));
  if(w!=width || h!=height || max!=255)throw std::runtime_error("PGM calibration/format mismatch: "+path);
  input.get();std::vector<std::uint8_t> data(w*h);input.read(reinterpret_cast<char*>(data.data()),data.size());
  if(input.gcount()!=static_cast<std::streamsize>(data.size()))throw std::runtime_error("Truncated PGM: "+path);
  return data;
}
std::array<double,9> rotation(const std::array<double,7>& pose) {
  const double x=pose[3],y=pose[4],z=pose[5],w=pose[6];
  return {{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
}
}
int main(int argc,char** argv) {
 try {
  std::map<std::string,std::string> args;bool paced=true;
  for(int i=1;i<argc;++i) {std::string a=argv[i];if(a=="--as-fast")paced=false;else if(i+1<argc)args[a]=argv[++i];else throw std::runtime_error("Missing argument");}
  if(!args.count("--frames") || !args.count("--output")) {
    std::cerr<<"Usage: original_ptam_replay --frames frames.csv --output trajectory.csv --fx F --fy F --cx C --cy C [--init-first 0 --init-second 18 --max-frames N --as-fast]\n";return 2;
  }
  original_ptam::Camera camera;
  auto number=[&](const std::string& key,double fallback){return args.count(key)?std::stod(args[key]):fallback;};
  camera.width=number("--width",640);camera.height=number("--height",480);camera.fx=number("--fx",460);camera.fy=number("--fy",460);camera.cx=number("--cx",319.5);camera.cy=number("--cy",239.5);camera.w=number("--w",0);
  const int first=number("--init-first",0),second=number("--init-second",18),limit=number("--max-frames",2147483647);
  const std::string frames_path=args["--frames"],prefix=frames_path.substr(0,frames_path.find_last_of('/')+1);
  std::ifstream source(frames_path);if(!source)throw std::runtime_error("Cannot read frames.csv");
  std::string line;std::getline(source,line);if(!line.empty()&&line.back()=='\r')line.pop_back();if(line!="timestamp,path")throw std::runtime_error("Expected timestamp,path CSV header");
  struct Frame {double time;std::string path;};std::vector<Frame> frames;
  while(std::getline(source,line)) {if(!line.empty()&&line.back()=='\r')line.pop_back();auto comma=line.find(',');if(comma==std::string::npos)continue;frames.push_back({std::stod(line.substr(0,comma)),prefix+line.substr(comma+1)});}
  if(frames.empty())throw std::runtime_error("No exposures");
  std::ofstream output(args["--output"]);if(!output)throw std::runtime_error("Cannot create output CSV");
  output<<"timestamp,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,quality,points,keyframes,found,attempted,initialized,finite,elapsed_ms,scheduling_lag_ms,map_id\n"<<std::setprecision(17);
  std::cerr<<"estimator=official upstream PTAM r114; original asynchronous mapper; timestamp_motion_prior=false; source_paced="<<paced<<"; lens_model=FOV(w="<<camera.w<<")\n";
  original_ptam::Engine engine(camera);
  const auto start=std::chrono::steady_clock::now();
  for(int i=0;i<static_cast<int>(frames.size()) && i<limit;++i) {
    const auto target=start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(frames[i].time-frames[0].time));
    if(paced)std::this_thread::sleep_until(target);
    const auto image=pgm(frames[i].path,camera.width,camera.height);
    const auto begin=std::chrono::steady_clock::now();
    const double lag=std::max(0.,std::chrono::duration<double,std::milli>(begin-target).count());
    const auto s=engine.process(image.data(),camera.width,frames[i].time,i==first||i==second);
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    output<<s.timestamp;for(int j=0;j<3;++j)output<<','<<s.world_from_camera[j];for(double v:rotation(s.world_from_camera))output<<','<<v;
    output<<','<<s.quality<<','<<s.map_points<<','<<s.keyframes<<','<<s.found<<','<<s.attempted<<','<<s.initialized<<','<<s.finite<<','<<elapsed<<','<<lag<<",0\n";
    output.flush();
    if(!output)throw std::runtime_error("Trajectory write/flush failed");
    if(i%100==0)std::cerr<<"frame="<<i<<" quality="<<s.quality<<" found="<<s.found<<" points="<<s.map_points<<" keyframes="<<s.keyframes<<" elapsed_ms="<<elapsed<<" lag_ms="<<lag<<'\n';
  }
  return 0;
 } catch(const std::exception& e) {std::cerr<<"adapter_error: "<<e.what()<<'\n';return 1;}
}
