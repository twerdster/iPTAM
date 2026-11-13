// Manual image-only diagnostic probe; intentionally not a CTest dataset test.
#include "Tracker.h"
#include "MobileConfig.h"
#include <fstream>
#include <iostream>
#include <string>
static std::string token(std::istream& in) {std::string s;while(in>>s){if(s[0]=='#'){std::getline(in,s);continue;}return s;}throw std::runtime_error("PGM header");}
int main(int argc,char** argv) {
 if(argc<2||argc>3)return 2;
 auto& config=IPTAMMobile::Config();config.coarseTracking=config.partialPoints=config.cullMeasurements=config.cullKeyframes=config.idleMapping=false;
 config.repairDepthIntervals=argc==3;
 ATANCamera camera("mapping-probe");camera.SetImageSize(CVD::ImageRef(640,480));camera.SetParameters(TooN::makeVector(460./640,460./480,.5,.5,0.));
 Map map;MapMaker maker(map,camera);Tracker tracker(CVD::ImageRef(640,480),camera,map,maker);
 std::ifstream csv(std::string(argv[1])+"/frames.csv");std::string line;std::getline(csv,line);
 for(int i=0;i<200&&std::getline(csv,line);++i) {
  if(!line.empty()&&line.back()=='\r')line.pop_back();auto comma=line.find(',');double timestamp=std::stod(line.substr(0,comma));
  std::ifstream pgm(std::string(argv[1])+"/"+line.substr(comma+1),std::ios::binary);
  if(token(pgm)!="P5")return 3;int w=std::stoi(token(pgm)),h=std::stoi(token(pgm));if(token(pgm)!="255")return 3;pgm.get();
  CVD::Image<CVD::byte> image(CVD::ImageRef(w,h));pgm.read(reinterpret_cast<char*>(image.data()),w*h);
  if(i==0||i==5)tracker.StartTracking();tracker.SetFrameTimestamp(timestamp);tracker.TrackFrame(image,0,false);
  if(i==5 || (i>5 && i%10==0)) {
   auto& d=maker.Diagnostics();std::cout<<"frame="<<i<<" P="<<map.vpPoints.size()<<" KF="<<map.vpKeyFrames.size()<<" attempts="<<d.epipolarAttempts<<" accepted="<<d.epipolarAccepted<<" invalid_depth="<<d.invalidDepthIntervals<<" depth="<<d.lastDepthMean<<" sigma="<<d.lastDepthSigma<<" interval="<<d.lastDepthStart<<","<<d.lastDepthEnd<<" rejected=";
   for(auto n:d.rejected)std::cout<<n<<",";std::cout<<std::endl;
  }
 }
}
