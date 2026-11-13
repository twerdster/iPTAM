#include "Bundle.h"
#include "Cholesky.h"
#include <fstream>
#include <sstream>
#include <set>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
static void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
static int solve(Bundle& bundle) { bool abort=false; return bundle.Compute(&abort); }
int main() {
 try {
  auto start=std::chrono::steady_clock::now();
  ATANCamera camera("bundle-regression");
  camera.SetParameters(TooN::makeVector(400./640.,400./480.,.5,.5,0.));
  {
   // All input coordinates are finite. Squaring this image residual overflows.
   // The historical outer LM loop never advanced its iteration counter here.
   Bundle bundle(camera); int c=bundle.AddCamera(TooN::SE3<>(),true);
   int p=bundle.AddPoint(TooN::makeVector(.1,.2,2.));
   bundle.AddMeas(c,p,TooN::makeVector(1e200,1e200),1.);
   require(solve(bundle)<0,"overflowing finite residual was not rejected");
   require(!bundle.FailureReason().empty(),"solver rejection has no diagnostic");
   require(TooN::norm(bundle.GetPoint(p)-TooN::makeVector(.1,.2,2.))==0,"failed trial changed a point");
  }
  {
   Bundle bundle(camera); int c=bundle.AddCamera(TooN::SE3<>(),false);
   int p=bundle.AddPoint(TooN::makeVector(0.,0.,2.));
   bundle.AddMeas(c,p,camera.Project(TooN::makeVector(0.,0.)),1.);
   require(solve(bundle)<0,"singular camera system was not rejected");
   require(bundle.NumericalRejections()>0,"singular solve did not record bounded retries");
   require(TooN::norm(bundle.GetCamera(c).ln())==0,"singular trial changed camera pose");
  }
  for(double variance:{0.,-1.,std::numeric_limits<double>::infinity()}) {
   Bundle bundle(camera);int c=bundle.AddCamera(TooN::SE3<>(),true);
   int p=bundle.AddPoint(TooN::makeVector(0.,0.,2.));
   bundle.AddMeas(c,p,TooN::makeVector(320.,240.),variance);
   require(solve(bundle)<0,"invalid measurement variance was accepted");
  }
  {
   Bundle bundle(camera);int c=bundle.AddCamera(TooN::SE3<>(),true);
   int p=bundle.AddPoint(TooN::makeVector(std::numeric_limits<double>::quiet_NaN(),0.,2.));
   bundle.AddMeas(c,p,TooN::makeVector(320.,240.),1.);
   require(solve(bundle)<0,"NaN point was accepted");
  }
  {
   // A calibrated, finite, constrained two-view system remains solvable.
   Bundle bundle(camera);TooN::SE3<> moved;
   moved.get_translation()=TooN::makeVector(.1,0.,0.);
   int a=bundle.AddCamera(TooN::SE3<>(),true),b=bundle.AddCamera(moved,false);
   for(int y=-2;y<=2;++y)for(int x=-3;x<=3;++x) {
    auto point=TooN::makeVector(x*.15,y*.15,2.+.1*((x*x+y*y)%3));
    int p=bundle.AddPoint(point);
    bundle.AddMeas(a,p,camera.Project(TooN::project(point)),1.);
    bundle.AddMeas(b,p,camera.Project(TooN::project(moved*point)),1.);
   }
   require(solve(bundle)>=0,"valid calibrated bundle was rejected");
   require(bundle.FailureReason().empty(),"valid solve retained a rejection diagnostic");
  }
  {
   // Real iPhone failure: a symmetric, finite Schur system with a negative
   // LDL pivot after16accepted improvements and4later trial failures.
   std::ifstream input(std::string(IPTAM_TEST_FIXTURE_DIR)+"/phone_schur_20261112.txt");
   std::string line; std::ostringstream data;
   while(std::getline(input,line)) if(!line.empty()&&line[0]!='#') data<<line<<'\n';
   std::istringstream values(data.str()); TooN::Matrix<6> matrix;
   for(int r=0;r<6;++r)for(int c=0;c<6;++c) require(bool(values>>matrix[r][c]),"truncated phone Schur fixture");
   auto diagonal=TooN::Cholesky<6>(matrix).get_D();bool rejected=false;
   for(int i=0;i<6;++i) if(diagonal[i][i]<=0 || !std::isfinite(diagonal[i][i])) rejected=true;
   require(rejected,"recorded phone trial incorrectly classified as positive definite");
  }
  for(bool defaultPolicy:{true,false}) {
   // This finite multi-depth cloud with image outliers naturally reaches a
   // late singular trial after17 accepted steps. Preserve actual optimization
   // and committed outliers, rather than fabricating state flags in the test.
   std::ifstream input(std::string(IPTAM_TEST_FIXTURE_DIR)+"/bundle_late_singular_20261112.txt");
   std::string line;std::ostringstream data;
   while(std::getline(input,line)) if(!line.empty()&&line[0]!='#') data<<line<<'\n';
   std::istringstream values(data.str());std::string label;TooN::Vector<6> twist;
   require(bool(values>>label)&&label=="camera_twist","missing fixture camera twist");
   for(int j=0;j<6;++j) require(bool(values>>twist[j]),"truncated fixture camera twist");
   double baseline;int count;
   require(bool(values>>label>>baseline)&&label=="baseline","missing fixture baseline");
   require(bool(values>>label>>count)&&label=="points"&&count==80,"invalid fixture point count");
   TooN::SE3<> truth;truth.get_translation()=TooN::makeVector(baseline,0.,0.);
   TooN::SE3<> moved=TooN::SE3<>::exp(twist)*truth;
   std::unique_ptr<Bundle> configured(defaultPolicy?new Bundle(camera):new Bundle(camera,1e-6));
   Bundle& bundle=*configured;
   int a=bundle.AddCamera(TooN::SE3<>(),true),b=bundle.AddCamera(moved,false);
   for(int i=0;i<count;++i) {
    TooN::Vector<3> point;TooN::Vector<2> first,second;
    for(int j=0;j<3;++j)require(bool(values>>point[j]),"truncated fixture point");
    for(int j=0;j<2;++j)require(bool(values>>first[j]),"truncated fixture first measurement");
    for(int j=0;j<2;++j)require(bool(values>>second[j]),"truncated fixture second measurement");
    int id=bundle.AddPoint(point);bundle.AddMeas(a,id,first,1.);bundle.AddMeas(b,id,second,1.);
   }
   int accepted=solve(bundle);
   require(accepted>0,"late singular trial discarded accepted optimization");
   if(defaultPolicy) {
    require(bundle.StoppedAtAcceptedEstimate(),"late singular fixture did not retain accepted iterate");
    require(!bundle.Converged(),"bounded singular stop falsely reported convergence");
   } else {
    require(!bundle.StoppedAtAcceptedEstimate(),"experimental damping floor did not prevent late singularity");
    require(bundle.NumericalRejections()==0,"experimental damping fixture rejected a trial system");
   }
   require(int(bundle.AcceptedObjectives().size())==accepted,"accepted objective trace incomplete");
   for(const auto& objective:bundle.AcceptedObjectives())
    require(std::isfinite(objective.first)&&std::isfinite(objective.second)&&objective.second<objective.first,"accepted objective was not finite and decreasing");
   for(int i=0;i<count;++i)for(int j=0;j<3;++j)require(std::isfinite(bundle.GetPoint(i)[j]),"retained nonfinite point");
   std::set<std::pair<int,int>> unique;
   for(const auto& outlier:bundle.GetOutlierMeasurements()) {
    require(outlier.first>=0&&outlier.first<count&&outlier.second>=0&&outlier.second<2,"invalid committed outlier index");
    require(unique.insert(outlier).second,"duplicate committed outlier index");
   }
   std::cout<<(defaultPolicy?"Default minimum0: ":"Experimental minimum1e-6: ")<<"fixture retained "<<accepted<<" accepted steps and "<<unique.size()<<" committed outliers\n";
  }
  require(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<2.,"solver regression exceeded bounded runtime");
  std::cout<<"Bundle solver: overflow, singularity, invalid input, rollback and valid-system checks passed\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
