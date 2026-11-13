#include "Bundle.h"
#include "MobileMappingBudget.h"
#include "MobileReplaySupport.h"
#include <iostream>
#include <stdexcept>
static void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
int main() {
  try {
    using namespace IPTAMMobile;
    require(!MappingBudgetExpired(),"default deadline unexpectedly active");
    const double future=MappingSteadySeconds()+60;
    {
      ScopedMappingDeadline outer(future);
      {ScopedMappingDeadline inner(future+60);require(MappingDeadlineSeconds()==future,"nested budget extended caller deadline");}
      require(MappingDeadlineSeconds()==future,"nested budget did not restore caller");
    }
    require(MappingDeadlineSeconds()==0,"budget leaked outside scope");
    ATANCamera camera("budget");camera.SetParameters(TooN::makeVector(.625,.833333333333,.5,.5,0.));
    Bundle bundle(camera);const auto original=TooN::makeVector(.1,.2,2.);
    int c=bundle.AddCamera(TooN::SE3<>(),true),p=bundle.AddPoint(original);
    bundle.AddMeas(c,p,camera.Project(TooN::project(original)),1.);
    bool abort=false;
    {
      ScopedMappingDeadline expired(MappingSteadySeconds()-.001);
      require(bundle.Compute(&abort)==0,"expired budget should yield without a trial");
      require(bundle.BudgetStopped(),"budget yield missing diagnostic");
      require(!bundle.Converged(),"budget yield falsely marked convergence");
      require(bundle.FailureReason().empty(),"budget yield falsely marked numerical failure");
      require(TooN::norm(bundle.GetPoint(p)-original)==0,"budget yield changed committed point");
    }
    require(bundle.Compute(&abort)>=0,"budget-yielded bundle could not resume");
    require(!bundle.BudgetStopped(),"budget flag survived a fresh unbudgeted compute");
    ApplyProfile("diagnostics");require(!Config().coarseTracking && !Config().repairDepthIntervals && !Config().idleMapping,"diagnostics profile changes estimator");
    ApplyOption("--mobile-depth-repair","1");require(Config().repairDepthIntervals,"isolated repair override missing");
    bool rejected=false;try {ApplyOption("--mobile-idle-ms","nan");} catch(...) {rejected=true;}
    require(rejected,"nonfinite budget accepted");
    rejected=false;try {ApplyOption("--mobile-idle-ms","0");} catch(...) {rejected=true;}
    require(rejected,"zero budget could silently starve mapping");
    ApplyOption("--mobile-idle-mapping","0");require(!Config().idleMapping,"explicit deterministic scheduling control missing");
    ApplyProfile("auto");require(Config().automaticInitialization && Config().coarseTracking,"auto profile incomplete");
    ResetConfig();std::cout<<"mobile budget tests passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
