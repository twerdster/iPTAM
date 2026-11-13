#include "MobileMappingGeometry.h"
#include <iostream>
#include <limits>
#include <stdexcept>
static void check(bool good,const char* message){if(!good)throw std::runtime_error(message);}
int main() {
 using IPTAMMobile::EpipolarDepthInterval;
 auto original=EpipolarDepthInterval(12.4154,6.40717,.1,false);
 check(!original.valid && original.lower>original.upper,"actual right-camera reversal was not reproduced");
 auto repaired=EpipolarDepthInterval(12.4154,6.40717,.1,true);
 check(repaired.valid && repaired.repaired && repaired.lower==original.lower && repaired.upper==12.4154+6.40717,"scene-derived depth repair failed");
 auto scaled=EpipolarDepthInterval(7*12.4154,7*6.40717,.7,true);
 check(scaled.valid && std::abs(scaled.lower/7-repaired.lower)<1e-10 && std::abs(scaled.upper/7-repaired.upper)<1e-10,"depth repair changed with monocular gauge scale");
 auto normal=EpipolarDepthInterval(2.,.5,.1,true);
 check(normal.valid && !normal.repaired && normal.lower==1.5 && normal.upper==2.5,"valid original interval changed");
 check(!EpipolarDepthInterval(12.,0.,.1,true).valid,"zero-width unobservable interval accepted");
 check(!EpipolarDepthInterval(std::numeric_limits<double>::infinity(),1.,.1,true).valid,"nonfinite scene accepted");
 check(!EpipolarDepthInterval(2.,-1.,.1,true).valid,"negative depth uncertainty accepted");
 std::cout<<"Depth-window regression: reversed real scene, gauge scaling, unchanged valid bounds, invalid geometry passed\n";
}
