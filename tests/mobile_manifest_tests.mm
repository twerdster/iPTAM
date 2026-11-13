#import <Foundation/Foundation.h>
#include "../ios/MobileReplay.h"
#include <iostream>
static bool Accept(NSDictionary *m) {
  try {IPTAMConfigureRollingShutter(m);return true;} catch(...) {return false;}
}
int main() {@autoreleasepool {
  using namespace IPTAMMobile;
  NSMutableDictionary *m=[@{@"width":@640,@"height":@480} mutableCopy];
  if(!Accept(m) || Config().rollingShutter) return 1;
  NSMutableDictionary *r=[@{@"enabled":@YES,@"verified":@YES,@"coordinate_space":@"prepared_image",
    @"source":@"synthetic fixture calibrated in supplied pixel coordinates",@"axis":@"rows",
    @"readout_seconds":@.02,@"direction":@1,@"timestamp_fraction":@.5} mutableCopy];
  m[@"rolling_shutter"]=r;
  if(!Accept(m) || !Config().rollingShutter || Config().readoutSeconds!=.02) return 2;
  for(NSString *key in @[@"verified",@"source",@"axis",@"direction",@"timestamp_fraction",@"readout_seconds"]){
    id value=r[key];[r removeObjectForKey:key];
    if(Accept(m) || Config().rollingShutter) return 3;
    r[key]=value;
  }
  r[@"direction"]=@1.5;if(Accept(m)) return 4;r[@"direction"]=@(-1);
  r[@"coordinate_space"]=@"raw_sensor";if(Accept(m)) return 5;r[@"coordinate_space"]=@"prepared_image";
  r[@"readout_seconds"]=@(NAN);if(Accept(m)) return 6;r[@"readout_seconds"]=@.01;
  r[@"timestamp_fraction"]=@YES;if(Accept(m)) return 7;r[@"timestamp_fraction"]=@.5;
  if(!Accept(m) || Config().readoutDirection!=-1) return 8;
  m[@"rolling_shutter"]=@{@"enabled":@NO};if(!Accept(m) || Config().rollingShutter) return 9;
  std::cout<<"mobile manifest tests passed\n";return 0;
}}
