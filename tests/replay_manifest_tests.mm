#import <Foundation/Foundation.h>
#include "../ios/ReplayManifest.h"
#include "../ios/AppLaunch.h"
#include <iostream>
#include <cstdlib>

static NSMutableDictionary *Fixture() {
    return [@{@"width":@640,@"height":@480,@"fx":@500,@"fy":@500,@"cx":@320,@"cy":@240,
        @"init_first":@0,@"init_second":@1,
        @"frames":@[@{@"path":@"frames/000.pgm",@"timestamp":@(-0.0015)},
                    @{@"path":@"frames/001.pgm",@"timestamp":@0.03}]} mutableCopy];
}
static int checks=0;
static void Check(id value,bool expected,const char *name) {
    NSString *reason=nil;bool accepted=IPTAMReplayManifest(value,&reason)!=nil;checks++;
    if(accepted!=expected || (!accepted && !reason.length)) {
        std::cerr<<"FAIL "<<name<<" reason="<<(reason?reason.UTF8String:"")<<'\n';std::exit(1);
    }
}
int main() {@autoreleasepool {
    NSString *path=nil;
    if(IPTAMLaunch(@[@"iPTAM"],&path)!=IPTAMLaunchMode::Recordings || path ||
       IPTAMLaunch(@[@"iPTAM",@"--live"],nil)!=IPTAMLaunchMode::Live ||
       IPTAMLaunch(@[@"iPTAM",@"--replay"],nil)!=IPTAMLaunchMode::InvalidReplay ||
       IPTAMLaunch(@[@"iPTAM",@"--replay",@"--live"],nil)!=IPTAMLaunchMode::InvalidReplay ||
       IPTAMLaunch(@[@"iPTAM",@"--live",@"--replay",@"replays/test/manifest.json"],&path)!=IPTAMLaunchMode::Replay ||
       ![path isEqualToString:@"replays/test/manifest.json"])
        {std::cerr<<"FAIL recording-first launch routing\n";return 1;}
    Check(Fixture(),true,"negative initial timestamp accepted");
    auto native=Fixture();native[@"lens_model"]=@"equidistant";Check(native,true,"native equidistant lens");
    native=Fixture();native[@"lens_model"]=@"pinhole";Check(native,true,"explicit pinhole lens");
    native=Fixture();native[@"lens_model"]=@"radtan";Check(native,false,"unsupported lens");
    native=Fixture();native[@"lens_model"]=NSNull.null;Check(native,false,"invalid lens type");
    Check(@[],false,"array root");Check(NSNull.null,false,"null root");
    auto m=Fixture();m[@"fx"]=@YES;Check(m,false,"Boolean focal length");
    m=Fixture();[m removeObjectForKey:@"cx"];Check(m,false,"missing principal point");
    m=Fixture();m[@"width"]=@640.5;Check(m,false,"fractional dimensions");
    m=Fixture();m[@"height"]=@63;Check(m,false,"undersized image");
    m=Fixture();m[@"height"]=@481;Check(m,false,"pyramid alignment");
    m=Fixture();m[@"fx"]=@(-1);Check(m,false,"negative focal length");
    m=Fixture();m[@"fx"]=@(NAN);Check(m,false,"nonfinite focal length");
    m=Fixture();m[@"frames"]=@[@1,NSNull.null];Check(m,false,"invalid frame types");
    m=Fixture();m[@"frames"]=@[@{@"path":NSNull.null,@"timestamp":@0},@{}];Check(m,false,"null path");
    m=Fixture();m[@"frames"]=@[@{@"path":@"/tmp/frame.pgm",@"timestamp":@0},@{}];Check(m,false,"absolute path");
    m=Fixture();m[@"frames"]=@[@{@"path":@"../frame.pgm",@"timestamp":@0},@{}];Check(m,false,"parent path");
    m=Fixture();m[@"frames"]=@[@{@"path":@"a.pgm",@"timestamp":@0},@{@"path":@"b.pgm",@"timestamp":@0}];Check(m,false,"duplicate timestamp");
    m=Fixture();m[@"frames"]=@[@{@"path":@"a.pgm",@"timestamp":@"0"},@{}];Check(m,false,"string timestamp");
    m=Fixture();m[@"init_second"]=@0;Check(m,false,"coincident initialization");
    m=Fixture();m[@"init_second"]=@2;Check(m,false,"initialization outside sequence");
    m=Fixture();m[@"init_first"]=@0.5;Check(m,false,"fractional initialization");
    m=Fixture();m[@"calibration"]=NSNull.null;Check(m,false,"invalid calibration");
    std::cout<<"Replay manifest: "<<checks<<" validation regressions passed\n";
}}
