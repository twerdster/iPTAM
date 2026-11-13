#pragma once
#import "ReplayManifest.h"
#include "MobileConfig.h"
#include "RollingShutter.h"
#include <stdexcept>

// Timing must describe the supplied, prepared pixels. A sensor-row number is
// insufficient after arbitrary rectification, cropping or image rotation.
static inline void IPTAMConfigureRollingShutter(NSDictionary *manifest) {
    using namespace IPTAMMobile;
    Config().rollingShutter=false;
    id raw=manifest[@"rolling_shutter"];
    if(!raw) return;
    if(![raw isKindOfClass:NSDictionary.class]) throw std::runtime_error("rolling_shutter must be an object");
    NSDictionary *r=raw;
    if([r[@"enabled"] isEqual:@NO]) return;
    if(![r[@"enabled"] isEqual:@YES] || ![r[@"verified"] isEqual:@YES] ||
       ![r[@"coordinate_space"] isEqual:@"prepared_image"] ||
       ![r[@"source"] isKindOfClass:NSString.class] || ![(NSString*)r[@"source"] length])
        throw std::runtime_error("Rolling shutter requires verified timing, a source and prepared_image coordinates");
    for(NSString *key in @[@"readout_seconds",@"direction",@"timestamp_fraction"])
        if(!IPTAMNumber(r[key])) throw std::runtime_error("Missing or invalid rolling shutter numeric field");
    RollingShutterCalibration calibration;
    calibration.known=true;calibration.width=[manifest[@"width"] intValue];calibration.height=[manifest[@"height"] intValue];
    if([r[@"axis"] isEqual:@"columns"]) calibration.axis=0;
    else if([r[@"axis"] isEqual:@"rows"]) calibration.axis=1;
    else throw std::runtime_error("Rolling shutter axis must be rows or columns");
    double direction=[r[@"direction"] doubleValue];
    if(direction!=1. && direction!=-1.) throw std::runtime_error("Rolling shutter direction must be +1 or -1");
    calibration.direction=int(direction);calibration.readoutSeconds=[r[@"readout_seconds"] doubleValue];
    calibration.timestampFraction=[r[@"timestamp_fraction"] doubleValue];
    std::string reason;if(!calibration.Validate(reason)) throw std::runtime_error(reason);
    Config().readoutSeconds=calibration.readoutSeconds;Config().readoutAxis=calibration.axis;
    Config().readoutDirection=calibration.direction;Config().exposureTimestampFraction=calibration.timestampFraction;
    Config().rollingShutter=true;
}
