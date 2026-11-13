#pragma once
#import <Foundation/Foundation.h>
#include <cmath>
#include <limits>

// Validate before any keyed subscripting or numeric conversion in the replay loop.
static inline bool IPTAMNumber(id value) {
    return [value isKindOfClass:NSNumber.class] && CFGetTypeID((__bridge CFTypeRef)value)!=CFBooleanGetTypeID()
        && std::isfinite([value doubleValue]);
}
static inline NSDictionary *IPTAMReplayManifest(id value, NSString **reason) {
    auto fail=[reason](NSString *text)->NSDictionary* {if(reason) *reason=text;return nil;};
    if(![value isKindOfClass:NSDictionary.class]) return fail(@"Replay root must be an object");
    NSDictionary *m=value;
    id lens=m[@"lens_model"];
    if(lens && (![lens isKindOfClass:NSString.class] ||
                (![(NSString*)lens isEqualToString:@"pinhole"] && ![(NSString*)lens isEqualToString:@"equidistant"])))
        return fail(@"Replay lens_model must be pinhole or equidistant");
    for(NSString *key in @[@"width",@"height",@"fx",@"fy",@"cx",@"cy",@"init_first",@"init_second"])
        if(!IPTAMNumber(m[key])) return fail([@"Missing or invalid numeric replay field: " stringByAppendingString:key]);
    double w=[m[@"width"] doubleValue],h=[m[@"height"] doubleValue];
    if(w<64 || h<64 || w>2048 || h>2048 || std::floor(w)!=w || std::floor(h)!=h || (int)w%8 || (int)h%8)
        return fail(@"Replay dimensions must be multiples of 8 between 64 and 2048");
    if([m[@"fx"] doubleValue]<=0 || [m[@"fy"] doubleValue]<=0) return fail(@"Replay focal lengths must be positive");
    NSArray *frames=m[@"frames"];
    if(![frames isKindOfClass:NSArray.class] || frames.count<2) return fail(@"Replay must contain at least two frames");
    double first=[m[@"init_first"] doubleValue],second=[m[@"init_second"] doubleValue];
    if(first<0 || std::floor(first)!=first || std::floor(second)!=second || second<=first || second>=frames.count)
        return fail(@"Replay initialization indices must satisfy 0 <= first < second < frame count");
    if(m[@"calibration"] && ![m[@"calibration"] isKindOfClass:NSDictionary.class]) return fail(@"Replay calibration must be an object");
    double previous=-std::numeric_limits<double>::infinity();
    for(NSUInteger i=0;i<frames.count;i++) {
        id raw=frames[i];
        if(![raw isKindOfClass:NSDictionary.class]) return fail([NSString stringWithFormat:@"Replay frame %lu must be an object",(unsigned long)i]);
        NSDictionary *entry=raw;
        NSString *path=entry[@"path"];
        if(![path isKindOfClass:NSString.class] || !path.length || path.isAbsolutePath || [path.pathComponents containsObject:@".."])
            return fail([NSString stringWithFormat:@"Replay frame %lu needs a relative image path",(unsigned long)i]);
        if(!IPTAMNumber(entry[@"timestamp"]) || [entry[@"timestamp"] doubleValue]<=previous)
            return fail([NSString stringWithFormat:@"Replay frame %lu timestamp must be finite and increasing",(unsigned long)i]);
        previous=[entry[@"timestamp"] doubleValue];
    }
    return m;
}
