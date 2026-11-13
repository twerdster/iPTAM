#pragma once
#import <Foundation/Foundation.h>

enum class IPTAMLaunchMode { Recordings, Replay, Live, InvalidReplay };

// Recorded playback takes precedence. A missing replay argument must never
// fall through to live capture or trigger a permission prompt.
static inline IPTAMLaunchMode IPTAMLaunch(NSArray<NSString*> *arguments, NSString **path) {
    if(path) *path=nil;
    NSUInteger index=[arguments indexOfObject:@"--replay"];
    if(index!=NSNotFound) {
        if(index+1>=arguments.count || !arguments[index+1].length || [arguments[index+1] hasPrefix:@"--"])
            return IPTAMLaunchMode::InvalidReplay;
        if(path) *path=arguments[index+1];
        return IPTAMLaunchMode::Replay;
    }
    return [arguments containsObject:@"--live"]?IPTAMLaunchMode::Live:IPTAMLaunchMode::Recordings;
}
