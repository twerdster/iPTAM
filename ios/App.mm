// Modern capture and test shell around the original PTAM implementation.
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <simd/simd.h>
#import <CommonCrypto/CommonDigest.h>
#import "ReplayManifest.h"
#import "AppLaunch.h"
#ifdef IPTAM_UPSTREAM
#include "OriginalPTAM.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#else
#include "Tracker.h"
#include "MapPoint.h"
#endif
#ifdef IPTAM_MOBILE_PAPER
#include "MobileReplaySupport.h"
#include "MobileReplay.h"
#endif
#include <memory>
#include <fstream>
#include <chrono>
#include <cmath>
#include <sstream>
#include <atomic>
#include <limits>
#include <mach/mach.h>

#ifdef IPTAM_MOBILE_PAPER
static NSDictionary *MobileJSON(const std::string& json) {
    NSData *data=[NSData dataWithBytes:json.data() length:json.size()];
    return [NSJSONSerialization JSONObjectWithData:data options:0 error:nil];
}
static void ConfigureMobileReplay() {
    NSArray<NSString*> *arguments=NSProcessInfo.processInfo.arguments;
    NSUInteger profile=[arguments indexOfObject:@"--mobile-profile"];
    if(profile!=NSNotFound && profile+1>=arguments.count) throw std::runtime_error("Missing mobile profile");
    IPTAMMobile::ApplyProfile(profile==NSNotFound?"auto":arguments[profile+1].UTF8String);
    for(NSUInteger i=0;i<arguments.count;++i) if([arguments[i] hasPrefix:@"--mobile-"]) {
        if(i+1>=arguments.count || [arguments[i+1] hasPrefix:@"--"]) throw std::runtime_error("Missing mobile option value");
        if(![arguments[i] isEqualToString:@"--mobile-profile"])
            IPTAMMobile::ApplyOption(arguments[i].UTF8String,arguments[i+1].UTF8String);
        ++i;
    }
}
#endif

static NSString *Documents() {
    return NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
}
static NSString *DigestString(const unsigned char *digest) {
    NSMutableString *result=[NSMutableString stringWithCapacity:CC_SHA256_DIGEST_LENGTH*2];
    for(int i=0;i<CC_SHA256_DIGEST_LENGTH;i++) [result appendFormat:@"%02x",digest[i]];
    return result;
}
static UIImage *GrayImage(const CVD::Image<CVD::byte>& image) {
    const size_t w=image.size().x, h=image.size().y;
    NSData *data=[NSData dataWithBytes:image.data() length:w*h];
    CGDataProviderRef provider=CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    CGColorSpaceRef space=CGColorSpaceCreateDeviceGray();
    CGImageRef cg=CGImageCreate(w,h,8,8,w,space,kCGImageAlphaNone,provider,nullptr,false,kCGRenderingIntentDefault);
    UIImage *result=[UIImage imageWithCGImage:cg];
    CGImageRelease(cg); CGColorSpaceRelease(space); CGDataProviderRelease(provider);
    return result;
}
static bool ReadPGM(NSString *path, CVD::Image<CVD::byte>& image) {
    std::ifstream f(path.fileSystemRepresentation, std::ios::binary);
    auto token=[&f]() {
        std::string s;
        while(f >> s) {
            if(s[0]=='#') {std::getline(f,s); continue;}
            return s;
        }
        return std::string();
    };
    if(token()!="P5") return false;
    try {
        int w=std::stoi(token()), h=std::stoi(token()), max=std::stoi(token());
        if(w<64 || h<64 || w>2048 || h>2048 || max!=255) return false;
        char separator=0; f.get(separator);
        if(!std::isspace(static_cast<unsigned char>(separator))) return false;
        if(separator=='\r' && f.peek()=='\n') f.get();
        image.resize(CVD::ImageRef(w,h));
        f.read(reinterpret_cast<char*>(image.data()),size_t(w)*h);
        return f.gcount()==size_t(w)*h;
    } catch(...) {return false;}
}

#ifdef IPTAM_UPSTREAM
class Engine {
public:
    int width,height;
    double fx,fy,cx,cy;
    std::unique_ptr<original_ptam::Engine> original;
    bool initializePending=false;
    Engine(int w,int h,double x,double y,double u,double v,bool equidistant=false)
      :width(w),height(h),fx(x),fy(y),cx(u),cy(v) {
        if(equidistant) throw std::runtime_error("Upstream comparison supports the common pinhole recordings only");
        original_ptam::Camera camera;
        camera.width=w;camera.height=h;camera.fx=x;camera.fy=y;camera.cx=u;camera.cy=v;camera.w=0;
        original.reset(new original_ptam::Engine(camera));
    }
    void initialize() {initializePending=true;}
    NSDictionary *process(CVD::Image<CVD::byte>& image,double timestamp) {
        auto start=std::chrono::steady_clock::now();
        auto state=original->process(image.data(),image.size().x,timestamp,initializePending);
        initializePending=false;
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        NSMutableArray *matrix=[NSMutableArray arrayWithCapacity:12];
        for(double a:state.camera_from_world) [matrix addObject:std::isfinite(a)?@(a):NSNull.null];
        bool valid=state.initialized && state.quality==2 && state.found>=10 && state.finite;
        NSMutableDictionary *result=[@{@"timestamp":@(timestamp),@"initialized":@(state.initialized),
          @"quality":@(state.quality),@"valid_visual_pose":@(valid),@"finite":@(state.finite),
          @"found":@(state.found),@"attempted":@(state.attempted),
          @"pose_support_diagnostics_available":@NO,
          @"trails":@(state.trails),@"map_points":@(state.map_points),@"keyframes":@(state.keyframes),
          @"solver_diagnostics_available":@(state.solver_diagnostics_available),
          @"processing_ms":@(ms),@"camera_from_world_3x4":matrix,
          @"message":[NSString stringWithUTF8String:state.message.c_str()]?:@""} mutableCopy];
        if(state.finite) {
            NSMutableArray *inverse=[NSMutableArray arrayWithCapacity:7];
            for(double a:state.world_from_camera) [inverse addObject:@(a)];
            result[@"world_from_camera_t_qxyzw"]=inverse;
        }
        return result;
    }
};
#else
class Engine {
public:
    ATANCamera camera;
    Map map;
    std::unique_ptr<MapMaker> mapper;
    std::unique_ptr<Tracker> tracker;
    int width,height;
    double fx,fy,cx,cy;
    Engine(int w,int h,double x,double y,double u,double v,
           bool equidistant=false):camera("Camera"),width(w),height(h),fx(x),fy(y),cx(u),cy(v) {
        if(w<64 || h<64 || w%8 || h%8 || !std::isfinite(x+y+u+v) || x<=0 || y<=0)
            throw std::runtime_error("Invalid image size or intrinsics");
        camera.SetParameters(TooN::makeVector(x/w,y/h,(u+.5)/w,(v+.5)/h,0.0));
        camera.SetProjectionModel(equidistant?ATANCamera::ProjectionModel::Equidistant:ATANCamera::ProjectionModel::FOV);
        camera.SetImageSize(CVD::ImageRef(w,h));
        mapper.reset(new MapMaker(map,camera,false));
        tracker.reset(new Tracker(CVD::ImageRef(w,h),camera,map,*mapper));
    }
    ~Engine() {tracker.reset();mapper.reset();map.Reset();}
    void initialize() {tracker->StartTracking();}
#ifdef IPTAM_MOBILE_PAPER
    unsigned idle(double deadline) {return mapper->ProcessIdleWork(deadline,IPTAMMobile::Config().idleBudgetMs);}
#endif
    NSDictionary *process(CVD::Image<CVD::byte>& image, double timestamp) {
        auto start=std::chrono::steady_clock::now();
        tracker->SetFrameTimestamp(timestamp);
        tracker->TrackFrame(image,0,false);
        auto pose=tracker->GetCurrentPose();
        NSMutableArray *matrix=[NSMutableArray arrayWithCapacity:12];
        bool finite=true;
        for(int r=0;r<3;r++) {
            for(int c=0;c<3;c++) {double a=pose.get_rotation().get_matrix()[r][c];finite &= std::isfinite(a);[matrix addObject:std::isfinite(a)?@(a):NSNull.null];}
            double a=pose.get_translation()[r];finite &= std::isfinite(a);[matrix addObject:std::isfinite(a)?@(a):NSNull.null];
        }
        int quality=tracker->TrackingQuality();
        // A finite predicted pose is not proof of a visual measurement.
        bool valid=map.IsGood() && quality==2 && tracker->MeasurementsFound()>=10 && tracker->PoseInliers()>=10 && finite;
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        NSString *message=[NSString stringWithUTF8String:tracker->GetMessageForUser().c_str()];
        NSMutableDictionary *d=[@{@"timestamp":@(timestamp),@"initialized":@(map.IsGood()),
          @"quality":@(quality),@"valid_visual_pose":@(valid),@"finite":@(finite),
          @"found":@(tracker->MeasurementsFound()),@"attempted":@(tracker->MeasurementsAttempted()),
          @"pose_inliers":@(tracker->PoseInliers()),
          @"trails":@(tracker->InitialTrailCount()),@"map_points":@(map.vpPoints.size()),
          @"retired_map_points":@(map.vpPointsTrash.size()),
          @"initial_displacement_px":@(tracker->InitialMedianDisplacement()),
          @"solver_failures":@(mapper->NumericalFailureCount()),
          @"solver_retained_stops":@(mapper->RetainedSolveCount()),
          @"last_solver_failure":[NSString stringWithUTF8String:mapper->LastNumericalFailure().c_str()]?:@"",
          @"keyframes":@(map.vpKeyFrames.size()),@"processing_ms":@(ms),
          @"camera_from_world_3x4":matrix,@"message":message?:@""} mutableCopy];
#ifdef IPTAM_MOBILE_PAPER
        d[@"mobile_mapping"]=MobileJSON(IPTAMMobile::DiagnosticsJSON(*mapper));
        d[@"retained_keyframes"]=@(map.vpRetainedKeyFrames.size());
        d[@"mobile_blur_fallback"]=@(tracker->MobileBlurFallback());
        d[@"mobile_coarse_matches"]=@(tracker->MobileCoarseMatches());
        d[@"mobile_initialization_stage"]=[NSString stringWithUTF8String:tracker->MobileInitializationStage().c_str()];
        d[@"rolling_shutter_observations"]=@(tracker->MobileRSObservations());
        d[@"rolling_shutter_corrections"]=@(tracker->MobileRSCorrections());
        if(mapper->ResourceLimitReached()) throw std::runtime_error("Mobile map resource limit: "+mapper->ResourceLimitReason());
#endif
        if(finite) {
            auto inverse=pose.inverse();
            auto r=inverse.get_rotation().get_matrix();
            simd_double3x3 m={simd_make_double3(r[0][0],r[1][0],r[2][0]),
                             simd_make_double3(r[0][1],r[1][1],r[2][1]),
                             simd_make_double3(r[0][2],r[1][2],r[2][2])};
            simd_quatd q=simd_quaternion(m);
            auto t=inverse.get_translation();
            d[@"world_from_camera_t_qxyzw"]=@[@(t[0]),@(t[1]),@(t[2]),@(q.vector.x),@(q.vector.y),@(q.vector.z),@(q.vector.w)];
        }
        return d;
    }
};
#endif

@interface CameraController : UIViewController <AVCaptureVideoDataOutputSampleBufferDelegate> {
    std::unique_ptr<Engine> engine;
    CVD::Image<CVD::byte> gray;
    dispatch_queue_t worker,captureControl;
    AVCaptureSession *session;
    AVCaptureDevice *device;
    UIImageView *preview;
    UILabel *status,*details,*guidance;
    UIButton *initializeButton,*resetButton;
    NSFileHandle *logFile;
    NSString *runDirectory,*runID,*mode;
    NSMutableDictionary *summary;
    NSUInteger frames,validFrames,lostFrames,droppedFrames,warmupFrames,uninitializedFrames;
    double lastTimestamp,totalMs,maxMs;
    BOOL focusLocked,configured,active,replaying,stopped,runOpen;
    std::atomic<bool> cancellationRequested,uiPending;
    NSDictionary *replayManifest;
    NSString *replayBase;
    NSUInteger replayIndex;
    CC_SHA256_CTX replayPixelHash;
    NSUInteger pixelHashFrames;
    BOOL replayHashOpen;
    BOOL sourcePaced;
    double replayFirstTimestamp,replayMaxLag;
    std::chrono::steady_clock::time_point replayWallStart;
}
@property(nonatomic,copy) NSString *requestedReplayPath;
@end

@implementation CameraController
- (void)viewDidLoad {
    [super viewDidLoad];
    UIApplication.sharedApplication.idleTimerDisabled=YES;
    self.view.backgroundColor=[UIColor colorWithWhite:.06 alpha:1];
    worker=dispatch_queue_create("iptam.serial",DISPATCH_QUEUE_SERIAL);
    captureControl=dispatch_queue_create("iptam.capture",DISPATCH_QUEUE_SERIAL);
    cancellationRequested=false;uiPending=false;
    preview=[UIImageView new]; preview.contentMode=UIViewContentModeScaleAspectFit;
    preview.backgroundColor=UIColor.blackColor; preview.translatesAutoresizingMaskIntoConstraints=NO;
    status=[UILabel new]; status.font=[UIFont boldSystemFontOfSize:23];status.textColor=UIColor.whiteColor;
    details=[UILabel new];details.font=[UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightRegular];
    details.textColor=UIColor.lightGrayColor;details.numberOfLines=0;
    guidance=[UILabel new];guidance.font=[UIFont systemFontOfSize:15];guidance.textColor=UIColor.whiteColor;guidance.numberOfLines=0;
    initializeButton=[UIButton buttonWithType:UIButtonTypeSystem];
    initializeButton.enabled=NO;
    [initializeButton setTitle:@"Initialize" forState:UIControlStateNormal];
    [initializeButton addTarget:self action:@selector(initializeMap) forControlEvents:UIControlEventTouchUpInside];
    resetButton=[UIButton buttonWithType:UIButtonTypeSystem];[resetButton setTitle:@"Reset" forState:UIControlStateNormal];
    [resetButton addTarget:self action:@selector(resetMap) forControlEvents:UIControlEventTouchUpInside];
    UIButton *exportButton=[UIButton buttonWithType:UIButtonTypeSystem];[exportButton setTitle:@"Export log" forState:UIControlStateNormal];
    [exportButton addTarget:self action:@selector(exportLog) forControlEvents:UIControlEventTouchUpInside];
    UIStackView *buttons=[[UIStackView alloc] initWithArrangedSubviews:@[initializeButton,resetButton,exportButton]];
    buttons.distribution=UIStackViewDistributionFillEqually;
    UILabel *attribution=[UILabel new];attribution.font=[UIFont systemFontOfSize:10];attribution.textColor=UIColor.grayColor;
    attribution.text=@"PTAM © 2008 Isis Innovation Limited · iPTAM by Aaron Wetzler";
    UIStackView *panel=[[UIStackView alloc] initWithArrangedSubviews:@[status,details,guidance,buttons,attribution]];
    panel.axis=UILayoutConstraintAxisVertical;panel.spacing=10;panel.translatesAutoresizingMaskIntoConstraints=NO;
    [self.view addSubview:preview];[self.view addSubview:panel];
    UILayoutGuide *safe=self.view.safeAreaLayoutGuide;
    [NSLayoutConstraint activateConstraints:@[
      [preview.topAnchor constraintEqualToAnchor:safe.topAnchor],[preview.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor],
      [preview.trailingAnchor constraintEqualToAnchor:safe.trailingAnchor],[preview.bottomAnchor constraintEqualToAnchor:panel.topAnchor constant:-12],
      [panel.leadingAnchor constraintEqualToAnchor:safe.leadingAnchor constant:16],[panel.trailingAnchor constraintEqualToAnchor:safe.trailingAnchor constant:-16],
      [panel.bottomAnchor constraintEqualToAnchor:safe.bottomAnchor constant:-8],
      [preview.heightAnchor constraintGreaterThanOrEqualToConstant:120]]];
    status.text=@"iPTAM";guidance.text=@"Point at a textured scene. Tap Initialize, move sideways, then tap again.";
    active=YES;lastTimestamp=-std::numeric_limits<double>::infinity();
    NSNotificationCenter *nc=NSNotificationCenter.defaultCenter;
    [nc addObserver:self selector:@selector(background:) name:UIApplicationDidEnterBackgroundNotification object:nil];
    [nc addObserver:self selector:@selector(foreground:) name:UIApplicationWillEnterForegroundNotification object:nil];
    [nc addObserver:self selector:@selector(runtimeError:) name:AVCaptureSessionRuntimeErrorNotification object:nil];
    [nc addObserver:self selector:@selector(interrupted:) name:AVCaptureSessionWasInterruptedNotification object:nil];
    [nc addObserver:self selector:@selector(resumed:) name:AVCaptureSessionInterruptionEndedNotification object:nil];
    NSString *path=self.requestedReplayPath;
    IPTAMLaunchMode launch=IPTAMLaunch(NSProcessInfo.processInfo.arguments,&path);
    if(self.requestedReplayPath) {path=self.requestedReplayPath;launch=IPTAMLaunchMode::Replay;}
    if(launch==IPTAMLaunchMode::Replay) {
        replaying=YES;initializeButton.enabled=NO;resetButton.enabled=NO;
        self.title=@"Replay";
        guidance.text=@"Processing recorded images with their camera calibration and source timestamps.";
        if(!path.isAbsolutePath) path=[Documents() stringByAppendingPathComponent:path];
        dispatch_async(worker, ^{[self runReplay:path];});
    } else if(launch==IPTAMLaunchMode::Live) {
#if defined(IPTAM_UPSTREAM) || defined(IPTAM_MOBILE_PAPER)
        replaying=YES;
        dispatch_async(worker, ^{[self failRun:@"This comparison app uses existing recordings. Select a prepared recording."];});
#else
        dispatch_async(worker, ^{
            if(![self beginRun:@"camera startup" calibration:@{}]) return;
            self->summary[@"camera_authorization"]=@"requested";
            if(![self saveSummary:@"running"]) {[self failRun:self->summary[@"error"]];return;}
            [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted){
                dispatch_async(self->worker, ^{
                    self->summary[@"camera_authorization"]=granted?@"granted":@"denied";
                    if(self->runOpen && ![self saveSummary:@"running"]) {[self failRun:self->summary[@"error"]];return;}
                    if(granted) [self configureCamera];
                    else [self failRun:@"Camera access required. Enable iPTAM in Settings → Privacy & Security → Camera."];
                });
            }];
        });
#endif
    } else {
        replaying=YES;
        dispatch_async(worker, ^{[self failRun:@"Select a recording, or supply a manifest path after --replay."];});
    }
}
- (void)viewWillDisappear:(BOOL)animated {
    [super viewWillDisappear:animated];
    if(!self.isMovingFromParentViewController) return;
    UIApplication.sharedApplication.idleTimerDisabled=NO;
    cancellationRequested=true;
    [NSNotificationCenter.defaultCenter removeObserver:self];
    dispatch_async(worker, ^{
        self->active=NO;
        [self finishRun:@"user_stopped"];
        self->replayManifest=nil;self->engine.reset();
        AVCaptureSession *current=self->session;
        dispatch_async(self->captureControl, ^{[current stopRunning];});
    });
}
- (void)showError:(NSString*)message {
    NSLog(@"IPTAM_ERROR %@",message);
    dispatch_async(dispatch_get_main_queue(), ^{self->status.text=@"Stopped";self->status.textColor=UIColor.systemOrangeColor;self->guidance.text=message;self->initializeButton.enabled=NO;});
}
- (void)storageFailure:(NSString*)message {
    summary[@"storage_error"]=@YES;summary[@"error"]=message;summary[@"status"]=@"failed";
    NSLog(@"IPTAM_STORAGE_ERROR %@",message);
}
- (BOOL)beginRun:(NSString*)kind calibration:(NSDictionary*)calibration {
    if(![self finishRun:@"replaced"]) return NO;
    mode=kind;runID=NSUUID.UUID.UUIDString;
    runDirectory=[[Documents() stringByAppendingPathComponent:@"runs"] stringByAppendingPathComponent:runID];
    frames=validFrames=lostFrames=droppedFrames=uninitializedFrames=0;totalMs=maxMs=0;
    summary=[@{@"run_id":runID,@"mode":kind,@"calibration":calibration,
#ifdef IPTAM_UPSTREAM
      @"estimator":@"Oxford PTAM v1.0-r114 d9dca71; original asynchronous mapper",
      @"engine_variant":@"upstream",@"solver_diagnostics_available":@NO,
      @"pose_support_diagnostics_available":@NO,
      @"visual_acceptance":@"original GOOD plus >=10 raw matches and finite pose; final-pose support unavailable",
      @"processing_time_scope":@"tracker plus observational export; excludes concurrent mapper CPU",
#elif defined(IPTAM_MOBILE_PAPER)
      @"estimator":@"ISMAR2009-inspired mobile variant of modern iPTAM; serial mapping",
      @"engine_variant":@"mobile",@"solver_diagnostics_available":@YES,
      @"pose_support_diagnostics_available":@YES,
      @"camera_permission_requested":@NO,
      @"visual_acceptance":@"GOOD plus >=10 raw matches, >=10 final-pose robust contributors and finite pose",
      @"processing_time_scope":@"tracker and foreground serial mapper plus export; additional idle mapping logged separately",
#else
      @"estimator":@"modernized iPTAM; serial mapping",
      @"engine_variant":@"current",@"solver_diagnostics_available":@YES,
      @"pose_support_diagnostics_available":@YES,
      @"visual_acceptance":@"GOOD plus >=10 raw matches, >=10 final-pose robust contributors and finite pose",
      @"processing_time_scope":@"tracker and serial mapper plus export",
#endif
      @"lens_model":@"pinhole",
      @"live_resource_policy":@"stop and save at 96 keyframes or 10000 retired points; reset to start a new map",
      @"device":UIDevice.currentDevice.model,@"os":UIDevice.currentDevice.systemVersion,
      @"pose_convention":@"world_from_camera translation + quaternion xyzw; monocular scale arbitrary",
#ifdef IPTAM_UPSTREAM
      @"motion_model_uses_timestamps":@NO,
#else
      @"motion_model_uses_timestamps":@YES,
#endif
      @"storage_error":@NO,@"started":[[NSISO8601DateFormatter new] stringFromDate:NSDate.date]} mutableCopy];
    runOpen=YES;
    NSError *error=nil;
    NSString *path=[runDirectory stringByAppendingPathComponent:@"frames.jsonl"];
    if(![NSFileManager.defaultManager createDirectoryAtPath:runDirectory withIntermediateDirectories:YES attributes:nil error:&error]
       || ![[NSData data] writeToFile:path options:0 error:&error]
       || !(logFile=[NSFileHandle fileHandleForWritingToURL:[NSURL fileURLWithPath:path] error:&error])) {
        [self storageFailure:error.localizedDescription?:@"Cannot create diagnostics log"];
        [self failRun:summary[@"error"]];return NO;
    }
    if(![self saveSummary:@"running"]) {[self failRun:summary[@"error"]];return NO;}
    NSLog(@"IPTAM_RUN %@ %@",runID,kind);
    return YES;
}
- (BOOL)saveSummary:(NSString*)state {
    if(!summary) return YES;
    summary[@"status"]=state;summary[@"frames"]=@(frames);summary[@"valid_visual_frames"]=@(validFrames);
    summary[@"lost_frames"]=@(lostFrames);summary[@"capture_drops"]=@(droppedFrames);
    summary[@"uninitialized_frames"]=@(uninitializedFrames);summary[@"visual_coverage"]=@(frames?double(validFrames)/frames:0);
    summary[@"tracking_result"]=validFrames?@"visual measurements obtained; inspect coverage and trajectory accuracy":@"no accepted visual tracking";
    summary[@"mean_processing_ms"]=@(frames?totalMs/frames:0);summary[@"max_processing_ms"]=@(maxMs);
    NSInteger thermal=(NSInteger)NSProcessInfo.processInfo.thermalState;
    summary[@"thermal_state"]=@(thermal);
    summary[@"max_thermal_state"]=@(std::max(thermal,[summary[@"max_thermal_state"] integerValue]));
    summary[@"thermal_state_codes"]=@"0 nominal, 1 fair, 2 serious, 3 critical";
    summary[@"low_power_mode"]=@(NSProcessInfo.processInfo.lowPowerModeEnabled);
    NSError *error=nil;
    NSData *data=[NSJSONSerialization dataWithJSONObject:summary options:NSJSONWritingPrettyPrinted error:&error];
    if(data && [data writeToFile:[runDirectory stringByAppendingPathComponent:@"summary.json"] options:NSDataWritingAtomic error:&error]
       && [data writeToFile:[Documents() stringByAppendingPathComponent:@"last-run.json"] options:NSDataWritingAtomic error:&error]) return YES;
    [self storageFailure:error.localizedDescription?:@"Cannot save diagnostics summary"];
    // Best-effort failure evidence. NSLog remains evidence if storage itself is unavailable.
    data=[NSJSONSerialization dataWithJSONObject:summary options:NSJSONWritingPrettyPrinted error:nil];
    [data writeToFile:[runDirectory stringByAppendingPathComponent:@"summary.json"] atomically:YES];
    [data writeToFile:[Documents() stringByAppendingPathComponent:@"last-run.json"] atomically:YES];
    return NO;
}
- (BOOL)finishRun:(NSString*)reason {
    if(!runOpen) return YES;
    runOpen=NO;NSError *error=nil;BOOL ok=YES;
    if(logFile) {
        if(![logFile synchronizeAndReturnError:&error]) {ok=NO;[self storageFailure:error.localizedDescription];}
        if(![logFile closeAndReturnError:&error]) {ok=NO;[self storageFailure:error.localizedDescription];}
        logFile=nil;
    }
    if([summary[@"storage_error"] boolValue]) ok=NO;
    if(replayHashOpen) {
        unsigned char digest[CC_SHA256_DIGEST_LENGTH];CC_SHA256_Final(digest,&replayPixelHash);
        summary[@"rectified_pixels_concatenated_sha256"]=DigestString(digest);
        summary[@"pixel_hash_frame_count"]=@(pixelHashFrames);replayHashOpen=NO;
    }
    summary[@"ended"]=[[NSISO8601DateFormatter new] stringFromDate:NSDate.date];
    ok=[self saveSummary:ok?reason:@"failed"] && ok;
    if(!ok) {stopped=YES;[self showError:summary[@"error"]?:@"Diagnostic storage failed"];}
    return ok;
}
- (void)failRun:(NSString*)message {
    if(!runOpen) [self beginRun:replaying?@"recorded replay":@"camera startup" calibration:@{}];
    if(summary) summary[@"error"]=message?:@"Unknown error";
    stopped=YES;replayManifest=nil;engine.reset();
    [self finishRun:@"failed"];
    AVCaptureSession *current=session;
    dispatch_async(captureControl, ^{[current stopRunning];});
    [self showError:message?:@"Unknown error"];
}
- (BOOL)record:(NSDictionary*)frame {
    NSError *error=nil;
    NSData *data=[NSJSONSerialization dataWithJSONObject:frame options:0 error:&error];
    if(!data || !logFile) {[self failRun:error.localizedDescription?:@"Frame diagnostics log unavailable"];return NO;}
    NSMutableData *line=[data mutableCopy];[line appendData:[@"\n" dataUsingEncoding:NSUTF8StringEncoding]];
    if(![logFile writeData:line error:&error]) {[self storageFailure:error.localizedDescription];[self failRun:summary[@"error"]];return NO;}
    frames++;validFrames += [frame[@"valid_visual_pose"] boolValue];
    summary[@"solver_numerical_rejections"]=frame[@"solver_failures"]?:@0;
    summary[@"solver_retained_stops"]=frame[@"solver_retained_stops"]?:@0;
    summary[@"last_solver_failure"]=frame[@"last_solver_failure"]?:@"";
#ifdef IPTAM_MOBILE_PAPER
    summary[@"mobile_mapping"]=frame[@"mobile_mapping"]?:@{};
    summary[@"retained_keyframes"]=frame[@"retained_keyframes"]?:@0;
    task_vm_info_data_t memory={};mach_msg_type_number_t count=TASK_VM_INFO_COUNT;
    if(task_info(mach_task_self(),TASK_VM_INFO,(task_info_t)&memory,&count)==KERN_SUCCESS) {
        summary[@"peak_resident_bytes"]=@(std::max((uint64_t)[summary[@"peak_resident_bytes"] unsignedLongLongValue],(uint64_t)memory.resident_size));
        summary[@"peak_phys_footprint_bytes"]=@(std::max((uint64_t)[summary[@"peak_phys_footprint_bytes"] unsignedLongLongValue],(uint64_t)memory.phys_footprint));
    }
#endif
    if(frames==1 && !replaying) {
        NSData *snapshot=UIImageJPEGRepresentation(GrayImage(gray),.85);
        if(!snapshot || ![snapshot writeToFile:[runDirectory stringByAppendingPathComponent:@"camera-preview.jpg"] options:NSDataWritingAtomic error:&error]) {
            [self storageFailure:error.localizedDescription?:@"Could not save camera verification image"];
            [self failRun:summary[@"error"]];return NO;
        }
    }
    lostFrames += [frame[@"initialized"] boolValue] && ![frame[@"valid_visual_pose"] boolValue];
    uninitializedFrames += ![frame[@"initialized"] boolValue];
    totalMs += [frame[@"processing_ms"] doubleValue];maxMs=std::max(maxMs,[frame[@"processing_ms"] doubleValue]);
    if(frames%30==0 && (![logFile synchronizeAndReturnError:&error] || ![self saveSummary:@"running"])) {
        if(error) [self storageFailure:error.localizedDescription];
        [self failRun:summary[@"error"]];return NO;
    }
    if(![frame[@"finite"] boolValue]) {[self failRun:@"Non-finite estimator pose; evidence saved"];return NO;}
    if(!replaying && ([frame[@"keyframes"] unsignedIntegerValue]>=96 || [frame[@"retired_map_points"] unsignedIntegerValue]>=10000)) {
        [self failRun:@"Map session limit reached. Diagnostics saved. Tap Reset to start a new map."];return NO;
    }
    if((frames%3==0 || replaying) && !uiPending.exchange(true)) {
        UIImage *image=GrayImage(gray);
        NSString *frameMode=[mode copy];NSUInteger frameNumber=frames;
        dispatch_async(dispatch_get_main_queue(), ^{
            self->preview.image=image;
            BOOL initialized=[frame[@"initialized"] boolValue], valid=[frame[@"valid_visual_pose"] boolValue];
            BOOL weak=initialized && [frame[@"quality"] intValue]==1 && [frame[@"found"] intValue]>=10;
            self->status.text=valid?@"Visual tracking":(weak?@"Weak visual tracking":(initialized?@"Tracking lost":@"Initializing"));
            self->status.textColor=valid?UIColor.systemGreenColor:(initialized?UIColor.systemOrangeColor:UIColor.whiteColor);
            self->details.text=[NSString stringWithFormat:@"%@ · frame %@\n%@/%@ matches · %@ points · %@ keyframes · %.1f ms",frameMode,@(frameNumber),frame[@"found"],frame[@"attempted"],frame[@"map_points"],frame[@"keyframes"],[frame[@"processing_ms"] doubleValue]];
            self->guidance.text=self->replaying?@"Processing recorded images. Visual tracking counts accepted image measurements; map scale is arbitrary.":
              initialized?(valid?@"Map coordinates have arbitrary monocular scale.":@"Return to a mapped view, or reset and initialize again."):
              ([frame[@"trails"] unsignedIntegerValue]?
                ([frame[@"initial_displacement_px"] doubleValue]<12?@"Move slowly sideways. Keep the same textured surface in view and avoid turning.":@"Tap Create map. If it fails, move a little farther sideways and try again."):
                @"Aim at a textured surface and tap Initialize. Move sideways before creating the map.");
            self->initializeButton.enabled=!initialized && !self->replaying;
            [self->initializeButton setTitle:[frame[@"trails"] unsignedIntegerValue]?@"Create map":@"Initialize" forState:UIControlStateNormal];
            self->uiPending=false;
        });
    }
    return YES;
}
- (void)configureCamera {
    if(configured) return;
    stopped=NO;
    BOOL editing=NO,deviceLocked=NO;
    @try {
    session=[AVCaptureSession new];[session beginConfiguration];
    editing=YES;
    session.sessionPreset=AVCaptureSessionPreset640x480;
    device=[AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionBack];
    NSError *error=nil;
    if(!device) {[session commitConfiguration];editing=NO;[self failRun:@"Rear camera unavailable"];return;}
    AVCaptureDeviceInput *input=[AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if(!input || ![session canAddInput:input]) {[session commitConfiguration];editing=NO;[self failRun:error.localizedDescription?:@"Rear camera unavailable"];return;}
    [session addInput:input];
    AVCaptureVideoDataOutput *output=[AVCaptureVideoDataOutput new];output.alwaysDiscardsLateVideoFrames=YES;
    output.videoSettings=@{(id)kCVPixelBufferPixelFormatTypeKey:@(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)};
    [output setSampleBufferDelegate:self queue:worker];
    if(![session canAddOutput:output]) {[session commitConfiguration];editing=NO;[self failRun:@"Camera output unavailable"];return;}
    [session addOutput:output];
    AVCaptureConnection *connection=[output connectionWithMediaType:AVMediaTypeVideo];
    if(connection.isVideoOrientationSupported) connection.videoOrientation=AVCaptureVideoOrientationLandscapeRight;
    if(connection.isVideoStabilizationSupported) connection.preferredVideoStabilizationMode=AVCaptureVideoStabilizationModeOff;
    if(!connection.isCameraIntrinsicMatrixDeliverySupported) {[session commitConfiguration];editing=NO;[self failRun:@"This camera format does not supply calibration."];return;}
    connection.cameraIntrinsicMatrixDeliveryEnabled=YES;
    if([device lockForConfiguration:&error]) {
        deviceLocked=YES;
        device.videoZoomFactor=1;
        BOOL supports30=NO;
        for(AVFrameRateRange *range in device.activeFormat.videoSupportedFrameRateRanges)
            supports30 |= range.minFrameRate<=30 && range.maxFrameRate>=30;
        if(supports30) {
            device.activeVideoMinFrameDuration=CMTimeMake(1,30);device.activeVideoMaxFrameDuration=CMTimeMake(1,30);
        }
        if(device.isGeometricDistortionCorrectionSupported) device.geometricDistortionCorrectionEnabled=YES;
        if([device isFocusModeSupported:AVCaptureFocusModeContinuousAutoFocus]) device.focusMode=AVCaptureFocusModeContinuousAutoFocus;
        [device unlockForConfiguration];deviceLocked=NO;
    } else {[session commitConfiguration];editing=NO;[self failRun:error.localizedDescription?:@"Cannot configure camera"];return;}
    [session commitConfiguration];editing=NO;configured=YES;
    if(active) {AVCaptureSession *current=session;dispatch_async(captureControl, ^{[current startRunning];});}
    } @catch(NSException *exception) {
        if(deviceLocked) [device unlockForConfiguration];
        if(editing) {
            @try {[session commitConfiguration];}
            @catch(NSException *cleanupError) {NSLog(@"IPTAM_CAPTURE_CLEANUP_ERROR %@",cleanupError.reason);}
        }
        // No startRunning has been queued on this path. Release the failed
        // session instead of calling stopRunning inside an unfinished transaction.
        session=nil;device=nil;
        configured=NO;[self failRun:exception.reason?:@"Camera configuration failed"];
    }
}
- (void)captureOutput:(AVCaptureOutput*)output didDropSampleBuffer:(CMSampleBufferRef)buffer fromConnection:(AVCaptureConnection*)connection {droppedFrames++;}
- (void)captureOutput:(AVCaptureOutput*)output didOutputSampleBuffer:(CMSampleBufferRef)sample fromConnection:(AVCaptureConnection*)connection {
    if(!active || stopped || cancellationRequested) return;
    @autoreleasepool {
        CVPixelBufferRef pixel=CMSampleBufferGetImageBuffer(sample);
        double timestamp=CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sample));
        if(!std::isfinite(timestamp) || timestamp<=lastTimestamp) return;
        // A long interruption invalidates the velocity prior and map continuity.
        if(std::isfinite(lastTimestamp) && timestamp-lastTimestamp>1) {engine.reset();if(![self finishRun:@"timestamp_gap"]) return;focusLocked=NO;warmupFrames=0;}
        lastTimestamp=timestamp;
        CFTypeRef attachment=CMGetAttachment(sample,kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix,nullptr);
        if(!attachment || CFGetTypeID(attachment)!=CFDataGetTypeID() || CFDataGetLength((CFDataRef)attachment)!=sizeof(matrix_float3x3)) {
            [self failRun:@"No per-frame camera calibration was delivered."];return;
        }
        matrix_float3x3 k;memcpy(&k,CFDataGetBytePtr((CFDataRef)attachment),sizeof(k));
        if(!pixel || CVPixelBufferGetPlaneCount(pixel)<2 || CVPixelBufferGetPixelFormatType(pixel)!=kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) {
            [self failRun:@"Unsupported camera pixel format"];return;
        }
        int w=(int)CVPixelBufferGetWidthOfPlane(pixel,0),h=(int)CVPixelBufferGetHeightOfPlane(pixel,0);
        double fx=k.columns[0][0],fy=k.columns[1][1],cx=k.columns[2][0],cy=k.columns[2][1];
        if(w<64 || h<64 || w>2048 || h>2048 || w%8 || h%8 || !std::isfinite(fx+fy+cx+cy) || fx<=0 || fy<=0) {
            [self failRun:@"Invalid camera dimensions or intrinsics"];return;
        }
        if(CVPixelBufferLockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly)!=kCVReturnSuccess) {
            [self failRun:@"Cannot read camera luminance buffer"];return;
        }
        const auto *base=(const uint8_t*)CVPixelBufferGetBaseAddressOfPlane(pixel,0);
        size_t stride=CVPixelBufferGetBytesPerRowOfPlane(pixel,0);
        if(!base || stride<(size_t)w) {CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);[self failRun:@"Invalid camera row stride"];return;}
        gray.resize(CVD::ImageRef(w,h));
        for(int y=0;y<h;y++) memcpy(gray.data()+y*w,base+y*stride,w);
        CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);
        if(!focusLocked) {
            if(warmupFrames++<20 || (device.adjustingFocus && warmupFrames<90)) {
                if(!uiPending.exchange(true)) {
                    UIImage *image=GrayImage(gray);
                    dispatch_async(dispatch_get_main_queue(), ^{self->preview.image=image;self->guidance.text=@"Focusing camera…";self->uiPending=false;});
                }
                return;
            }
            NSError *error=nil;
            if([device isFocusModeSupported:AVCaptureFocusModeLocked] && [device lockForConfiguration:&error]) {
                device.focusMode=AVCaptureFocusModeLocked;focusLocked=device.focusMode==AVCaptureFocusModeLocked;
                [device unlockForConfiguration];
            }
            if(!focusLocked) {[self failRun:error.localizedDescription?:@"Cannot lock camera focus for calibration"];return;}
            // Use a subsequent frame's intrinsics after the focus lock.
            return;
        }
        try {
            if(engine && (w!=engine->width || h!=engine->height || std::abs(fx/engine->fx-1)>.005 || std::abs(fy/engine->fy-1)>.005 || std::abs(cx-engine->cx)>.5 || std::abs(cy-engine->cy)>.5)) {
                engine.reset();if(![self finishRun:@"calibration_changed"]) return;
            }
            if(!engine) {
                engine.reset(new Engine(w,h,fx,fy,cx,cy));
                if(![self beginRun:@"live camera" calibration:@{@"width":@(w),@"height":@(h),@"fx":@(fx),@"fy":@(fy),@"cx":@(cx),@"cy":@(cy),@"distortion":@"pinhole approximation; rear wide camera; stabilization off; focus locked",@"geometric_distortion_correction":@(device.isGeometricDistortionCorrectionEnabled)}]) return;
            }
            [self record:engine->process(gray,timestamp)];
        } catch(const std::exception& e) {[self failRun:[NSString stringWithUTF8String:e.what()]];}
    }
}
- (void)initializeMap {dispatch_async(worker, ^{if(self->engine && !self->stopped && !self->replaying) self->engine->initialize();});}
- (void)resetMap {
    if(replaying) return;
    dispatch_async(worker, ^{
        [self finishRun:@"reset"];self->engine.reset();self->focusLocked=NO;self->warmupFrames=0;
        self->stopped=NO;self->lastTimestamp=-std::numeric_limits<double>::infinity();
        NSError *error=nil;
        if([self->device isFocusModeSupported:AVCaptureFocusModeContinuousAutoFocus] && [self->device lockForConfiguration:&error]) {
            self->device.focusMode=AVCaptureFocusModeContinuousAutoFocus;[self->device unlockForConfiguration];
        }
        if(!self->configured) [self configureCamera];
        else if(self->active) {AVCaptureSession *current=self->session;dispatch_async(self->captureControl, ^{[current startRunning];});}
        dispatch_async(dispatch_get_main_queue(), ^{self->status.text=@"Initializing";self->initializeButton.enabled=NO;self->guidance.text=@"Focusing camera…";});
    });
}
- (void)exportLog {
    dispatch_async(worker, ^{
        if(self->runOpen && ![self saveSummary:@"running"]) {[self failRun:self->summary[@"error"]];return;}
        NSError *error=nil;
        if(self->logFile && ![self->logFile synchronizeAndReturnError:&error]) {[self storageFailure:error.localizedDescription];[self failRun:self->summary[@"error"]];return;}
        NSString *path=self->runDirectory;
        dispatch_async(dispatch_get_main_queue(), ^{
            if(!path) return;
            UIActivityViewController *share=[[UIActivityViewController alloc] initWithActivityItems:@[[NSURL fileURLWithPath:path]] applicationActivities:nil];
            share.popoverPresentationController.sourceView=self.view;
            [self presentViewController:share animated:YES completion:nil];
        });
    });
}
- (void)runReplay:(NSString*)manifestPath {
    if(cancellationRequested) return;
    NSError *error=nil;
    NSData *data=[NSData dataWithContentsOfFile:manifestPath options:NSDataReadingMappedIfSafe error:&error];
    id candidate=data?[NSJSONSerialization JSONObjectWithData:data options:0 error:&error]:nil;
    NSString *reason=nil;
    NSDictionary *m=IPTAMReplayManifest(candidate,&reason);
    if(error || !m) {
        if([self beginRun:@"recorded replay" calibration:@{}]) {
            summary[@"manifest"]=manifestPath;
            if(data) {unsigned char digest[CC_SHA256_DIGEST_LENGTH];CC_SHA256(data.bytes,(CC_LONG)data.length,digest);summary[@"manifest_sha256"]=DigestString(digest);}
            [self failRun:error.localizedDescription?:reason?:@"Replay manifest missing or invalid"];
        }
        return;
    }
    try {
        NSString *lens=m[@"lens_model"]?:@"pinhole";
#ifdef IPTAM_MOBILE_PAPER
        ConfigureMobileReplay();
        IPTAMConfigureRollingShutter(m);
#endif
        engine.reset(new Engine([m[@"width"] intValue],[m[@"height"] intValue],[m[@"fx"] doubleValue],[m[@"fy"] doubleValue],[m[@"cx"] doubleValue],[m[@"cy"] doubleValue],[lens isEqualToString:@"equidistant"]));
        if(![self beginRun:@"recorded replay" calibration:m[@"calibration"]?:@{@"width":m[@"width"],@"height":m[@"height"],@"fx":m[@"fx"],@"fy":m[@"fy"],@"cx":m[@"cx"],@"cy":m[@"cy"]}]) return;
        unsigned char digest[CC_SHA256_DIGEST_LENGTH];CC_SHA256(data.bytes,(CC_LONG)data.length,digest);
        summary[@"manifest"]=manifestPath;summary[@"manifest_sha256"]=DigestString(digest);
        summary[@"lens_model"]=lens;
#ifdef IPTAM_MOBILE_PAPER
        summary[@"mobile_config"]=MobileJSON(IPTAMMobile::ConfigJSON());
        summary[@"rolling_shutter_status"]=IPTAMMobile::Config().rollingShutter?@"enabled using verified prepared-image timing":@"disabled: no enabled verified prepared-image timing";
        if(m[@"rolling_shutter"]) summary[@"rolling_shutter_calibration"]=m[@"rolling_shutter"];
        summary[@"camera_permission_requested"]=@NO;
        summary[@"initialization_policy"]=IPTAMMobile::Config().automaticInitialization?@"one predeclared start; image-only automatic second view":@"predeclared fixed image pair";
        summary[@"mapping_budget"]=@"cooperative stage/solver boundaries; indivisible operations can overrun; idle time logged separately";
#endif
        summary[@"input_frame_count"]=@([m[@"frames"] count]);
        sourcePaced=[NSProcessInfo.processInfo.arguments containsObject:@"--source-paced"];
        summary[@"source_paced"]=@(sourcePaced);
        summary[@"replay_timing"]=sourcePaced?@"target wall start plus source offset; every image processed, late images retained":@"as fast as possible; every image processed";
        summary[@"initialization_indices"]=@[m[@"init_first"],m[@"init_second"]];
        if(![self saveSummary:@"running"]) {[self failRun:summary[@"error"]];return;}
        replayManifest=m;replayBase=manifestPath.stringByDeletingLastPathComponent;replayIndex=0;
        replayFirstTimestamp=[m[@"frames"][0][@"timestamp"] doubleValue];
        replayWallStart=std::chrono::steady_clock::now();replayMaxLag=0;
        lastTimestamp=-std::numeric_limits<double>::infinity();pixelHashFrames=0;
        CC_SHA256_Init(&replayPixelHash);replayHashOpen=YES;
        dispatch_async(worker, ^{[self runReplayStep];});
    } catch(const std::exception& e) {
        if(!runOpen && ![self beginRun:@"recorded replay" calibration:@{}]) return;
        summary[@"manifest"]=manifestPath;
        unsigned char digest[CC_SHA256_DIGEST_LENGTH];CC_SHA256(data.bytes,(CC_LONG)data.length,digest);
        summary[@"manifest_sha256"]=DigestString(digest);
        [self failRun:[NSString stringWithUTF8String:e.what()]];
    }
}
- (void)runReplayStep {
    if(!active || cancellationRequested || stopped || !replayManifest) return;
    @autoreleasepool {
        try {
            NSArray *input=replayManifest[@"frames"];
            if(replayIndex>=input.count) {
                summary[@"replay_wall_seconds"]=@(std::chrono::duration<double>(std::chrono::steady_clock::now()-replayWallStart).count());
                summary[@"max_schedule_lag_ms"]=@(replayMaxLag);
                BOOL saved=[self finishRun:@"completed"];replayManifest=nil;
                if(!saved) return;
                NSLog(@"IPTAM_REPLAY_COMPLETE %@ frames=%lu valid=%lu",runID,(unsigned long)frames,(unsigned long)validFrames);
                NSString *result=[NSString stringWithFormat:@"Replay complete: %lu/%lu frames accepted for visual tracking. Diagnostics saved in Files → iPTAM → runs.",(unsigned long)validFrames,(unsigned long)frames];
                dispatch_async(dispatch_get_main_queue(), ^{self->status.text=@"Replay complete";self->guidance.text=result;});return;
            }
            NSDictionary *entry=input[replayIndex];
            NSString *path=[replayBase stringByAppendingPathComponent:entry[@"path"]];
            double timestamp=[entry[@"timestamp"] doubleValue];
            double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-replayWallStart).count();
            double target=timestamp-replayFirstTimestamp;
            if(sourcePaced && target>elapsed) {
#ifdef IPTAM_MOBILE_PAPER
                const double absoluteTarget=std::chrono::duration<double>(replayWallStart.time_since_epoch()).count()+target;
                unsigned work=IPTAMMobile::Config().idleMapping?engine->idle(absoluteTarget):0;
                elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-replayWallStart).count();
                if(work && elapsed<target) {dispatch_async(worker, ^{[self runReplayStep];});return;}
#endif
                dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(std::max(0.,target-elapsed)*NSEC_PER_SEC)),worker, ^{[self runReplayStep];});
                return;
            }
            if(!ReadPGM(path,gray) || gray.size().x!=[replayManifest[@"width"] intValue] || gray.size().y!=[replayManifest[@"height"] intValue] || timestamp<=lastTimestamp) {
                [self failRun:[NSString stringWithFormat:@"Invalid replay image at frame %lu: %@",(unsigned long)replayIndex,path]];return;
            }
            lastTimestamp=timestamp;
            CC_SHA256_Update(&replayPixelHash,gray.data(),(CC_LONG)gray.size().area());pixelHashFrames++;
            bool secondCommand=true;
#ifdef IPTAM_MOBILE_PAPER
            secondCommand=!IPTAMMobile::Config().automaticInitialization;
#endif
            if(replayIndex==[replayManifest[@"init_first"] unsignedIntegerValue] || (secondCommand && replayIndex==[replayManifest[@"init_second"] unsignedIntegerValue])) engine->initialize();
            double delivered=std::chrono::duration<double>(std::chrono::steady_clock::now()-replayWallStart).count();
            double lag=sourcePaced?std::max(0.0,(delivered-target)*1000.0):0;
            replayMaxLag=std::max(replayMaxLag,lag);
            NSMutableDictionary *frame=[engine->process(gray,timestamp) mutableCopy];
            frame[@"delivery_wall_seconds"]=@(delivered);frame[@"schedule_lag_ms"]=@(lag);
            summary[@"replay_wall_seconds"]=@(std::chrono::duration<double>(std::chrono::steady_clock::now()-replayWallStart).count());
            summary[@"max_schedule_lag_ms"]=@(replayMaxLag);
            if(![self record:frame]) return;
            replayIndex++;
            // One frame per block lets lifecycle, export and cancellation work run.
            dispatch_async(worker, ^{[self runReplayStep];});
        } catch(const std::exception& e) {[self failRun:[NSString stringWithUTF8String:e.what()]];}
    }
}
- (void)background:(NSNotification*)note {
    UIApplication.sharedApplication.idleTimerDisabled=NO;
    cancellationRequested=true;
    dispatch_async(worker, ^{
        self->active=NO;AVCaptureSession *current=self->session;dispatch_async(self->captureControl, ^{[current stopRunning];});
        [self finishRun:@"backgrounded"];self->engine.reset();self->replayManifest=nil;self->focusLocked=NO;self->warmupFrames=0;
    });
}
- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    dispatch_async(worker, ^{
        [self failRun:@"Memory pressure stopped this map. Diagnostics saved. Tap Reset to start again."];
    });
}
- (void)foreground:(NSNotification*)note {
    UIApplication.sharedApplication.idleTimerDisabled=YES;
    dispatch_async(worker, ^{
    self->active=YES;
    if(self->replaying) {[self showError:@"Replay interrupted by backgrounding. Relaunch the replay to start a fresh run."];return;}
    self->cancellationRequested=false;
    if(self->configured && !self->stopped) {AVCaptureSession *current=self->session;dispatch_async(self->captureControl, ^{[current startRunning];});}
});}
- (void)runtimeError:(NSNotification*)note {
    NSError *error=note.userInfo[AVCaptureSessionErrorKey];
    dispatch_async(worker, ^{if(note.object==self->session) [self failRun:error.localizedDescription?:@"Camera stopped. Tap Reset to retry."];});
}
- (void)interrupted:(NSNotification*)note {dispatch_async(worker, ^{
    if(note.object!=self->session) return;
    [self finishRun:@"interrupted"];self->engine.reset();self->focusLocked=NO;self->warmupFrames=0;
    dispatch_async(dispatch_get_main_queue(), ^{self->status.text=@"Camera interrupted";self->initializeButton.enabled=NO;});
});}
- (void)resumed:(NSNotification*)note {dispatch_async(worker, ^{
    if(note.object==self->session && self->active && self->configured && !self->stopped) {AVCaptureSession *current=self->session;dispatch_async(self->captureControl, ^{if(!current.running) [current startRunning];});}
});}
@end

@interface RecordingLibraryController : UITableViewController
@property(nonatomic,copy) NSArray<NSDictionary*> *recordings;
@end
@implementation RecordingLibraryController
- (void)viewDidLoad {
    [super viewDidLoad];
    self.title=@"Recordings";
    self.tableView.accessibilityIdentifier=@"recordings-library";
    self.refreshControl=[UIRefreshControl new];
    [self.refreshControl addTarget:self action:@selector(loadRecordings) forControlEvents:UIControlEventValueChanged];
}
- (void)viewWillAppear:(BOOL)animated {
    [super viewWillAppear:animated];
    UIApplication.sharedApplication.idleTimerDisabled=NO;
    [self loadRecordings];
}
- (void)loadRecordings {
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED,0), ^{
        NSString *root=[Documents() stringByAppendingPathComponent:@"replays"];
        NSArray *folders=[NSFileManager.defaultManager contentsOfDirectoryAtPath:root error:nil]?:@[];
        NSMutableArray *items=[NSMutableArray array];
        NSMutableSet *seen=[NSMutableSet set];
        for(NSString *folder in [folders sortedArrayUsingSelector:@selector(localizedStandardCompare:)]) {
            NSString *path=[[root stringByAppendingPathComponent:folder] stringByAppendingPathComponent:@"manifest.json"];
            NSData *bytes=[NSData dataWithContentsOfFile:path];
            if(!bytes) continue;
            NSDictionary *manifest=IPTAMReplayManifest([NSJSONSerialization JSONObjectWithData:bytes options:0 error:nil],nil);
            if(!manifest) continue;
            unsigned char digest[CC_SHA256_DIGEST_LENGTH];CC_SHA256(bytes.bytes,(CC_LONG)bytes.length,digest);
            NSString *identity=DigestString(digest);
            if([seen containsObject:identity]) continue;
            [seen addObject:identity];
            NSString *title=manifest[@"dataset"];
            if(![title isKindOfClass:NSString.class] || !title.length) title=folder;
            // Device test caches append a manifest hash to the folder name.
            NSRegularExpression *suffix=[NSRegularExpression regularExpressionWithPattern:@"-[0-9a-f]{12}$" options:0 error:nil];
            title=[suffix stringByReplacingMatchesInString:title options:0 range:NSMakeRange(0,title.length) withTemplate:@""];
            title=[[title stringByReplacingOccurrencesOfString:@"_" withString:@" "] capitalizedString];
            NSArray *frames=manifest[@"frames"];
            double duration=[frames.lastObject[@"timestamp"] doubleValue]-[frames.firstObject[@"timestamp"] doubleValue];
            NSString *detail=[NSString stringWithFormat:@"%lu images · %.1f seconds · %@",(unsigned long)frames.count,duration,manifest[@"lens_model"]?:@"pinhole"];
            [items addObject:@{@"title":title,@"detail":detail,@"manifest":path,@"manifest_sha256":identity}];
        }
        NSDictionary *receipt=@{@"mode":@"recordings",@"camera_permission_requested":@NO,
            @"camera_authorization_status":@([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo]),
            @"recordings":items,@"updated":[[NSISO8601DateFormatter new] stringFromDate:NSDate.date]};
        [[NSJSONSerialization dataWithJSONObject:receipt options:NSJSONWritingPrettyPrinted error:nil]
            writeToFile:[Documents() stringByAppendingPathComponent:@"recording-library.json"] atomically:YES];
        dispatch_async(dispatch_get_main_queue(), ^{
            self.recordings=items;
            UILabel *empty=[UILabel new];empty.textAlignment=NSTextAlignmentCenter;empty.numberOfLines=0;
            empty.text=@"Add a prepared recording in Files → iPTAM → replays, then pull to refresh.";
            self.tableView.backgroundView=items.count?nil:empty;
            [self.tableView reloadData];[self.refreshControl endRefreshing];
        });
    });
}
- (NSInteger)tableView:(UITableView*)tableView numberOfRowsInSection:(NSInteger)section {return self.recordings.count;}
- (UITableViewCell*)tableView:(UITableView*)tableView cellForRowAtIndexPath:(NSIndexPath*)indexPath {
    UITableViewCell *cell=[tableView dequeueReusableCellWithIdentifier:@"recording"];
    if(!cell) cell=[[UITableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:@"recording"];
    NSDictionary *item=self.recordings[indexPath.row];
    cell.textLabel.text=item[@"title"];cell.detailTextLabel.text=item[@"detail"];
    cell.accessoryType=UITableViewCellAccessoryDisclosureIndicator;
    return cell;
}
- (void)tableView:(UITableView*)tableView didSelectRowAtIndexPath:(NSIndexPath*)indexPath {
    CameraController *controller=[CameraController new];
    controller.requestedReplayPath=self.recordings[indexPath.row][@"manifest"];
    [self.navigationController pushViewController:controller animated:YES];
}
@end

@interface AppDelegate : UIResponder <UIApplicationDelegate>
@property(nonatomic,strong) UIWindow *window;
@end
@implementation AppDelegate
- (BOOL)application:(UIApplication*)application didFinishLaunchingWithOptions:(NSDictionary*)options {
    self.window=[[UIWindow alloc] initWithFrame:UIScreen.mainScreen.bounds];
    IPTAMLaunchMode launch=IPTAMLaunch(NSProcessInfo.processInfo.arguments,nil);
    if(launch==IPTAMLaunchMode::Recordings)
        self.window.rootViewController=[[UINavigationController alloc] initWithRootViewController:[RecordingLibraryController new]];
    else self.window.rootViewController=[CameraController new];
    [self.window makeKeyAndVisible];return YES;
}
@end
int main(int argc,char**argv) {@autoreleasepool {return UIApplicationMain(argc,argv,nil,NSStringFromClass(AppDelegate.class));}}
