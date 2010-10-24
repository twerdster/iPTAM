

#import <UIKit/UIKit.h>

@class EAGLView;

@interface GLVideoFrameAppDelegate : NSObject <UIApplicationDelegate> {
    UIWindow *window;
    EAGLView *glView;
	IBOutlet UILabel *userString;
}

@property (nonatomic, retain) IBOutlet UIWindow *window;
@property (nonatomic, retain) IBOutlet EAGLView *glView;
@property (nonatomic,retain) IBOutlet UILabel *userString;

@end

