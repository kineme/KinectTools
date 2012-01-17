#include <libusb.h>
#include "libfreenect.h"
#import <pthread.h>


@interface KinectPatch : QCPatch
{
	QCIndexPort *inputDeviceID;

	QCBooleanPort *outputConnected;
	QCImagePort *outputColorImage;
	QCImagePort *outputDepthImage;
	QCNumberPort *outputAccelerationX;
	QCNumberPort *outputAccelerationY;
	QCNumberPort *outputAccelerationZ;

	NSThread *usbHandlerThread;
	pthread_mutex_t usbHandlerUpdatingLock;

	// set in usbHandlerThread, use usbHandlerUpdatingLock
	bool connected;
	QCImage *colorImage;
	QCImage *depthImage;
	double ax,ay,az;
}

+(BOOL)allowsSubpatchesWithIdentifier:(id)identifier;
+(QCPatchExecutionMode)executionModeWithIdentifier:(id)identifier;
-(id)initWithIdentifier:(id)identifier;
-(void)disable:(QCOpenGLContext*)context;
-(BOOL)execute:(QCOpenGLContext*)context time:(double)time arguments:(NSDictionary*)arguments;

- (void)updateDepthImageWithBuffer:(freenect_depth *)depth;
- (void)updateColorImageWithBuffer:(freenect_pixel *)rgb;

@end
