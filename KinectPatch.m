#import "KinectPatch.h"

#include <sched.h>


uint8_t t_gamma[2048];


void KinectPatchDepthBufferCallback(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	KinectPatch *k = (KinectPatch *)freenect_get_user(dev);
	[k updateDepthImageWithBuffer:v_depth];
}

void KinectPatchColorBufferCallback(freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp)
{
	KinectPatch *k = (KinectPatch *)freenect_get_user(dev);
	[k updateColorImageWithBuffer:rgb];
}



@implementation KinectPatch

+(BOOL)allowsSubpatchesWithIdentifier:(id)identifier
{
	return NO;
}

+(QCPatchExecutionMode)executionModeWithIdentifier:(id)identifier
{
	return kQCPatchExecutionModeProvider;
}

-(id)initWithIdentifier:(id)identifier
{
	if(self = [super initWithIdentifier:identifier])
	{
		[[self userInfo] setObject:@"Kineme Kinect Patch" forKey:@"name"];

		int i;
		for (i=0; i<2048; i++) {
			float v = i/2048.0;
			v = powf(v, 3)* 6;
			t_gamma[i] = 255-((int)(v*6*256)>>3)&0xff;
		}
		
		pthread_mutex_init(&usbHandlerUpdatingLock,NULL);
	}
	return self;
}
- (void)dealloc
{
	pthread_mutex_destroy(&usbHandlerUpdatingLock);
	[super dealloc];
}

-(void)disable:(QCOpenGLContext*)context
{
	[usbHandlerThread cancel];
	while([usbHandlerThread isExecuting])
		sched_yield();

	if(colorImage)
		[colorImage release];
	if(depthImage)
		[colorImage release];

	[usbHandlerThread release];
	usbHandlerThread = nil;
}



- (void)updateDepthImageWithBuffer:(freenect_depth *)depth
{
//	SLog();
	CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericRGB);
	QCImagePixelBuffer *pb = [[QCImagePixelBuffer alloc] initWithFormat:[QCPixelFormat pixelFormatARGB8] pixelsWide:FREENECT_FRAME_W pixelsHigh:FREENECT_FRAME_H options:nil];
	[pb beginUpdatePixels:TRUE colorSpace:cs];

	char *base = [pb baseAddress];
	int basestride = FREENECT_FRAME_W*4;
	if ([[pb pixelBufferRepresentation] isFlipped])
	{
//		SLog(@"Flipping.");
		base += (FREENECT_FRAME_H-1)*FREENECT_FRAME_W*4;
		basestride *= -1;
	}

	// @@@ todo: make this not so horrifically inefficient, now that I've kinda figured out what's going on here.
	for(NSUInteger y=0;y<FREENECT_FRAME_H;++y,base+=basestride)
	{
		for(NSUInteger x=0;x<FREENECT_FRAME_W;++x)
		{
			uint16_t v = *(depth+y*FREENECT_FRAME_W+x);

			// 0x7ff is apparently "I couldn't figure out what depth this pixel is"
			if(v == 0x7ff)
				base[x*4+0]= 0;
			else
			{
				base[x*4+0]= 255;

				base[x*4+1]=
				base[x*4+2]=
				base[x*4+3]= t_gamma[v];
			}
		}
	}


	[pb endUpdatePixels];
	CGColorSpaceRelease(cs);
	pthread_mutex_lock(&usbHandlerUpdatingLock);
	if(depthImage)
		[depthImage release];
	depthImage = [[QCImage alloc] initWithQCImageBuffer:pb options:nil]; 
	pthread_mutex_unlock(&usbHandlerUpdatingLock);
	[pb release]; 
}
- (void)updateColorImageWithBuffer:(freenect_pixel *)rgb
{
//	SLog();
	CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericRGB); 
	QCImagePixelBuffer *pb = [[QCImagePixelBuffer alloc] initWithFormat:[QCPixelFormat pixelFormatRGB8] baseAddress:rgb releaseCallback:nil releaseInfo:nil bytesPerRow:FREENECT_FRAME_W*3 pixelsWide:FREENECT_FRAME_W pixelsHigh:FREENECT_FRAME_H flipped:NO colorSpace:cs options:nil];

	CGColorSpaceRelease(cs);
	pthread_mutex_lock(&usbHandlerUpdatingLock);
	if(colorImage)
		[colorImage release];
	colorImage = [[QCImage alloc] initWithQCImageBuffer:pb options:nil]; 
	pthread_mutex_unlock(&usbHandlerUpdatingLock);
	[pb release]; 
}

-(BOOL)execute:(QCOpenGLContext*)context time:(double)time arguments:(NSDictionary*)arguments
{
//	SLog();

	if([inputDeviceID wasUpdated])
	{
		[usbHandlerThread cancel];
		while([usbHandlerThread isExecuting])
			sched_yield();

		[usbHandlerThread release];
		usbHandlerThread = nil;

		usbHandlerThread = [[NSThread alloc] initWithTarget:self selector:@selector(_usbHandlerThread) object:nil];
		[usbHandlerThread setName:@"KinectTools"];
		[usbHandlerThread start];
	}

	pthread_mutex_lock(&usbHandlerUpdatingLock);
	[outputConnected setBooleanValue:connected];
	[outputColorImage setImageValue:colorImage];
	[outputDepthImage setImageValue:depthImage];
	[outputAccelerationX setDoubleValue:ax];
	[outputAccelerationY setDoubleValue:ay];
	[outputAccelerationZ setDoubleValue:az];
	pthread_mutex_unlock(&usbHandlerUpdatingLock);

	return YES;
}



-(void)_usbHandlerThread
{
	NSAutoreleasePool *pool = [NSAutoreleasePool new];
	NSThread *t = [NSThread currentThread];
	[NSThread setThreadPriority:0.75];

	SLog(@"%@: Starting capture session.",t);

	pthread_mutex_lock(&usbHandlerUpdatingLock);
	colorImage=depthImage=nil;
	pthread_mutex_unlock(&usbHandlerUpdatingLock);

	freenect_context *f_ctx;

	if(freenect_init(&f_ctx,NULL) < 0)
	{
		SLog(@"%@: Failed to initialize libfreenect.",t);
		[pool drain];
		return;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_WARNING);

	freenect_device *f_dev;
	if(freenect_open_device(f_ctx,&f_dev,[inputDeviceID indexValue]) < 0)
	{
		SLog(@"%@: Could not open USB device.",t);
		freenect_shutdown(f_ctx);
		[pool drain];
		return;
	}
	SLog(@"%@: Opened USB device %i",t,[inputDeviceID indexValue]);
	connected=YES;

	freenect_set_user(f_dev,self);

	freenect_set_depth_callback(f_dev, KinectPatchDepthBufferCallback);
	freenect_set_rgb_callback(f_dev, KinectPatchColorBufferCallback);
	freenect_set_rgb_format(f_dev, FREENECT_FORMAT_RGB);
	freenect_set_depth_format(f_dev, FREENECT_FORMAT_11_BIT);

	freenect_start_depth(f_dev);
	freenect_start_rgb(f_dev);



	while([t isCancelled] == NO)
	{
		if(freenect_process_events(f_ctx)<0)
		{
			SLog(@"%@: freenect_process_events() returned < 0",t);
			break;
		}

		double dx,dy,dz;
		if(freenect_get_mks_accel(f_dev, &dx, &dy, &dz)<0)
		{
			SLog(@"%@: freenect_get_mks_accel() returned < 0",t);
			break;
		}

		pthread_mutex_lock(&usbHandlerUpdatingLock);
		ax=dx;
		ay=dy;
		az=dz;
		pthread_mutex_unlock(&usbHandlerUpdatingLock);

		sched_yield();
	}
	
	SLog(@"%@: Closing capture session.",t);

	freenect_stop_depth(f_dev);
	freenect_stop_rgb(f_dev);

	pthread_mutex_lock(&usbHandlerUpdatingLock);
	colorImage=depthImage=nil;
	ax=ay=az=0;
	connected=NO;
	pthread_mutex_unlock(&usbHandlerUpdatingLock);

	freenect_close_device(f_dev);

	freenect_shutdown(f_ctx);

	SLog(@"%@: Done.",t);
	[pool drain];
}

@end
