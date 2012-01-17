#import "KinectToolsPrincipal.h"
#import "KinectPatch.h"

@implementation KinectToolsPrincipal

+(void)registerNodesWithManager:(QCNodeManager*)manager
{
	KIRegisterPatch(KinectPatch);
}

@end
