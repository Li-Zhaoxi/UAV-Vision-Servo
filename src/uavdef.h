#ifndef _UAV_DEF_
#define _UAV_DEF_

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined MBZIRC2017_EXPORTS
#define UAV_EXPORTS __declspec(dllexport)
#else
#define UAV_EXPORTS
#endif


#endif
