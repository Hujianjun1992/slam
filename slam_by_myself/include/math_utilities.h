#ifndef QRK_MATH_UTILITIES_H
#define QRK_MATH_UTILITIES_H
 
#include "detect_os.h"
#if defined(WINDOWS_OS)
#define _USE_MATH_DEFINES
#endif
#include <math.h>

  
 #ifndef M_PI
 
 
 #define M_PI 3.14159265358979323846
 #endif
 
 #if defined(MSC)
 extern long lrint(double x);
 #endif
 
#endif /* !QRK_MATH_UTILITIES_H */
