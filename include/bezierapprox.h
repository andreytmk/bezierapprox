#pragma once

#if defined(_MSC_VER)
    //  Microsoft 
    #define EXPORT __declspec(dllexport)
    #define IMPORT __declspec(dllimport)
#elif defined(__GNUC__)
    //  GCC
    #define EXPORT __attribute__((visibility("default")))
    #define IMPORT
#else
    //  do nothing and hope for the best?
    #define EXPORT
    #define IMPORT
    #pragma warning Unknown dynamic link import/export semantics.
#endif

#if BEZIERAPPROXLIB_COMPILING
#   define BEZIERAPPROXLIB_PUBLIC EXPORT
#else
#   define BEZIERAPPROXLIB_PUBLIC IMPORT
#endif

#define BEZIER_APPROX_OK 0
#define BEZIER_APPROX_FAILED -1
#define BEZIER_APPROX_ARGUMENTS_ERROR -2
#define BEZIER_APPROX_NOT_ENOUGH_POINTS_ERROR -3
#define BEZIER_APPROX_BUFFER_TOO_SMALL -4

BEZIERAPPROXLIB_PUBLIC
typedef struct _BezierApproxPoint {
    double x;
    double y;
} BezierApproxPoint;

BEZIERAPPROXLIB_PUBLIC
typedef struct _BezierApproxCurve3Controls {
    BezierApproxPoint P0;
    BezierApproxPoint P1;
    BezierApproxPoint P2;
    BezierApproxPoint P3;
} BezierApproxCurve3Controls;

BEZIERAPPROXLIB_PUBLIC
int bezierApprox(
    const BezierApproxPoint points[],
    int pointsSize,
    double precision,
    BezierApproxCurve3Controls* controlsBuffer,
    int* controlsBufferSize
);

BEZIERAPPROXLIB_PUBLIC
int bezierApproxByOneCurve(
    const BezierApproxPoint points[],
    int firstPointIndex,
    int lastPointIndex,
    BezierApproxCurve3Controls * controls
);

BEZIERAPPROXLIB_PUBLIC
BezierApproxPoint bezierApproxGetCurveValue(
    const BezierApproxCurve3Controls controls,
    double t
);
