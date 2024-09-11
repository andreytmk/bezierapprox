#include <bezierapprox.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#if defined(_MSC_VER)
    #define _CRTDBG_MAP_ALLOC
#endif

#include <stdlib.h>

#if defined(_MSC_VER)
    #include <crtdbg.h>
#endif

#define EPS 0.05

const BezierApproxPoint P0 = { 50.0, 300.0 };
const BezierApproxPoint P1 = { 150.0, -150.0 };
const BezierApproxPoint P2 = { 250.0, 450.0 };
const BezierApproxPoint P3 = { 350.0, 50.0 };

const int N = 8;
const double t[] = { 0.0, 0.05, 0.2, 0.4, 0.6, 0.8, 0.95, 1.0 };
const BezierApproxPoint points[] = {
    { 50.00, 300.00 },
    { 65.00, 240.12 },
    { 110.00, 139.60 },
    { 170.00, 132.80 },
    { 230.00, 181.20 },
    { 290.00, 186.40 },
    { 335.00, 102.76 },
    { 350.00, 50.00 },
};

static inline bool epsNear(double a, double b) {
    return fabs(a - b) < EPS;
}

static inline int bezierRandom(int min, int max) {
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

bool test_bezierApproxGetCurveValue() {
    BezierApproxCurve3Controls controls = { P0, P1, P2, P3 };

    bool success = true;
    for (int i = 0; i < N && success; ++i) {
        BezierApproxPoint ans = bezierApproxGetCurveValue(controls, t[i]);
        success &= epsNear(ans.x, points[i].x);
        success &= epsNear(ans.y, points[i].y);
        if (!success) {
            printf("test_bezierApproxGetCurveValue failed. Expected: (%lf, %lf). Actual: (%lf, %lf).",
                points[i].x, points[i].y,
                ans.x, ans.y
            );
        }
    }
    return success;
}

bool test_bezierApproxByOneCurve() {
    bool success = true;
    BezierApproxCurve3Controls ans;
    int result = bezierApproxByOneCurve(
        points,
        0,
        N - 1,
        &ans
    );
    if (result != BEZIER_APPROX_OK) {
        success = false;
        goto cleanup;
    }

cleanup:
    return success;
}

static inline bool checkOnePoint(
    BezierApproxPoint* points,
    BezierApproxCurve3Controls* controlsBuffer,
    BezierApproxPoint p1
) {
    bool success = true;
    const int pointsSize = 1;
    int controlsBufferSize = 1;

    points[0].x = p1.x;
    points[0].y = p1.y;

    int result = bezierApprox(
        points,
        pointsSize,
        1.0,
        controlsBuffer,
        &controlsBufferSize
    );

    success &= (result == BEZIER_APPROX_OK);
    success &= (controlsBufferSize == 1);

    BezierApproxPoint t0val = bezierApproxGetCurveValue(*controlsBuffer, 0.0);
    success &= epsNear(p1.x, t0val.x) && epsNear(p1.y, t0val.y);

    BezierApproxPoint t05val = bezierApproxGetCurveValue(*controlsBuffer, 0.5);
    success &= epsNear(p1.x, t05val.x) && epsNear(p1.y, t05val.y);

    BezierApproxPoint t1val = bezierApproxGetCurveValue(*controlsBuffer, 1.0);
    success &= epsNear(p1.x, t1val.x) && epsNear(p1.y, t1val.y);

    return success;
}

bool test_onePoint() {
    bool success = true;
    BezierApproxPoint *points = NULL;
    BezierApproxCurve3Controls* controlsBuffer = NULL;
    int controlsBufferSize = 1;

    points = (BezierApproxPoint*)malloc(1 * sizeof(BezierApproxPoint));
    if (!points) {
        success = false;
        goto cleanup;
    }

    controlsBuffer = (BezierApproxCurve3Controls*)
        malloc(controlsBufferSize * sizeof(BezierApproxCurve3Controls));
    if (!controlsBuffer) {
        success = false;
        goto cleanup;
    }

    {
        BezierApproxPoint p1 = { 2.0, 2.0 };
        success &= checkOnePoint(points, controlsBuffer, p1);
    }

    {
        BezierApproxPoint p1 = { 0.0, 0.0 };
        success &= checkOnePoint(points, controlsBuffer, p1);
    }

    {
        BezierApproxPoint p1 = { 1.0, 0.0 };
        success &= checkOnePoint(points, controlsBuffer, p1);
    }

cleanup:
    if (controlsBuffer) {
        free(controlsBuffer);
        controlsBuffer = NULL;
    }
    if (points) {
        free(points);
        points = NULL;
    }
    return success;
}

static inline bool checkTwoPoints(
    BezierApproxPoint* points,
    BezierApproxCurve3Controls* controlsBuffer,
    BezierApproxPoint p1,
    BezierApproxPoint p2
) {
    bool success = true;
    const int pointsSize = 2;
    int controlsBufferSize = 1;

    points[0].x = p1.x;
    points[0].y = p1.y;

    points[1].x = p2.x;
    points[1].y = p2.y;

    int result = bezierApprox(
        points,
        pointsSize,
        1.0,
        controlsBuffer,
        &controlsBufferSize
    );

    success &= (result == BEZIER_APPROX_OK);
    success &= (controlsBufferSize == 1);

    BezierApproxPoint t0val = bezierApproxGetCurveValue(*controlsBuffer, 0.0);
    success &= epsNear(p1.x, t0val.x) && epsNear(p1.y, t0val.y);

    BezierApproxPoint t1val = bezierApproxGetCurveValue(*controlsBuffer, 1.0);
    success &= epsNear(p2.x, t1val.x) && epsNear(p2.y, t1val.y);

    return success;
}

bool test_twoPoints() {
    bool success = true;
    BezierApproxPoint* points = NULL;
    const int pointsSize = 2;
    BezierApproxCurve3Controls* controlsBuffer = NULL;
    int controlsBufferSize = 1;

    points = (BezierApproxPoint*)malloc(pointsSize * sizeof(BezierApproxPoint));
    if (!points) {
        success = false;
        goto cleanup;
    }

    controlsBuffer = (BezierApproxCurve3Controls*)
        malloc(controlsBufferSize * sizeof(BezierApproxCurve3Controls));
    if (!controlsBuffer) {
        success = false;
        goto cleanup;
    }

    {
        BezierApproxPoint p1 = { 1.0, 1.0 };
        BezierApproxPoint p2 = { 2.0, 2.0 };
        success &= checkTwoPoints(points, controlsBuffer, p1, p2);
    }

    {
        BezierApproxPoint p1 = { 0.0, 0.0 };
        BezierApproxPoint p2 = { 1.0, 0.0 };
        success &= checkTwoPoints(points, controlsBuffer, p1, p2);
    }

cleanup:
    if (controlsBuffer) {
        free(controlsBuffer);
        controlsBuffer = NULL;
    }
    if (points) {
        free(points);
        points = NULL;
    }
    return success;
}

static inline bool checkPointsArray(
    BezierApproxPoint* points,
    int pointsSize,
    BezierApproxCurve3Controls* controlsBuffer,
    int controlsBufferSize
) {
    bool success = true;

    int result = bezierApprox(
        points,
        pointsSize,
        1.0,
        controlsBuffer,
        &controlsBufferSize
    );

    success &= (result == BEZIER_APPROX_OK);
    success &= (controlsBufferSize >= 1);

    BezierApproxPoint tFirstVal = bezierApproxGetCurveValue(controlsBuffer[0], 0.0);
    success &= epsNear(points[0].x, tFirstVal.x) && epsNear(points[0].y, tFirstVal.y);

    BezierApproxPoint tLastVal = bezierApproxGetCurveValue(controlsBuffer[controlsBufferSize - 1], 1.0);
    success &= epsNear(points[pointsSize - 1].x, tLastVal.x) && epsNear(points[pointsSize - 1].y, tLastVal.y);

    return success;
}

bool test_3pointsInLine() {
    bool success = true;
    BezierApproxPoint* points = NULL;
    const int pointsSize = 3;
    BezierApproxCurve3Controls* controlsBuffer = NULL;
    int controlsBufferSize = 1;

    points = (BezierApproxPoint*)malloc(pointsSize * sizeof(BezierApproxPoint));
    if (!points) {
        success = false;
        goto cleanup;
    }

    controlsBuffer = (BezierApproxCurve3Controls*)
        malloc(controlsBufferSize * sizeof(BezierApproxCurve3Controls));
    if (!controlsBuffer) {
        success = false;
        goto cleanup;
    }

    {
        points[0].x = 1.0;
        points[0].y = 1.0;
        points[1].x = 2.0;
        points[1].y = 2.0;
        points[2].x = 3.0;
        points[2].y = 3.0;
        success &= checkPointsArray(
            points,
            pointsSize,
            controlsBuffer,
            controlsBufferSize
        );
    }

    {
        points[0].x = 1.0;
        points[0].y = 1.0;
        points[1].x = 2.0;
        points[1].y = 1.0;
        points[2].x = 3.0;
        points[2].y = 1.0;
        success &= checkPointsArray(
            points,
            pointsSize,
            controlsBuffer,
            controlsBufferSize
        );
    }

cleanup:
    if (controlsBuffer) {
        free(controlsBuffer);
        controlsBuffer = NULL;
    }
    if (points) {
        free(points);
        points = NULL;
    }
    return success;
}

bool runRandomTest(
    int pointsSize
) {
    bool success = true;
    BezierApproxPoint* points = NULL;
    BezierApproxCurve3Controls* controlsBuffer = NULL;
    int controlsBufferSize = pointsSize - 1;
    const int RND_STEP = 10;

    if (pointsSize < 1) {
        success = false;
        goto cleanup;
    }

    points = (BezierApproxPoint*)malloc(pointsSize * sizeof(BezierApproxPoint));
    if (!points) {
        success = false;
        goto cleanup;
    }

    controlsBuffer = (BezierApproxCurve3Controls*)
        malloc(controlsBufferSize * sizeof(BezierApproxCurve3Controls));
    if (!controlsBuffer) {
        success = false;
        goto cleanup;
    }

    points[0].x = bezierRandom(0, RND_STEP);
    points[0].y = bezierRandom(0, RND_STEP);
    for (int i = 1; i < pointsSize; ++i) {
        int dx = bezierRandom(0, RND_STEP);
        int dy = bezierRandom(0, RND_STEP);
        while (dx == 0 && dy == 0) {
            dx = bezierRandom(0, RND_STEP);
            dy = bezierRandom(0, RND_STEP);
        }
        points[i].x = points[i - 1].x + dx;
        points[i].y = points[i - 1].y + dy;
    }

    success &= checkPointsArray(
        points,
        pointsSize,
        controlsBuffer,
        controlsBufferSize
    );

cleanup:
    if (controlsBuffer) {
        free(controlsBuffer);
        controlsBuffer = NULL;
    }
    if (points) {
        free(points);
        points = NULL;
    }
    return success;
}

bool test_randomPoints() {
    srand(1212);
    const int RANDOM_TEST_COUNT = 100;
    const int MAX_POINTS = 100;

    bool success = true;
    success &= runRandomTest(3);
    success &= runRandomTest(4);
    for (int t = 0; t < RANDOM_TEST_COUNT; ++t) {
        int pointsCount = bezierRandom(4, MAX_POINTS);
        success &= runRandomTest(pointsCount);
        if (!success) {
            printf("Failed t=%d\n", t);
        }
    }
    return success;
}

bool runAllTests() {
    bool success = true;
    success &= test_bezierApproxGetCurveValue();
    success &= test_bezierApproxByOneCurve();
    success &= test_onePoint();
    success &= test_twoPoints();
    success &= test_3pointsInLine();
    success &= test_randomPoints();
    return success;
}

int main() {
    bool success = runAllTests();
    int errorCode = 0;
    if (success) {
        printf("SUCCESS");
    }
    else {
        printf("FAILED");
        errorCode = 1;
    }

#if defined(_MSC_VER)
    _CrtDumpMemoryLeaks();
#endif
    return errorCode;
}