#include "bezierapprox.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define EPS_ZERO 1.0e-9

const double B3[] = {
    1,
    3,
    3,
    1
};

typedef struct _BezierControlsStackEntry {
    BezierApproxCurve3Controls controls;
    BezierApproxPoint e1;
    BezierApproxPoint e2;
    int fistIdx;
    int lastIdx;
} BezierControlsStackEntry;

static inline double bValue(int j, double t) {
    return B3[j] * pow(1 - t, 3 - j) * pow(t, j);
}

static inline BezierApproxPoint substructPoint(BezierApproxPoint a, BezierApproxPoint b) {
    BezierApproxPoint c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    return c;
}

static inline double getPointNorm(BezierApproxPoint *a) {
    return sqrt(a->x * a->x + a->y * a->y);
}

static inline int normalizePoint(BezierApproxPoint *a) {
    double norm = getPointNorm(a);
    if (fabs(norm) < EPS_ZERO) {
        return BEZIER_APPROX_ARGUMENTS_ERROR;
    }
    a->x /= norm;
    a->y /= norm;
    return BEZIER_APPROX_OK;
}

static inline double* initTdist(
    const BezierApproxPoint points[],
    int pointsSize
) {
    double* tDist = NULL;
    tDist = (double*)malloc(pointsSize * sizeof(double));
    if (!tDist) {
        return tDist;
    }

    tDist[0] = 0.0;
    for (int i = 1; i < pointsSize; ++i) {
        BezierApproxPoint diff = substructPoint(points[i], points[i - 1]);
        tDist[i] = tDist[i - 1] + getPointNorm(&diff);
    }
    return tDist;
}

static inline double getTValue(
    const double tDist[],
    int idx,
    int firstIdx,
    int lastIdx
) {
    assert(tDist[lastIdx] - tDist[firstIdx] > EPS_ZERO);
    return (tDist[idx] - tDist[firstIdx]) / (tDist[lastIdx] - tDist[firstIdx]);
}

static inline void getMaxDistance(
    const BezierApproxCurve3Controls controls,
    const BezierApproxPoint points[],
    const double tDist[],
    int firstIdx,
    int lastIdx,
    double *maxDist,
    int* maxDistIdx
) {
    *maxDist = -1.0;
    *maxDistIdx = -1;
    for (int i = firstIdx; i <= lastIdx; ++i) {
        double tVal = getTValue(tDist, i, firstIdx, lastIdx);
        BezierApproxPoint approxPoint = bezierApproxGetCurveValue(controls, tVal);
        BezierApproxPoint diffVect = substructPoint(approxPoint, points[i]);
        double dist = getPointNorm(&diffVect);
        if (dist > *maxDist) {
            *maxDist = dist;
            *maxDistIdx = i;
        }
    }
    assert(*maxDist >= -0.001);
    assert(*maxDistIdx >= firstIdx);
    assert(*maxDistIdx <= lastIdx);
}

static int inline bezierApproxByOneCurveByInitVectors(
    const BezierApproxPoint points[],
    int firstPointIndex,
    int lastPointIndex,
    const BezierApproxPoint e1,
    const BezierApproxPoint e2,
    const double tDist[],
    BezierApproxCurve3Controls* controls
) {
    int result = BEZIER_APPROX_FAILED;
    const int pointsCount = lastPointIndex - firstPointIndex + 1;

    if (pointsCount < 2) {
        result = BEZIER_APPROX_NOT_ENOUGH_POINTS_ERROR;
        goto cleanup;
    }

    double z1 = 0.0;
    double z2 = 0.0;
    double x0 = points[firstPointIndex].x;
    double y0 = points[firstPointIndex].y;
    double x3 = points[lastPointIndex].x;
    double y3 = points[lastPointIndex].y;

    if (pointsCount == 2) {
        z1 = 1.0;
        z2 = 1.0;
        goto fillcontrols;
    }

    double A11 = 0.0;
    double A12 = 0.0;
    double A22 = 0.0;
    double D1 = 0.0;
    double D2 = 0.0;
    for (int i = firstPointIndex; i <= lastPointIndex; ++i) {
        double tVal = getTValue(tDist, i, firstPointIndex, lastPointIndex);

        double b0Val = bValue(0, tVal);
        double b1Val = bValue(1, tVal);
        double b2Val = bValue(2, tVal);
        double b3Val = bValue(3, tVal);

        A11 += b1Val * b1Val;
        A12 += b1Val * b2Val;
        A22 += b2Val * b2Val;

        double a = points[i].x;
        double b = points[i].y;

        double dPart1 = a - x0 * (b0Val + b1Val) - x3 * (b2Val + b3Val);
        double dPart2 = b - y0 * (b0Val + b1Val) - y3 * (b2Val + b3Val);

        D1 += dPart1 * e1.x * b1Val + dPart2 * e1.y * b1Val;
        D2 += dPart1 * e2.x * b2Val + dPart2 * e2.y * b2Val;
    }
    A12 = (e1.x * e2.x + e1.y * e2.y) * A12;

    double detA = A11 * A22 - A12 * A12;
    if (fabs(detA) < EPS_ZERO) {
        z1 = 1.0;
        z2 = 1.0;
        goto fillcontrols;
    }

    z1 = (A22 * D1 - A12 * D2) / detA;
    z2 = (A11 * D2 - A12 * D1) / detA;

fillcontrols:
    controls->P0.x = x0;
    controls->P0.y = y0;
    controls->P1.x = x0 + z1 * e1.x;
    controls->P1.y = y0 + z1 * e1.y;
    controls->P2.x = x3 + z2 * e2.x;
    controls->P2.y = y3 + z2 * e2.y;
    controls->P3.x = x3;
    controls->P3.y = y3;

    result = BEZIER_APPROX_OK;
cleanup:
    return result;
}

int bezierApprox(
    const BezierApproxPoint points[],
    int pointsSize,
    double precision,
    BezierApproxCurve3Controls* controlsBuffer,
    int* controlsBufferSize
) {
    int result = BEZIER_APPROX_FAILED;
    double* tDist = NULL;

    const int controlsAnsCapacity = pointsSize - 1;
    int controlsAnsSize = 0;
    BezierApproxCurve3Controls* controlsAns = NULL;

    const int controlsStackCapacity = pointsSize - 1;
    int controlsStackSize = 0;
    BezierControlsStackEntry *controlsStack = NULL;

    if (pointsSize < 1) {
        result = BEZIER_APPROX_NOT_ENOUGH_POINTS_ERROR;
        goto cleanup;
    }
    if (pointsSize == 1) {
        if (*controlsBufferSize < 1) {
            result = BEZIER_APPROX_BUFFER_TOO_SMALL;
            *controlsBufferSize = controlsAnsSize;
            goto cleanup;
        }
        *controlsBufferSize = 1;
        controlsBuffer[0].P0.x = points[0].x;
        controlsBuffer[0].P0.y = points[0].y;
        controlsBuffer[0].P1.x = points[0].x;
        controlsBuffer[0].P1.y = points[0].y;
        controlsBuffer[0].P2.x = points[0].x;
        controlsBuffer[0].P2.y = points[0].y;
        controlsBuffer[0].P3.x = points[0].x;
        controlsBuffer[0].P3.y = points[0].y;
        result = BEZIER_APPROX_OK;
        goto cleanup;
    }

    BezierApproxPoint e1 = substructPoint(points[1], points[0]);
    if (normalizePoint(&e1) != BEZIER_APPROX_OK) {
        result = BEZIER_APPROX_ARGUMENTS_ERROR;
        goto cleanup;
    }

    BezierApproxPoint e2 = substructPoint(points[pointsSize - 2], points[pointsSize - 1]);
    if (normalizePoint(&e2) != BEZIER_APPROX_OK) {
        result = BEZIER_APPROX_ARGUMENTS_ERROR;
        goto cleanup;
    }

    controlsAns = (BezierApproxCurve3Controls*)malloc(
        sizeof(BezierApproxCurve3Controls) * controlsAnsCapacity
    );

    if (!controlsAns) {
        goto cleanup;
    }

    tDist = initTdist(points, pointsSize);
    if (!tDist) {
        goto cleanup;
    }

    int leftIdx = 0;
    int rightIdx = pointsSize - 1;

    BezierApproxCurve3Controls controls = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}
    };
    result = bezierApproxByOneCurveByInitVectors(
        points,
        0,
        pointsSize - 1,
        e1,
        e2,
        tDist,
        &controls
    );
    if (result != BEZIER_APPROX_OK) {
        goto cleanup;
    }

    controlsStack = (BezierControlsStackEntry*)malloc(
        sizeof(BezierControlsStackEntry) * controlsStackCapacity
    );
    if (!controlsStack) {
        goto cleanup;
    }

    controlsStack[0].controls = controls;
    controlsStack[0].e1 = e1;
    controlsStack[0].e2 = e2;
    controlsStack[0].fistIdx = 0;
    controlsStack[0].lastIdx = pointsSize - 1;
    controlsStackSize = 1;

    while (controlsStackSize > 0) {
        --controlsStackSize;
        BezierControlsStackEntry entry = controlsStack[controlsStackSize];
        double maxDist;
        int maxDistIdx;
        getMaxDistance(entry.controls, points, tDist, entry.fistIdx, entry.lastIdx, &maxDist, &maxDistIdx);
        if (maxDist <= precision) {
            assert(controlsAnsSize + 1 <= controlsAnsCapacity);
            controlsAns[controlsAnsSize] = entry.controls;
            ++controlsAnsSize;
            continue;
        }

        BezierApproxPoint eSplit = substructPoint(points[maxDistIdx + 1], points[maxDistIdx - 1]);
        if (normalizePoint(&eSplit) != BEZIER_APPROX_OK) {
            result = BEZIER_APPROX_ARGUMENTS_ERROR;
            goto cleanup;
        }

        result = bezierApproxByOneCurveByInitVectors(
            points,
            maxDistIdx,
            entry.lastIdx,
            eSplit,
            entry.e2,
            tDist,
            &controls
        );
        if (result != BEZIER_APPROX_OK) {
            goto cleanup;
        }

        assert(controlsStackSize + 1 <= controlsStackCapacity);
        controlsStack[controlsStackSize].controls = controls;
        controlsStack[controlsStackSize].e1 = eSplit;
        controlsStack[controlsStackSize].e2 = entry.e2;
        controlsStack[controlsStackSize].fistIdx = maxDistIdx;
        controlsStack[controlsStackSize].lastIdx = entry.lastIdx;
        ++controlsStackSize;

        BezierApproxPoint eSplitInv = eSplit;
        eSplitInv.x = -eSplitInv.x;
        eSplitInv.y = -eSplitInv.y;

        result = bezierApproxByOneCurveByInitVectors(
            points,
            entry.fistIdx,
            maxDistIdx,
            entry.e1,
            eSplitInv,
            tDist,
            &controls
        );
        if (result != BEZIER_APPROX_OK) {
            goto cleanup;
        }

        assert(controlsStackSize + 1 <= controlsStackCapacity);
        controlsStack[controlsStackSize].controls = controls;
        controlsStack[controlsStackSize].e1 = entry.e1;
        controlsStack[controlsStackSize].e2 = eSplitInv;
        controlsStack[controlsStackSize].fistIdx = entry.fistIdx;
        controlsStack[controlsStackSize].lastIdx = maxDistIdx;
        ++controlsStackSize;
    }

    if (controlsAnsSize > *controlsBufferSize) {
        result = BEZIER_APPROX_BUFFER_TOO_SMALL;
        *controlsBufferSize = controlsAnsSize;
        goto cleanup;
    }

    memcpy(controlsBuffer, controlsAns, controlsAnsSize * sizeof(BezierApproxCurve3Controls));
    *controlsBufferSize = controlsAnsSize;
    result = BEZIER_APPROX_OK;

cleanup:
    if (controlsStack) {
        free(controlsStack);
        controlsStack = NULL;
    }
    if (tDist) {
        free(tDist);
        tDist = NULL;
    }
    if (controlsAns) {
        free(controlsAns);
        controlsAns = NULL;
    }
    return result;
}

int bezierApproxByOneCurve(
    const BezierApproxPoint points[],
    int firstPointIndex,
    int lastPointIndex,
    BezierApproxCurve3Controls* controls
) {
    int result = BEZIER_APPROX_FAILED;
    double* tDist = NULL;

    if (firstPointIndex + 1 > lastPointIndex) {
        result = BEZIER_APPROX_NOT_ENOUGH_POINTS_ERROR;
        goto cleanup;
    }

    BezierApproxPoint e1 = substructPoint(points[firstPointIndex + 1], points[firstPointIndex]);
    if (normalizePoint(&e1) != BEZIER_APPROX_OK) {
        result = BEZIER_APPROX_ARGUMENTS_ERROR;
        goto cleanup;
    }

    BezierApproxPoint e2 = substructPoint(points[lastPointIndex - 1], points[lastPointIndex]);
    if (normalizePoint(&e2) != BEZIER_APPROX_OK) {
        result = BEZIER_APPROX_ARGUMENTS_ERROR;
        goto cleanup;
    }

    int tSize = lastPointIndex - firstPointIndex + 1;
    tDist = initTdist(points, tSize);
    if (!tDist) {
        goto cleanup;
    }

    result = bezierApproxByOneCurveByInitVectors(
        points,
        firstPointIndex,
        lastPointIndex,
        e1,
        e2,
        tDist,
        controls
    );

cleanup:
    if (tDist) {
        free(tDist);
        tDist = NULL;
    }
    return result;
}

BezierApproxPoint bezierApproxGetCurveValue(
    const BezierApproxCurve3Controls controls,
    double t
) {
    double x = 
        bValue(0, t) * controls.P0.x +
        bValue(1, t) * controls.P1.x +
        bValue(2, t) * controls.P2.x +
        bValue(3, t) * controls.P3.x;

    double y =
        bValue(0, t) * controls.P0.y +
        bValue(1, t) * controls.P1.y +
        bValue(2, t) * controls.P2.y +
        bValue(3, t) * controls.P3.y;

    BezierApproxPoint result;
    result.x = x;
    result.y = y;
    return result;
}
