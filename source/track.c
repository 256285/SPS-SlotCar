#include "include/track.h"
#include "include/logging.h"        // SAMPLE_RATE_HZ

static Segment seg[MAX_SEGMENTS];   // sem se ukládá jízda
static MapSeg  lapMap[MAX_SEGMENTS];// hotová mapa kola
static uint8_t segCount = 0;        // poslední prvek je otevřený úsek
static uint8_t sigCount = 0;        // počet úseků podpisu (0 = ještě není)
static uint8_t pendingType = 0xFF;
static uint8_t pendingCnt = 0;
static uint32_t pendingYawSum = 0;
static uint8_t lapDone = 0;
static uint8_t overflow = 0;
static uint8_t mapCnt = 0;
static uint32_t lapMm = 0;


/* Localization */

static uint8_t raceIndex = 0;
static uint16_t raceTicks = 0;
static uint32_t raceYawSum = 0;

static uint8_t racePendingType = 0xFF;
static uint8_t racePendingCnt = 0;
static uint32_t racePendingYawSum = 0;

static uint8_t raceLapCount = 0;


/* Helpers */

static uint32_t absYaw(int32_t yaw)
{
    return (yaw < 0) ? (uint32_t)(-yaw) : (uint32_t)yaw;
}


uint32_t Track_lenMm(uint16_t samples)
{
    return ((uint32_t)samples * REF_SPEED_MM_S) / SAMPLE_RATE_HZ;
}


static void push(uint8_t type, uint16_t len, uint32_t yawSum)
{
    if (segCount < MAX_SEGMENTS)
    {
        seg[segCount].type   = type;
        seg[segCount].len    = len;
        seg[segCount].yawSum = yawSum;

        segCount++;
    }
    else
    {
        overflow = 1;
    }
}


static uint8_t similar(const Segment *a, const Segment *b)
{
    uint32_t la = Track_lenMm(a->len);
    uint32_t lb = Track_lenMm(b->len);
    uint32_t diff = (la > lb) ? (la - lb) : (lb - la);
    uint32_t tol  = (la * LEN_TOL_PCT) / 100;

    if (tol < LEN_TOL_ABS_MM)
        tol = LEN_TOL_ABS_MM;

    return (a->type == b->type) && (diff <= tol);
}


/* Podpis = celé uzavřené úseky od seg[1], dokud součet nedosáhne SIG_MM */
static void updateSignature(void)
{
    uint32_t sum = 0;
    uint8_t i;

    if (sigCount)
        return;

    for (i = 1; i + 1 < segCount; i++)          // uzavřené: seg[1..segCount-2]
        sum += Track_lenMm(seg[i].len);

    if (sum >= SIG_MM)
        sigCount = segCount - 2;
}


static void buildMap(uint8_t m)                  // mapa = seg[1]..seg[m-1]
{
    uint8_t i;
    uint32_t pos = 0;

    mapCnt = m - 1;

    for (i = 0; i < mapCnt; i++)
    {
        uint32_t l = Track_lenMm(seg[1 + i].len);

        lapMap[i].type    = seg[1 + i].type;
        lapMap[i].len     = seg[1 + i].len;
        lapMap[i].startMm = (uint16_t)pos;
        lapMap[i].lenMm   = (uint16_t)l;

        lapMap[i].yawSum =
            seg[1 + i].yawSum;

        lapMap[i].predictedLen =
            seg[1 + i].len;

        pos += l;
    }

    lapMm = pos;
}


/* Localization start */

static void initRaceTracking(void)
{
    if (mapCnt == 0)
        return;

    raceIndex = sigCount % mapCnt;

    raceTicks =
        seg[segCount - 1].len;

    raceYawSum =
        seg[segCount - 1].yawSum;

    racePendingType = 0xFF;
    racePendingCnt = 0;
    racePendingYawSum = 0;

    raceLapCount = 0;
}


static uint8_t checkLap(void)
{
    uint8_t closed = segCount - 1;
    uint8_t j, m;

    if (!sigCount)
        return 0;

    if (closed < 2 * sigCount + 1)
        return 0;                               // podpis a kandidát se nesmí překrývat

    m = closed - sigCount;

    for (j = 0; j < sigCount; j++)
    {
        if (!similar(&seg[1 + j], &seg[m + j]))
            return 0;
    }

    buildMap(m);

    lapDone = 1;

    initRaceTracking();

    return 1;
}


/* Prediction */

static void updatePrediction(uint8_t index, uint16_t actualTicks)
{
    uint32_t predicted;

    if (actualTicks == 0)
        return;

    if (index >= mapCnt)
        return;

    predicted = lapMap[index].predictedLen;

    lapMap[index].predictedLen =
        (uint16_t)(
            (3UL * predicted + actualTicks + 2UL)
            / 4UL
        );
}


/* Localization */

static void updateRaceTracking(uint8_t state, uint32_t yawAbs)
{
    uint8_t currentType;
    uint8_t nextIndex;
    uint8_t nextType;

    if (!lapDone || mapCnt == 0)
        return;

    currentType =
        lapMap[raceIndex].type;

    nextIndex =
        raceIndex + 1;

    if (nextIndex >= mapCnt)
        nextIndex = 0;

    nextType =
        lapMap[nextIndex].type;


    /* Current segment */

    if (state == currentType)
    {
        raceTicks +=
            1 + racePendingCnt;

        raceYawSum +=
            yawAbs + racePendingYawSum;

        racePendingType = 0xFF;
        racePendingCnt = 0;
        racePendingYawSum = 0;

        return;
    }


    /* Expected next segment */

    if (state == nextType)
    {
        if (racePendingType == state)
        {
            racePendingCnt++;

            racePendingYawSum +=
                yawAbs;
        }
        else
        {
            raceTicks +=
                racePendingCnt;

            raceYawSum +=
                racePendingYawSum;

            racePendingType = state;
            racePendingCnt = 1;

            racePendingYawSum =
                yawAbs;
        }


        if (racePendingCnt < MIN_SEG_SAMPLES)
            return;


        /* Transition confirmed */

        updatePrediction(
            raceIndex,
            raceTicks
        );


        raceIndex =
            nextIndex;


        if (raceIndex == 0)
            raceLapCount++;


        raceTicks =
            racePendingCnt;

        raceYawSum =
            racePendingYawSum;


        racePendingType = 0xFF;
        racePendingCnt = 0;
        racePendingYawSum = 0;

        return;
    }


    /* Unexpected state */

    raceTicks +=
        1 + racePendingCnt;

    raceYawSum +=
        yawAbs + racePendingYawSum;

    racePendingType = 0xFF;
    racePendingCnt = 0;
    racePendingYawSum = 0;
}


void Track_reset(void)
{
    segCount = 0;
    sigCount = 0;
    lapDone = 0;
    overflow = 0;
    mapCnt = 0;
    lapMm = 0;

    pendingType = 0xFF;
    pendingCnt = 0;
    pendingYawSum = 0;

    raceIndex = 0;
    raceTicks = 0;
    raceYawSum = 0;

    racePendingType = 0xFF;
    racePendingCnt = 0;
    racePendingYawSum = 0;

    raceLapCount = 0;
}


uint8_t Track_update(uint8_t state, int32_t yaw)
{
    Segment *cur;
    uint32_t yawAbs = absYaw(yaw);


    /* Race localization */

    if (lapDone)
    {
        updateRaceTracking(
            state,
            yawAbs
        );

        return 0;
    }


    /* First segment */

    if (segCount == 0)
    {
        push(
            state,
            1,
            yawAbs
        );

        return 0;
    }


    cur = &seg[segCount - 1];


    /* Same segment */

    if (state == cur->type)
    {
        cur->len +=
            1 + pendingCnt;

        cur->yawSum +=
            yawAbs + pendingYawSum;

        pendingCnt = 0;
        pendingType = 0xFF;
        pendingYawSum = 0;

        return 0;
    }


    /* Candidate */

    if (state == pendingType)
    {
        pendingCnt++;

        pendingYawSum +=
            yawAbs;
    }
    else
    {
        cur->len +=
            pendingCnt;

        cur->yawSum +=
            pendingYawSum;

        pendingType = state;
        pendingCnt = 1;

        pendingYawSum =
            yawAbs;
    }


    if (pendingCnt < MIN_SEG_SAMPLES)
        return 0;


    push(
        pendingType,
        pendingCnt,
        pendingYawSum
    );

    pendingType = 0xFF;
    pendingCnt = 0;
    pendingYawSum = 0;


    updateSignature();

    return checkLap();
}


uint8_t Track_sigReady(void)
{
    return sigCount != 0;
}


uint8_t Track_isDone(void)
{
    return lapDone;
}


uint8_t Track_overflow(void)
{
    return overflow;
}


uint8_t Track_mapCount(void)
{
    return mapCnt;
}


const MapSeg * Track_mapGet(uint8_t i)
{
    return &lapMap[i];
}


uint32_t Track_lapLenMm(void)
{
    return lapMm;
}


/* Localization */

uint8_t Track_currentType(void)
{
    if (!lapDone || mapCnt == 0)
        return 0xFF;

    return lapMap[raceIndex].type;
}


uint8_t Track_nextType(void)
{
    uint8_t nextIndex;

    if (!lapDone || mapCnt == 0)
        return 0xFF;

    nextIndex =
        raceIndex + 1;

    if (nextIndex >= mapCnt)
        nextIndex = 0;

    return lapMap[nextIndex].type;
}


uint8_t Track_currentIndex(void)
{
    return raceIndex;
}


uint16_t Track_segmentTicks(void)
{
    return raceTicks;
}


uint16_t Track_predictedTicks(void)
{
    if (!lapDone || mapCnt == 0)
        return 0;

    return lapMap[raceIndex].predictedLen;
}


/* Turn progress */

uint8_t Track_turnProgressReached(uint8_t percent)
{
    uint32_t refYaw;
    uint32_t threshold;

    if (!lapDone || mapCnt == 0)
        return 0;

    if (lapMap[raceIndex].type == STRAIGHT)
        return 0;

    if (percent > 100)
        percent = 100;

    refYaw =
        lapMap[raceIndex].yawSum;

    if (refYaw == 0)
        return 0;

    threshold =
        (refYaw / 100UL) * percent;

    threshold +=
        ((refYaw % 100UL) * percent)
        / 100UL;

    return raceYawSum >= threshold;
}


uint8_t Track_raceLapCount(void)
{
    return raceLapCount;
}