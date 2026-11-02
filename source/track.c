#include "include/track.h"
#include "include/logging.h"        // SAMPLE_RATE_HZ

static Segment seg[MAX_SEGMENTS];   // sem se ukládá jízda
static MapSeg  lapMap[MAX_SEGMENTS];// hotová mapa kola
static uint8_t segCount = 0;        // poslední prvek je otevřený úsek
static uint8_t sigCount = 0;        // počet úseků podpisu (0 = ještě není)
static uint8_t pendingType = 0xFF;
static uint8_t pendingCnt = 0;
static uint8_t lapDone = 0;
static uint8_t overflow = 0;
static uint8_t mapCnt = 0;
static uint32_t lapMm = 0;

uint32_t Track_lenMm(uint16_t samples)
{
    return ((uint32_t)samples * REF_SPEED_MM_S) / SAMPLE_RATE_HZ;
}

static void push(uint8_t type, uint16_t len)
{
    if (segCount < MAX_SEGMENTS)
    {
        seg[segCount].type = type;
        seg[segCount].len  = len;
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
    if (tol < LEN_TOL_ABS_MM) tol = LEN_TOL_ABS_MM;
    return (a->type == b->type) && (diff <= tol);
}

/* Podpis = celé uzavřené úseky od seg[1], dokud součet nedosáhne SIG_MM */
static void updateSignature(void)
{
    uint32_t sum = 0;
    uint8_t i;

    if (sigCount) return;

    for (i = 1; i + 1 < segCount; i++)          // uzavřené: seg[1..segCount-2]
        sum += Track_lenMm(seg[i].len);

    if (sum >= SIG_MM) sigCount = segCount - 2;
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
        pos += l;
    }
    lapMm = pos;
}

static uint8_t checkLap(void)
{
    uint8_t closed = segCount - 1;
    uint8_t j, m;

    if (!sigCount) return 0;
    if (closed < 2 * sigCount + 1) return 0;     // podpis a kandidát se nesmí překrývat

    m = closed - sigCount;
    for (j = 0; j < sigCount; j++)
    {
        if (!similar(&seg[1 + j], &seg[m + j])) return 0;
    }

    buildMap(m);
    lapDone = 1;
    return 1;
}

void Track_reset(void)
{
    segCount = 0; sigCount = 0; lapDone = 0; overflow = 0; mapCnt = 0; lapMm = 0;
    pendingType = 0xFF; pendingCnt = 0;
}

uint8_t Track_update(uint8_t state)
{
    Segment *cur;

    if (lapDone) return 1;

    if (segCount == 0) { push(state, 1); return 0; }

    cur = &seg[segCount - 1];

    if (state == cur->type)
    {
        cur->len += 1 + pendingCnt;
        pendingCnt = 0;
        pendingType = 0xFF;
        return 0;
    }

    if (state == pendingType)
    {
        pendingCnt++;
    }
    else
    {
        cur->len += pendingCnt;
        pendingType = state;
        pendingCnt = 1;
    }

    if (pendingCnt < MIN_SEG_SAMPLES) return 0;

    push(pendingType, pendingCnt);
    pendingType = 0xFF;
    pendingCnt = 0;

    updateSignature();
    return checkLap();
}

uint8_t Track_sigReady(void) { return sigCount != 0; }

uint8_t        Track_isDone(void)       { return lapDone; }
uint8_t        Track_overflow(void)     { return overflow; }
uint8_t        Track_mapCount(void)     { return mapCnt; }
const MapSeg * Track_mapGet(uint8_t i)  { return &lapMap[i]; }
uint32_t       Track_lapLenMm(void)     { return lapMm; }