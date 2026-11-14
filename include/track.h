#ifndef TRACK_H_
#define TRACK_H_

#include <stdint.h>

typedef enum { LEFT_TURN, STRAIGHT, RIGHT_TURN } TurnState;

typedef struct
{
    uint8_t  type;
    uint16_t len;            // vzorky
    uint32_t yawSum;         // integrál |yaw|
} Segment;

typedef struct
{
    uint8_t  type;
    uint16_t len;            // vzorky
    uint16_t startMm;        // poloha začátku od počátku mapy
    uint16_t lenMm;

    uint32_t yawSum;         // referenční integrál zatáčky
    uint16_t predictedLen;   // predikovaná délka úseku ve vzorcích
} MapSeg;

#define MAX_SEGMENTS     60
#define SIG_MM           1700   // délka podpisu trati
#define MIN_SEG_SAMPLES  5
#define LEN_TOL_PCT      24
#define LEN_TOL_ABS_MM   300
#define REF_SPEED_MM_S   1000
#define WARMUP_SAMPLES   150

static uint16_t warmup = 0;


/* Mapping */

void           Track_reset(void);
uint8_t        Track_update(uint8_t state, int32_t yaw);   // 1 = kolo dokončeno
uint8_t        Track_isDone(void);
uint8_t        Track_overflow(void);
uint32_t       Track_lenMm(uint16_t samples);
uint8_t        Track_mapCount(void);
const MapSeg * Track_mapGet(uint8_t i);
uint32_t       Track_lapLenMm(void);
uint8_t        Track_sigReady(void);


/* Localization */

uint8_t        Track_currentType(void);
uint8_t        Track_nextType(void);
uint8_t        Track_currentIndex(void);

uint16_t       Track_segmentTicks(void);
uint16_t       Track_predictedTicks(void);

uint8_t        Track_turnProgressReached(uint8_t percent);
uint8_t        Track_raceLapCount(void);


#endif