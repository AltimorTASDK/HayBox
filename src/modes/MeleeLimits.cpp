#include "modes/MeleeLimits.hpp"
#include "modes/Melee20Button.hpp"
#include "modes/Fixed.h"

#define HISTORYLEN 5//changes in target stick position

#define ANALOG_DEAD_MIN (128-22)/*this is in the deadzone*/
#define ANALOG_DEAD_MAX (128+22)/*this is in the deadzone*/
#define ANALOG_CROUCH (128-50)/*this y coordinate will hold a crouch*/
#define ANALOG_TAPJUMP (128+53)/*this Y coordinate will tap jump always (running is less)*/
#define ANALOG_DASH_LEFT (128-64)/*this x coordinate will dash left*/
#define ANALOG_DASH_RIGHT (128+64)/*this x coordinate will dash right*/
#define ANALOG_SDI_LEFT (128-56)/*this x coordinate will sdi left*/
#define ANALOG_SDI_RIGHT (128+56)/*this x coordinate will sdi right*/
#define MELEE_RIM_RAD1 6185/*if x^2+y^2 >= this, it's on the rim and 6ms*/
#define MELEE_RIM_RAD2 6858/*if x^2+y^2 >= this, it's past the rim and 7ms*/
#define MELEE_RIM_RAD3 8979/*if x^2+y^2 >= this, it's past the rim and 8ms*/

#define TRAVELTIME_EASY1 6//ms
#define TRAVELTIME_EASY2 6//7//ms
#define TRAVELTIME_EASY3 6//8//ms for 112+cubic it takes 83% to get to dash, for 80+linear it takes 80% to get to dash
#define TRAVELTIME_CROSS 12//ms to cross gate; unused
#define TRAVELTIME_INTERNAL 12//ms for "easy" to "internal"; 2/3 frame
#define TRAVELTIME_SLOW (4*16)//ms for tap SDI nerfing, 4 frames

#define TIMELIMIT_DOWNUP (16*3*250)//units of 4us; how long after a crouch to upward input should it begin a jump?
#define JUMP_TIME (16*2*250)//units of 4us; after a recent crouch to upward input, always hold full up for 2 frames

#define TIMELIMIT_FRAME 4167//(16.66...*250)//units of 4us; 1 frame, for reference
#define TIMELIMIT_HALFFRAME 2083//(8.33...*250)//units of 4us; 1/2 frame
#define TIMELIMIT_DEBOUNCE (6*250)//units of 4us; 6ms;
#define TIMELIMIT_SIMUL (2*250)//units of 4us; 3ms: if the latest inputs are less than 2 ms apart then don't nerf cardiag

#define TIMELIMIT_TAPSHUTOFF 16000//4 frames for tap jump shutoff

//not used #define TIMELIMIT_DASH 60000//(16*15*250)//units of 4us; last dash time prior to a pivot input; 15 frames

//not used #define TIMELIMIT_QCIRC 24000//(16*6*250)//units of 4us; 6 frames

#define TIMELIMIT_TAP 22000//(16*5.5*250)//units of 4us; 5.5 frames
#define TIMELIMIT_TAP_PLUS 34000//(16*8.5*250)//units of 4us; 3 additional frames

#define TIMELIMIT_CARDIAG 32000//(16*8*250)//units of 4us; 8 frames

#define TIMELIMIT_WANK 22000//(16*5.5*250)//units of 4us; 5.5 frames

#define TIMELIMIT_PIVOTTILT 32000//(16*8*250)//units of 4us; 8 frames

enum pivotdir{P_None, P_Leftright, P_Rightleft};
enum travelType{T_Lin, T_Quad, T_Cubic, T_Quart, T_Delay};

#define ZONE_DIR 0b0000'1111
#define ZONE_U   0b0000'0001
#define ZONE_D   0b0000'0010
#define ZONE_L   0b0000'0100
#define ZONE_R   0b0000'1000

#define BITS_SDI      0b1111'0000
#define BITS_SDI_WANK     0b0001'0000
#define BITS_SDI_TAP_CARD 0b0010'0000
#define BITS_SDI_TAP_DIAG 0b0100'0000
#define BITS_SDI_TAP_CRDG 0b1000'0000

typedef struct {
    uint16_t timestamp;//in samples
    uint8_t tt;//travel time used in ms
    uint8_t x;
    uint8_t y;
    uint8_t x_start;
    uint8_t y_start;
    uint8_t x_end;
    uint8_t y_end;
} shortstate;

//for sdi nerfs, we want to record only movement between sdi zones, ignoring movement within zones
typedef struct {
    uint16_t timestamp;//in samples
    uint8_t zone;
    bool stale;
} sdizonestate;

//for pivot nerfs, we want to record only movement between dash zones, ignoring movement within zones
typedef struct {
    uint16_t timestamp;//in samples
    uint8_t zone;
    bool stale;
} pivotzonestate;

uint8_t isEasy(const uint8_t x, const uint8_t y) {
    //is it on the rim?
    const uint8_t xnorm = (x > ANALOG_STICK_NEUTRAL ? (x-ANALOG_STICK_NEUTRAL) : (ANALOG_STICK_NEUTRAL-x));
    const uint8_t ynorm = (y > ANALOG_STICK_NEUTRAL ? (y-ANALOG_STICK_NEUTRAL) : (ANALOG_STICK_NEUTRAL-y));
    const uint16_t xsquared = xnorm*xnorm;
    const uint16_t ysquared = ynorm*ynorm;
    const uint16_t radSquared = xsquared+ysquared;
    if((radSquared) >= MELEE_RIM_RAD1) {
        //is it within 3 units of the diagonal? or is it a cardinal?
        const uint8_t diagMax = max(xnorm, ynorm);
        const uint8_t diagMin = min(xnorm, ynorm);
        const uint8_t diff = diagMax - diagMin;
        if(diff <= 6 || xnorm == 0 || ynorm == 0) {
            //if so, yes
            if(radSquared >= MELEE_RIM_RAD3) {
                return 3;
            } else if(radSquared >= MELEE_RIM_RAD2) {
                return 2;
            } else {
                return 1;
            }
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

uint8_t getRandom(uint16_t currentTime) {
    // Use the time of the first directional input to initialize the LCG
    static uint16_t random = currentTime;
    // Constant from https://arxiv.org/pdf/2001.05304.pdf
    random = 0xD9F5 * random + 1;
    // XOR all nibbles together, necessary to minimize patterns with a power of 2 LCG
    uint8_t xor1 = random ^ (random >> 8);
    uint8_t xor2 = xor1   ^ (xor1   >> 4);
    return xor2 % 16;
}
// 1 2 1 //4
// 2 4 2 //8
// 1 2 1 //4
// totals up to 16
void randomizeCoord(uint8_t &x, uint8_t &y, uint16_t currentTime) {
    const uint8_t random = getRandom(currentTime);
    const uint8_t left = ((random ^ 0b0) & 0b11) == 0;
    //middle is when random & 0b01 or 0b10
    const uint8_t right = ((random ^ 0b11) & 0b11) == 0;
    const uint8_t up = ((random ^ 0b0000) & 0b1100) == 0;
    //middle is when random & 0b01xx or 0b10xx
    const uint8_t down = ((random ^ 0b1100) & 0b1100) == 0;

    //don't randomize when x or y are neutral
    x = (x != ANALOG_STICK_NEUTRAL) ? x - left + right : x;
    y = (y != ANALOG_STICK_NEUTRAL) ? y - down + up : y;
}

uint8_t lookback(const uint8_t currentIndex,
                 const uint8_t samplesBack) {
    if(samplesBack > currentIndex) {
        return (HISTORYLEN-samplesBack) + currentIndex;
    } else {
        return currentIndex - samplesBack;
    }
}

//the output will have the following bits set:
//0b0000'0001 for up
//0b0000'0010 for down
//0b0000'0100 for left
//0b0000'1000 for right
//no bits set for neutral
//thresholds are dash for cardinals, and deadzone for diagonals
uint8_t sdiZone(const uint8_t x, const uint8_t y) {
    uint8_t result = 0b0000'0000;
    if(x >= ANALOG_DEAD_MIN && x <= ANALOG_DEAD_MAX) {
        if(y < ANALOG_SDI_LEFT) {
            result = result | ZONE_D;
        } else if(y > ANALOG_SDI_RIGHT) {
            result = result | ZONE_U;
        }
    } else if(x < ANALOG_DEAD_MIN) {
        if(y < ANALOG_DEAD_MIN) {
            result = result | ZONE_D | ZONE_L;
        } else if(y > ANALOG_DEAD_MAX) {
            result = result | ZONE_U | ZONE_L;
        } else if(x <= ANALOG_SDI_LEFT) {
            result = result | ZONE_L;
        }
    } else /*if(x > ANALOG_DEAD_MAX)*/ {
        if(y < ANALOG_DEAD_MIN) {
            result = result | ZONE_D | ZONE_R;
        } else if(y > ANALOG_DEAD_MAX) {
            result = result | ZONE_U | ZONE_R;
        } else if(x >= ANALOG_SDI_RIGHT) {
            result = result | ZONE_R;
        }
    }
    return result;
}

//the output will have the following bits set:
//0b0000'0100 for left
//0b0000'1000 for right
//no bits set for neutral
//thresholds are dash for the x-axis
uint8_t pivotZone(const uint8_t x) {
    uint8_t result = 0b0000'0000;
    if(x <= ANALOG_DASH_LEFT) {
        result = result | ZONE_L;
    } else if(x >= ANALOG_DASH_RIGHT) {
        result = result | ZONE_R;
    }
    return result;
}

uint8_t uptiltShutoffZone(const uint8_t y) {
    uint8_t result = 0b0000'0000;
    if(y > ANALOG_DEAD_MAX && y < ANALOG_TAPJUMP) {
        result = result | ZONE_U;
    }
    return result;
}

//not a general purpose popcount, this is specifically for zones
uint8_t popcount_zone(const uint8_t bitsIn) {
    uint8_t count = 0;
    for(uint8_t i = 0; i < 4; i++) {
        if((bitsIn >> i) & 0b0000'0001) {
            count++;
        }
    }
    return count;
}

uint8_t isTapSDI(const sdizonestate zoneHistory[HISTORYLEN],
                 const uint8_t currentIndex,
                 const bool currentTime,
                 const uint16_t sampleSpacing) {
    uint8_t output = 0;

    //grab the last five zones
    const uint8_t historyLength = min(5, HISTORYLEN);
    uint8_t zoneList[historyLength];
    uint16_t timeList[historyLength];
    bool staleList[historyLength];
    for(int i = 0; i < historyLength; i++) {
        const uint8_t index = lookback(currentIndex, i);
        zoneList[i] = zoneHistory[index].zone;
        timeList[i] = zoneHistory[index].timestamp;
        staleList[i] = zoneHistory[index].stale;
    }
    const uint8_t popCur = popcount_zone(zoneList[0]);
    const uint8_t popOne = popcount_zone(zoneList[1]);

    //detect repeated center-cardinal sequences, or repeated cardinal-diagonal sequences
    // if we're changing zones back and forth
    if(zoneList[0] != zoneList[1] && (zoneList[0] == zoneList[2]) && (zoneList[1] == zoneList[3])) {
        //check the time duration
        const uint16_t timeDiff0 = (currentTime - timeList[2])*sampleSpacing;//make sure things aren't reliant on long-past inputs
        const uint16_t timeDiff1 = (timeList[0] - timeList[2])*sampleSpacing;//rising edge to rising edge, or falling edge to falling edge
        //const uint16_t timeDiff2 = (timeList[0] - timeList[1])*sampleSpacing;//rising to falling, or falling to rising
        //We want to nerf it if there is more than one press every 6 frames, but not if the previous press or release duration is less than 1 frame
        if(!staleList[3] && (timeDiff0 < TIMELIMIT_TAP_PLUS && timeDiff1 < TIMELIMIT_TAP && timeDiff0 > TIMELIMIT_DEBOUNCE)) {
            if((zoneList[0] == 0) || (zoneList[1] == 0)) {//if one of the pairs of zones is zero, it's tapping a cardinal
                output = output | BITS_SDI_TAP_CARD;
            } else if(popCur+popOne == 3) { //one is cardinal and the other is diagonal
                output = output | BITS_SDI_TAP_DIAG;
            }
        }
    }
    //detect:
    //         center-cardinal-diagonal-center-cardinal (-diagonal)
    //center-cardinal-diagonal-cardinal-center-cardinal (-diagonal)
    //where the the diagonals are the same
    //the cardinals don't have to be the same in case they're inputting 2365 etc

    //simpler: if the last 5 inputs are in the origin, one cardinal, and one diagonal
    //and that there was a recent return to center
    //at least one of each zone, and at least two diagonals
    uint8_t diagZone = 0b1111'1111;
    uint8_t origCount = 0;
    uint8_t cardCount = 0;
    uint8_t diagCount = 0;
    for(int i = 0; i < historyLength; i++) {
        const uint8_t popcnt = popcount_zone(zoneList[i]);
        if(popcnt == 0) {
            origCount++;
        } else if(popcnt == 1) {
            cardCount++;
        } else {
            diagCount++;
            diagZone = diagZone & zoneList[i];//if two of these don't match, it'll have zero or one bits set
        }
    }
    {//to limit scope of these vars
        //check the bit count of diagonal matching
        const bool diagMatch = popcount_zone(diagZone) == 2;
        //check whether it returned to center recently
        //const bool recentOrig = (zoneList[1] & zoneList[2]) == 0;//may be too lenient in case people throw in modifier taps?
        //check whether the input was fast enough
        const bool shortTime = ((timeList[0] - timeList[4])*sampleSpacing < TIMELIMIT_CARDIAG) &&
                               ((timeList[0] - timeList[1])*sampleSpacing > TIMELIMIT_SIMUL) &&
                               !staleList[4];

        // if only the same diagonal was pressed
        //              if the origin, cardinal, and two diagonals were all entered
        //                                                         within the time limit
        if(diagMatch && origCount && cardCount && diagCount > 1 && shortTime) {
            output = output | BITS_SDI_TAP_CRDG;
        }
    }

    //3 input sdi
    //center-cardinal-diagonal-diagonal
    //center-cardinal-diagonal-same cardinal-diagonal
    //all directions except center must be the same
    uint8_t cardZone = 0b1111'1111;
    diagZone = 0b1111'1111;
    origCount = 0;
    cardCount = 0;
    diagCount = 0;
    for(int i = 0; i < historyLength; i++) {
        const uint8_t popcnt = popcount_zone(zoneList[i]);
        if(popcnt == 0) {
            origCount++;
            break;//stop counting once there's an origin
        } else if(popcnt == 1) {
            cardCount++;
            cardZone = cardZone & zoneList[i];//if there are two different cardinals then it won't count
        } else {
            diagCount++;
            diagZone = diagZone & zoneList[i];//if two of these don't match, it'll have zero or one bits set
        }
    }
    {//to limit scope of these vars
        //check the bit count of diagonal matching
        const bool adjacentDiag = popcount_zone(diagZone & cardZone) == 1;
        const bool shortTime = ((timeList[0] - timeList[3])*sampleSpacing < TIMELIMIT_WANK) &&
                               !staleList[3];
        //if it hit two different diagonals
        //                 hit origin, at least one cardinal, and two diagonals
        //                                                            within the time limit
        if(adjacentDiag && origCount && cardCount && diagCount > 1 && shortTime) {
            output = output | BITS_SDI_WANK;
        }
    }

    //return the last cardinal in the zone list before the last diagonal, useful for SDI diagonal nerfs.
    bool lookNow = false;
    bool alreadyWritten = false;
    for(int i = 0; i < historyLength; i++) {
        if((popcount_zone(zoneList[i]) == 2) && !alreadyWritten) {
            lookNow = true;
        }
        if((popcount_zone(zoneList[i]) == 1) && !alreadyWritten && lookNow) {
            output = output | zoneList[i];
            alreadyWritten = true;
        }
    }
    return output;
}

Fixed88 quadraticEasing(Fixed88 i){
    Fixed88 x2 = fixedMul(i,i);
    return x2;
}
Fixed88 cubicEasing(Fixed88 i){
    Fixed88 x2 = fixedMul(i,i);
    Fixed88 x3 = fixedMul(x2,i);
    return x3;
}
Fixed88 quarticEasing(Fixed88 i){
    Fixed88 x2 = fixedMul(i,i);
    Fixed88 x4 = fixedMul(x2,x2);
    return x4;
}

void travelTimeCalc(const uint16_t samplesElapsed,
                    const uint16_t sampleSpacing,//units of 4us
                    const uint8_t msTravel,
                    const uint8_t targetX,
                    const uint8_t targetY,
                    const uint8_t startX,
                    const uint8_t startY,
                    const uint8_t destX,
                    const uint8_t destY,
                    const travelType type,
                    bool &oldChange,//if the time gets too long, set it to true
                    bool &doneTraveling,//apply tt if false; when travel time is done, set it to true
                    uint8_t &outX,
                    uint8_t &outY) {
    //check for old data; this prevents overflow from causing issues
    if(samplesElapsed > 5*16*2) {//5 frames * 16 ms * max 2 samples per frame
        oldChange = true;
        doneTraveling = true;
    }
    const uint16_t timeElapsed = samplesElapsed*sampleSpacing;//units of 4 us
    if(type == T_Lin) {
        const uint16_t travelTimeElapsed = timeElapsed/msTravel;//250 times the fraction of the travel time elapsed

        //For the following 256s, they used to be 250, but AVR had division issues
        // and would only be able to reach a value of 6 when returning to neutral,
        // from the right only.
        //Switching to 256 fixed it somehow, at the expense of 2.4% greater travel time.
        const uint16_t cappedTT = min(256, travelTimeElapsed);

        const int16_t dX = ((destX-startX)*cappedTT)/256;
        const int16_t dY = ((destY-startY)*cappedTT)/256;

        const uint16_t newX = startX+dX;
        const uint16_t newY = startY+dY;

        outX = (uint8_t) newX;
        outY = (uint8_t) newY;
    } else if (type == T_Delay) {
        if(timeElapsed <= msTravel*250) {
            outX = startX;
            outY = startY;
        } else {
            outX = destX;
            outY = destY;
            doneTraveling = true;
        }
    } else{
        uint16_t usTravel4 = msTravel*250;//units of 4 us
        const uint16_t clampedElapsed = min(usTravel4, timeElapsed);
        if(clampedElapsed == usTravel4) {
            doneTraveling = true;
        }
        const Fixed88 timeElapsedPercent = fastDiv(intToFixed(int8_t(timeElapsed>>6)), intToFixed(int8_t(usTravel4>>6)));

        Fixed88 interpolatedTime = max(min(timeElapsedPercent, intToFixed(int8_t(1))), intToFixed(int8_t(0)));

        if(type == T_Quad) {
            interpolatedTime = quadraticEasing(interpolatedTime);
        } else if(type == T_Cubic) {
            interpolatedTime = cubicEasing(interpolatedTime);
        } else /*if(type == T_Quart)*/ {
            interpolatedTime = quarticEasing(interpolatedTime);
        }
        const Fixed88 x0 = intToFixed(int8_t(startX- 128)) >> 1;
        const Fixed88 x1 = intToFixed(int8_t(destX - 128)) >> 1;
        const Fixed88 y0 = intToFixed(int8_t(startY- 128)) >> 1;
        const Fixed88 y1 = intToFixed(int8_t(destY - 128)) >> 1;
        const Fixed88 fixedX = lerp(x0, x1, interpolatedTime) << 1;
        const Fixed88 fixedY = lerp(y0, y1, interpolatedTime) << 1;

        outX = uint8_t(fixedToInt(fixedX) + 128);
        outY = uint8_t(fixedToInt(fixedY) + 128);
    }

    if(oldChange || doneTraveling || msTravel == 0) {
        outX = destX;
        outY = destY;
    }
}

void limitWavedashAngle(uint8_t *prelimAX, uint8_t *prelimAY) {
    if(*prelimAX >= ANALOG_DEAD_MIN && *prelimAX <= ANALOG_DEAD_MAX) {
        return;
    }
    if(*prelimAY >= ANALOG_DEAD_MIN && *prelimAY <= ANALOG_DEAD_MAX) {
        return;
    }

    int8_t xCoord = *prelimAX - ANALOG_STICK_NEUTRAL;
    int8_t yCoord = *prelimAY - ANALOG_STICK_NEUTRAL;
    uint8_t xCoordAbs = abs(xCoord);
    uint8_t yCoordAbs = abs(yCoord);

    if (xCoordAbs * MX_SHIELD_TILT_Y > yCoordAbs * MX_SHIELD_TILT_X) {
        uint8_t xAdjusted = yCoordAbs * MX_SHIELD_TILT_X / MX_SHIELD_TILT_Y;
        *prelimAX = xCoord >= 0 ? ANALOG_STICK_NEUTRAL + xAdjusted : ANALOG_STICK_NEUTRAL - xAdjusted;
    } else if (yCoordAbs * MY_SHIELD_TILT_X > xCoordAbs * MY_SHIELD_TILT_Y) {
        uint8_t yAdjusted = xCoordAbs * MY_SHIELD_TILT_Y / MY_SHIELD_TILT_X;
        *prelimAY = yCoord >= 0 ? ANALOG_STICK_NEUTRAL + yAdjusted : ANALOG_STICK_NEUTRAL - yAdjusted;
    }
}

void limitOutputs(const uint16_t sampleSpacing,//in units of 4us
                  const abtest whichAB,
                  const InputState &inputs,
                  const OutputState &rawOutputIn,
                  OutputState &finalOutput) {
    //First, we want to check if the raw output has changed.
    //If it has changed, then we need to store it with a timestamp in our buffer.
    //Also check whether it's an "easy" coordinate or not (rim+origin = easy)
    //Then, we need to parse the buffer and detect whether:
    //  an empty pivot has likely occurred (0.5 to 1.5 frames?) ============ this depends on our travel time implementation unless we store the analog value history too
    //    how?
    //    (make you do a 2f jump input if you input an upward tilt coordinate within 10 frames)
    //    (lock out the A press if you input a downward tilt coordinate for 3 frames)
    //  the input is crouching
    //    (make you do a 2f jump input if you input an upward tilt coordinate within 4 frames)
    //  rapid quarter circling (wank di)
    //    lock out second diagonal (opposite axis). May not be important with neutral SOCD.
    //  rapid tapping (tap di)
    //    Temporarily increase travel time to... 16 ms? 24ms?
    //  rapid eighth-circling (cardinal diagonal neutral repeat)
    //    Increase travel time on cardinal and lock out later diagonals.

    static bool oldA = true;
    static bool doneTraveling = true;

    static uint16_t currentTime = 0;
    currentTime++;

    static shortstate aHistory[HISTORYLEN];
    static sdizonestate sdiZoneHist[HISTORYLEN];
    static pivotzonestate pivotZoneHist[HISTORYLEN];

    static bool initialized = false;
    if(!initialized) {
        for(int i = 0; i<HISTORYLEN; i++) {
            aHistory[i].timestamp = 0;
            aHistory[i].tt = 6;
            aHistory[i].x = ANALOG_STICK_NEUTRAL;
            aHistory[i].y = ANALOG_STICK_NEUTRAL;
            aHistory[i].x_start = ANALOG_STICK_NEUTRAL;
            aHistory[i].y_start = ANALOG_STICK_NEUTRAL;
            aHistory[i].x_end = ANALOG_STICK_NEUTRAL;
            aHistory[i].y_end = ANALOG_STICK_NEUTRAL;
            sdiZoneHist[i].timestamp = 0;
            sdiZoneHist[i].zone = 0;
            sdiZoneHist[i].stale = true;
            pivotZoneHist[i].timestamp = 0;
            pivotZoneHist[i].zone = 0;
            pivotZoneHist[i].stale = true;
        }
        initialized = true;
    }
    static uint8_t currentIndexA = 0;
    static uint8_t currentIndexSDI = 0;

    static travelType delayType = T_Cubic;

    //calculate travel from the previous step
    uint8_t prelimAX;
    uint8_t prelimAY;
    uint8_t prelimCX = rawOutputIn.rightStickX;
    uint8_t prelimCY = rawOutputIn.rightStickY;
    travelTimeCalc(currentTime-aHistory[currentIndexA].timestamp,
                   sampleSpacing,
                   aHistory[currentIndexA].tt,
                   aHistory[currentIndexA].x,
                   aHistory[currentIndexA].y,
                   aHistory[currentIndexA].x_start,
                   aHistory[currentIndexA].y_start,
                   aHistory[currentIndexA].x_end,
                   aHistory[currentIndexA].y_end,
                   delayType,
                   oldA,
                   doneTraveling,
                   prelimAX,
                   prelimAY);

    //Enforce a consistent minimum angle for the L/R NDM
    if(inputs.l || inputs.r) {
        limitWavedashAngle(&prelimAX, &prelimAY);
    }

    //detect an input that gives >50% pivot probability given travel time
    //if we are in a new pivot zone, record the new zone
    //we're keeping these in sequence so we copy them all
    if(pivotZoneHist[0].zone != pivotZone(prelimAX)) {
        for(int i = HISTORYLEN-1; i >= 0; i--) {
            pivotZoneHist[i+1].timestamp = pivotZoneHist[i].timestamp;
            pivotZoneHist[i+1].zone = pivotZoneHist[i].zone;
            pivotZoneHist[i+1].stale = pivotZoneHist[i].stale;
        }

        pivotZoneHist[0].timestamp = currentTime;
        pivotZoneHist[0].zone = pivotZone(prelimAX);
        pivotZoneHist[0].stale = false;
    }
    for(int i = 0; i < HISTORYLEN; i++) {
        if((currentTime-pivotZoneHist[i].timestamp) * (sampleSpacing>>1) > 15*16*125) { //15 frames
            pivotZoneHist[i].stale = true;
        }
    }

    //pivot inputs:
    //current--------------------past
    //---neutral ----left ---neutral ---right
    //---neutral ----left ---right
    //---neutral ---right ---neutral ---left
    //---neutral ---right ---left
    //start time of neutral minus
    //  start time of ^ between 0.5 and 1.5 frames
    //current time minus start time of neutral should be used to limit how long the nerf applies for
    pivotdir direction = P_None;
    if(pivotZoneHist[0].zone == 0) {
        if(pivotZoneHist[1].zone == ZONE_L && (pivotZoneHist[2].zone == ZONE_R || pivotZoneHist[3].zone == ZONE_R)) {
            direction = P_Rightleft;
        } else if(pivotZoneHist[1].zone == ZONE_R && (pivotZoneHist[2].zone == ZONE_L || pivotZoneHist[3].zone == ZONE_L)) {
            direction = P_Leftright;
        }
    }
    uint16_t pivotLength = (pivotZoneHist[0].timestamp - pivotZoneHist[1].timestamp)*sampleSpacing;
    if(pivotLength < TIMELIMIT_HALFFRAME || pivotLength > TIMELIMIT_FRAME+TIMELIMIT_HALFFRAME) {
        //less than 50% chance it was a successful pivot
        direction = P_None;
    }
    //check for staleness
    if(pivotZoneHist[3].stale || pivotZoneHist[2].stale || pivotZoneHist[1].stale || pivotZoneHist[0].stale) {
        //if the previous movement was more than 15 frames earlier
        direction = P_None;
    }

    uint16_t pivotAge = (currentTime - pivotZoneHist[0].timestamp)*sampleSpacing;
    if(pivotAge > TIMELIMIT_PIVOTTILT) {
        direction = P_None;
    }

    //tap jump shutoff
    static uint8_t uptiltSamples = 0;
    if(prelimAY > ANALOG_DEAD_MAX) {
        uptiltSamples = min(uptiltSamples+1,254);
    } else {
        uptiltSamples = 0;
    }
    const uint16_t timeSinceNotUptilt = uptiltSamples*sampleSpacing;

    //actually apply the nerfs
    //debug c-stick output
    /*
    if(direction != P_None) {
        prelimCY = 255;
    }
    */
    //if it's a downtilt coordinate...
    bool pivotTilt = false;
    bool upTilt = false;
    if(direction != P_None && prelimAY < ANALOG_DEAD_MIN) {
        pivotTilt = true;
    }
    //if it's a uptilt coordinate...
    if(direction != P_None && prelimAY > ANALOG_DEAD_MAX) {
        pivotTilt = true;
        upTilt = true;
    }
    if(pivotTilt) {
        //If there's a pivot tilt, preserve the angle as best as possible but maximize the radius
        int8_t xCoord = prelimAX - ANALOG_STICK_NEUTRAL;
        int8_t yCoord = prelimAY - ANALOG_STICK_NEUTRAL;
        uint8_t xCoordAbs = abs(xCoord);
        uint8_t yCoordAbs = abs(yCoord);

        if(upTilt && xCoordAbs > yCoordAbs) {
            //Force all upward angles to a minimum of 45deg to prevent pivot uftilt and ensure tap jump
            prelimAX = ANALOG_STICK_NEUTRAL + (xCoord >= 0 ? 127 : -127);
            prelimAY = ANALOG_STICK_NEUTRAL + 127;
        } else {
            //Scale magnitude as close as we can get to 127 in increments of 0.5
            //Quick way to ensure we get above 80 magnitude with minimal rounding errors
            uint8_t stretchMult = (uint8_t)(127 * 2) / max(xCoordAbs, yCoordAbs);
            prelimAX = ANALOG_STICK_NEUTRAL + xCoord * stretchMult / 2;
            prelimAY = ANALOG_STICK_NEUTRAL + yCoord * stretchMult / 2;
        }
        if(upTilt && timeSinceNotUptilt > TIMELIMIT_TAPSHUTOFF) {
            if(direction == P_Leftright) {
                prelimAX = ANALOG_STICK_NEUTRAL - 30;
            } else if(direction == P_Rightleft) {
                prelimAX = ANALOG_STICK_NEUTRAL + 30;
            }
            prelimAY = ANALOG_STICK_NEUTRAL +30;
        }
    }

    //if it's a crouch to upward coordinate too quickly, make Y jump even if a tilt was desired
    static uint16_t timeSinceCrouch = 100;//must be < 65535 when multiplied by 500, the longest possible sampleSpacing, but bigger than the (timelimit/250)
    static bool downUpJumping;
    static uint16_t timeSinceJump = 100;

    //increment timeSinceCrouch unless you have crouched
    timeSinceCrouch = min(timeSinceCrouch+1, 100);
    if(prelimAY < ANALOG_CROUCH) {
        timeSinceCrouch = 0;
    }

    //increment timeSinceJump unless you want to trigger a jump
    timeSinceJump = min(timeSinceJump+1, 100);
    if(timeSinceCrouch*sampleSpacing < TIMELIMIT_DOWNUP && prelimAY > ANALOG_DEAD_MAX && prelimAY < ANALOG_TAPJUMP && !downUpJumping) {
        downUpJumping = true;
        timeSinceJump = 0;
    }

    if(timeSinceJump*sampleSpacing < JUMP_TIME && downUpJumping) {
        prelimAY = 255;
        timeSinceCrouch = 100;//prevent an extra duration jump if the jump ends before the lockout window
    } else {
        downUpJumping = false;
    }

    //if it's wank sdi (TODO) or diagonal tap SDI, lock out the cross axis
    const uint8_t sdi = isTapSDI(sdiZoneHist, currentIndexSDI, currentTime, sampleSpacing);
    static bool sdiIsNerfed = false;
    if(sdi & (BITS_SDI_TAP_DIAG | BITS_SDI_TAP_CRDG | BITS_SDI_WANK)){
        if(sdi & (ZONE_L | ZONE_R)) {
            //lock the cross axis
            prelimAY = ANALOG_STICK_NEUTRAL;
            //make sure future cross axis travel time begins at the origin
            aHistory[currentIndexA].y_end = prelimAY;
            //the following is disabled because the sdi threshold is actually 0.7
            //prevent the cardinal axis from dipping below the sdi threshold by preserving its coordinate
            //prelimAX = aHistory[currentIndexA].x_start;
            //make sure that future cardinal travel time begins where it was before
            //aHistory[currentIndexA].x_end = prelimAX;
            sdiIsNerfed = true;
        } else if(sdi & (ZONE_U | ZONE_D)) {
            //lock the cross axis
            prelimAX = ANALOG_STICK_NEUTRAL;
            //make sure future cross axis travel time begins at the origin
            aHistory[currentIndexA].x_end = prelimAX;
            //the following is disabled because the sdi threshold is actually 0.7
            //prevent the cardinal axis from dipping below the sdi threshold by preserving its coordinate
            //prelimAY = aHistory[currentIndexA].y_start;
            //make sure that future cardinal travel time begins where it was before
            //aHistory[currentIndexA].y_end = prelimAY;
            sdiIsNerfed = true;
        }//one or the other should occur
        //debug to see if SDI was detected
        /*
        if(sdi & BITS_SDI_TAP_CARD) {
            prelimCX = 200;
        }
        if(sdi & BITS_SDI_TAP_CRDG) {
            prelimCY = 200;
        }
        if(sdi & BITS_SDI_WANK) {
            prelimCX = 10;
        }
        */
    } else if(sdiIsNerfed) {
        aHistory[currentIndexA].x_end = aHistory[currentIndexA].x;
        aHistory[currentIndexA].y_end = aHistory[currentIndexA].y;
        if(prelimAX == ANALOG_STICK_NEUTRAL && prelimAY == ANALOG_STICK_NEUTRAL) {
            sdiIsNerfed = false;
        }
    }

    //==================================recording history======================================//

    //if we are in a new SDI zone, record the new zone
    if(sdiZoneHist[currentIndexSDI].zone != sdiZone(rawOutputIn.leftStickX, rawOutputIn.leftStickY)) {
        currentIndexSDI = (currentIndexSDI + 1) % HISTORYLEN;

        sdiZoneHist[currentIndexSDI].timestamp = currentTime;
        sdiZoneHist[currentIndexSDI].zone = sdiZone(rawOutputIn.leftStickX, rawOutputIn.leftStickY);
        sdiZoneHist[currentIndexSDI].stale = false;
    }
    for(int i = 0; i < HISTORYLEN; i++) {
        if(currentTime-sdiZoneHist[i].timestamp > 8*16*2) {//8 frames * 16 ms * max 2 samples
            sdiZoneHist[i].stale = true;
        }
    }

    //if we have a new coordinate, record the new info, the travel time'd locked out stick coordinate, and set travel time
    if(aHistory[currentIndexA].x != rawOutputIn.leftStickX || aHistory[currentIndexA].y != rawOutputIn.leftStickY) {
        oldA = false;
        doneTraveling = false;
        const uint8_t oldIndexA = currentIndexA;
        currentIndexA = (currentIndexA + 1) % HISTORYLEN;

        const uint8_t xIn = rawOutputIn.leftStickX;
        const uint8_t yIn = rawOutputIn.leftStickY;
        uint8_t xInRand = xIn;
        uint8_t yInRand = yIn;
        randomizeCoord(xInRand, yInRand, currentTime);

        aHistory[currentIndexA].timestamp = currentTime;
        aHistory[currentIndexA].x = xIn;
        aHistory[currentIndexA].y = yIn;
        aHistory[currentIndexA].x_end = xInRand;
        aHistory[currentIndexA].y_end = yInRand;
        aHistory[currentIndexA].x_start = prelimAX;
        aHistory[currentIndexA].y_start = prelimAY;

        uint8_t oldX = aHistory[oldIndexA].x;
        uint8_t oldY = aHistory[oldIndexA].y;
        if(prelimAX == oldX) {
            if(xInRand > oldX) {
                aHistory[currentIndexA].x_start++;
            }
            if(xInRand < oldX) {
                aHistory[currentIndexA].x_start--;
            }
        }
        if(prelimAY == oldY) {
            if(yInRand > oldY) {
                aHistory[currentIndexA].y_start++;
            }
            if(yInRand < oldY) {
                aHistory[currentIndexA].y_start--;
            }
        }

        uint8_t prelimTT = TRAVELTIME_EASY1;
        //if the destination is not an "easy" coordinate
        const uint8_t easiness = isEasy(xIn, yIn);
        if(easiness == 1) {
            prelimTT = TRAVELTIME_EASY1;
            delayType = T_Cubic;
        } else if(easiness == 2) {
            prelimTT = TRAVELTIME_EASY2;
            delayType = T_Cubic;
        } else if(easiness == 3) {
            prelimTT = TRAVELTIME_EASY3;
            delayType = T_Cubic;
        } else {
            prelimTT = TRAVELTIME_INTERNAL;
            delayType = T_Cubic;
        }
        //if cardinal tap SDI
        if(sdi & BITS_SDI_TAP_CARD) {
            prelimTT = max(prelimTT, TRAVELTIME_SLOW);
            delayType = T_Lin;
        }
        /*
        //if the destination is on the opposite side from the current prelim coord
        if((xIn < ANALOG_STICK_NEUTRAL && ANALOG_STICK_NEUTRAL < prelimAX) ||
           (yIn < ANALOG_STICK_NEUTRAL && ANALOG_STICK_NEUTRAL < prelimAY) ||
           (xIn > ANALOG_STICK_NEUTRAL && ANALOG_STICK_NEUTRAL > prelimAX) ||
           (yIn > ANALOG_STICK_NEUTRAL && ANALOG_STICK_NEUTRAL > prelimAY)) {
            prelimTT = max(prelimTT, TRAVELTIME_CROSS);
        }
        */
        aHistory[currentIndexA].tt = prelimTT;
    }

    //===============================applying the nerfed coords=================================//
    finalOutput.a               = rawOutputIn.a;
    finalOutput.b               = rawOutputIn.b;
    finalOutput.x               = rawOutputIn.x;
    finalOutput.y               = rawOutputIn.y;
    finalOutput.buttonL         = rawOutputIn.buttonL;
    finalOutput.buttonR         = rawOutputIn.buttonR;
    finalOutput.triggerLDigital = rawOutputIn.triggerLDigital;
    finalOutput.triggerRDigital = rawOutputIn.triggerRDigital;
    finalOutput.start           = rawOutputIn.start;
    finalOutput.select          = rawOutputIn.select;
    finalOutput.home            = rawOutputIn.home;
    finalOutput.dpadUp          = rawOutputIn.dpadUp;
    finalOutput.dpadDown        = rawOutputIn.dpadDown;
    finalOutput.dpadLeft        = rawOutputIn.dpadLeft;
    finalOutput.dpadRight       = rawOutputIn.dpadRight;
    finalOutput.leftStickClick  = rawOutputIn.leftStickClick;
    finalOutput.rightStickClick = rawOutputIn.rightStickClick;
    finalOutput.leftStickX      = prelimAX;
    finalOutput.leftStickY      = prelimAY;
    finalOutput.rightStickX     = prelimCX;
    finalOutput.rightStickY     = prelimCY;
    finalOutput.triggerLAnalog  = rawOutputIn.triggerLAnalog;
    finalOutput.triggerRAnalog  = rawOutputIn.triggerRAnalog;
}
