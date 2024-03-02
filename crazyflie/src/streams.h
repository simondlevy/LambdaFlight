#include <datatypes.h>

extern demands_t openLoopDemands;
extern vehicleState_t vehicleState;
extern bool inHoverMode;
extern bool resetPids;

extern kalmanMode_e kalmanMode;
extern uint32_t kalmanNowMsec;
extern uint32_t kalmanNextPredictionMsec;
extern bool kalmanIsFlying;
extern measurement_t kalmanMeasurement;
