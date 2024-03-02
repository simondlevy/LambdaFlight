#include <streams.h>

demands_t openLoopDemands;
vehicleState_t vehicleState;
bool inHoverMode;
bool resetPids;

kalmanMode_e kalmanMode;
uint32_t kalmanNowMsec;
uint32_t kalmanNextPredictionMsec;
bool kalmanIsFlying;
