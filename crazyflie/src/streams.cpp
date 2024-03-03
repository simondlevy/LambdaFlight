#include <streams.h>

demands_t openLoopDemands;
vehicleState_t vehicleState;
bool inHoverMode;
bool resetPids;

copilotMode_e copilotMode;
uint32_t kalmanNowMsec;
uint32_t kalmanNextPredictionMsec;
bool kalmanIsFlying;
measurement_t kalmanMeasurement;
