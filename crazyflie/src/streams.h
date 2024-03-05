#include <datatypes.h>

extern demands_t openLoopDemands;
extern vehicleState_t vehicleState;
extern bool inHoverMode;
extern bool resetPids;

extern Axis3f stream_gyro;

extern copilotMode_e copilotMode;
extern uint32_t kalmanNowMsec;
extern uint32_t kalmanNextPredictionMsec;
extern bool kalmanIsFlying;
extern measurement_t kalmanMeasurement;
