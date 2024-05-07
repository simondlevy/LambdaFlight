#pragma once

class Ekf {

    public:
        
        void init(const float diag[EKF_N], const uint32_t now_msec)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {

                x[i] = 0;

                for (uint8_t j=0; j<EKF_N; ++j) {

                    p[i][j] = i == j ? diag[i] : 0;
                }
            }

            _lastProcessNoiseUpdateMsec = now_msec;

            _lastPredictionMsec = now_msec;

            _isUpdated = false;
        }

        void predict(const uint32_t now_msec)
        {
            const auto isDtPositive = 
                (now_msec - _lastProcessNoiseUpdateMsec) / 1000.0f;

            if (isDtPositive) {
                _lastProcessNoiseUpdateMsec = now_msec;
            }

            _lastPredictionMsec = now_msec;

            _isUpdated = true;
        }

        void finalize(void)
        {
            _isUpdated = false;
        }

    private:

        float x[EKF_N];

        float p[EKF_N][EKF_N];

        uint32_t _lastProcessNoiseUpdateMsec;
        uint32_t _lastPredictionMsec;
        bool _isUpdated;
};
