#pragma once

#include <string.h>

#define EKF_N 7


class Ekf {

    private:

        typedef struct {

            float dat[EKF_N];

        } vector_t;

        typedef struct {

            float dat[EKF_N][EKF_N];

        } matrix_t;

        matrix_t _p;

        vector_t _x;

        bool _isUpdated;

        uint32_t _lastPredictionMsec;

        uint32_t _lastProcessNoiseUpdateMsec;

        uint32_t _predictionIntervalMsec;

        void multiplyCovariance(const matrix_t & a)
        {
            matrix_t  at = {};
            transpose(a, at);  
            matrix_t ap = {};
            multiply(a, _p, ap);
            multiply(ap, at, _p);

        }

        void cleanupCovariance(void)
        {
            // Enforce symmetry of the covariance matrix, and ensure the
            // values stay bounded
            for (int i=0; i<EKF_N; i++) {

                for (int j=i; j<EKF_N; j++) {

                    const auto pval = (_p.dat[i][j] + _p.dat[j][i]) / 2;

                    _p.dat[i][j] = _p.dat[j][i] = 
                        pval > _max_covariance ?  _max_covariance :
                        (i==j && pval < _min_covariance) ?  _min_covariance :
                        pval;
                }
            }
        }

        static void makevec(const float dat[EKF_N], vector_t & x)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {
                x.dat[i] = dat[i];
            }
        }

        static void makemat(const float dat[EKF_N][EKF_N], matrix_t & a)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {
                for (uint8_t j=0; j<EKF_N; ++j) {
                    a.dat[i][j] = dat[i][j];
                }
            }
        }

        static void transpose(const matrix_t & a, matrix_t & at)
        {
            for (uint8_t i=0; i<EKF_N; ++i) {
                for (uint8_t j=0; j<EKF_N; ++j) {
                    auto tmp = a.dat[i][j];
                    at.dat[i][j] = a.dat[j][i];
                    at.dat[j][i] = tmp;
                }
            }
        }

        static float dot(const vector_t & x, const vector_t & y) 
        {
            float d = 0;

            for (uint8_t k=0; k<EKF_N; k++) {
                d += x.dat[k] * y.dat[k];
            }

            return d;
        }

        static float get(const matrix_t & a, const uint8_t i, const uint8_t j)
        {
            return a.dat[i][j];
        }

        static float get(const vector_t & x, const uint8_t i)
        {
            return x.dat[i];
        }

        static void set(vector_t & x, const uint8_t i, const float val)
        {
            x.dat[i] = val;
        }

        static void set(matrix_t & a, const uint8_t i, const uint8_t j, const float val)
        {
            a.dat[i][j] = val;
        }

        static float dot(
                const matrix_t & a, 
                const matrix_t & b, 
                const uint8_t i, 
                const uint8_t j)
        {
            float d = 0;

            for (uint8_t k=0; k<EKF_N; k++) {
                d += a.dat[i][k] * b.dat[k][j];
            }

            return d;
        }

        // Matrix * Matrix
        static void multiply( const matrix_t a, const matrix_t b, matrix_t & c)
        {
            for (uint8_t i=0; i<EKF_N; i++) {

                for (uint8_t j=0; j<EKF_N; j++) {

                    c.dat[i][j] = dot(a, b, i, j);
                }
            }
        }

        // Matrix * Vector
        static void multiply(const matrix_t & a, const vector_t & x, vector_t & y)
        {
            for (uint8_t i=0; i<EKF_N; i++) {
                y.dat[i] = 0;
                for (uint8_t j=0; j<EKF_N; j++) {
                    y.dat[i] += a.dat[i][j] * x.dat[j];
                }
            }
        }

        // Outer product
        static void multiply(const vector_t & x, const vector_t & y, matrix_t & a)
        {
            for (uint8_t i=0; i<EKF_N; i++) {
                for (uint8_t j=0; j<EKF_N; j++) {
                    a.dat[i][j] = x.dat[i] * y.dat[j];
                }
            }
        }

        float _min_covariance;
        float _max_covariance;

    protected:

        static void do_init(float diag[EKF_N]);

        static void get_prediction(
                const float dt,
                const bool didAddProcessNoise,
                const float xold[EKF_N],
                float xnew[EKF_N],
                float F[EKF_N][EKF_N]);

        static bool did_finalize(float x[EKF_N], float A[EKF_N][EKF_N]);

    public:

        void initialize(
                const uint32_t nowMsec,
                const uint32_t predictionIntervalMsec,
                const float min_covariance, 
                const float max_covariance)
        {
            _predictionIntervalMsec = predictionIntervalMsec;

            _lastProcessNoiseUpdateMsec = nowMsec;
            _lastPredictionMsec = nowMsec;
            _isUpdated = false;

            _min_covariance = min_covariance;
            _max_covariance = max_covariance;

            float diag[EKF_N] = {};

            do_init(diag);

            for (uint8_t i=0; i<EKF_N; ++i) {

                for (uint8_t j=0; j<EKF_N; ++j) {

                    set(_p, i, j, i==j ? diag[i] : 0);
                }

                set(_x, i, 0);
            }

        }

        void predict(const uint32_t nowMsec)
        {
            static uint32_t _nextPredictionMsec;

            _nextPredictionMsec = nowMsec > _nextPredictionMsec ?
                nowMsec + _predictionIntervalMsec :
                _nextPredictionMsec;

            if (nowMsec >= _nextPredictionMsec) {

                _isUpdated = true;

                float xnew[EKF_N] = {};

                for (uint8_t i=0; i<EKF_N; ++i) {
                    xnew[i] = get(_x, i);
                }

                float Fdat[EKF_N][EKF_N] = {};

                const auto shouldAddProcessNoise = 
                    nowMsec - _lastProcessNoiseUpdateMsec > 0;

                const float dt = (nowMsec - _lastPredictionMsec) / 1000.0f;

                get_prediction(dt, shouldAddProcessNoise, _x.dat, xnew, Fdat);

                matrix_t F = {};
                makemat(Fdat, F);

                multiplyCovariance(F);

                cleanupCovariance();

                _lastPredictionMsec = nowMsec;

                if (shouldAddProcessNoise) {

                    _lastProcessNoiseUpdateMsec = nowMsec;

                    for (uint8_t i=0; i<EKF_N; ++i) {
                        set(_x, i, xnew[i]);
                    }

                }
            }
        }

        void update(
                const float hdat[EKF_N], 
                const float error, 
                const float stdMeasNoise)
        {
            vector_t h = {};
            makevec(hdat, h);

            // ====== INNOVATION COVARIANCE ======
            vector_t ph = {};
            multiply(_p, h, ph);
            const auto r = stdMeasNoise * stdMeasNoise;
            const auto hphr = r + dot(h, ph); // HPH' + R

            // Compute the Kalman gain as a column vector
            vector_t g = {};
            for (uint8_t i=0; i<EKF_N; ++i) {
                set(g, i, get(ph, i) / hphr);
            }

            // Perform the state update
            for (uint8_t i=0; i<EKF_N; ++i) {
                set(_x, i, get(_x, i) + get(g, i) * error);
            }

            matrix_t GH = {};
            multiply(g, h, GH); 

            for (int i=0; i<EKF_N; i++) { 
                set(GH, i, i, get(GH, i, i) - 1);
            }

            multiplyCovariance(GH);

            // Add the measurement variance 
            for (int i=0; i<EKF_N; i++) {
                for (int j=0; j<EKF_N; j++) {
                    _p.dat[i][j] += j < i ? 0 : r * get(g, i) * get(g, j);
                    set(_p, i, j, get(_p, i, j));
                }
            }

            cleanupCovariance();

            _isUpdated = true;
        }

        void finalize(const uint32_t nowMsec)
        {
            if (_isUpdated) {

                matrix_t  A = {};

                // Move attitude error into attitude if any of the angle errors are
                // large enough
                if (did_finalize(_x.dat, A.dat)) {

                    multiplyCovariance(A);

                    cleanupCovariance();
                }

                _isUpdated = false;
            }
        }

        float * getState(void)
        {
            static float x[EKF_N];

            for (uint8_t i=0; i<EKF_N; ++i) {
                x[i] = get(_x, i);
            }

            return x;
        }
};
