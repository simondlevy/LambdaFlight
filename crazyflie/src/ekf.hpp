#pragma once

#include <string.h>

class Ekf {

    protected:

        void init(
                const float diag[EKF_N],
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

            memset(&_p, 0, sizeof(_p));

            for (uint8_t i=0; i<EKF_N; ++i) {
                set(_p, i, i, diag[i]);
            }

            memset(&_x, 0, sizeof(_x));
        }

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

        void scalarUpdate(
                const vector_t & h, 
                const float error, 
                const float stdMeasNoise)
        {

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

            // ====== COVARIANCE UPDATE ======

            matrix_t GH = {};
            multiply(g, h, GH); // KH

            for (int i=0; i<EKF_N; i++) { 
                set(GH, i, i, get(GH, i, i) - 1);
            } // KH - I

            matrix_t GHt = {};
            transpose(GH, GHt);      // (KH - I)'
            matrix_t GHIP = {};
            multiply(GH, _p, GHIP);  // (KH - I)*P
            multiply(GHIP, GHt, _p); // (KH - I)*P*(KH - I)'

            // Add the measurement variance 
            for (int i=0; i<EKF_N; i++) {
                for (int j=0; j<EKF_N; j++) {
                    _p.dat[i][j] += j < i ? 0 : r * get(g, i) * get(g, j);
                    set(_p, i, j, get(_p, i, j));
                }
            }

            updateCovarianceMatrix();
        }


        void updateCovarianceMatrix(void)
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

    private:

        float _min_covariance;
        float _max_covariance;

};
