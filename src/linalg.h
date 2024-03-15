#pragma once

static const uint8_t N = 7;

static void transpose(const float a[N][N], float at[N][N])
{
    for (uint8_t i=0; i<N; ++i) {
        for (uint8_t j=0; j<N; ++j) {
            auto tmp = a[i][j];
            at[i][j] = a[j][i];
            at[j][i] = tmp;
        }
    }
}

static void multiply(const float a[N][N], const float b[N][N], float c[N][N])
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t j=0; j<N; j++) {
            c[i][j] = 0;
            for (uint8_t k=0; k<N; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

static void multiply(const float a[N][N], const float x[N], float y[N])
{
    for (uint8_t i=0; i<N; i++) {
        y[i] = 0;
        for (uint8_t j=0; j<N; j++) {
            y[i] += a[i][j] * x[j];
        }
    }
}

static void multiply(const float x[N], const float y[N], float a[N][N])
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t j=0; j<N; j++) {
            a[i][j] = x[i] * y[j];
        }
    }
}
