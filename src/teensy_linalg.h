#pragma once

#include <stdio.h>
#include <stdint.h>

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

static float dot(const float a[N][N], const float b[N][N], 
        const uint8_t i, const uint8_t j)
{
    float d = 0;

    for (uint8_t k=0; k<N; k++) {
        d += a[i][k] * b[k][j];
    }

    return d;
}

/*
// Matrix + Matrix
static void add(const float a[N][N], const float b[N][N], float c[N][N])
{
    for (uint8_t i=0; i<N; i++) {

        for (uint8_t j=0; j<N; j++) {

            c[i][j] = a[i][j] + b[i][j];
        }
    }
}
*/

// Matrix * Matrix
static void multiply(const float a[N][N], const float b[N][N], float c[N][N], 
        const bool doit=true)
{
    for (uint8_t i=0; i<N; i++) {

        for (uint8_t j=0; j<N; j++) {

            c[i][j] = doit ? dot(a, b, i, j) : c[i][j];
        }
    }
}
