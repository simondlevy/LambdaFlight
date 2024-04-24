#pragma once

#include <stdio.h>
#include <stdint.h>

static const uint8_t N =
#ifdef _TEST
3
#else
4
#endif
;


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

static float dot(const float x[N], const float y[N]) 
{
    float d = 0;

    for (uint8_t k=0; k<N; k++) {
        d += x[k] * y[k];
    }

    return d;
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

// Matrix * Matrix
static void multiply(const float a[N][N], const float b[N][N], float c[N][N])
{
    for (uint8_t i=0; i<N; i++) {

        for (uint8_t j=0; j<N; j++) {

            c[i][j] = dot(a, b, i, j);
        }
    }
}

// Matrix * Vector
static void multiply(const float a[N][N], const float x[N], float y[N])
{
    for (uint8_t i=0; i<N; i++) {
        y[i] = 0;
        for (uint8_t j=0; j<N; j++) {
            y[i] += a[i][j] * x[j];
        }
    }
}

// Outer product
static void multiply(const float x[N], const float y[N], float a[N][N])
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t j=0; j<N; j++) {
            a[i][j] = x[i] * y[j];
        }
    }
}

#ifdef _TEST

static void show(const float a[N][N])
{
    for (uint8_t i=0; i<N; ++i) {

        for (uint8_t j=0; j<N; ++j) {
            printf("%3.0f ", (double)a[i][j]);
        }

        printf("\n");
    }
}

static void show(const float x[N])
{
    for (uint8_t i=0; i<N; ++i) {
        printf("%3.0f ", (double)x[i]);
    }
    printf("\n");
}
#endif
