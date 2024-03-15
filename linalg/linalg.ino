#include <arm_math.h>

#define N 3

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, 
        arm_matrix_instance_f32 * pDst) 
{
  arm_mat_trans_f32(pSrc, pDst);
}

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, 
        const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) 
{
  arm_mat_mult_f32(pSrcA, pSrcB, pDst);
}

/*
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
*/

void setup() 
{
    Serial.begin(115200);
}

void loop() 
{
    float h[N] = {10, 11, 12};

    float P[N][N] = { {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };

    arm_matrix_instance_f32 Pm;
    Pm.numRows = N;
    Pm.numCols = N;
    Pm.pData = (float*)P;

    arm_matrix_instance_f32 Hm = {1, 3, h};

    float HTd[N];
    arm_matrix_instance_f32 HTm = {N, 1, HTd};

    float PHTd[N];
    arm_matrix_instance_f32 PHTm = {N, 1, PHTd};

    mat_trans(&Hm, &HTm);
    mat_mult(&Pm, &HTm, &PHTm);

    Serial.printf("%f %f %f\n", PHTd[0], PHTd[1], PHTd[2]);

    delay(500);

}
