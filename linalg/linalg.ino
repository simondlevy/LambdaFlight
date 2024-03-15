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

static void multiply(const float x[N], const float y[N], float a[N][N])
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t j=0; j<N; j++) {
            a[i][j] = x[i] * y[j];
        }
    }
}

void setup() 
{
    Serial.begin(115200);
}

void loop() 
{
    float K[N] = {1, 2, 3};

    float H[N] = {4, 5, 6};

    arm_matrix_instance_f32 Hm = {1, N, H};
    arm_matrix_instance_f32 Km = {N, 1, K};
    float tmpNN1d[N*N] = {};
    arm_matrix_instance_f32 KH = {N, N, tmpNN1d };

    mat_mult(&Km, &Hm, &KH); // KH

    Serial.printf("%f %f %f\n%f %f %f\n%f %f %f\n\n", 
            tmpNN1d[0], tmpNN1d[1], tmpNN1d[2],
            tmpNN1d[3], tmpNN1d[4], tmpNN1d[5],
            tmpNN1d[6], tmpNN1d[7], tmpNN1d[8]);

    delay(100);

    float a[N][N] = {};

    multiply(K, H, a);

    Serial.printf("%f %f %f\n%f %f %f\n%f %f %f\n\n\n", 
            a[0][0], a[0][1], a[0][2],
            a[1][0], a[1][1], a[1][2],
            a[2][0], a[2][1], a[2][2]);

    multiply(K, H, a);

    delay(500);

}
