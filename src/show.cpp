#include <stdio.h>

#include "linalg.h"

int main(int argc, char ** argv)
{
    const float a[N][N] = { {2, 3, 5},
                            {7, 11, 13},
                            {17, 19, 23} };


    //float at[N][N] = {};
    //transpose(a, at);
    // show(at);

    const float x[N] = {29, 31, 37};

    //float y[N] = {};
    //multiply(a, x, y);
    //show(y);

    printf("%f\n", dot(x, x));

    return 0;
}
