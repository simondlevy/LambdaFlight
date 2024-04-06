#include <stdio.h>

#include "linalg.h"

int main(int argc, char ** argv)
{
    const float a[N][N] = { {2, 3, 5},
                            {7, 11, 13},
                            {17, 19, 23} };

    show(a);

    return 0;
}
