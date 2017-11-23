#include <stdio.h>

#include "april.h"
#include "tag25h9.h"

#include <assert.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "matd.h"

using namespace cv;
static uint8_t randi()
{
    uint8_t v = random();

    return v;
}

uint64_t detector(char* filename)
{
    Mat A;

    A = imread(filename, 0);

    printf("%d,%d\n", A.rows, A.cols);
    matd_t* B = matd_create_data(A.rows, A.cols, A.data);
    // matd_print(B, "%5d");
    int quad_size = A.cols / 9;
    printf("%d\n", A.cols / 9);
    int count = (quad_size * quad_size * 0.8);
    if (count == 0)
        count = 1;
    matd_t* x = matd_reduce(B, quad_size, 10, count);
    matd_print(x, "%5d");

    printf("\n");
    matd_t* C = matd_select(x, 2, 6, 2, 6);
    matd_print(C, "%5d");
    printf("\n");
    uint64_t v = matd_value(C);

    printf("v=%llx\n", v);

    matd_destroy(B);
    matd_destroy(x);
    matd_destroy(C);

    return v;
}

int main(int argc, char** argv)
{
    uint64_t v = detector(argv[1]);
    struct quick_decode_entry entry;
    apriltag_family_t* family = tag25h9_create();
    quick_decode_init(family, 3);
    quick_decode_codeword(family, v, &entry);
    printf("rcode=%llx, id=%u, hamming=%d, rotation=%d\n",
        entry.rcode, entry.id, entry.hamming, entry.rotation);
}