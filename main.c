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
#include <sys/time.h>
#include <time.h>

#include "matd.h"

using namespace cv;

int64_t utime_now() // blacklist-ignore
{
    struct timeval tv;
    gettimeofday(&tv, NULL); // blacklist-ignore
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

int64_t utime_get_seconds(int64_t v)
{
    return v / 1000000;
}

int64_t utime_get_useconds(int64_t v)
{
    return v % 1000000;
}

uint64_t detector(char* filename)
{
    Mat A;

    A = imread(filename, 0);

    matd_t* B = matd_create_data(A.rows, A.cols, A.data);
    // matd_print(B, "%5d");
    int quad_size = A.cols / 9;

    int count = (quad_size * quad_size * 0.6);
    if (count == 0)
        count = 1;
    // printf("%d,%d:quad_size=%d, count=%d\n", A.rows, A.cols, quad_size, count);
    matd_t* x = matd_reduce(B, quad_size, 10, count);
    // matd_print(x, "%5d");
    // printf("\n");

    matd_t* C = matd_select(x, 2, 6, 2, 6);
    // matd_print(C, "%5d");
    // printf("\n");

    uint64_t v = matd_value(C);

    // printf("v=%llx\n", v);

    matd_destroy(B);
    matd_destroy(x);
    matd_destroy(C);

    return v;
}

int main(int argc, char** argv)
{
    struct quick_decode_entry entry;
    int64_t t3, t2, t1, t0;
    int i;

    t0 = utime_now();

    t1 = utime_now();
    apriltag_family_t* family = tag25h9_create();
    quick_decode_init(family, 2);
    t2 = utime_now();
    printf("decode init time  %8.3f ms\n", utime_get_useconds(t2 - t1) / 1000.0);

    for (i = 0; i < 10; i++) {
        t1 = utime_now();
        t3 = utime_now();
        uint64_t v = detector(argv[1]);
        t2 = utime_now();
        printf("decode image time %8.3f ms\n", utime_get_useconds(t2 - t1) / 1000.0);

        t1 = utime_now();
        quick_decode_codeword(family, v, &entry);
        t2 = utime_now();
        printf("codeword time     %8.3f ms\n", utime_get_useconds(t2 - t1) / 1000.0);

        printf("rcode=%llx, id=%u, hamming=%d, rotation=%d, time %8.3f ms\n",
            entry.rcode, entry.id, entry.hamming, entry.rotation, utime_get_useconds(t2 - t3) / 1000.0);
    }
    printf("all time          %8.3f ms\n", utime_get_useconds(t2 - t0) / 1000.0);
}