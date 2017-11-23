#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "april.h"

static inline int imin(int a, int b)
{
    return (a < b) ? a : b;
}

static inline int imax(int a, int b)
{
    return (a > b) ? a : b;
}

/** if the bits in w were arranged in a d*d grid and that grid was
 * rotated, what would the new bits in w be?
 * The bits are organized like this (for d = 3):
 *
 *  8 7 6       2 5 8      0 1 2
 *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
 *  2 1 0       0 3 6      6 7 8
 **/
static uint64_t rotate90(uint64_t w, uint32_t d)
{
    uint64_t wr = 0;

    for (int32_t r = d - 1; r >= 0; r--) {
        for (int32_t c = 0; c < d; c++) {
            int32_t b = r + d * c;

            wr = wr << 1;

            if ((w & (((uint64_t)1) << b)) != 0)
                wr |= 1;
        }
    }

    return wr;
}

void quick_decode_add(struct quick_decode* qd, uint64_t code, int id, int hamming)
{
    uint32_t bucket = code % qd->nentries;

    while (qd->entries[bucket].rcode != UINT64_MAX) {
        bucket = (bucket + 1) % qd->nentries;
    }

    qd->entries[bucket].rcode = code;
    qd->entries[bucket].id = id;
    qd->entries[bucket].hamming = hamming;
}

void quick_decode_uninit(apriltag_family_t* fam)
{
    if (!fam->impl)
        return;

    struct quick_decode* qd = (struct quick_decode*)fam->impl;
    free(qd->entries);
    free(qd);
    fam->impl = NULL;
}

void quick_decode_init(apriltag_family_t* family, int maxhamming)
{
    assert(family->impl == NULL);
    assert(family->ncodes < 65535);

    struct quick_decode* qd = (struct quick_decode*)calloc(1, sizeof(struct quick_decode));
    int capacity = family->ncodes;

    int nbits = family->d * family->d;

    if (maxhamming >= 1)
        capacity += family->ncodes * nbits;

    if (maxhamming >= 2)
        capacity += family->ncodes * nbits * (nbits - 1);

    if (maxhamming >= 3)
        capacity += family->ncodes * nbits * (nbits - 1) * (nbits - 2);

    qd->nentries = capacity * 3;

    //    printf("capacity %d, size: %.0f kB\n",
    //           capacity, qd->nentries * sizeof(struct quick_decode_entry) / 1024.0);

    qd->entries = (struct quick_decode_entry*)calloc(qd->nentries, sizeof(struct quick_decode_entry));
    if (qd->entries == NULL) {
        printf("apriltag.c: failed to allocate hamming decode table. Reduce max hamming size.\n");
        exit(-1);
    }

    for (int i = 0; i < qd->nentries; i++)
        qd->entries[i].rcode = UINT64_MAX;

    for (int i = 0; i < family->ncodes; i++) {
        uint64_t code = family->codes[i];

        // add exact code (hamming = 0)
        quick_decode_add(qd, code, i, 0);

        if (maxhamming >= 1) {
            // add hamming 1
            for (int j = 0; j < nbits; j++)
                quick_decode_add(qd, code ^ (1L << j), i, 1);
        }

        if (maxhamming >= 2) {
            // add hamming 2
            for (int j = 0; j < nbits; j++)
                for (int k = 0; k < j; k++)
                    quick_decode_add(qd, code ^ (1L << j) ^ (1L << k), i, 2);
        }

        if (maxhamming >= 3) {
            // add hamming 3
            for (int j = 0; j < nbits; j++)
                for (int k = 0; k < j; k++)
                    for (int m = 0; m < k; m++)
                        quick_decode_add(qd, code ^ (1L << j) ^ (1L << k) ^ (1L << m), i, 3);
        }

        if (maxhamming > 3) {
            printf("apriltag.c: maxhamming beyond 3 not supported\n");
        }
    }

    family->impl = qd;

    if (0) {
        int longest_run = 0;
        int run = 0;
        int run_sum = 0;
        int run_count = 0;

        // This accounting code doesn't check the last possible run that
        // occurs at the wrap-around. That's pretty insignificant.
        for (int i = 0; i < qd->nentries; i++) {
            if (qd->entries[i].rcode == UINT64_MAX) {
                if (run > 0) {
                    run_sum += run;
                    run_count++;
                }
                run = 0;
            } else {
                run++;
                longest_run = imax(longest_run, run);
            }
        }

        printf("quick decode: longest run: %d, average run %.3f\n", longest_run, 1.0 * run_sum / run_count);
    }
}

// returns an entry with hamming set to 255 if no decode was found.
void quick_decode_codeword(apriltag_family_t* tf, uint64_t rcode, struct quick_decode_entry* entry)
{
    struct quick_decode* qd = (struct quick_decode*)tf->impl;

    for (int ridx = 0; ridx < 4; ridx++) {

        for (int bucket = rcode % qd->nentries;
             qd->entries[bucket].rcode != UINT64_MAX;
             bucket = (bucket + 1) % qd->nentries) {

            if (qd->entries[bucket].rcode == rcode) {
                *entry = qd->entries[bucket];
                entry->rotation = ridx;
                return;
            }
        }

        rcode = rotate90(rcode, tf->d);
    }

    entry->rcode = 0;
    entry->id = 65535;
    entry->hamming = 255;
    entry->rotation = 0;
}