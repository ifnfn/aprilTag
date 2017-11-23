/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "matd.h"

#define sq(x) ((x) * (x))
#define max(a, b) (a) > (b) ? (a) : (b)

static matd_t* matd_create_scalar(TYPE v)
{
    matd_t* m = (matd_t*)calloc(1, sizeof(matd_t) + sizeof(TYPE));
    m->nrows = 0;
    m->ncols = 0;
    m->data[0] = v;

    return m;
}

static inline int matd_is_scalar(const matd_t* a)
{
    assert(a != NULL);
    return a->ncols == 0 || a->nrows == 0;
}

static inline TYPE matd_get_scalar(const matd_t* m)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    return (m->data[0]);
}

static inline void matd_put_scalar(matd_t* m, TYPE value)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    m->data[0] = value;
}

matd_t* matd_create(int rows, int cols)
{
    assert(rows >= 0);
    assert(cols >= 0);

    if (rows == 0 || cols == 0)
        return matd_create_scalar(0);

    matd_t* m = (matd_t*)calloc(1, sizeof(matd_t) + (rows * cols * sizeof(TYPE)));
    m->nrows = rows;
    m->ncols = cols;

    return m;
}

matd_t* matd_create_data(int rows, int cols, const uint8_t* data)
{
    if (rows == 0 || cols == 0)
        return matd_create_scalar(data[0]);

    matd_t* m = matd_create(rows, cols);
    for (int i = 0; i < rows * cols; i++)
        m->data[i] = data[i];

    return m;
}

matd_t* matd_identity(int dim)
{
    if (dim == 0)
        return matd_create_scalar(1);

    matd_t* m = matd_create(dim, dim);
    for (int i = 0; i < dim; i++)
        MATD_EL(m, i, i) = 1;

    return m;
}

// row and col are zero-based
TYPE matd_get(const matd_t* m, int row, int col)
{
    assert(m != NULL);
    assert(!matd_is_scalar(m));
    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    return MATD_EL(m, row, col);
}

// row and col are zero-based
void matd_put(matd_t* m, int row, int col, TYPE value)
{
    assert(m != NULL);

    if (matd_is_scalar(m)) {
        matd_put_scalar(m, value);
        return;
    }

    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    MATD_EL(m, row, col) = value;
}

matd_t* matd_copy(const matd_t* m)
{
    assert(m != NULL);

    matd_t* x = matd_create(m->nrows, m->ncols);
    if (matd_is_scalar(m))
        x->data[0] = m->data[0];
    else
        memcpy(x->data, m->data, sizeof(TYPE) * m->ncols * m->nrows);

    return x;
}

matd_t* matd_select(const matd_t* a, int r0, int r1, int c0, int c1)
{
    assert(a != NULL);

    assert(r0 >= 0 && r0 < a->nrows);
    assert(c0 >= 0 && c0 < a->ncols);

    int nrows = r1 - r0 + 1;
    int ncols = c1 - c0 + 1;

    matd_t* r = matd_create(nrows, ncols);

    for (int row = r0; row <= r1; row++)
        for (int col = c0; col <= c1; col++)
            MATD_EL(r, row - r0, col - c0) = MATD_EL(a, row, col);

    return r;
}

void matd_print(const matd_t* m, const char* fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int i = 0; i < m->nrows; i++) {
            for (int j = 0; j < m->ncols; j++) {
                printf(fmt, MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_print_transpose(const matd_t* m, const char* fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int j = 0; j < m->ncols; j++) {
            for (int i = 0; i < m->nrows; i++) {
                printf(fmt, MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_destroy(matd_t* m)
{
    if (!m)
        return;

    assert(m != NULL);
    free(m);
}

matd_t* matd_multiply(const matd_t* a, const matd_t* b)
{
    assert(a != NULL);
    assert(b != NULL);

    if (matd_is_scalar(a))
        return matd_scale(b, a->data[0]);
    if (matd_is_scalar(b))
        return matd_scale(a, b->data[0]);

    assert(a->ncols == b->nrows);
    matd_t* m = matd_create(a->nrows, b->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            TYPE acc = 0;
            for (int k = 0; k < a->ncols; k++) {
                acc += MATD_EL(a, i, k) * MATD_EL(b, k, j);
            }
            MATD_EL(m, i, j) = acc;
        }
    }

    return m;
}

matd_t* matd_scale(const matd_t* a, TYPE s)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] * s);

    matd_t* m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = s * MATD_EL(a, i, j);
        }
    }

    return m;
}

void matd_scale_inplace(matd_t* a, TYPE s)
{
    assert(a != NULL);

    if (matd_is_scalar(a)) {
        a->data[0] *= s;
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) *= s;
        }
    }
}

matd_t* matd_add(const matd_t* a, const matd_t* b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] + b->data[0]);

    matd_t* m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) + MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_add_inplace(matd_t* a, const matd_t* b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] += b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) += MATD_EL(b, i, j);
        }
    }
}

matd_t* matd_subtract(const matd_t* a, const matd_t* b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] - b->data[0]);

    matd_t* m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) - MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_subtract_inplace(matd_t* a, const matd_t* b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] -= b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) -= MATD_EL(b, i, j);
        }
    }
}

matd_t* matd_transpose(const matd_t* a)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0]);

    matd_t* m = matd_create(a->ncols, a->nrows);

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(m, j, i) = MATD_EL(a, i, j);
        }
    }
    return m;
}

TYPE matd_max(matd_t* m)
{
    TYPE d = 0;
    for (int x = 0; x < m->nrows; x++) {
        for (int y = 0; y < m->ncols; y++) {
            if (MATD_EL(m, x, y) > d)
                d = MATD_EL(m, x, y);
        }
    }

    return d;
}

int matd_nonzero(matd_t* m)
{
    int count = 0;
    for (int x = 0; x < m->nrows; x++) {
        for (int y = 0; y < m->ncols; y++) {
            if (MATD_EL(m, x, y) != 0)
                count++;
        }
    }

    return count;
}

matd_t* matd_reduce(matd_t* m, int dim, int thresh, int num)
{
    int new_r = m->nrows / dim;
    int new_c = m->ncols / dim;

    matd_t* t = matd_create(new_r, new_c);
    for (int x = 0; x < m->nrows; x++) {
        for (int y = 0; y < m->ncols; y++) {
            int r = x / dim;
            int c = y / dim;
            // printf("%d = %d\n", MATD_EL(m, x, y), thresh);
            if (r < new_r && c < new_c) {
                // printf("r=%d,c=%d\n", r, c);
                if (MATD_EL(m, x, y) >= thresh)
                    MATD_EL(t, r, c) += 1;
            }
        }
    }

    for (int x = 0; x < t->nrows; x++) {
        for (int y = 0; y < t->ncols; y++) {
            // printf("%d = %d\n", MATD_EL(t, x, y), num);
            if (MATD_EL(t, x, y) >= num) {
                MATD_EL(t, x, y) = 1;
            } else {
                MATD_EL(t, x, y) = 0;
            }
        }
    }

    return t;
}

uint64_t matd_value(matd_t* t)
{
    uint64_t v = 0;
    for (int x = 0; x < t->nrows; x++) {
        for (int y = 0; y < t->ncols; y++) {
            v = v << 1;
            if (MATD_EL(t, x, y) == 1) {
                v += 1;
            }
        }
    }
    return v;
}

uint64_t matd_reduce_value(matd_t* m, int dim, int thresh, int num)
{
    uint64_t v = 0;

    int new_r = m->nrows / dim;
    int new_c = m->ncols / dim;
    matd_t* t = matd_create(new_r, new_c);
    for (int x = 0; x < m->nrows; x++) {
        for (int y = 0; y < m->ncols; y++) {
            int r = x / dim;
            int c = y / dim;
            // printf("%d = %d\n", MATD_EL(m, x, y), thresh);
            if (MATD_EL(m, x, y) >= thresh)
                MATD_EL(t, r, c) += 1;
        }
    }

    for (int x = 0; x < t->nrows; x++) {
        for (int y = 0; y < t->ncols; y++) {
            v = v << 1;
            if (MATD_EL(t, x, y) >= num) {
                v += 1;
            }
        }
    }

    return v;
}
