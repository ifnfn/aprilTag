#ifndef APRIL_H
#define APRIL_H

#include <stdint.h>
#include <stdlib.h>

typedef struct apriltag_family {
    // How many codes are there in this tag family?
    uint32_t ncodes;

    // The codes in the family.
    uint64_t* codes;

    // how wide (in bit-sizes) is the black border? (usually 1)
    uint32_t black_border;

    // how many bits tall and wide is it? (e.g. 36bit tag ==> 6)
    uint32_t d;

    // minimum hamming distance between any two codes. (e.g. 36h11 => 11)
    uint32_t h;

    // a human-readable name, e.g., "tag36h11"
    char* name;

    // some detector implementations may preprocess codes in order to
    // accelerate decoding.  They put their data here. (Do not use the
    // same apriltag_family instance in more than one implementation)
    void* impl;
} apriltag_family_t;

struct quick_decode_entry {
    uint64_t rcode; // the queried code
    uint16_t id; // the tag ID (a small integer)
    uint8_t hamming; // how many errors corrected?
    uint8_t rotation; // number of rotations [0, 3]
};

struct quick_decode {
    int nentries;
    struct quick_decode_entry* entries;
};

void quick_decode_init(apriltag_family_t* family, int maxhamming);
void quick_decode_codeword(apriltag_family_t* tf, uint64_t rcode, struct quick_decode_entry* entry);

#endif
