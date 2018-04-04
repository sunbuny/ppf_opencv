#pragma once


#include <stdio.h>
#include <stdlib.h>


namespace ppf_match_3d {

//! @addtogroup surface_matching
//! @{

    typedef uint KeyType;

    typedef struct hashnode_i {
        KeyType key;
        void *data;
        struct hashnode_i *next;
    } hashnode_i;

    typedef struct HSHTBL_i {
        size_t size;
        struct hashnode_i **nodes;

        size_t (*hashfunc)(uint);
    } hashtable_int;


/** @brief Round up to the next highest power of 2

from http://www-graphics.stanford.edu/~seander/bithacks.html
*/
    inline static uint next_power_of_two(uint value) {

        --value;
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        ++value;

        return value;
    }

    hashtable_int *hashtableCreate(size_t size, size_t (*hashfunc)(uint));

    void hashtableDestroy(hashtable_int *hashtbl);

    int hashtableInsert(hashtable_int *hashtbl, KeyType key, void *data);

    int hashtableInsertHashed(hashtable_int *hashtbl, KeyType key, void *data);

    int hashtableRemove(hashtable_int *hashtbl, KeyType key);

    void *hashtableGet(hashtable_int *hashtbl, KeyType key);

    hashnode_i *hashtableGetBucketHashed(hashtable_int *hashtbl, KeyType key);

    int hashtableResize(hashtable_int *hashtbl, size_t size);

    hashtable_int *hashtable_int_clone(hashtable_int *hashtbl);

    hashtable_int *hashtableRead(FILE *f);

    int hashtableWrite(const hashtable_int *hashtbl, const size_t dataSize, FILE *f);

    void hashtablePrint(hashtable_int *hashtbl);

//! @}

} // namespace ppf_match_3d

