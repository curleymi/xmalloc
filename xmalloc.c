/*
 *  Michael Curley
 *  cs3650
 *  ch02
 *
 *  notes:
 *   - bucket style allocator
 *
 *   - all pointers returned to caller are preceeded by a uin8_t flag
 *   - flag defs:
 *      - 0x00 - ARENA_NUM: bucket flag, indicates previous 4 bytes
 *              are the offset to the mmap page header, flag itself is
 *              the arena the pointer was xmalloced in
 *      - 0xFF: non bucket flag, indicates previous 8 bytes are the
 *              size of the allocation
 *   
 *   - every mmap is of size 2^21 (2 MB), however, all the pages
 *     without the page_header data are initially madvised as
 *     DONTNEED, thereby making the actual allocations to physical RAM
 *     much smaller
 *   
 *   - on xmalloc, the top of the g_Bucket_Stack is 'popped', if the
 *     stack is empty, and new ALLOC_CHUNK sized page is pushed
 *   
 *   - on xfree, the pointer is pushed back onto the stack by updating
 *     the page_header's bitmap at its offset location, if an entire
 *     page that does not have any page_header data is free the page
 *     is madvised with DONTNEED
 *   
 *   - on xrealloc, the pointer is attempted to be returned unchanged
 *     if the data still fits in the bucket and is greater than the
 *     previous bucket
 *
 *   - each stack has its own mutex for pushing and popping an entire
 *     mmap chunk to an arena stack
 *
 *   - arena style, each thread has its own favorite stack, if it
 *     fails to lock the stack it will move to the next arena stack
 *
 *   - due to the size of the allocated chunks, the mmap headers are
 *     quite large (about 4 to 5 4K pages), however since each
 *     mmap allows for a minimum of ~255 and a maximum of ~160000
 *     stack 'pops' there should be decent time between mmapping new
 *     memory, and since madvise DONTNEED is utilized, the physical
 *     mapping to RAM may not even occur
 *
 *   - each header has a last offset index
 *      - since the allocations occur in the bitmap from left to
 *        right, the last offset value is set
 *      - in worst case, every bit is checked and a new mmap occurs
 *      - in best case number of comparisons to pop is 1
 */

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/mman.h>
#include <assert.h>
#include <pthread.h>
#include <string.h>

#include "xmalloc.h"


// --------- PREPROCSSOR DEFINITIONS --------------------------------



// total number of buckets
#define BUCKET_NUM 21

// minimum size of a bucket
#define BUCKET_MIN 8

// maximum size of a bucket
#define BUCKET_MAX 8192

// the number of arenas for thread safe allocs/frees
#define ARENA_NUM 8

// the page size for mmap
#define SMALL_PAGE 4096

// the size allocated for each mmap
#define ALLOC_CHUNK 2097152


// the number of longs needed to represent the bitmap (see notes)
// calculation:
// (ALLOC_CHUNK - sizeof(uint8_t) - sizeof(page_header*) -
// sizeof(uint32_t) - (BITMAP_LONGS * sizeof(uint64_t))
//      = free_size
// free_size / (BUCKET_MIN + sizeof(uint8_t) + sizeof(uint32_t))
//      = free_slots
// free_slots / (sizeof(uint64_t) * 8 bits)
//      = BITMAP_LONGS
// value allows for up to 159808 buckets per mmap
#define BITMAP_LONGS 2497



// --------- CONSTRUCTOR/DESTRUCTOR PROTOTYPES ----------------------



// constructor attribute... initializes all mutexes on startup,
// no thread safe properties
void initialize_mutexes (void) __attribute__ ((constructor));

// destructor attribute... frees all buckets on when program
// terminates
void free_all_buckets (void) __attribute__ ((destructor));



// --------- TYPEDEFS -----------------------------------------------



// every page has a header if it appears in a bucket
// a header has an encoded size, pointer to next page and a bitmap of
// free buckets for the page
typedef struct page_header {
    uint8_t size;
    struct page_header* next_page;
    uint32_t last_offset;
    uint64_t bitmap[BITMAP_LONGS];
} page_header;



// --------- CONSTANTS ----------------------------------------------



// the non bucket xmalloc flag, indicates previous 8 bytes are the
// size of the mmap
const uint8_t c_Non_Bucket_Flag =            0xFF;

// the bucket xmalloc flag, indicates previous 4 bytes are the offset
// to the mmap page_header
const uint8_t c_Bucket_Flag =                0x00;

// the metadata size for a non bucket, one uint8_t flag and size_t
const uint8_t c_Non_Bucket_Metadata_Size =   0x09;

// the meta data size for a bucket, one uint8_t flag and one uint32_t
const uint8_t c_Bucket_Metadata_Size =       0x05;

// the minimum number of pages needed for the header
const uint8_t c_Header_Pages_Needed =        0x05;

// used for checking bitmaps, the most significant bit at position 63
// is the only one set to 1
const uint64_t c_64_MSB_High =               0x8000000000000000;

// used for checking bitmaps, all bits high corresponds no free slots
// to pop from stack
const uint64_t c_64_All_High =               0xFFFFFFFFFFFFFFFF;

// mask to get an address alligned to its mmapped page
const uint64_t c_4K_Mask =                   0xFFFFFFFFFFFF1000;

/*                           bucket sizes = { 8,            12,           16,           24,
                                              32,           48,           64,           96,
                                              128,          192,          256,          384,
                                              512,          768,          1024,         1536,
                                              2048,         3072,         4096,         6144,
                                              8192       } */
// the bucket sizes for the bucket stacks
const uint32_t c_Bucket_Sizes[BUCKET_NUM] = { 0x00000008,   0x0000000C,   0x00000010,   0x00000018,  
                                              0x00000020,   0x00000030,   0x00000040,   0x00000060,
                                              0x00000080,   0x000000C0,   0x00000100,   0x00000180,
                                              0x00000200,   0x00000300,   0x00000400,   0x00000600,
                                              0x00000800,   0x00000C00,   0x00001000,   0x00001800,
                                              0x00002000 };

// the number of ALLOC_CHUNKS by bucket index, chunks are chosen such
// that the resulting number of free slots does not exceed the bitmap,
// but only grow by a factor of two between adjacent buckets so the
// total allocations do not grow too large
const uint8_t c_MMAP_Chunks[BUCKET_NUM] =  { 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02,
                                             0x04, 0x04, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08,
                                             0x10, 0x10, 0x10, 0x10, 0x20 };


// --------- THREAD LOCALS ------------------------------------------


// each threads favorite arena to use based on the bucket index
// these are the second indexer to the global g_Bucket_Stacks
__thread uint8_t t_Favorite_Arenas[BUCKET_NUM];



// --------- GLOBALS ------------------------------------------------



// the stacks of buckets with at least one free page
static page_header* g_Bucket_Stacks[BUCKET_NUM][ARENA_NUM];

// there is a mutex for each bucket
static pthread_mutex_t g_Free_Bucket_Mutexes[BUCKET_NUM][ARENA_NUM];



// --------- ENCODED SIZE FUNCTIONS ---------------------------------



// a uint8_t size in a header is encoded such that the most
// significant bit represents whether the size is intermediate (12,
// 24, 48 etc..) or a base 2 value (4, 8, 16 etc..) and the least
// significant bits represent the power of the bucket (2^n)
size_t parse_header_size(uint8_t size) {

    // get the intermediate value at bit 7
    uint8_t intermediate = (uint8_t)(size >> 0x07);
    
    // mask the value to get the power of the size
    uint8_t powertwo = size & 0x7F;
    size_t size_p = 0x1;

    // get the base two value of the bucket 
    while (powertwo--) {
        size_p *= 0x2;
    }

    // return the value, intermediate will be a 1 or a 0 so size is
    // either only the base two value or the next bucket up
    return size_p + (intermediate * size_p / 2);
}

// generates an encoded size_t in only 1 byte for the header
uint8_t gen_header_size(size_t size) {
    assert(size >= BUCKET_MIN);

    // all intermediate values are divisble by 3
    uint8_t intermediate = size % 0x3 == 0 ? 0x80 : 0x00;
    uint8_t powertwo = 0x00;

    // find the power of the size
    while (size != 0x01) {
        powertwo++;
        size /= 0x2;
    }

    // return the encoded size
    return intermediate | powertwo;
}



// --------- MMAP FUNCTIONS -----------------------------------------



// mmaps an constant sized piece of memory, all pages other than
// the ones with the page_header data are madvised as DONTNEED
void* mmap_bucket(int bucket_i) {
    assert(bucket_i >= 0 && bucket_i < BUCKET_NUM);

    // set the size of the mmap and the number of pages needed so the
    // header is NOT madvised as DONTNEED
    size_t mmap_size = c_MMAP_Chunks[bucket_i] * ALLOC_CHUNK;
    
    // mmap and madvise if needed, check errors
    void* new_bucket = mmap(0, mmap_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (!new_bucket) {
        fprintf(stderr, "mmap failed for size %lu\n", mmap_size);
        exit(1);
    }
    
    // mark all pages without header data as DONTNEED
    if (madvise(new_bucket + (SMALL_PAGE * c_Header_Pages_Needed), mmap_size - (SMALL_PAGE * c_Header_Pages_Needed), MADV_DONTNEED)) {
        fprintf(stderr, "madvise error at %p of size %lu\n", new_bucket, mmap_size);
        exit(1);
    }

    // write header data
    ((page_header*)new_bucket)->size = gen_header_size(c_Bucket_Sizes[bucket_i]);

    return new_bucket;
}

// mmaps memory for data that does not lie within a valid bucket range
void* mmap_non_bucket(size_t size) {
    assert(size > BUCKET_MAX);

    // set size to include metadata and get the total number of bytes
    // needed alligned to a 4K page
    size += c_Non_Bucket_Metadata_Size;
    size = ((size / SMALL_PAGE) + (size % SMALL_PAGE == 0 ? 0x0 : 0x1)) * SMALL_PAGE;
   
    // mmap the size
    void* ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (!ptr) {
        fprintf(stderr, "mmap failed for large size %lu\n", size);
        exit(1);
    }

    // set the metadata
    *((size_t*)ptr) = size;
    ptr += sizeof(size_t);
    *((uint8_t*)ptr) = c_Non_Bucket_Flag;
    ptr += sizeof(uint8_t);

    // return the pointer after the metadata
    return ptr;
}



// --------- XMALLOC PUSH/POP FUNCTIONS FOR STACKS ------------------



// pops a bucket size to return to calling program
void* pop_bucket(int bucket_i) {
    uint8_t bitmap_shift;
    uint32_t offset;
    uint16_t bitmap_i;

    // set the bitmap size max, assert the size can fit in the bitmap
    uint32_t bitmap_size = ((c_MMAP_Chunks[bucket_i] * ALLOC_CHUNK) - sizeof(page_header)) / (c_Bucket_Sizes[bucket_i] + c_Bucket_Metadata_Size);
    assert(bitmap_size <= BITMAP_LONGS * 64);

    // try to lock favorite arena, on lock success return is 0
    if (pthread_mutex_trylock(&g_Free_Bucket_Mutexes[bucket_i][t_Favorite_Arenas[bucket_i]])) {
        // change arenas, lock the new stack
        t_Favorite_Arenas[bucket_i] = (t_Favorite_Arenas[bucket_i] + 1) % ARENA_NUM;
        pthread_mutex_lock(&g_Free_Bucket_Mutexes[bucket_i][t_Favorite_Arenas[bucket_i]]);
    }

    // set the header and bucket found to false
    page_header* header = g_Bucket_Stacks[bucket_i][t_Favorite_Arenas[bucket_i]];
    uint8_t bucket_found = 0x00;

    // loop until null header is found, indicating no free buckets
    while (header) {
        // initially check the next slot after the last offset
        offset = (header->last_offset + 1) % bitmap_size;

        // loop until the offset exceeds the available slots in the
        // allocate data according to the header
        while (offset != header->last_offset) {
            bitmap_i = offset / (sizeof(uint64_t) * 0x08);
            bitmap_shift = offset % (sizeof(uint64_t) * 0x08);

            // check if the index has any free bits every time the
            // offset starts a new index in the bitmap
            if ((bitmap_shift == (uint8_t)0x00) && (header->bitmap[bitmap_i] == c_64_All_High)) {
                offset += sizeof(uint64_t) * 0x08;
                continue;
            }
            
            // if current offset is free in the bitmap set bucket
            // found and break
            if ((header->bitmap[bitmap_i] & (c_64_MSB_High >> bitmap_shift)) == 0x00) {
                bucket_found = 0x01;
                break;
            }

            // check the next bit
            offset = (offset + 0x01) % bitmap_size;
        }

        // break on bucket found
        if (bucket_found) {
            break;
        }

        // continue to check the next header
        header = header->next_page;
    }

    // no bucket found, push a page and get the newest added bucket
    if (!bucket_found) {
        // assert all headers were checked
        assert(!header);

        // mmap a new bucket stack and push it
        header = mmap_bucket(bucket_i);
        header->next_page = g_Bucket_Stacks[bucket_i][t_Favorite_Arenas[bucket_i]];
        g_Bucket_Stacks[bucket_i][t_Favorite_Arenas[bucket_i]] = header;

        // a new stack will always have an initial offset of 0 free
        offset = 0x0000;
        bitmap_shift = 0x00;
        bitmap_i = 0x00;
    }

    // modify the header bitmap
    header->last_offset = offset;
    header->bitmap[bitmap_i] = header->bitmap[bitmap_i] | (c_64_MSB_High >> bitmap_shift);

    // initialize the return pointer to the offset position
    void* ptr = ((void*)header) + sizeof(page_header) + (offset * (c_Bucket_Sizes[bucket_i] + c_Bucket_Metadata_Size));
    
    // write the metadata
    *((uint32_t*)ptr) = (uint32_t)(ptr - (void*)header);
    ptr += sizeof(uint32_t);
    *((uint8_t*)ptr) = t_Favorite_Arenas[bucket_i];
    ptr += sizeof(uint8_t);

    // unlock the favorite arenas stack
    pthread_mutex_unlock(&g_Free_Bucket_Mutexes[bucket_i][t_Favorite_Arenas[bucket_i]]);
    
    return ptr;
}

// pushes a bucket back onto the stack for the given arena
void push_bucket(int bucket_i, int arena_i, page_header* header, void* addr) {
    assert(bucket_i >= 0 && bucket_i < BUCKET_NUM);

    // set the offset, get the bitmap index and shift
    uint32_t offset = (uint32_t)((uint64_t)addr - (uint64_t)header + sizeof(page_header)) / (c_Bucket_Sizes[bucket_i] + c_Bucket_Metadata_Size);
    uint16_t bitmap_i = offset / (sizeof(uint64_t) * 0x08);
    uint8_t bitmap_shift = offset % (sizeof(uint64_t) * 0x08);

    // lock the arenas stack
    pthread_mutex_lock(&g_Free_Bucket_Mutexes[bucket_i][arena_i]);
    
    // update the bitmap
    header->bitmap[bitmap_i] = header->bitmap[bitmap_i] & ~(c_64_MSB_High >> bitmap_shift);

    // unlock the arenas stack
    pthread_mutex_unlock(&g_Free_Bucket_Mutexes[bucket_i][arena_i]);
}



// --------- XMALLOC HEADER PROTOTYPE IMPLEMENTATIONS ---------------



// 'mallocs' a given number of bytes
void* xmalloc(size_t bytes) {
    // if bytes is greater than the max bucket do regular mmap
    if (bytes > BUCKET_MAX) {
        return mmap_non_bucket(bytes);
    }

    // determine bucket index from size
    int bucket_i = 0;
    while (bytes > c_Bucket_Sizes[bucket_i] && bucket_i < BUCKET_NUM - 1) {
        bucket_i++;
    }
    assert(bytes <= c_Bucket_Sizes[bucket_i] && (bucket_i == 0 ? 1 : bytes > c_Bucket_Sizes[bucket_i - 1]));

    // pop a bucket and return to caller
    return pop_bucket(bucket_i);    
}

// 'frees' a given xmalloced pointer
void xfree(void* ptr) {
    // do nothing for null pointer
    if (!ptr) {
        return;
    }

    // set the flag
    uint8_t flag = *((uint8_t*)(ptr - 1));

    // if non bucket do regular munmap
    if (flag == c_Non_Bucket_Flag) {
        ptr -= c_Non_Bucket_Metadata_Size;

        // munmap and check error
        if (munmap(ptr, *((size_t*)ptr))) {
            fprintf(stderr, "munmap error: %p\n", ptr);
            exit(1);
        }
        return;
    }

    page_header* header;
    
    // check if flag is a valid arena
    if (flag < ARENA_NUM) {
        ptr -= c_Bucket_Metadata_Size;
        header = (page_header*)(ptr - *((uint32_t*)ptr));
    }
    else {
        fprintf(stderr, "arena flag error at %p, flag: %hhu\n", ptr, flag);
        exit(1);
    }

    // get the bucket size and find its index
    size_t bucket_size = parse_header_size(header->size);
    int bucket_i = 0;

    while (bucket_i < BUCKET_NUM && c_Bucket_Sizes[bucket_i] != bucket_size) {
        bucket_i++;
    }
    assert(bucket_i < BUCKET_NUM);

    // push the bucket back onto the stack
    push_bucket(bucket_i, flag, header, ptr);
}

// 'reallocs' the given pointer to new size, preserves data
void* xrealloc(void* prev, size_t bytes) {
    // do nothing with null pointer
    if (!prev) {
        return prev;
    }
    
    void* ptr = prev;
    uint8_t flag = *((uint8_t*)(prev - 1));
    size_t prev_bytes = 0x00;

    // if non bucket flag
    if (flag == c_Non_Bucket_Flag) {
        ptr -= c_Non_Bucket_Metadata_Size;
        prev_bytes = *((size_t*)ptr);

        // if new bytes is bigger than prev, or new bytes is less than
        // 3/4 the previous size, create new pointer and copy old data
        if (prev_bytes < bytes || bytes < (prev_bytes * 3 / 4)) {
            ptr = xmalloc(bytes);
            memcpy(ptr, prev, bytes < prev_bytes ? bytes : prev_bytes);
            xfree(prev);
            return ptr;
        }

        // prev_bytes * 3/4 <= bytes <= prev_bytes
        return prev;
    }
    
    page_header* header;

    // if valid flag, set header
    if (flag < ARENA_NUM) {
        ptr -= c_Bucket_Metadata_Size;
        header = (page_header*)(ptr - *((uint32_t*)ptr));
    }
    else {
        fprintf(stderr, "realloc flag error at %p, flag: %hhu\n", ptr, flag);
        exit(1);
    }

    // get prev byte num and bucket index
    int bucket_i = 0;
    prev_bytes = parse_header_size(header->size);

    // if new bytes does not fit in old (or any) bucket, xmalloc new
    // and copy data
    if (bytes > BUCKET_MAX || bytes > prev_bytes || (bytes < (prev_bytes * 2 / 3) && prev_bytes != BUCKET_MIN)) {
        ptr = xmalloc(bytes);
        memcpy(ptr, prev, bytes < prev_bytes ? bytes : prev_bytes);
        xfree(prev);
        return ptr;
    }

    // data fits, so return prev
    return prev;
}



// --------- CONSTRUCTOR/DESTRUCTOR FUNCTIONS -----------------------



// called when program starts
void initialize_mutexes(void) {
    // assert preprocessor definitions allign with constants
    assert(c_Bucket_Sizes[0] == BUCKET_MIN && c_Bucket_Sizes[BUCKET_NUM - 1] == BUCKET_MAX);
    
    int bucket_index;
    int arena_index;

    // loop over all buckets
    for (bucket_index = 0; bucket_index < BUCKET_NUM; bucket_index++) {
        // loop over each arean for the bucket
        for (arena_index = 0; arena_index < ARENA_NUM; arena_index++) {
            // initialize the mutexes
            pthread_mutex_init(&g_Free_Bucket_Mutexes[bucket_index][arena_index], 0);
            page_header* header = mmap_bucket(bucket_index);
            header->next_page = g_Bucket_Stacks[bucket_index][arena_index];
            g_Bucket_Stacks[bucket_index][arena_index] = header;
        }
    }
}

// called when program terminates
void free_all_buckets(void) {
    int bucket_index;
    int arena_index;
    page_header* header;
    page_header* next;

    // loop over all buckets
    for (bucket_index = 0; bucket_index < BUCKET_NUM; bucket_index++) {
        // loop over each arena per bucket
        for (arena_index = 0; arena_index < ARENA_NUM; arena_index++) {
            // lock the arena bucket
            pthread_mutex_lock(&g_Free_Bucket_Mutexes[bucket_index][arena_index]);
            
            // munmap all headers
            header = g_Bucket_Stacks[bucket_index][arena_index];
            while (header) {
                next = header->next_page;
                if (munmap(header, c_MMAP_Chunks[bucket_index] * ALLOC_CHUNK)) {
                    // don't break, continue to trying free the rest of the
                    // headers
                    printf("munmap error on destruction\n");
                }
                header = next;
            }

            // unlock the arena
            pthread_mutex_unlock(&g_Free_Bucket_Mutexes[bucket_index][arena_index]);
        }
    }
}



// --------- END OF FILE --------------------------------------------



