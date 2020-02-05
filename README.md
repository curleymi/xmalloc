# xmalloc: Thread-Safe Dynamic Memory Allocator

## xmalloc
```
the top of the g_Bucket_Stack is 'popped', if the stack is empty, a new ALLOC_CHUNK sized page is pushed
```

## xfree
```
the pointer is pushed back onto the stack by updating the page_header's bitmap at its offset location, if an
entire page that does not have any page_header data is free the page is madvised with MADV_DONTNEED
```

## xrealloc
```
the pointer is attempted to be returned unchanged if the data still fits in the bucket and is greater than the
previous bucket
```

### notes

- bucket style allocator, with each bucket size owning a stack of memory chunks
  ```
  each stack has its own mutex for pushing and popping an entire mmap chunk to an arena stack
  ```

- arena style thread managemnt
  ```
  each thread has its own favorite stack, if it fails to lock the stack it will move to the next arena stack
  ```

- the mmap headers are quite large (about 4 to 5 4K pages)
  ```
  however since each mmap allows for a minimum of ~255 and a maximum of ~160000 stack 'pops' there should be
  decent time between mmapping new memory, and since madvise MADV_DONTNEED is utilized, the physical mapping to
  RAM may not even occur
  ```

- each header utilizes a cyclic bitmap
  ```
  since the allocations occur in the bitmap from left to right, the last offset value is set and rotates
  around back to 0 on the next allocation
  in worst case, every bit is checked and a new mmap occurs
  in best case number of comparisons to pop is 1
  depending on how many allocations per bucket, there should be a reasobale amount of time before the worst
  case scenario occurs
  ```

- all pointers returned to caller are preceeded by an 8-bit flag
  #### flag definitions
    - 0x00 - AREMA_NUM
      ```
      bucket flag, indicates previous 4 bytes are the offset to the mmap page header, flag itself is the
      arena the pointer was xmalloced in
      ```
    - 0xFF
      ```
      non bucket flag, indicates previous 8 bytes are the size of the allocation
      ```
      
- every mmap is of size 2^21 (2 MB)
  ```
  all the pages without the page_header data are initially madvised as MADV_DONTNEED, thereby making the
  actual allocations to physical RAM much smaller
  ```
