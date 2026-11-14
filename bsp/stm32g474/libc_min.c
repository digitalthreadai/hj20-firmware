/**
 * @file libc_min.c
 * @brief The two libc routines GCC may emit calls to in a -nostdlib image.
 *
 * Compiled with -fno-tree-loop-distribute-patterns so these loops are not
 * themselves turned into calls to memset/memcpy.
 */
#include <stddef.h>
#include <stdint.h>

void *memset(void *dst, int c, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    while (n--) {
        *d++ = (uint8_t)c;
    }
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    while (n--) {
        *d++ = *s++;
    }
    return dst;
}
