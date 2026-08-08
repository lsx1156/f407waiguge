#pragma once

#include <stdint.h>
#include <stddef.h>

#define LZ4_VERSION_MAJOR    1
#define LZ4_VERSION_MINOR    9
#define LZ4_VERSION_RELEASE  4

int LZ4_decompress_safe(const char* src, char* dst, int compressedSize, int dstCapacity);
int LZ4_decompress_fast(const char* src, char* dst, int originalSize);

typedef struct { uint32_t dummy; } LZ4_streamDecode_t;

int LZ4_setStreamDecode(LZ4_streamDecode_t* LZ4_streamDecode, const char* dict, int dictSize);
int LZ4_decompress_safe_continue(LZ4_streamDecode_t* LZ4_streamDecode, const char* src, char* dst, int compressedSize, int dstCapacity);
