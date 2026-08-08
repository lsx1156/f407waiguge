#define LZ4_DISABLE_DEPRECATE_WARNINGS 1
#include "lz4.h"
#include <string.h>

typedef uint8_t BYTE;

/*---------- 常量 ----------*/
#define MINMATCH 4
#define WILDCOPYLENGTH 8
#define LASTLITERALS 5
#define MFLIMIT (WILDCOPYLENGTH + MINMATCH)

/*---------- 本地工具 ----------*/
static uint16_t LZ4_read16(const void* p) { uint16_t v; memcpy(&v, p, 2); return v; }
static uint32_t LZ4_read32(const void* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static void LZ4_write16(void* p, uint16_t v) { memcpy(p, &v, 2); }
static void LZ4_write32(void* p, uint32_t v) { memcpy(p, &v, 4); }

static void LZ4_wildCopy(BYTE* dst, const BYTE* src, const BYTE* end)
{
    while (dst < end) {
        LZ4_write32(dst,   LZ4_read32(src));
        LZ4_write32(dst+4, LZ4_read32(src+4));
        dst += 8; src += 8;
    }
}

/*---------- 核心解压 (内联函数, 替代原宏, 编译器100%兼容) ----------*/
static int LZ4_decompress_generic(
    const char* src, char* dst,
    int srcSize, int dstCapacity,
    int safe, int full,
    const char* dict)
{
    const BYTE* ip = (const BYTE*)src;
    const BYTE* const iend = ip + srcSize;
    BYTE* op = (BYTE*)dst;
    BYTE* const oend = op + dstCapacity;
    BYTE* const oexit = op + (full ? dstCapacity : 0);
    const BYTE* const dictStart = (const BYTE*)dict;

    while (1) {
        unsigned token = *ip++;
        unsigned length = token >> 4;
        if (length == 0xF) {
            unsigned s;
            do { s = *ip++; length += s; } while (s == 255);
        }
        BYTE* const cpy = op + length;
        if (safe && (cpy > oexit - MFLIMIT)) return -1;
        LZ4_wildCopy(op, ip, cpy);
        ip += length; op = cpy;

        if (safe && (ip >= iend - LASTLITERALS)) break;

        unsigned offset = LZ4_read16(ip); ip += 2;
        const BYTE* match = op - offset;
        if (safe && (offset == 0 || match < dictStart)) return -1;
        length = token & 0xF;
        if (length == 0xF) {
            unsigned s;
            do { s = *ip++; length += s; } while (s == 255);
        }
        length += MINMATCH;
        BYTE* const matchEnd = op + length;
        if (safe && (matchEnd > oend - LASTLITERALS)) return -1;
        if (offset < 8) {
            LZ4_write32(op,   LZ4_read32(match));
            LZ4_write32(op+4, LZ4_read32(match+4));
            op += 8; match += 8;
            while (op < matchEnd) { *op++ = *match++; }
        } else {
            LZ4_wildCopy(op, match, matchEnd);
            op = matchEnd;
        }
    }
    return (int)(op - (BYTE*)dst);
}

/*========== 对外 API ==========*/

int LZ4_decompress_safe(const char* src, char* dst, int compressedSize, int dstCapacity)
{
    return LZ4_decompress_generic(src, dst, compressedSize, dstCapacity, 1, 0, dst);
}

int LZ4_decompress_fast(const char* src, char* dst, int originalSize)
{
    return LZ4_decompress_generic(src, dst, 0, originalSize, 0, 1, dst);
}

/* 占位，防链接报错 */
int LZ4_setStreamDecode(LZ4_streamDecode_t* LZ4_streamDecode, const char* dict, int dictSize)
{
    (void)LZ4_streamDecode; (void)dict; (void)dictSize;
    return 0;
}

int LZ4_decompress_safe_continue(LZ4_streamDecode_t* LZ4_streamDecode, const char* src, char* dst, int compressedSize, int dstCapacity)
{
    (void)LZ4_streamDecode;
    return LZ4_decompress_safe(src, dst, compressedSize, dstCapacity);
}
