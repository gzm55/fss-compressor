#include <stddef.h>

#ifndef __FPAQ0F2_H__
#define __FPAQ0F2_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Compress [in, in + len) bytes into [out, out + return) bytes if buffer is larger enough,
 * otherwise return bufszie + 1, and first bufsize compressed bytes will be filled into
 * the out buffer. On error, return SIZE_MAX.
 */
size_t fpaq0f2_compress(const void * in, size_t len, void * out, size_t bufsize);

/* Decompress [in, in + len) bytes into [out, out + return) bytes if buffer is larger enough,
 * otherwise return bufszie + 1, and first bufsize decompressed bytes will be filled into
 * the out buffer. On error, return SIZE_MAX.
 */
size_t fpaq0f2_decompress(const void * in, size_t len, void * out, size_t bufsize);

#ifdef __cplusplus
}
#endif

#endif /* __FPAQ0F2_H__ */
