#ifndef __PSLIB_MP_H__
#define __PSLIB_MP_H__

#include <libps/pslib.h>

#ifdef __cplusplus
extern "C" {
#endif

PSLIB_API void PSLIB_CALL
PS_mp_init(void);

PSLIB_API void * PSLIB_CALL
PS_mp_malloc(PSDoc *p, size_t size, const char *caller);

PSLIB_API void * PSLIB_CALL
PS_mp_realloc(PSDoc *p, void *mem, size_t size, const char *caller);

PSLIB_API void PSLIB_CALL
PS_mp_free(PSDoc *p, void *mem);

PSLIB_API void PSLIB_CALL
PS_mp_list_unfreed();

#ifdef __cplusplus
}
#endif

#endif
