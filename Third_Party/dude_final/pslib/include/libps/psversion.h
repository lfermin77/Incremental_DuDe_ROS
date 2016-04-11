/*
 * pslibversion.h : compile-time version informations for the pslib.
 *
 * See Copyright for the status of this software.
 *
 * Uwe@steinmann.cx
 */

#ifndef __PSLIB_VERSION_H__
#define __PSLIB_VERSION_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * use those to be sure nothing nasty will happen if
 * your library and includes mismatch
 */
#define LIBPS_DOTTED_VERSION "0.4.5"
#define LIBPS_MAJOR_VERSION 0
#define LIBPS_MINOR_VERSION 4
#define LIBPS_MICRO_VERSION 5

/*
 * Whether Debugging module is configured in
 */
#if 1
#define LIBPS_DEBUG_ENABLED
#else
#define LIBPS_DEBUG_DISABLED
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif


