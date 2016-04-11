#ifndef __PSLIB_H__
#define __PSLIB_H__

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32

#define PSLIB_CALL __cdecl

#ifdef PSLIB_EXPORTS
#define PSLIB_API __declspec(dllexport) /* prepare a DLL (internal use only) */
#elif defined(PSLIB_DLL)
#define PSLIB_API __declspec(dllimport) /* PSlib clients: import PSlib DLL */
#else /* !PSLIB_DLL */
#define PSLIB_API /* */  /* default: generate or use static library */

#endif  /* !PSLIB_DLL */

#endif /* !WIN32 */

#ifndef PSLIB_CALL
#define PSLIB_CALL
#endif
#ifndef PSLIB_API
#define PSLIB_API
#endif

#define PS_MemoryError 1
#define PS_IOError 2
#define PS_RuntimeError 3
#define PS_Warning 100

#define PS_LINECAP_BUTT 0
#define PS_LINECAP_ROUND 1
#define PS_LINECAP_SQUARED 2

#define PS_LINEJOIN_MITER 0
#define PS_LINEJOIN_ROUND 1
#define PS_LINEJOIN_BEVEL 2

#define PS_GOTO_NEXT_PAGE -1
#define PS_GOTO_PREV_PAGE -2

#define ps_true 1
#define ps_false 0

typedef int ps_bool;

typedef struct PSDoc_ PSDoc;

PSLIB_API int PSLIB_CALL
PS_get_majorversion(void);

PSLIB_API int PSLIB_CALL
PS_get_minorversion(void);

PSLIB_API int PSLIB_CALL
PS_get_subminorversion(void);

PSLIB_API void PSLIB_CALL
PS_boot(void);

PSLIB_API void PSLIB_CALL
PS_shutdown(void);

PSLIB_API void PSLIB_CALL
PS_set_info(PSDoc *p, const char *key, const char *val);

PSLIB_API PSDoc * PSLIB_CALL
PS_new(void);

PSLIB_API PSDoc * PSLIB_CALL
PS_new2(void  (*errorhandler)(PSDoc *p, int type, const char *msg, void *data),
	void* (*allocproc)(PSDoc *p, size_t size, const char *caller),
	void* (*reallocproc)(PSDoc *p, void *mem, size_t size, const char *caller),
	void  (*freeproc)(PSDoc *p, void *mem),
	void *opaque);

PSLIB_API int PSLIB_CALL
PS_open_fp(PSDoc *psdoc, FILE *fp);

PSLIB_API void * PSLIB_CALL
PS_get_opaque(PSDoc *psdoc);

PSLIB_API int PSLIB_CALL
PS_open_mem(PSDoc *p, size_t (*writeproc)(PSDoc *p, void *data, size_t size));

PSLIB_API int PSLIB_CALL
PS_open_file(PSDoc *psdoc, const char *filename);

PSLIB_API void PSLIB_CALL
PS_close(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_delete(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_begin_page(PSDoc *psdoc, float width, float height);

PSLIB_API void PSLIB_CALL
PS_end_page(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_set_parameter(PSDoc *psdoc, const char *name, const char *value);

PSLIB_API const char * PSLIB_CALL
PS_get_parameter(PSDoc *psdoc, const char *name, float modifier);

PSLIB_API void PSLIB_CALL
PS_set_value(PSDoc *psdoc, const char *name, float value);

PSLIB_API float PSLIB_CALL
PS_get_value(PSDoc *psdoc, const char *name, float modifier);

PSLIB_API void PSLIB_CALL
PS_list_values(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_list_parameters(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_list_resources(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_set_text_pos(PSDoc *psdoc, float x, float y);

PSLIB_API void PSLIB_CALL
PS_setlinewidth(PSDoc *p, float width);

PSLIB_API void PSLIB_CALL
PS_setlinecap(PSDoc *p, int type);

PSLIB_API void PSLIB_CALL
PS_setlinejoin(PSDoc *p, int type);

PSLIB_API void PSLIB_CALL
PS_setmiterlimit(PSDoc *p, float value);

PSLIB_API void PSLIB_CALL
PS_setflat(PSDoc *p, float value);

PSLIB_API void PSLIB_CALL
PS_setoverprintmode(PSDoc *p, int mode);

PSLIB_API void PSLIB_CALL
PS_setsmoothness(PSDoc *p, float smoothness);

PSLIB_API void PSLIB_CALL
PS_setdash(PSDoc *p, float on, float off);

PSLIB_API void PSLIB_CALL
PS_setpolydash(PSDoc *p, float *arr, int length);

PSLIB_API void PSLIB_CALL
PS_curveto(PSDoc *psdoc, float x1, float y1, float x2, float y2, float x3, float y3);

PSLIB_API void PSLIB_CALL
PS_rect(PSDoc *psdoc, float x, float y, float width, float height);

PSLIB_API void PSLIB_CALL
PS_circle(PSDoc *psdoc, float x, float y, float radius);

PSLIB_API void PSLIB_CALL
PS_arc(PSDoc *psdoc, float x, float y, float radius, float alpha, float beta);

PSLIB_API void PSLIB_CALL
PS_arcn(PSDoc *psdoc, float x, float y, float radius, float alpha, float beta);

PSLIB_API void PSLIB_CALL
PS_clip(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_setgray(PSDoc *psdoc, float gray);

PSLIB_API void PSLIB_CALL
PS_closepath(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_closepath_stroke(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_fill_stroke(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_stroke(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_fill(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_save(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_restore(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_show(PSDoc *psdoc, const char *text);

PSLIB_API void PSLIB_CALL
PS_show2(PSDoc *psdoc, const char *text, int xlen);

PSLIB_API void PSLIB_CALL
PS_show_xy(PSDoc *psdoc, const char *text, float x, float y);

PSLIB_API void PSLIB_CALL
PS_show_xy2(PSDoc *psdoc, const char *text, int xlen, float x, float y);

PSLIB_API int PSLIB_CALL
PS_show_boxed(PSDoc *psdoc, const char *text, float left, float bottom, float width, float height, const char *hmode, const char *feature);

PSLIB_API void PSLIB_CALL
PS_continue_text(PSDoc *psdoc, const char *text);

PSLIB_API void PSLIB_CALL
PS_continue_text2(PSDoc *psdoc, const char *text, int len);

PSLIB_API void PSLIB_CALL
PS_setcolor(PSDoc *psdoc, const char *type, const char *colorspace, float c1, float c2, float c3, float c4);

PSLIB_API int PSLIB_CALL
PS_makespotcolor(PSDoc *psdoc, const char *name, int reserved);

PSLIB_API int PSLIB_CALL
PS_findfont(PSDoc *psdoc, const char *fontname, const char *encoding, int embed);

PSLIB_API float PSLIB_CALL
PS_stringwidth(PSDoc *psdoc, const char *text, int fontid, float size);

PSLIB_API float PSLIB_CALL
PS_stringwidth2(PSDoc *psdoc, const char *text, int xlen, int fontid, float size);

PSLIB_API float PSLIB_CALL
PS_string_geometry(PSDoc *psdoc, const char *text, int xlen, int fontid, float size, float *dimension);

PSLIB_API void PSLIB_CALL
PS_deletefont(PSDoc *psdoc, int fontid);

PSLIB_API void PSLIB_CALL
PS_setfont(PSDoc *psdoc, int fontid, float size);

PSLIB_API int PSLIB_CALL
PS_getfont(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_moveto(PSDoc *psdoc, float x, float y);

PSLIB_API void PSLIB_CALL
PS_lineto(PSDoc *psdoc, float x, float y);

PSLIB_API void PSLIB_CALL
PS_rotate(PSDoc *psdoc, float x);

PSLIB_API void PSLIB_CALL
PS_translate(PSDoc *psdoc, float x, float y);

PSLIB_API void PSLIB_CALL
PS_scale(PSDoc *psdoc, float x, float y);

PSLIB_API const char * PSLIB_CALL
PS_get_buffer(PSDoc *psdoc, long *size);

PSLIB_API int PSLIB_CALL
PS_open_image(PSDoc *psdoc, const char *type, const char *source, const char *data, long length, int width, int height, int components, int bpc, const char *params);

PSLIB_API int PSLIB_CALL
PS_open_image_file(PSDoc *psdoc, const char *type, const char *filename, const char *stringparam, int intparam);

PSLIB_API void PSLIB_CALL
PS_place_image(PSDoc *psdoc, int imageid, float x, float y, float scale);

PSLIB_API void PSLIB_CALL
PS_close_image(PSDoc *psdoc, int imageid);

PSLIB_API void PSLIB_CALL
PS_add_weblink(PSDoc *psdoc, float llx, float lly, float urx, float ury, const char *url);

PSLIB_API void PSLIB_CALL
PS_add_pdflink(PSDoc *psdoc, float llx, float lly, float urx, float ury, const char *filename, int page, const char *dest);

PSLIB_API void PSLIB_CALL
PS_add_locallink(PSDoc *psdoc, float llx, float lly, float urx, float ury, const int page, const char *dest);

PSLIB_API void PSLIB_CALL
PS_add_launchlink(PSDoc *psdoc, float llx, float lly, float urx, float ury, const char *filename);

PSLIB_API int PSLIB_CALL
PS_add_bookmark(PSDoc *psdoc, const char *text, int parent, int open);

PSLIB_API void PSLIB_CALL
PS_add_note(PSDoc *psdoc, float llx, float lly, float urx, float ury, const char *contents, const char *title, const char *icon, int open);

PSLIB_API void PSLIB_CALL
PS_set_border_style(PSDoc *psdoc, const char *style, float width);

PSLIB_API void PSLIB_CALL
PS_set_border_color(PSDoc *psdoc, float red, float green, float blue);

PSLIB_API void PSLIB_CALL
PS_set_border_dash(PSDoc *psdoc, float black, float white);

PSLIB_API int PSLIB_CALL
PS_begin_template(PSDoc *psdoc, float width, float height);

PSLIB_API void PSLIB_CALL
PS_end_template(PSDoc *psdoc);

PSLIB_API int PSLIB_CALL
PS_begin_pattern(PSDoc *psdoc, float width, float height, float xstep, float ystep, int painttype);

PSLIB_API void PSLIB_CALL
PS_end_pattern(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_shfill(PSDoc *psdoc, int shading);

PSLIB_API int PSLIB_CALL
PS_shading(PSDoc *psdoc, const char *shtype, float x0, float y0, float x1, float y1, float c1, float c2, float c3, float c4, const char *optlist);

PSLIB_API int PSLIB_CALL
PS_shading_pattern(PSDoc *psdoc, int shading, const char *optlist);

PSLIB_API int PSLIB_CALL
PS_create_gstate(PSDoc *psdoc, const char *optlist);

PSLIB_API int PSLIB_CALL
PS_hyphenate(PSDoc *psdoc, const char *text, char **hyphens);

PSLIB_API void PSLIB_CALL
PS_symbol(PSDoc *psdoc, unsigned char c);

PSLIB_API float PSLIB_CALL
PS_symbol_width(PSDoc *psdoc, unsigned char c, int fontid, float size);

PSLIB_API void PSLIB_CALL
PS_symbol_name(PSDoc *psdoc, unsigned char c, int fontid, char *name, int size);

PSLIB_API int PSLIB_CALL
PS_include_file(PSDoc *psdoc, const char *filename);

PSLIB_API int PSLIB_CALL
PS_begin_font(PSDoc *psdoc, const char *fontname, int reserverd, double a, double b, double c, double d, double e, double f, const char *optlist);

PSLIB_API void PSLIB_CALL
PS_end_font(PSDoc *psdoc);

PSLIB_API int PSLIB_CALL
PS_begin_glyph(PSDoc *psdoc, const char *glyphname, double wx, double llx, double lly, double urx, double ury);

PSLIB_API void PSLIB_CALL
PS_end_glyph(PSDoc *psdoc);

PSLIB_API void PSLIB_CALL
PS_add_kerning(PSDoc *psdoc, int fontid, const char *glyphname1, const char *glyphname2, int kern);

PSLIB_API void PSLIB_CALL
PS_add_ligature(PSDoc *psdoc, int fontid, const char *glyphname1, const char *glyphname2, const char *glyphname3);

PSLIB_API void PSLIB_CALL
PS_glyph_show(PSDoc *psdoc, const char *name);

PSLIB_API char** PSLIB_CALL
PS_glyph_list(PSDoc *psdoc, int fontid, char ***charlist, int *len);

PSLIB_API float PSLIB_CALL
PS_glyph_width(PSDoc *psdoc, const char *glyphname, int fontid, float size);

PSLIB_API void PSLIB_CALL
PS_free_glyph_list(PSDoc *psdoc, char **charlist, int len);

#ifdef __cplusplus
}
#endif

#endif
