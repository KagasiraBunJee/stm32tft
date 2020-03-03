#ifndef _ILI9341_t3_font_Nunito_
#define _ILI9341_t3_font_Nunito_

#include <stdint.h>

typedef struct {
	const unsigned char *index;
	const unsigned char *unicode;
	const unsigned char *data;
	unsigned char version;
	unsigned char reserved;
	unsigned char index1_first;
	unsigned char index1_last;
	unsigned char index2_first;
	unsigned char index2_last;
	unsigned char bits_index;
	unsigned char bits_width;
	unsigned char bits_height;
	unsigned char bits_xoffset;
	unsigned char bits_yoffset;
	unsigned char bits_delta;
	unsigned char line_space;
	unsigned char cap_height;
} packedbdf_t;
typedef packedbdf_t ILI9341_t3_font_t;

typedef struct {             // Data stored PER GLYPH
  uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
  uint8_t  width, height;    // Bitmap dimensions in pixels
  uint8_t  xAdvance;         // Distance to advance cursor (x axis)
  int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;

typedef struct {         // Data stored for FONT AS A WHOLE:
  uint8_t * bitmap;      // Glyph bitmaps, concatenated
  GFXglyph *glyph;       // Glyph array
  uint8_t   first, last; // ASCII extents
  uint8_t   yAdvance;    // Newline distance (y axis)

  // Added(pimvanpelt) for framebuffer rendering.
  int8_t    font_height;      // Maximum per-glyph height
  int8_t    font_width;       // Maximum per-glyph width
  int8_t    font_min_xOffset; // Left-most glyph xOffset
  int8_t    font_min_yOffset; // Left-most glyph yOffset
} GFXfont;

enum GFXfont_t {
  GFXFONT_NONE     =0,
  GFXFONT_INTERNAL =1,
  GFXFONT_FILE     =2,
};

extern const ILI9341_t3_font_t Nunito_8;
extern const ILI9341_t3_font_t Nunito_9;
extern const ILI9341_t3_font_t Nunito_10;
extern const ILI9341_t3_font_t Nunito_11;
extern const ILI9341_t3_font_t Nunito_12;
extern const ILI9341_t3_font_t Nunito_13;
extern const ILI9341_t3_font_t Nunito_14;
extern const ILI9341_t3_font_t Nunito_16;
extern const ILI9341_t3_font_t Nunito_18;
extern const ILI9341_t3_font_t Nunito_20;
extern const ILI9341_t3_font_t Nunito_24;
extern const ILI9341_t3_font_t Nunito_28;
extern const ILI9341_t3_font_t Nunito_32;
extern const ILI9341_t3_font_t Nunito_40;
extern const ILI9341_t3_font_t Nunito_48;
extern const ILI9341_t3_font_t Nunito_60;
extern const ILI9341_t3_font_t Nunito_72;
extern const ILI9341_t3_font_t Nunito_96;

#ifdef __cplusplus
} // extern "C"
#endif

#endif
