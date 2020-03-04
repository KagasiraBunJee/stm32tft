/* GIMP RGB C-Source image dump (1231.c) */

#include <stdint.h>

static const struct {
  unsigned int 	 width;
  unsigned int 	 height;
  unsigned int 	 bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */ 
  unsigned char	 pixel_data[240 * 320 * 2 + 1];
} gimp_image = {
  240, 320, 2,
  ""
};

