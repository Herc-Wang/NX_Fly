/********************************************************************************************
 * apps/graphics/nxglyphs/src/glyph_lcdclock102x48.cxx
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>

#include "graphics/nxwidgets/crlepalettebitmap.hxx"
#include "graphics/nxglyphs.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

#define BITMAP_NROWS     48
#define BITMAP_NCOLUMNS  102
#define BITMAP_NLUTCODES 53

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NXWidgets;

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32
// RGB24 (8-8-8) Colors

static const uint32_t g_lcdClockNormalLut[BITMAP_NLUTCODES] =
{
  0x808c80, 0x748474, 0x607060, 0x7c887c, 0x6c786c, 0x445844, 0x3c503c, 0x385038,  /* Codes 0-7 */
  0x243c24, 0x002000, 0x707c70, 0x344c34, 0x102c10, 0x6c7c6c, 0x4c604c, 0x405440,  /* Codes 8-15 */
  0x042404, 0x0c2c0c, 0x687868, 0x042004, 0x788478, 0x284428, 0x2c442c, 0x546854,  /* Codes 16-23 */
  0x5c6c5c, 0x788878, 0x143014, 0x183418, 0x506450, 0x546454, 0x1c381c, 0x485c48,  /* Codes 24-31 */
  0x244024, 0x647464, 0x082408, 0x586858, 0x082808, 0x586c58, 0x486048, 0x405840,  /* Codes 32-39 */
  0x304830, 0x103010, 0x3c543c, 0x687468, 0x708070, 0x284028, 0x203820, 0x748074,  /* Codes 40-47 */
  0x143414, 0x203c20, 0x5c705c, 0x7c8c7c, 0xd4d828,  /* Codes 48-52 */
};

static const uint32_t g_lcdClockBrightLut[BITMAP_NLUTCODES] =
{
  0x9fa89f, 0x96a296, 0x879387, 0x9ca59c, 0x909990, 0x728172, 0x6c7b6c, 0x697b69,  /* Codes 0-7 */
  0x5a6c5a, 0x3f573f, 0x939c93, 0x667866, 0x4b604b, 0x909c90, 0x788778, 0x6f7e6f,  /* Codes 8-15 */
  0x425a42, 0x486048, 0x8d998d, 0x425742, 0x99a299, 0x5d725d, 0x607260, 0x7e8d7e,  /* Codes 16-23 */
  0x849084, 0x99a599, 0x4e634e, 0x516651, 0x7b8a7b, 0x7e8a7e, 0x546954, 0x758475,  /* Codes 24-31 */
  0x5a6f5a, 0x8a968a, 0x455a45, 0x818d81, 0x455d45, 0x819081, 0x758775, 0x6f816f,  /* Codes 32-39 */
  0x637563, 0x4b634b, 0x6c7e6c, 0x8d968d, 0x939f93, 0x5d6f5d, 0x576957, 0x969f96,  /* Codes 40-47 */
  0x4e664e, 0x576c57, 0x849384, 0x9ca89c, 0xdee15d,  /* Codes 48-52 */
};

#elif CONFIG_NXWIDGETS_BPP == 16
// RGB16 (565) Colors (four of the colors in this map are duplicates)

static const uint16_t g_lcdClockNormalLut[BITMAP_NLUTCODES] =
{
  0x8470, 0x742e, 0x638c, 0x7c4f, 0x6bcd, 0x42c8, 0x3a87, 0x3a87, 0x21e4, 0x0100,  /* Codes 0-9 */
  0x73ee, 0x3266, 0x1162, 0x6bed, 0x4b09, 0x42a8, 0x0120, 0x0961, 0x6bcd, 0x0100,  /* Codes 10-19 */
  0x7c2f, 0x2a25, 0x2a25, 0x534a, 0x5b6b, 0x7c4f, 0x1182, 0x19a3, 0x532a, 0x532a,  /* Codes 20-29 */
  0x19c3, 0x4ae9, 0x2204, 0x63ac, 0x0921, 0x5b4b, 0x0941, 0x5b6b, 0x4b09, 0x42c8,  /* Codes 30-39 */
  0x3246, 0x1182, 0x3aa7, 0x6bad, 0x740e, 0x2a05, 0x21c4, 0x740e, 0x11a2, 0x21e4,  /* Codes 40-49 */
  0x5b8b, 0x7c6f, 0xd6c5,  /* Codes 50-52 */
};

static const uint16_t g_lcdClockBrightLut[BITMAP_NLUTCODES] =
{
  0x9d53, 0x9512, 0x8490, 0x9d33, 0x94d2, 0x740e, 0x6bcd, 0x6bcd, 0x5b6b, 0x3aa7,  /* Codes 0-9 */
  0x94f2, 0x63cc, 0x4b09, 0x94f2, 0x7c2f, 0x6bed, 0x42c8, 0x4b09, 0x8cd1, 0x42a8,  /* Codes 10-19 */
  0x9d13, 0x5b8b, 0x638c, 0x7c6f, 0x8490, 0x9d33, 0x4b09, 0x532a, 0x7c4f, 0x7c4f,  /* Codes 20-29 */
  0x534a, 0x742e, 0x5b6b, 0x8cb1, 0x42c8, 0x8470, 0x42e8, 0x8490, 0x742e, 0x6c0d,  /* Codes 30-39 */
  0x63ac, 0x4b09, 0x6bed, 0x8cb1, 0x94f2, 0x5b6b, 0x534a, 0x94f2, 0x4b29, 0x536a,  /* Codes 40-49 */
  0x8490, 0x9d53, 0xdf0b,  /* Codes 50-52 */
};

#elif CONFIG_NXWIDGETS_BPP == 8
// 8-bit color lookups.  NOTE:  This is really dumb!  The lookup index is 8-bits and it used
// to lookup an 8-bit value.  There is no savings in that!  It would be better to just put
// the 8-bit color/greyscale value in the run-length encoded image and save the cost of these
// pointless lookups.  But these p;ointless lookups do make the logic compatible with the
// 16- and 24-bit types.
///

#  ifdef CONFIG_NXWIDGETS_GREYSCALE
// 8-bit Greyscale

static const uint8_t g_lcdClockNormalLut[BITMAP_NLUTCODES] =
{
  0x87, 0x7d, 0x69, 0x83, 0x73, 0x4f, 0x47, 0x46, 0x32, 0x12, 0x77, 0x42, 0x20, 0x75, 0x57, 0x4b,  /* Codes 0-15 */
  0x16, 0x1e, 0x71, 0x14, 0x7f, 0x38, 0x3a, 0x5f, 0x65, 0x81, 0x24, 0x28, 0x5b, 0x5d, 0x2c, 0x53,  /* Codes 16-31 */
  0x34, 0x6d, 0x18, 0x61, 0x1a, 0x63, 0x56, 0x4e, 0x3e, 0x22, 0x4a, 0x6f, 0x79, 0x36, 0x2e, 0x7b,  /* Codes 32-47 */
  0x26, 0x30, 0x67, 0x85, 0xc2,  /* Codes 48-52 */
}

static const uint8_t g_lcdClockBrightLut[BITMAP_NLUTCODES] =
{
  0xa4, 0x9d, 0x8e, 0xa1, 0x95, 0x7a, 0x74, 0x73, 0x64, 0x4d, 0x98, 0x70, 0x57, 0x97, 0x80, 0x77,  /* Codes 0-15 */
  0x50, 0x56, 0x94, 0x4e, 0x9e, 0x69, 0x6a, 0x86, 0x8b, 0xa0, 0x5a, 0x5d, 0x83, 0x85, 0x60, 0x7d,  /* Codes 16-31 */
  0x66, 0x91, 0x51, 0x88, 0x53, 0x89, 0x7f, 0x79, 0x6d, 0x59, 0x76, 0x92, 0x9a, 0x67, 0x61, 0x9b,  /* Codes 32-47 */
  0x5c, 0x63, 0x8c, 0xa3, 0xd1,  /* Codes 48-52 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */
// RGB8 (332) Colors

static const nxgl_mxpixel_t g_lcdClockNormalLut[BITMAP_NLUTCODES] =
{
  0x92, 0x71, 0x6d, 0x72, 0x6d, 0x49, 0x29, 0x28, 0x24, 0x04, 0x6d, 0x28, 0x04, 0x6d, 0x4d, 0x49,  /* Codes 0-15 */
  0x04, 0x04, 0x6d, 0x04, 0x71, 0x28, 0x28, 0x4d, 0x4d, 0x71, 0x04, 0x04, 0x4d, 0x4d, 0x04, 0x49,  /* Codes 16-31 */
  0x28, 0x6d, 0x04, 0x4d, 0x04, 0x4d, 0x4d, 0x49, 0x28, 0x04, 0x29, 0x6d, 0x71, 0x28, 0x24, 0x71,  /* Codes 32-47 */
  0x04, 0x24, 0x4d, 0x72, 0xd8,  /* Codes 48-52 */
};

static const uint8_t g_lcdClockBrightLut[BITMAP_NLUTCODES] =
{
  0xb6, 0x96, 0x92, 0x96, 0x92, 0x71, 0x6d, 0x6d, 0x4d, 0x49, 0x92, 0x6d, 0x4d, 0x92, 0x71, 0x71,  /* Codes 0-15 */
  0x49, 0x4d, 0x92, 0x49, 0x96, 0x4d, 0x6d, 0x92, 0x92, 0x96, 0x4d, 0x4d, 0x71, 0x92, 0x4d, 0x71,  /* Codes 16-31 */
  0x4d, 0x92, 0x49, 0x92, 0x49, 0x92, 0x71, 0x71, 0x6d, 0x4d, 0x71, 0x92, 0x96, 0x4d, 0x4d, 0x96,  /* Codes 32-47 */
  0x4d, 0x4d, 0x92, 0x96, 0xfd,  /* Codes 48-52 */
};

#  endif
#else
#  error Unsupported pixel format
#endif

static const struct SRlePaletteBitmapEntry g_lcdClockRleEntries[] =
{
  {102,   0},                                                                                        // Row 0
  {102,   0},                                                                                        // Row 1
  {102,   0},                                                                                        // Row 2
  { 23,   0}, {  1,   1}, { 12,   2}, {  1,   3}, { 21,   0}, {  1,   4}, { 12,   2}, { 31,   0},    // Row 3
  { 13,   0}, {  1,   5}, {  1,   6}, {  7,   0}, {  1,   7}, {  1,   8}, { 12,   9}, {  1,  10},    // Row 4
  {  2,  11}, {  1,   3}, { 17,   0}, {  1,  11}, {  1,  12}, { 12,   9}, {  1,  13}, {  1,   2},
  {  8,   0}, {  1,   5}, { 12,  11}, {  8,   0},
  { 12,   0}, {  1,  14}, {  2,   9}, {  1,  15}, {  5,   0}, {  1,   7}, { 14,   9}, {  1,  10},    // Row 5
  {  2,   9}, {  1,  16}, { 16,   0}, {  1,  17}, { 13,   9}, {  1,  18}, {  1,   2}, {  1,  19},
  {  1,  20}, {  6,   0}, {  1,  14}, { 13,   9}, {  1,   0}, {  1,  21}, {  1,  17}, {  5,   0},
  { 11,   0}, {  1,   3}, {  1,  11}, {  3,   9}, {  1,  18}, {  5,   0}, {  1,  22}, { 11,   9},    // Row 6
  {  1,  16}, {  1,  23}, {  1,  17}, {  3,   9}, { 16,   0}, {  1,  22}, { 11,   9}, {  1,  16},
  {  1,  23}, {  1,  24}, {  3,   9}, {  5,   0}, {  1,  25}, {  1,  26}, { 11,   9}, {  1,  27},
  {  1,   0}, {  1,  28}, {  2,   9}, {  1,  11}, {  4,   0},
  { 11,   0}, {  1,  20}, {  4,   9}, {  1,  18}, {  6,   0}, {  1,  29}, {  1,  30}, {  9,   9},    // Row 7
  {  1,  26}, {  1,  29}, {  1,  16}, {  3,   9}, { 17,   0}, {  1,  29}, {  1,  30}, {  9,   9},
  {  1,  26}, {  1,   0}, {  1,  17}, {  3,   9}, {  5,   0}, {  1,  22}, {  1,   6}, {  1,  29},
  {  1,  31}, {  9,   9}, {  1,  28}, {  1,   0}, {  1,  30}, {  2,   9}, {  1,  11}, {  4,   0},
  { 10,   0}, {  1,   3}, {  4,   9}, {  1,   6}, { 10,   0}, {  8,   9}, {  1,   0}, {  1,   6},    // Row 8
  {  4,   9}, { 19,   0}, {  9,   9}, {  1,   0}, {  1,   6}, {  4,   9}, {  4,   0}, {  1,  32},
  {  2,   9}, {  1,   2}, {  1,   0}, {  1,   5}, {  7,   9}, {  1,  32}, {  1,   0}, {  1,   3},
  {  3,   9}, {  1,  11}, {  4,   0},
  { 10,   0}, {  1,  32}, {  4,   9}, {  1,   6}, { 18,   0}, {  1,  18}, {  5,   9}, { 28,   0},    // Row 9
  {  1,  20}, {  1,  32}, {  4,   9}, {  4,   0}, {  1,  32}, {  3,   9}, {  1,  28}, {  1,  33},
  {  8,   0}, {  1,   2}, {  4,   9}, {  1,  11}, {  4,   0},
  { 10,   0}, {  1,  32}, {  4,   9}, {  1,   6}, { 18,   0}, {  1,  30}, {  4,   9}, {  1,  23},    // Row 10
  { 28,   0}, {  1,  18}, {  5,   9}, {  4,   0}, {  1,  32}, {  4,   9}, {  1,   6}, {  8,   0},
  {  1,  22}, {  4,   9}, {  1,  11}, {  4,   0},
  { 10,   0}, {  1,  32}, {  4,   9}, {  1,   6}, { 18,   0}, {  5,   9}, {  1,  20}, { 28,   0},    // Row 11
  {  1,  18}, {  4,   9}, {  1,  20}, {  4,   0}, {  1,  32}, {  3,   9}, {  1,  17}, {  9,   0},
  {  1,  22}, {  3,   9}, {  1,  34}, {  5,   0},
  { 10,   0}, {  1,  32}, {  3,   9}, {  1,  16}, {  1,  35}, { 18,   0}, {  5,   9}, {  1,  20},    // Row 12
  { 28,   0}, {  1,   6}, {  4,   9}, {  1,  20}, {  4,   0}, {  1,  32}, {  3,   9}, {  1,  17},
  {  9,   0}, {  1,  22}, {  3,   9}, {  1,  34}, {  5,   0},
  {  9,   0}, {  1,  24}, {  1,  34}, {  3,   9}, {  1,  17}, { 19,   0}, {  5,   9}, {  1,  20},    // Row 13
  { 28,   0}, {  5,   9}, {  1,  20}, {  3,   0}, {  1,  24}, {  1,  34}, {  3,   9}, {  1,  17},
  {  8,   0}, {  1,   2}, {  1,  36}, {  3,   9}, {  1,  34}, {  5,   0},
  {  9,   0}, {  1,  28}, {  4,   9}, {  1,  17}, { 19,   0}, {  5,   9}, {  1,  20}, { 28,   0},    // Row 14
  {  5,   9}, {  1,  20}, {  3,   0}, {  1,  28}, {  4,   9}, {  1,  17}, {  8,   0}, {  1,  37},
  {  4,   9}, {  1,  34}, {  5,   0},
  {  9,   0}, {  1,  28}, {  4,   9}, {  1,  17}, { 19,   0}, {  4,   9}, {  1,   8}, {  1,   3},    // Row 15
  { 28,   0}, {  5,   9}, {  1,  20}, {  3,   0}, {  1,  28}, {  4,   9}, {  1,  17}, {  8,   0},
  {  1,  37}, {  4,   9}, {  1,  34}, {  5,   0},
  {  9,   0}, {  1,  28}, {  4,   9}, {  1,  17}, { 18,   0}, {  1,  32}, {  4,   9}, {  1,  38},    // Row 16
  { 29,   0}, {  5,   9}, {  1,  20}, {  3,   0}, {  1,  28}, {  4,   9}, {  1,  17}, {  8,   0},
  {  1,  37}, {  4,   9}, {  1,  34}, {  5,   0},
  {  9,   0}, {  1,  39}, {  4,   9}, {  1,  32}, { 18,   0}, {  1,  26}, {  4,   9}, {  1,  38},    // Row 17
  { 28,   0}, {  1,  26}, {  4,   9}, {  1,  38}, {  4,   0}, {  1,  39}, {  4,   9}, {  1,  32},
  {  8,   0}, {  1,  31}, {  4,   9}, {  6,   0},
  {  9,   0}, {  5,   9}, { 19,   0}, {  1,  26}, {  4,   9}, {  1,  38}, { 28,   0}, {  1,  26},    // Row 18
  {  4,   9}, {  1,  38}, {  4,   0}, {  5,   9}, {  9,   0}, {  5,   9}, {  6,   0},
  {  9,   0}, {  5,   9}, { 19,   0}, {  1,  26}, {  4,   9}, {  1,  38}, { 28,   0}, {  1,  26},    // Row 19
  {  4,   9}, {  1,  38}, {  4,   0}, {  5,   9}, {  9,   0}, {  5,   9}, {  6,   0},
  {  9,   0}, {  5,   9}, { 19,   0}, {  1,  26}, {  4,   9}, {  1,  38}, { 28,   0}, {  1,  26},    // Row 20
  {  4,   9}, {  1,  38}, {  4,   0}, {  5,   9}, {  9,   0}, {  5,   9}, {  6,   0},
  {  9,   0}, {  1,  40}, {  4,   9}, { 19,   0}, {  1,  26}, {  3,   9}, {  1,  41}, {  1,   4},    // Row 21
  { 28,   0}, {  1,  26}, {  3,   9}, {  1,  41}, {  1,   4}, {  4,   0}, {  4,   9}, {  1,   6},
  {  9,   0}, {  5,   9}, {  6,   0},
  {  9,   0}, {  1,   3}, {  1,   8}, {  3,   9}, {  9,   0}, { 10,  19}, {  2,  33}, {  2,   9},    // Row 22
  {  1,  30}, { 19,   0}, {  1,  24}, {  8,  19}, {  1,  42}, {  1,   3}, {  3,   9}, {  1,  30},
  {  4,   0}, {  1,  19}, {  2,   9}, {  1,  16}, {  2,   3}, {  8,  19}, {  1,  20}, {  1,  35},
  {  4,   9}, {  6,   0},
  { 11,   0}, {  1,  32}, {  2,   9}, {  6,   0}, {  1,   1}, {  1,   8}, { 12,   9}, {  1,  23},    // Row 23
  {  1,  42}, {  1,  16}, {  1,  30}, { 17,   0}, {  1,  43}, {  1,  23}, { 11,   9}, {  1,  24},
  {  1,  28}, {  1,  16}, {  1,  30}, {  4,   0}, {  2,   9}, {  1,  30}, {  1,   3}, {  1,  40},
  { 10,   9}, {  1,  27}, {  1,  44}, {  1,  45}, {  1,   9}, {  1,  37}, {  6,   0},
  { 11,   0}, {  1,   3}, {  2,  23}, {  6,   0}, {  1,  46}, { 14,   9}, {  1,   5}, {  1,  24},    // Row 24
  {  1,   2}, { 16,   0}, {  1,  45}, {  1,  12}, { 12,   9}, {  1,  16}, {  1,  23}, {  1,  24},
  {  1,   2}, {  4,   0}, {  2,  23}, {  1,  11}, {  1,  32}, { 12,   9}, {  1,  27}, {  1,  45},
  {  1,  23}, {  1,  47}, {  6,   0},
  { 10,   0}, {  1,   3}, {  1,  47}, {  1,   9}, {  1,   2}, {  3,   0}, {  1,  18}, {  2,   9},    // Row 25
  {  1,   3}, {  1,  11}, { 12,   9}, {  1,  27}, { 19,   0}, {  1,   2}, { 13,   9}, {  1,  27},
  {  1,   3}, {  1,  17}, {  1,  30}, {  6,   0}, {  1,  45}, { 13,   9}, {  1,  29}, {  1,   3},
  {  1,  28}, {  1,  37}, {  6,   0},
  {  9,   0}, {  1,  14}, {  1,  48}, {  2,   9}, {  1,   2}, {  3,   0}, {  1,  18}, {  2,   9},    // Row 26
  {  1,  41}, {  1,   4}, {  1,  11}, {  9,   9}, {  1,  48}, {  1,  44}, { 21,   0}, {  1,  28},
  {  1,  40}, {  8,   9}, {  1,  16}, {  1,  40}, {  1,  44}, {  1,  15}, {  2,   9}, {  1,  30},
  {  7,   0}, {  1,  22}, { 10,   9}, {  1,  22}, {  1,   7}, {  1,  14}, {  2,   9}, {  1,  37},
  {  6,   0},
  {  8,   0}, {  1,  49}, {  4,   9}, {  1,   2}, {  3,   0}, {  1,  18}, {  3,   9}, {  1,  26},    // Row 27
  {  1,  49}, {  9,  50}, {  1,  13}, { 24,   0}, {  8,  50}, {  1,   2}, {  1,  28}, {  1,  49},
  {  3,   9}, {  1,  30}, {  7,   0}, {  1,   3}, { 10,  50}, {  1,   8}, {  1,  27}, {  3,   9},
  {  1,  37}, {  6,   0},
  {  7,   0}, {  1,  22}, {  5,   9}, {  1,   2}, {  3,   0}, {  1,  18}, {  5,   9}, { 42,   0},    // Row 28
  {  1,  10}, {  5,   9}, { 18,   0}, {  1,  11}, {  5,   9}, {  1,  37}, {  6,   0},
  {  7,   0}, {  1,  22}, {  4,   9}, {  1,  27}, {  1,  13}, {  3,   0}, {  1,   7}, {  5,   9},    // Row 29
  { 42,   0}, {  1,  10}, {  5,   9}, { 18,   0}, {  1,  11}, {  5,   9}, {  1,  37}, {  6,   0},
  {  7,   0}, {  1,  22}, {  4,   9}, {  1,  11}, {  4,   0}, {  6,   9}, { 42,   0}, {  1,  10},    // Row 30
  {  5,   9}, { 18,   0}, {  1,  11}, {  4,   9}, {  1,   8}, {  1,  20}, {  6,   0},
  {  7,   0}, {  1,  22}, {  4,   9}, {  1,  11}, {  4,   0}, {  5,   9}, {  1,  20}, { 42,   0},    // Row 31
  {  1,  10}, {  5,   9}, { 18,   0}, {  1,  11}, {  4,   9}, {  1,  22}, {  7,   0},
  {  7,   0}, {  1,  22}, {  4,   9}, {  1,  11}, {  4,   0}, {  5,   9}, {  1,  20}, { 42,   0},    // Row 32
  {  1,  11}, {  5,   9}, { 18,   0}, {  1,  11}, {  4,   9}, {  1,  22}, {  7,   0},
  {  6,   0}, {  1,  50}, {  1,  16}, {  4,   9}, {  1,  11}, {  4,   0}, {  5,   9}, {  1,  20},    // Row 33
  { 42,   0}, {  6,   9}, { 18,   0}, {  1,  11}, {  4,   9}, {  1,  22}, {  7,   0},
  {  6,   0}, {  1,  37}, {  5,   9}, {  1,  11}, {  4,   0}, {  5,   9}, {  1,  20}, { 42,   0},    // Row 34
  {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9}, {  1,  22}, {  7,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  16}, {  1,   2}, {  3,   0}, {  1,   5}, {  5,   9},    // Row 35
  {  1,  20}, { 42,   0}, {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9}, {  1,  24},
  {  7,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  34}, {  4,   0}, {  1,  26}, {  5,   9}, {  1,  20},    // Row 36
  { 42,   0}, {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9}, {  8,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  34}, {  4,   0}, {  1,  26}, {  4,   9}, {  1,  38},    // Row 37
  { 43,   0}, {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9}, {  8,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  34}, {  4,   0}, {  1,  26}, {  4,   9}, {  1,  38},    // Row 38
  { 43,   0}, {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9}, {  8,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  34}, {  4,   0}, {  1,  26}, {  2,   9}, {  1,  10},    // Row 39
  {  1,   3}, {  1,  51}, { 43,   0}, {  5,   9}, {  1,  10}, { 17,   0}, {  1,   2}, {  5,   9},
  {  8,   0},
  {  6,   0}, {  1,  37}, {  4,   9}, {  1,  34}, {  4,   0}, {  1,   7}, {  1,  20}, {  2,  29},    // Row 40
  {  1,   9}, {  1,  40}, {  8,  29}, {  1,  47}, { 21,   0}, {  1,  18}, { 11,  29}, {  1,  47},
  {  1,  32}, {  3,   9}, {  1,  48}, {  1,   1}, {  6,   0}, { 10,  29}, {  1,  25}, {  1,   4},
  {  1,  12}, {  4,   9}, {  8,   0},
  {  6,   0}, {  1,  47}, {  1,  46}, {  3,   9}, {  1,  24}, {  4,   0}, {  1,  11}, {  1,  46},    // Row 41
  { 12,   9}, {  1,   2}, { 19,   0}, {  1,  11}, {  1,  32}, {  1,  12}, { 11,   9}, {  1,  30},
  {  1,  10}, {  3,   9}, {  1,   5}, {  4,   0}, {  1,  33}, {  2,  32}, { 10,   9}, {  1,  49},
  {  1,   6}, {  1,  11}, {  4,   9}, {  8,   0},
  {  7,   0}, {  1,  40}, {  2,   9}, {  1,  16}, {  5,   0}, {  1,  26}, { 14,   9}, { 19,   0},    // Row 42
  {  1,  26}, { 14,   9}, {  1,   4}, {  1,  19}, {  2,   9}, {  1,   5}, {  4,   0}, { 14,   9},
  {  1,  30}, {  1,   3}, {  1,  34}, {  2,   9}, {  9,   0},
  {  8,   0}, {  2,   9}, {  1,  37}, {  5,   0}, {  1,  26}, { 14,   9}, {  1,  28}, { 18,   0},    // Row 43
  {  1,  15}, { 14,   9}, {  1,  17}, {  1,   5}, {  1,   9}, {  1,  36}, {  1,  24}, {  4,   0},
  {  1,   8}, { 14,   9}, {  1,  38}, {  1,  34}, {  1,   9}, {  1,  49}, {  9,   0},
  {  8,   0}, {  1,   2}, {  1,  49}, {  1,   1}, {  5,   0}, {  1,  33}, { 14,   9}, {  1,  17},    // Row 44
  { 19,   0}, {  1,  28}, { 13,   9}, {  1,  19}, {  1,  12}, {  1,   9}, {  1,  33}, {  5,   0},
  {  1,  37}, { 14,   9}, {  1,  12}, {  1,   9}, {  1,  30}, {  1,  47}, {  9,   0},
  { 18,   0}, {  1,   5}, { 13,  17}, {  1,   5}, { 18,   0}, {  1,   4}, {  2,  17}, { 12,   9},    // Row 45
  {  1,   5}, {  9,   0}, {  1,  17}, { 13,   9}, {  1,  20}, { 11,   0},
  { 54,   0}, { 11,   6}, {  1,   5}, { 11,   0}, { 13,   6}, {  1,   3}, { 11,   0},                // Row 46
  {  1,  52}, {101,   0},                                                                            // Row 47
 };

/********************************************************************************************
 * Public Bitmap Structure Definitions
 ********************************************************************************************/

const struct SRlePaletteBitmap NXWidgets::g_lcdClockBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_lcdClockNormalLut, //          Index 0: Unselected LUT
    g_lcdClockBrightLut, //          Index 1: Selected LUT
  },
  g_lcdClockRleEntries   // data   - Pointer to the beginning of the RLE data
};
