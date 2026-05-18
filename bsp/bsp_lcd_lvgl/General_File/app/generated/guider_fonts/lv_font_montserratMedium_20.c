/*******************************************************************************
 * Size: 20 px
 * Bpp: 4
 * Opts: undefined
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl.h"
#include "spi_w25q.h"
#endif

#ifndef LV_CUSTOMER_FONT_MONTSERRATMEDIUM_20
#define LV_CUSTOMER_FONT_MONTSERRATMEDIUM_20 1
#endif

#if LV_CUSTOMER_FONT_MONTSERRATMEDIUM_20
#define W25Q_FONT_OFFSET  0x030000

static uint8_t w25q_glyph_buf[256];

static const uint8_t *m20_get_bitmap_cb(const lv_font_t *font, uint32_t unicode)
{
    static lv_font_glyph_dsc_t glyph;
    
    if(lv_font_get_glyph_dsc_fmt_txt(font, &glyph, unicode, unicode) == false) {
        return NULL;
    }

    /* 从字体内部描述符表中获取 bitmap_index */
    lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;
    uint32_t bitmap_index = fdsc->glyph_dsc[fdsc->cache->last_glyph_id].bitmap_index;

    uint32_t read_len = glyph.box_w * glyph.box_h * 4 / 8;
    if(read_len == 0) read_len = 1;
    if(read_len > sizeof(w25q_glyph_buf)) read_len = sizeof(w25q_glyph_buf);

    W25Q_ReadData(W25Q_FONT_OFFSET + bitmap_index, w25q_glyph_buf, read_len);
    return w25q_glyph_buf;
}
/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 86, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 270, .box_w = 17, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 119, .adv_w = 123, .box_w = 6, .box_h = 2, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 125, .adv_w = 73, .box_w = 4, .box_h = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 131, .adv_w = 113, .box_w = 9, .box_h = 20, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 221, .adv_w = 213, .box_w = 13, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 312, .adv_w = 118, .box_w = 6, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 354, .adv_w = 184, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 431, .adv_w = 183, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 508, .adv_w = 214, .box_w = 14, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 606, .adv_w = 184, .box_w = 11, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 683, .adv_w = 197, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 767, .adv_w = 191, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 851, .adv_w = 206, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 935, .adv_w = 197, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1019, .adv_w = 73, .box_w = 4, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1041, .adv_w = 231, .box_w = 14, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1139, .adv_w = 203, .box_w = 10, .box_h = 14, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1209, .adv_w = 190, .box_w = 10, .box_h = 14, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1279, .adv_w = 260, .box_w = 13, .box_h = 14, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1370, .adv_w = 188, .box_w = 12, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1454, .adv_w = 360, .box_w = 22, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1608, .adv_w = 191, .box_w = 10, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1663, .adv_w = 218, .box_w = 12, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1753, .adv_w = 183, .box_w = 11, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1814, .adv_w = 196, .box_w = 12, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1880, .adv_w = 221, .box_w = 12, .box_h = 15, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 1970, .adv_w = 89, .box_w = 4, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2000, .adv_w = 197, .box_w = 12, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2090, .adv_w = 338, .box_w = 19, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2195, .adv_w = 218, .box_w = 11, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2256, .adv_w = 203, .box_w = 12, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2322, .adv_w = 131, .box_w = 7, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2361, .adv_w = 132, .box_w = 8, .box_h = 14, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2417, .adv_w = 217, .box_w = 11, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2478, .adv_w = 177, .box_w = 11, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2539, .adv_w = 134, .box_w = 8, .box_h = 8, .ofs_x = 0, .ofs_y = 7}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x5, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12,
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a,
    0x23, 0x26, 0x2c, 0x2e, 0x34, 0x37, 0x41, 0x42,
    0x43, 0x45, 0x47, 0x49, 0x4b, 0x4d, 0x4e, 0x4f,
    0x52, 0x54, 0x55, 0x58, 0x90
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 145, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 37, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 0, 1, 2, 3, 4, 5, 0,
    6, 7, 8, 9, 10, 11, 12, 5,
    13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 24, 25, 20, 20,
    21, 26, 27, 24, 28, 29
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 0, 1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 5, 11, 12, 13,
    14, 15, 16, 16, 16, 17, 18, 19,
    20, 21, 21, 21, 22, 20, 23, 23,
    21, 23, 24, 25, 26, 27
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    -39, 6, 10, 0, 0, -6, 3, 3,
    11, 6, -5, 6, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -9, -12, 1, -2, 0, 2,
    -6, -4, -6, 2, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -20, -2,
    0, 32, -4, -4, 3, 3, -3, 0,
    -4, 3, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -20, 0, -22, -31, -22, -6, 10, 0,
    0, -21, 0, 4, -7, 0, -5, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 15, 0, 2, -4, -6,
    0, -1, -1, -3, 0, 0, -2, 0,
    0, 0, 0, 0, -6, -8, 0, 0,
    0, 0, 0, 3, 0, -3, 1, 3,
    -2, 3, 3, -1, 0, 0, 0, -6,
    0, -1, 0, 0, 0, 0, 0, -2,
    -4, 0, 0, -3, 0, 0, 0, -1,
    -2, 0, -3, 0, 0, 0, 0, 0,
    -2, -2, 0, -3, -4, 0, 0, 0,
    0, 0, -2, -3, 0, 0, 0, 0,
    0, 0, 0, -4, -5, -10, 3, 6,
    9, 0, -8, -1, -4, 0, -1, -15,
    3, -2, 2, 3, 0, -17, -17, 9,
    0, 4, 0, 0, 1, 0, -4, -10,
    -3, 0, 0, 0, 0, 0, -2, -2,
    0, -2, -4, 0, 0, 0, 0, 0,
    -3, -2, 0, 0, 0, 0, 0, 0,
    0, -3, -4, -6, 2, 3, 3, 0,
    0, 0, 0, 0, 0, -2, 0, 0,
    0, 0, 0, -3, -3, 0, 0, 3,
    0, 0, 0, 0, -5, -6, 1, -16,
    -17, -13, -6, 3, 0, -3, -21, -6,
    0, -6, 0, -6, -6, 0, 0, 2,
    -12, 0, -16, -8, -8, -4, -9, -6,
    2, -3, 0, 3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -3, -5, 0, 0, 0, 0, 0,
    0, 0, -2, 0, 0, 0, 0, 15,
    0, 0, 0, 0, 0, 0, 2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -1, 4, 0, -7, 3, -2, -1, -8,
    -3, 0, -4, -3, -2, -5, 0, -3,
    -2, -2, 0, -7, 0, 0, 0, -5,
    -5, 3, 0, 0, -3, -13, -4, 5,
    0, 0, -15, -5, 3, -5, 2, 0,
    -3, 0, 2, 0, -6, 0, -5, -3,
    -3, 0, -5, -6, 10, 0, -3, 10,
    7, -7, -12, 0, 1, -10, 0, -16,
    -2, -3, 6, -4, 0, -21, -17, 1,
    0, -2, 0, 0, -2, -2, -2, -21,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -11, -10, -6, -6,
    13, 6, 3, -28, -2, 6, -3, 0,
    -3, -3, 0, 3, -3, -9, 0, -18,
    -4, -4, 1, -4, -11, 13, 0, -7,
    -12, -13, -8, 10, 0, 1, -23, -3,
    3, -5, -2, -7, -7, -5, -3, 0,
    -18, 0, -18, -4, -11, -1, -10, -11,
    9, -9, 0, 0, 0, 0, -7, -2,
    0, 0, 0, -7, 0, -4, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, -6, -9, 3, -4, 0,
    0, -9, -3, -7, 0, 0, -9, 0,
    -3, 0, 0, 0, -26, -13, -2, 0,
    0, 0, 0, 0, 0, -6, -5, 0,
    -4, 6, 0, -2, -7, -2, -5, -6,
    0, -4, -2, -2, 2, -1, 0, -28,
    -4, 0, -2, -3, 0, 0, 2, 0,
    -5, 5, -2, 3, 0, 0, 0, -9,
    -3, -6, 0, 0, -9, 0, -3, 0,
    0, 0, -31, -6, -5, 0, 0, 0,
    0, 0, 0, -5, -5, 0, 0, 0,
    0, 0, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -27, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -7, 3, 0, -3, 3, 6, 3,
    -10, 0, -1, -3, 3, 0, 0, 0,
    -8, -3, -3, 0, -7, -2, -2, 0,
    -3, -6, 0, 0, -3, -3, -13, 1,
    -2, 1, -2, -9, 1, 7, 1, 3,
    1, -8, -4, -12, -9, -3, -2, -4,
    -2, -2, 5, 0, -2, 10, 0, -3,
    3, 0, -5, -6, -2, 0, -9, -2,
    -7, -2, -4, 0, 0, 0, 0, 0,
    0, 0, -6, 0, 0, -4, 0, 0,
    -2, 0, -7, 0, 3, -3, 3, 0,
    0, -11, 0, -2, -1, 0, -3, -3,
    0, -13, -7, -3, 0, -6, 0, 0,
    0, -3, 0, 3, 7, 0, -20, -18,
    1, 14, 10, 5, -13, 2, 13, 0,
    12, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 29,
    .right_class_cnt     = 27,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = NULL,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_classes,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t lv_font_montserratMedium_20 = {
#else
lv_font_t lv_font_montserratMedium_20 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = m20_get_bitmap_cb,    /*Function pointer to get glyph's bitmap*/
    .line_height = 20,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if LV_CUSTOMER_FONT_MONTSERRATMEDIUM_20*/

