/*******************************************************************************
 * Size: 12 px
 * Bpp: 4
 * Opts: undefined
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl.h"
#include "spi_w25q.h"
#endif

#ifndef LV_CUSTOMER_FONT_MONTSERRATMEDIUM_12
#define LV_CUSTOMER_FONT_MONTSERRATMEDIUM_12 1
#endif

#if LV_CUSTOMER_FONT_MONTSERRATMEDIUM_12
#define W25Q_FONT_OFFSET  0x020000

static uint8_t w25q_glyph_buf[256];

static const uint8_t *m12_get_bitmap_cb(const lv_font_t *font, uint32_t unicode)
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
/* Bitmap data stored in W25Q external flash */  // ������
/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 71, .box_w = 4, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 18, .adv_w = 110, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 50, .adv_w = 110, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 82, .adv_w = 128, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 118, .adv_w = 110, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 49, .range_length = 5, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 0, 1, 2, 3, 4
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 1, 2, 3, 4, 5
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    0, 0, 0, -4, 0, 0, -1, -1,
    0, -2, -5, -1, -2, 0, -1, 0,
    -1, -1, 0, -1
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 4,
    .right_class_cnt     = 5,
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
const lv_font_t lv_font_montserratMedium_12 = {
#else
lv_font_t lv_font_montserratMedium_12 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = m12_get_bitmap_cb,    /*Function pointer to get glyph's bitmap*/
    .line_height = 12,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
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



#endif /*#if LV_CUSTOMER_FONT_MONTSERRATMEDIUM_12*/

