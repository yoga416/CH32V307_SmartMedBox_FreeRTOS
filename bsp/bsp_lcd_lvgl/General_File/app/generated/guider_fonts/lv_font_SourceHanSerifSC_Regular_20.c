/*******************************************************************************
 * 字体大小: 20 px
 * 位深: 4 位/像素 (bpp = 4)
 * 编译选项: 未定义
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl.h"
#include "spi_w25q.h"
#endif

#ifndef LV_FONT_SOURCEHANSERIFSC_REGULAR_20
#define LV_FONT_SOURCEHANSERIFSC_REGULAR_20 1
#endif

#if LV_FONT_SOURCEHANSERIFSC_REGULAR_20

/* 中文字体在 W25Q 闪存中的存储起始地址（0x000000） */
#define W25Q_FONT_OFFSET  0x000000

/* 字形数据缓冲区（512字节足够容纳大多数汉字字形） */
static uint8_t w25q_glyph_buf[512];

/**
 * @brief 自定义获取字形位图的回调函数
 * @param font  LVGL 字体结构体指针
 * @param unicode 要获取的字符的 Unicode 编码
 * @return 指向字形位图数据的指针，失败返回 NULL
 * 
 * @note 该函数在 LVGL 渲染每个字符时被调用，
 *       从 W25Q 外部闪存中读取对应的字形数据
 */
static const uint8_t *han_get_bitmap_cb(const lv_font_t *font, uint32_t unicode)
{
    static lv_font_glyph_dsc_t glyph;
    static int call_count = 0;

    call_count++;

    /* 获取字形的描述信息（宽度、高度、偏移等） */
    if(lv_font_get_glyph_dsc_fmt_txt(font, &glyph, unicode, unicode) == false) {
        if(call_count <= 5) printf("x HAN: U+%04lX not found\r\n", (unsigned long)unicode);
        return NULL;
    }

    /* 从字体描述符表中获取该字形的 bitmap_index（在字体文件中的偏移） */
    lv_font_fmt_txt_dsc_t *fdsc = (lv_font_fmt_txt_dsc_t *)font->dsc;
    uint32_t bitmap_index = fdsc->glyph_dsc[fdsc->cache->last_glyph_id].bitmap_index;

    /* 计算需要读取的数据量：宽度 x 高度 x 4位/像素 ÷ 8位/字节 */
    uint32_t read_len = glyph.box_w * glyph.box_h * 4 / 8;
    if(read_len == 0) read_len = 1;                    // 至少读取1字节
    if(read_len > sizeof(w25q_glyph_buf)) read_len = sizeof(w25q_glyph_buf);  // 不超过缓冲区大小

    /* 调试打印：前10次调用输出详细信息 */
    if(call_count <= 10) {
        printf("HAN U+%04lX: idx=%d box=%dx%d len=%d\r\n",
               (unsigned long)unicode, (int)bitmap_index,
               (int)glyph.box_w, (int)glyph.box_h, (int)read_len);
    }

    /* 从 W25Q 闪存的对应地址读取字形数据 */
    W25Q_ReadData(W25Q_FONT_OFFSET + bitmap_index, w25q_glyph_buf, read_len);
    return w25q_glyph_buf;
}

/*---------------------
 *  字形描述表 (GLYPH DESCRIPTION)
 * 每一项定义了字符的形状和显示信息
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    /* .bitmap_index = 位图数据在字体文件中的偏移地址 */
    /* .adv_w = 字符前进宽度（水平占位） */
    /* .box_w = 字形边界框宽度 */
    /* .box_h = 字形边界框高度 */
    /* .ofs_x = 字形在边界框中的X偏移 */
    /* .ofs_y = 字形在边界框中的Y偏移 */
    
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 保留 */,
    {.bitmap_index = 0, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 190, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 380, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 570, .adv_w = 320, .box_w = 17, .box_h = 19, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 732, .adv_w = 320, .box_w = 19, .box_h = 18, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 903, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 1103, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1293, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1493, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1693, .adv_w = 320, .box_w = 19, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1874, .adv_w = 320, .box_w = 21, .box_h = 18, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 2063, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2263, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 2453, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2653, .adv_w = 320, .box_w = 19, .box_h = 19, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 2834, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 3024, .adv_w = 320, .box_w = 19, .box_h = 18, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 3195, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 3385, .adv_w = 320, .box_w = 19, .box_h = 19, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 3566, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 3756, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 3946, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 4146, .adv_w = 320, .box_w = 20, .box_h = 20, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 4346, .adv_w = 320, .box_w = 19, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 4527, .adv_w = 320, .box_w = 20, .box_h = 18, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 4707, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 4897, .adv_w = 320, .box_w = 20, .box_h = 19, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 5087, .adv_w = 320, .box_w = 20, .box_h = 18, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 5267, .adv_w = 320, .box_w = 20, .box_h = 18, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 5447, .adv_w = 320, .box_w = 17, .box_h = 19, .ofs_x = 2, .ofs_y = -2}
};

/*---------------------
 *  字符映射表 (CHARACTER MAPPING)
 * 将 Unicode 编码映射到字形描述表中的索引
 *--------------------*/

/* Unicode 编码列表（要支持的字符的 Unicode 值） */
static const uint16_t unicode_list_0[] = {
    0x0, 0x146, 0x258, 0x378, 0x5f6, 0x6d9, 0x91b, 0x1099,
    0x10aa, 0x1148, 0x11b6, 0x1561, 0x1763, 0x17ac, 0x17e9, 0x1900,
    0x19d8, 0x1a3b, 0x1e1a, 0x201c, 0x2072, 0x2102, 0x257a, 0x271b,
    0x28c5, 0x28fe, 0x3562, 0x3a33, 0x3da3, 0x47e7
};

/* 字符映射表结构体 */
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 19981,                      /* Unicode 起始值 */
        .range_length = 18408,                     /* 范围长度 */
        .glyph_id_start = 1,                       /* 对应的字形起始ID */
        .unicode_list = unicode_list_0,            /* Unicode 列表 */
        .glyph_id_ofs_list = NULL,                 /* 字形偏移列表（NULL表示连续映射） */
        .list_length = 30,                         /* 列表长度 */
        .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY    /* 映射类型：稀疏映射 */
    }
};

/*--------------------
 *  字体自定义数据区
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/* LVGL v8 需要缓存来存储最近使用的字形 */
static lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = NULL,                          /* 位图数据为空（从外部读取） */
    .glyph_dsc = glyph_dsc,                        /* 字形描述表 */
    .cmaps = cmaps,                                /* 字符映射表 */
    .kern_dsc = NULL,                              /* 字距调整数据（无） */
    .kern_scale = 0,                               /* 字距缩放因子 */
    .cmap_num = 1,                                 /* 字符映射表数量 */
    .bpp = 4,                                      /* 每像素位数：4位（16级灰度） */
    .kern_classes = 0,                             /* 是否使用字距类别 */
    .bitmap_format = 0,                            /* 位图格式：0=原始位图 */
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache                                /* 字形缓存（LVGL v8） */
#endif
};

/*-----------------
 *  导出字体结构体
 *----------------*/

/* 初始化一个公共的通用字体描述符 */
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t lv_font_SourceHanSerifSC_Regular_20 = {
#else
lv_font_t lv_font_SourceHanSerifSC_Regular_20 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,   /* 获取字形描述信息的函数指针 */
    .get_glyph_bitmap = han_get_bitmap_cb,            /* 获取字形位图数据的函数指针 */
    .line_height = 20,                                /* 字体的最大行高 */
    .base_line = 3,                                   /* 基线位置（从行底向上测量） */
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,                      /* 子像素渲染：无 */
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,                         /* 下划线位置 */
    .underline_thickness = 1,                         /* 下划线粗细 */
#endif
    .dsc = &font_dsc,                                 /* 自定义字体数据指针 */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,                                 /* 后备字体（无） */
#endif
    .user_data = NULL,                                /* 用户自定义数据（无） */
};

#endif /*#if LV_FONT_SOURCEHANSERIFSC_REGULAR_20*/