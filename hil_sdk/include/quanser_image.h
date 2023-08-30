#if !defined(_quanser_image_h)
#define _quanser_image_h

typedef enum tag_image_data_type
{
    IMAGE_DATA_TYPE_UINT8,
    IMAGE_DATA_TYPE_UINT16,
    IMAGE_DATA_TYPE_UINT32,
    IMAGE_DATA_TYPE_SINGLE,
    IMAGE_DATA_TYPE_DOUBLE,

    NUMBER_OF_IMAGE_DATA_TYPES
} t_image_data_type;

typedef enum tag_image_colur_format
{
    IMAGE_COLOR_FORMAT_MONOCHROME,   /* greyscale image */
    IMAGE_COLOR_FORMAT_COLOR,        /* colour image */

    NUMBER_OF_COLOR_FORMATS
} t_image_color_format;

#endif

