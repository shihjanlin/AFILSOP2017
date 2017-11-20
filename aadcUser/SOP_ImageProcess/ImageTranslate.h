

//#define S_IMGW 320
//#define S_IMGH 240
#include "Algorithm/CameraEnvironment.h"


#define IMAGE_WIDTH  S_IMGW
#define IMAGE_HEIGHT S_IMGH

typedef struct
{
	unsigned char Y[IMAGE_WIDTH * IMAGE_HEIGHT];
	unsigned char U[(IMAGE_WIDTH / 2) * IMAGE_HEIGHT];
	unsigned char V[(IMAGE_WIDTH / 2) * IMAGE_HEIGHT];

}IMAGE_BUFFER;

typedef struct
{
	long video_input_buffer_width;
	long video_input_buffer_height;
	//RGB24
	unsigned int rgb24_down_sample_col_table[IMAGE_WIDTH];
	unsigned long rgb24_down_sample_row_table[IMAGE_WIDTH];
	//YUY2
	unsigned int YUY2_down_sample_col_table_Y[IMAGE_WIDTH];
	unsigned int YUY2_down_sample_col_table_UandV[IMAGE_WIDTH/2];
	unsigned long YUY2_down_sample_row_table[IMAGE_WIDTH];

}IMAGE_TRANSLATE;



unsigned char uCharLimitSet(int);
void DownsamplingArrayPrepare_YUY2(int, int, IMAGE_TRANSLATE *);
void ImageBufferDownsamplingYUY2_to_YUV(int, int, unsigned char *, IMAGE_BUFFER *, IMAGE_TRANSLATE *);
void DownsamplingArrayPrepare_RGB24(int, int, IMAGE_TRANSLATE *);
void ImageBufferDownsamplingRGB24_to_YUV(int, int, unsigned char*,IMAGE_BUFFER *, IMAGE_TRANSLATE *, char);



void YUVtoRGB(int*, int*, int*, int, int, int);
void ImageBufferYUV_to_RGB24(int, int, int, unsigned char*, IMAGE_BUFFER *);
void ImageBufferYUV_to_Gray_scale(int Im_width, int Im_height, int row_size, unsigned char *bits, IMAGE_BUFFER *buffer);
void ImageBufferUpsampling_RGB24(int, int, int, int, int, unsigned char*, IMAGE_BUFFER *);

