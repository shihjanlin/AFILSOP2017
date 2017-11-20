//---------------------------------------------------------------------------


#include "ImageTranslate.h"
#include<math.h>
//---------------------------------------------------------------------------


unsigned char uCharLimitSet(int value)
{
	if(value >255)
		return (unsigned char)255;
	else if(value < 0)
		return (unsigned char)0;
	else
		return (unsigned char)value;
}

void DownsamplingArrayPrepare_YUY2(int Im_width, int Im_height, IMAGE_TRANSLATE *im_T)
{
	// YUY2 buffer sequence  YUYVYUYV
	int row, col;
	int index = 0;
	double video_width_sample_frequency = 0;
	double video_height_sample_frequency = 0;

	video_width_sample_frequency = (double)((double)im_T->video_input_buffer_width / (double)Im_width);
	video_height_sample_frequency = (double)((double)im_T->video_input_buffer_height / (double)Im_height);

	for(col = 0; col < Im_width; col++)
	{

		im_T->YUY2_down_sample_col_table_Y[col] = int(int(video_width_sample_frequency * col) * 2);
		if(col % 2 == 0)
		{
			im_T->YUY2_down_sample_col_table_UandV[index] = int(int(video_width_sample_frequency * index) * 4);
			index++;
		}
	}
	for(row = 0; row < Im_height; row++)
	{
		im_T->YUY2_down_sample_row_table[row] = int(int(video_height_sample_frequency * row) * im_T->video_input_buffer_width * 2);
	}
}

void ImageBufferDownsamplingYUY2_to_YUV(int Im_width, int Im_height, unsigned char *Im_buffer, IMAGE_BUFFER *VinSource, IMAGE_TRANSLATE *im_T)
{
	int row, col;
	int index = 0;
	int index2 = 0;
	int index3 = 0;
	for(row = 0; row < Im_height; row++)
	{
		for(col = 0; col < Im_width; col++)
		{
			VinSource->Y[index] = (unsigned char)Im_buffer[im_T->YUY2_down_sample_row_table[row] + im_T->YUY2_down_sample_col_table_Y[col]];
			index++;
			if(index % 2 == 0)
			{
				VinSource->U[index2] = (unsigned char)Im_buffer[im_T->YUY2_down_sample_row_table[row] + im_T->YUY2_down_sample_col_table_UandV[index3] + 1];
				VinSource->V[index2] = (unsigned char)Im_buffer[im_T->YUY2_down_sample_row_table[row] + im_T->YUY2_down_sample_col_table_UandV[index3] + 3];
				index2++;
				index3++;
			}
		}
		index3 = 0;
	}
}



void DownsamplingArrayPrepare_RGB24(int Im_width, int Im_height, IMAGE_TRANSLATE *im_T)
{
	int row, col;
	double video_width_down_sample_frequency = 0;
	double video_height_down_sample_frequency = 0;
	video_width_down_sample_frequency = (double)((double)im_T->video_input_buffer_width / (double)Im_width);
	video_height_down_sample_frequency = (double)((double)im_T->video_input_buffer_height / (double)Im_height);


	for(col = 0; col < Im_width; col++)
	{
		im_T->rgb24_down_sample_col_table[col] = int(int(video_width_down_sample_frequency * col) * 3);
	}
	for(row = 0; row < Im_height; row++)
	{
		if(row == 0)
		{
			im_T->rgb24_down_sample_row_table[row] = int((im_T->video_input_buffer_height - 1) * im_T->video_input_buffer_width * 3);
		}
		else
		{
			im_T->rgb24_down_sample_row_table[row] = int(int(im_T->video_input_buffer_height - (video_height_down_sample_frequency * row + 1)) * im_T->video_input_buffer_width * 3);
		}
	}
}

void ImageBufferDownsamplingRGB24_to_YUV(int Im_width, int Im_height, unsigned char *Im_buffer, IMAGE_BUFFER *VinSource, IMAGE_TRANSLATE *im_T, char type)
{
	int row, col;
	int red = 0;
	int green = 0;
	int blue = 0;
	int index = 0;
	int index2 = 0;
	for(row = 0; row < Im_height; row++)
	{
		for(col = 0; col < Im_width; col++)
		{
			blue  = (int)Im_buffer[im_T->rgb24_down_sample_row_table[row] + im_T->rgb24_down_sample_col_table[col]];
			green = (int)Im_buffer[im_T->rgb24_down_sample_row_table[row] + im_T->rgb24_down_sample_col_table[col] + 1];
			red   = (int)Im_buffer[im_T->rgb24_down_sample_row_table[row] + im_T->rgb24_down_sample_col_table[col] + 2];

			VinSource->Y[index] = uCharLimitSet((int)((0.299*(float)red) + (0.587*(float)green) + (0.114*(float)blue)));
			index++;
			if(index % 2 == 0)
			{
				if(type == 0){
					//es ist schneller aber unklar
					VinSource->U[index2] = uCharLimitSet(((-43*red-85*green+128*blue)>> 8)+128);
					VinSource->V[index2] = uCharLimitSet(((128*red-107*green-21*blue)>> 8)+128);
				}
				else if(type == 1){
					//es ist langsamer aber klärer
					VinSource->U[index2] = uCharLimitSet(-0.169*red-0.331*green+0.499*blue+128);
					VinSource->V[index2] = uCharLimitSet(0.499*red-0.418*green-0.0813*blue+128);
				}
				index2++;
			}
		}
	}
}

void ImageBufferYUV_to_RGB24(int Im_width, int Im_height, int row_size, unsigned char *bits, IMAGE_BUFFER *buffer)
{
	unsigned char *BitsPtr;
	int row, col;
        int Y, U = 0, V = 0;
	int red, green, blue;
	int index = 0;
	int index2 = 0;

	for (row = 0; row < Im_height; row++)
	{
		BitsPtr = bits + (row_size * row);
		for (col = 0; col < Im_width; col++)
		{

			Y = (int)buffer->Y[index];
			if(index % 2 == 0)
			{
				U = (int)buffer->U[index2];
				V = (int)buffer->V[index2];
				index2++;
			}
			red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
			green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
			blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);

			*BitsPtr++ = (unsigned char)blue;    // Blue 'noise'
			*BitsPtr++ = (unsigned char)green;   // Green
			*BitsPtr++ = (unsigned char)red;     // Red
			index++;
		}
	}
}

void ImageBufferYUV_to_Gray_scale(int Im_width, int Im_height, int row_size, unsigned char *bits, IMAGE_BUFFER *buffer)
{
	unsigned char *BitsPtr;
	int row, col;
	int Y, U = 128, V = 128;
	int red, green, blue;
	int index = 0;
	//int index2 = 0;

	for (row = 0; row < Im_height; row++)
	{
		BitsPtr = bits + (row_size * row);
		for (col = 0; col < Im_width; col++)
		{

			Y = (int)buffer->Y[index];
		  /*	if(index % 2 == 0)
			{
				U = (int)buffer->U[index2];
				V = (int)buffer->V[index2];
				index2++;
			} */
			red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
			green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
			blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);

			*BitsPtr++ = (unsigned char)blue;    // Blue 'noise'
			*BitsPtr++ = (unsigned char)green;   // Green
			*BitsPtr++ = (unsigned char)red;     // Red
			index++;
		}
	}
}

void ImageBufferUpsampling_RGB24(int DIm_width, int DIm_height, int SIm_width, int SIm_height, int row_size, unsigned char *bits, IMAGE_BUFFER *buffer)
{
	unsigned char *BitsPtr;
	int row, col;
	int Y, U, V;
	int red, green, blue;
	int index = 0;
	int index2 = 0;
	int temp = 0;
	int row_counter = 0;

	float width_up_sample_frequency = (float)DIm_width / (float)SIm_width;
	float height_up_sample_frequency = (float)DIm_height / (float)SIm_height;


	for (row = 0; row < DIm_height; row++)
	{
		BitsPtr = bits + (row_size * row);

		for (col = 0; col < DIm_width; col++)
		{

			Y = (int)buffer->Y[index];
            U = (int)buffer->U[index2];
			V = (int)buffer->V[index2];

			red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
			green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
			blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);


			*BitsPtr++ = (unsigned char)blue;    // Blue 'noise'
			*BitsPtr++ = (unsigned char)green;   // Green
			*BitsPtr++ = (unsigned char)red;     // Red


			temp = fmod(col, width_up_sample_frequency);
			if(temp == 0 )
			{
				index++;
			}

			if(index % 2 == 0)
			{
				index2 = index/2;
			}

		}

		temp = fmod(row, height_up_sample_frequency);
		if(temp == 0 )
		{
			row_counter++;
			index = SIm_width * row_counter;
			index2 = index / 2;
		}
		else
		{
			index = SIm_width * row_counter;
			index2 = index / 2;
		}
	}
}

void YUVtoRGB(int *red, int *green, int *blue, int Y, int U, int V)
{
	*red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
	*green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
	*blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);
}

