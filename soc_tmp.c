#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"

#include <math.h>
#include <stdio.h>
#include <float.h>

#define WIDTH 960 
#define HEIGHT 720

void grayScale_transform (unsigned char* in, int const height, int const width, int const channel, unsigned char* out);
void FILTER_GAUSSIAN (unsigned char* in, int const height, int const width, int const channel, unsigned char* out); 
void FILTER_SOBEL (unsigned char* in, int const height, int const width, int const channel, unsigned char* out); 
void FILTER_CANNY (unsigned char* in, int const height, int const width, int const channel, unsigned char* out); 
double imagePSNR(unsigned char* frame1, unsigned char* frame2, unsigned int size);

int main()
{
 
	int height;
	int width;
	int channel;

    float PSNR_up,PSNR_bilinear;

 	unsigned char* imgIn = stbi_load("test_up.bmp", &width, &height, &channel, 3);
	unsigned char* imgOut_grayScale = (unsigned char*) malloc (sizeof(unsigned char)*3*WIDTH*HEIGHT);
	unsigned char* imgOut_gaussian = (unsigned char*) malloc (sizeof(unsigned char)*3*WIDTH*HEIGHT);
	unsigned char* imgOut_edge = (unsigned char*) malloc (sizeof(unsigned char)*3*WIDTH*HEIGHT);
	unsigned char* imgOut_guassian_sobel = (unsigned char*) malloc (sizeof(unsigned char)*3*WIDTH*HEIGHT);

 	grayScale_transform(imgIn, height, width, channel, imgOut_grayScale);
	FILTER_GAUSSIAN(imgOut_grayScale, height, width, channel, imgOut_gaussian);
	FILTER_CANNY(imgOut_grayScale, height, width, channel, imgOut_edge);
	FILTER_CANNY(imgOut_gaussian, height, width, channel, imgOut_guassian_sobel);
				
	stbi_write_bmp("image_grayscale.bmp", width, height, channel, imgOut_grayScale);
	stbi_write_bmp("image_gaussian.bmp", width, height, channel, imgOut_gaussian);
	stbi_write_bmp("image_sobel.bmp", width, height, channel, imgOut_edge);
	stbi_write_bmp("image_gaussian_sobel.bmp", width, height, channel, imgOut_guassian_sobel);
	
	stbi_image_free(imgIn);
	free(imgOut_grayScale);
	free(imgOut_gaussian);
	free(imgOut_guassian_sobel);
 

	return 0;
}

void grayScale_transform (unsigned char* in, int const height, int const width, int const channel, unsigned char* out) {
	int x, y, c;

	for(y=0; y<height; y++)
		for(x=0; x<width; x++)
			for(c=0; c<channel; c++)
				out[y*WIDTH*3 + x*3+c]=(in[y*WIDTH*3+x*3+0] + in[y*WIDTH*3+x*3+1] + in[y*WIDTH*3+x*3+2])/3;
}

void FILTER_GAUSSIAN(unsigned char* in, int const height, int const width, int const channel, unsigned char* out) {

	int x,y,c,i,j;
	int sum_R, sum_G, sum_B ;

	unsigned int newValue;
	int kernel_Gaussian[5][5]=
						{{ 2,4,5,4,2 },
						 { 4,9,12,9,4 },
						 { 5,12,15,12,5},
						 { 4,9,12,9,4 }, 
						 { 2,4,5,4,2 }};

	for(y=2; y< height-2 ; y++) {
		for(x=2; x< width-2 ; x++) {
			sum_R=0;
			sum_G=0;
			sum_B=0;

			for(i=0; i<5; i++) {
				for(j=0; j<5; j++) {
					sum_R += in[(y+i-1)*WIDTH*3+(x+j-1)*3+0]*kernel_Gaussian[i][j]; 		
					sum_G += in[(y+i-1)*WIDTH*3+(x+j-1)*3+1]*kernel_Gaussian[i][j]; 	
					sum_B += in[(y+i-1)*WIDTH*3+(x+j-1)*3+2]*kernel_Gaussian[i][j]; 				
				}
			}			

			out[y*WIDTH*3 + x*3 +0] = (int)(sum_R/159);
			out[y*WIDTH*3 + x*3 +1] = (int)(sum_G/159);
			out[y*WIDTH*3 + x*3 +2] = (int)(sum_B/159);
			
		}	
	}
}

void FILTER_SOBEL(unsigned char* in, int const height, int const width, int const channel, unsigned char* out) {

	int x,y,c,i,j;
	int sumX, sumY;
	int maskSobelX[3][3]=
						{{ -1, 0, 1},
						 { -2, 0, 2},
						 { -1, 0, 1}};
	int maskSobelY[3][3]=
						{{ -1, -2, -1},
						 {  0,  0,  0},
						 {  1,  2, 1}};

	unsigned int newValue;

	for(y=1; y<(height-1); y++) {
		for(x=1; x<(width-1); x++) {
			sumX = 0;
			sumY = 0;
			for(i=0; i<3; i++) {
				for(j=0; j<3; j++) {
					sumX += in[(y+i-1)*WIDTH*3+(x+j-1)*3+0]*maskSobelX[i][j]; 
					sumY += in[(y+i-1)*WIDTH*3+(x+j-1)*3+0]*maskSobelY[i][j]; 
				}
			}
			newValue = (sumX > 0 ? sumX : -sumX) + (sumY > 0 ? sumY : -sumY);
			for(c=0; c<channel; c++)
				out[y*WIDTH*3 + x*3+c] = newValue;
		}
	}
	
}

void FILTER_CANNY(unsigned char* in, int const height, int const width, int const channel, unsigned char* out) {

	int x,y,c,i,j;
	int sumX, sumY, slope, direction, local_max;
	unsigned int magnitude ; 

	unsigned int* sumX_arr = (unsigned int*) malloc (sizeof(unsigned int)*WIDTH*HEIGHT);
	unsigned int* sumY_arr = (unsigned int*) malloc (sizeof(unsigned int)*WIDTH*HEIGHT);
	unsigned int* mag_arr = (unsigned int*) malloc (sizeof(unsigned int)*WIDTH*HEIGHT);
	unsigned int* out_arr_tmp = (unsigned int*) malloc (sizeof(unsigned int)*WIDTH*HEIGHT);

	unsigned int** stack_top, **stack_bottom; 
	stack_top = (unsigned int**) malloc (sizeof(unsigned int*)*WIDTH*HEIGHT);
	stack_bottom = stack_top;

	#define CANNY_PUSH(p) *(p) = 255, *(stack_top++) = (p)
	#define CANNY_POP() *(--stack_top)


	int th_low = 50; 
	int th_high = 100; 

	int maskSobelX[3][3]=
						{{ -1, 0, 1},
						 { -2, 0, 2},
						 { -1, 0, 1}};
	int maskSobelY[3][3]=
						{{ -1, -2, -1},
						 {  0,  0,  0},
						 {  1,  2, 1}};

	unsigned int newValue;

	for(y=1; y<(height-1); y++) {
		for(x=1; x<(width-1); x++) {
			sumX = 0;
			sumY = 0;
			for(i=0; i<3; i++) {
				for(j=0; j<3; j++) {
					sumX_arr[y*WIDTH+x] += in[(y+i-1)*WIDTH*3+(x+j-1)*3+0]*maskSobelX[i][j]; 
					sumY_arr[y*WIDTH+x] += in[(y+i-1)*WIDTH*3+(x+j-1)*3+0]*maskSobelY[i][j]; 
				}
			} 
			sumX = sumX_arr[y*WIDTH+x];
			sumY = sumY_arr[y*WIDTH+x];

			newValue = (sumX > 0 ? sumX : -sumX) + (sumY > 0 ? sumY : -sumY);
			mag_arr[y*WIDTH + x] = newValue;
		}
	}

	for(y=1; y<(height-1); y++) {
		for(x=1; x<(width-1); x++) {
			magnitude = mag_arr[y*WIDTH + x]; 
			
			if(magnitude > th_low)  {
				sumX = sumX_arr[y*WIDTH + x];
				sumY = sumY_arr[y*WIDTH + x]; 

				if	(sumX!=0) slope = (sumY << 10) /sumX ;
            	
					if(slope > 0) {
						if(slope < 424) //slope < tan225
							direction = 0;
						else if (slope <2472) // tan225 < slope < tan675
							direction = 1;
						else 
							direction = 2;
						}

					else {
						if(-slope < 2472 )  
							direction = 2;
						else if (-slope < 424)  
							direction = 3;
						else 
							direction = 0;
						}
				
			}

			else direction = 2; 
         		
			local_max = 1; 

			if(direction==0){
				if( ( magnitude  < mag_arr[y*WIDTH + (x-1)]) ||  (magnitude  < mag_arr[y*WIDTH + (x+1)]) )
				local_max = 0; 
			}

			else if(direction==1){
				if(( magnitude  < mag_arr[(y+1)*WIDTH + (x+1)]) ||  (magnitude  < mag_arr[(y-1)*WIDTH + (x-1)]) )
				local_max = 0; 
			}

			else if(direction==2){
				if( (magnitude  < mag_arr[(y+1)*WIDTH + x]) ||  (magnitude  < mag_arr[(y-1)*WIDTH + (x+1)]) )
				local_max = 0; 
			}

			else{
				if( (magnitude  < mag_arr[(y+1)*WIDTH + (x-1)]) ||  (magnitude < mag_arr[(y-1)*WIDTH + (x+1)]) )
				local_max = 0; 
			}

			if(local_max){
				if(magnitude > th_high){ //strong edges
					out_arr_tmp[(y*WIDTH + x)+0] = 255; 
				}
				else { //week edges
					out_arr_tmp[(y*WIDTH + x)+0] = 100;  
				}
			}

		}
	}

	while(stack_top!=stack_bottom) {
		unsigned int * p = CANNY_POP();

		if(*(p+1)==100)	CANNY_PUSH(p+1);
		if(*(p-1)==100)	CANNY_PUSH(p+1);
		if(*(p+WIDTH)==100) CANNY_PUSH(p+WIDTH);
		if(*(p-WIDTH)==100) CANNY_PUSH(p-WIDTH);
		if(*(p+WIDTH+1)==100) CANNY_PUSH(p+WIDTH+1);
		if(*(p+WIDTH-1)==100) CANNY_PUSH(p+WIDTH-1);
		if(*(p-WIDTH+1)==100) CANNY_PUSH(p-WIDTH+1);
		if(*(p-WIDTH-1)==100) CANNY_PUSH(p-WIDTH-1);

	} 

	for(y=1; y<(height-1); y++) {
		for(x=1; x<(width-1); x++) {
				int tmp_value = out_arr_tmp[(y*WIDTH + x)+0] ; 
				out[3*(y*WIDTH + x)+0] = (tmp_value == 255) ? 255 : 0 ; 
				out[3*(y*WIDTH + x)+1] = (tmp_value == 255) ? 255 : 0 ; 
				out[3*(y*WIDTH + x)+2] = (tmp_value == 255) ? 255 : 0 ; 

		}
	}
}


//Calculates image PSNR value
double imagePSNR(unsigned char* frame1, unsigned char* frame2, unsigned int size){

	int i = 0;
	int a = 0;
	double MSE = 0;
	//double PSNR = 10;
	
	for (i = 0; i < (size); i++) {
		
		a = a + (frame1[i] - frame2[i]) * (frame1[i] - frame2[i]);

	}
	MSE = a / (size);

	double PSNR = 0;

	PSNR = (20 * log10(255)) - (10 * log10(MSE));
	
	return PSNR;
}
