#include "hls_video.h"
#define IMG_WIDTH_OR_COLS 240
#define IMG_HEIGHT_OR_ROWS 320

#define KERNEL_DIM 3

#define INPUT_IMAGE_CORE "E://Photo/Lena/Lena.bmp"
#define OUTPUT_IMAGE_CORE "E://Photo/Lena/Result.bmp"
#define OUTPUT_IMAGE_REF "E://Photo/Lena/Ref.bmp"

#include <ap_axi_sdata.h>
typedef ap_axiu<8,2,5,6> uint_8_side_channel;
typedef ap_axiu<8,2,5,6> int_8_side_channel;

short convolve2d(hls::Window<KERNEL_DIM, KERNEL_DIM, char> *window, char kernel[KERNEL_DIM*KERNEL_DIM]);
short sumWindow(hls::Window<KERNEL_DIM, KERNEL_DIM, short> *window);
short minWindow(hls::Window<KERNEL_DIM, KERNEL_DIM, short> *window);
short maxWindow(hls::Window<KERNEL_DIM, KERNEL_DIM, short> *window);

void doImgProc(hls::stream<uint_8_side_channel> &inStream, hls::stream<int_8_side_channel> &outStream, char kernel[KERNEL_DIM*KERNEL_DIM], int operation);
