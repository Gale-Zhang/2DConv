#include "core.h"

void doImgProc(hls::stream<uint_8_side_channel> &inStream, hls::stream<int_8_side_channel> &outStream, char kernel[KERNEL_DIM*KERNEL_DIM], int operation)
{

#pragma HLS INTERFACE axis port=inStream
#pragma HLS INTERFACE axis port=outStream
#pragma HLS INTERFACE s_axilite port=return bundle=CRTL_BUS
#pragma HLS INTERFACE s_axilite port=operation bundle=CRTL_BUS
#pragma HLS INTERFACE s_axilite port=kernel bundle=KERNEL_BUS

	hls::LineBuffer<KERNEL_DIM, IMG_WIDTH_OR_COLS, unsigned char> lineBuff;
	hls::Window<KERNEL_DIM, KERNEL_DIM, short> window;

	int idxCol = 0;
	int idxRow = 0;
	int pixConvolved = 0;
	int waitTicks = (IMG_WIDTH_OR_COLS * (KERNEL_DIM - 1) + KERNEL_DIM)/2;
	int countWait = 0;
	int sentPixels = 0;

	int_8_side_channel dataOutSideChannel;
	uint_8_side_channel currPixelSideChannel;

	for(int idxPixel = 0; idxPixel < (IMG_WIDTH_OR_COLS * IMG_HEIGHT_OR_ROWS); idxPixel++)
	{
#pragma HLS PIPELINE
		currPixelSideChannel = inStream.read();
		unsigned char pixelIn = currPixelSideChannel.data;

		lineBuff.shift_up(idxCol);
		lineBuff.insert_top(pixelIn, idxCol);

		for(int idxWinRow = 0; idxWinRow < KERNEL_DIM; idxWinRow++)
		{
			for(int idxWinCol = 0; idxWinCol < KERNEL_DIM; idxWinCol++)
			{
				short val = (short)lineBuff.getval(idxWinRow, idxWinCol + pixConvolved);
				val = (short)kernel[(idxWinRow*KERNEL_DIM) + idxWinCol ] * val;
				window.insert(val,idxWinRow,idxWinCol);
			}
		}

		short valOutput = 0;
		if ((idxRow >= KERNEL_DIM-1) && (idxCol >= KERNEL_DIM-1))
		{
			switch (operation) {
			case 0:
			{
				// Convolution
				valOutput = sumWindow(&window);
				valOutput = valOutput / 8;
				// Avoid negative values
				if (valOutput < 0)
					valOutput = 0;
				break;
			}
			case 1:
			{
				// Erode
				valOutput = minWindow(&window);
				break;
			}
			case 2:
			{
				// Dilate
				valOutput = maxWindow(&window);
				break;
			}
			}

			pixConvolved++;
		}

		// Calculate row and col index
		if (idxCol < IMG_WIDTH_OR_COLS-1)
		{
			idxCol++;
		}
		else
		{
			// New line
			idxCol = 0;
			idxRow++;
			pixConvolved = 0;
		}

		/*
		 * Fix the line buffer delay, on a 320x240 image with 3x3 kernel, the delay will be
		 * ((240*2) + 3)/2 = 241
		 * So we wait for 241 ticks send the results than put more 241 zeros
		 */

		// Put data on output stream (side-channel(tlast) way...)
		/*dataOutSideChannel.data = valOutput;
		dataOutSideChannel.keep = currPixelSideChannel.keep;
		dataOutSideChannel.strb = currPixelSideChannel.strb;
		dataOutSideChannel.user = currPixelSideChannel.user;
		dataOutSideChannel.last = currPixelSideChannel.last;
		dataOutSideChannel.id = currPixelSideChannel.id;
		dataOutSideChannel.dest = currPixelSideChannel.dest;

		// Send to the stream (Block if the FIFO receiver is full)
		outStream.write(dataOutSideChannel);*/
		countWait++;
		if (countWait > waitTicks)
		{
			dataOutSideChannel.data = valOutput;
			dataOutSideChannel.keep = currPixelSideChannel.keep;
			dataOutSideChannel.strb = currPixelSideChannel.strb;
			dataOutSideChannel.user = currPixelSideChannel.user;
			dataOutSideChannel.last = 0;
			dataOutSideChannel.id = currPixelSideChannel.id;
			dataOutSideChannel.dest = currPixelSideChannel.dest;
			// Send to the stream (Block if the FIFO receiver is full)
			outStream.write(dataOutSideChannel);
			sentPixels++;
		}
	}

	// Now send the remaining zeros (Just the (Number of delayed ticks)
	for (countWait = 0; countWait < waitTicks; countWait++)
	{
		dataOutSideChannel.data = 0;
		dataOutSideChannel.keep = currPixelSideChannel.keep;
		dataOutSideChannel.strb = currPixelSideChannel.strb;
		dataOutSideChannel.user = currPixelSideChannel.user;
		// Send last on the last item
		if (countWait < waitTicks - 1)
			dataOutSideChannel.last = 0;
		else
			dataOutSideChannel.last = 1;
		dataOutSideChannel.id = currPixelSideChannel.id;
		dataOutSideChannel.dest = currPixelSideChannel.dest;
		// Send to the stream (Block if the FIFO receiver is full)
		outStream.write(dataOutSideChannel);
	}
}

short minWindow(hls::Window<KERNEL_DIM,KERNEL_DIM,short> *window)
{
	unsigned char minVal = 255;
	// Look for the smalles value on the array
	for (int idxRow = 0; idxRow < KERNEL_DIM; idxRow++)
	{
		for (int idxCol = 0; idxCol < KERNEL_DIM; idxCol++)
		{
			unsigned char valInWindow;
			valInWindow = (unsigned char)window->getval(idxRow,idxCol);
			if (valInWindow < minVal)
				minVal = valInWindow;
		}
	}
	return minVal;
}

// Dilate will get the maximum value on the window, which in our case will always be retangular and filled with one
short maxWindow(hls::Window<KERNEL_DIM,KERNEL_DIM,short> *window)
{
	unsigned char maxVal = 0;
	// Look for the smalles value on the array
	for (int idxRow = 0; idxRow < KERNEL_DIM; idxRow++)
	{
		for (int idxCol = 0; idxCol < KERNEL_DIM; idxCol++)
		{
			unsigned char valInWindow;
			valInWindow = (unsigned char)window->getval(idxRow,idxCol);
			if (valInWindow > maxVal)
				maxVal = valInWindow;
		}
	}
	return maxVal;
}


// Sum all values inside window (Already multiplied by the kernel)
short sumWindow(hls::Window<KERNEL_DIM,KERNEL_DIM,short> *window)
{
	short accumulator = 0;

	// Iterate on the window multiplying and accumulating the kernel and sampling window
	for (int idxRow = 0; idxRow < KERNEL_DIM; idxRow++)
	{
		for (int idxCol = 0; idxCol < KERNEL_DIM; idxCol++)
		{
			accumulator = accumulator + (short)window->getval(idxRow,idxCol);
		}
	}
	return accumulator;
}
