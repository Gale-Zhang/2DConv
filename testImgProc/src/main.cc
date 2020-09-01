/*
 * Commands to download memory to file (Download 76800 bytes from address 0x1300000 to file logresp.txt
 * set logfile [open "E:\\ZCU106\\StudyDemos\\2DConv\\log.txt" "w"]
 * puts $logfile [mrd -size b 0x1300000 76800]
 * close $logfile
 *
 *
 * */

#include <stdio.h>
#include "xaxidma.h"
#include "xdoimgproc.h"
#include "LenaOnCode.h"
#include "AxiTimerHelper.h"

#define SIZE_ARR (320*240)
// 3x3 kernel
#define KERNEL_DIM 3

// Memory used by DMA
#define MEM_BASE_ADDR 	0x01000000
#define TX_BUFFER_BASE	(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE	(MEM_BASE_ADDR + 0x00300000)
// Get a pointer to the TX and RX dma buffer (CONFIGURE DMA)
// The pointers are for 8-bit memory but their addresses are 32 bit (u32)
unsigned char *m_dma_buffer_TX = (unsigned char*)TX_BUFFER_BASE;
unsigned char *m_dma_buffer_RX = (unsigned char*)RX_BUFFER_BASE;

unsigned char imgIn_HW[SIZE_ARR];


XAxiDma axiDma;
int initDMA()
{
	XAxiDma_Config *CfgPtr;
	CfgPtr = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	XAxiDma_CfgInitialize(&axiDma,CfgPtr);

	// Disable interrupts
	XAxiDma_IntrDisable(&axiDma,XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&axiDma,XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE);

	return XST_SUCCESS;
}

XDoimgproc doImgProc;
int initDoImgProc()
{
	int status;

	XDoimgproc_Config *doImgProc_cfg;
	doImgProc_cfg = XDoimgproc_LookupConfig(XPAR_DOIMGPROC_0_DEVICE_ID);
	if (!doImgProc_cfg)
	{
		printf("Error loading config for doHist_cfg\n");
	}
	status = XDoimgproc_CfgInitialize(&doImgProc,doImgProc_cfg);
	if (status != XST_SUCCESS)
	{
		printf("Error initializing for doHist\n");
	}

	return status;
}

// Impulse
/*char kernel[KERNEL_DIM*KERNEL_DIM] = {
		0, 0, 0,
		0, 1, 0,
		0, 0, 0,
};*/

// Edge
/*char kernel[KERNEL_DIM*KERNEL_DIM] = {
		-1, -1, -1,
		-1, 8, -1,
		-1, -1, -1,
};*/

// Use with morphological (Erode, Dilate)
char kernel[KERNEL_DIM*KERNEL_DIM] = {
		1, 1, 1,
		1, 1, 1,
		1, 1, 1,
};

int main()
{
	initDMA();
	initDoImgProc();

	printf("Doing convolution on HW\n");

	// Populate data (Get image from header and put on memory)
	for (int idx = 0; idx < SIZE_ARR; idx++)
	{
		imgIn_HW[idx] = img[idx];
	}

	AxiTimerHelper axiTimer;

	printf("Starting.... HW\n");
	// Ask for a convolution
	XDoimgproc_Write_kernel_Bytes(&doImgProc,0,kernel,9);
	printf("Kernel total bytes: %d Bitwidth:%d Base: 0x%X\n",XDoimgproc_Get_kernel_TotalBytes(&doImgProc), XDoimgproc_Get_kernel_BitWidth(&doImgProc), XDoimgproc_Get_kernel_BaseAddress(&doImgProc));
	XDoimgproc_Set_operation(&doImgProc,2);
	XDoimgproc_Start(&doImgProc);

	// Do the DMA transfer to push and get our image
	axiTimer.startTimer();
	Xil_DCacheFlushRange((u64)imgIn_HW,SIZE_ARR*sizeof(unsigned char));
	Xil_DCacheFlushRange((u64)m_dma_buffer_RX,SIZE_ARR*sizeof(unsigned char));

	XAxiDma_SimpleTransfer(&axiDma,(u64)imgIn_HW,SIZE_ARR*sizeof(unsigned char),XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_SimpleTransfer(&axiDma,(u64)m_dma_buffer_RX,SIZE_ARR*sizeof(unsigned char),XAXIDMA_DEVICE_TO_DMA);

	//Wait transfers to finish
	while(XAxiDma_Busy(&axiDma,XAXIDMA_DMA_TO_DEVICE));
	while(XAxiDma_Busy(&axiDma,XAXIDMA_DEVICE_TO_DMA));

	// Invalidate the cache to avoid reading garbage
	Xil_DCacheInvalidateRange((u64)m_dma_buffer_RX,SIZE_ARR*sizeof(unsigned char));
	axiTimer.stopTimer();

	double HW_elapsed = axiTimer.getElapsedTimerInSeconds();
	printf("HW execution time: %f sec\n", HW_elapsed);


	return 0;
}
