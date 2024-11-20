// Test of parallel AD9226 ADC using Raspberry Pi SMI (Secondary Memory Interface)
// Afeka Project

#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "rpi_dma_utils.h"
#include "rpi_smi_defs.h"
#include <fftw3.h>
#include <math.h>

typedef struct {
    uint16_t *time_data;  // Pointer to the time-domain data array
    double *frequency;    // Array to hold frequency data
    double *magnitude;    // Array to hold magnitude data
    int nsamples;         // Number of samples in the time_data
} DataBundle;



// Set zero for single value, non-zero for block read
#define USE_DMA         1
// Use test pin in place of GPIO mode setting (to check timing)
#define USE_TEST_PIN    0

// SMI cycle timings
#define SMI_NUM_BITS    SMI_16_BITS
#define SMI_TIMING      SMI_TIMING_25M

// Timings of SMI
#define SMI_TIMING_1M   10, 38, 74, 38  // 1 MS/s
#define SMI_TIMING_10M   6,  6, 13,  6  // 10 MS/s
#define SMI_TIMING_20M   4,  5,  9,  5  // 19.74 MS/s
#define SMI_TIMING_25M   4,  3,  8,  4  // 25 MS/s
#define SMI_TIMING_31M   4,  3,  6,  3  // 31.25 MS/s


// Number of raw bytes per ADC sample
#define SAMPLE_SIZE     2

// Number of samples to be captured, and number to be discarded
#define NSAMPLES        16384
#define PRE_SAMP        50
#define NCYCLES			25000 //Approx. 15 seconds
#define FFT_NSAMPLES	(2*NSAMPLES)

// Voltage calibration
#define ADC_ZERO        2026
#define ADC_SCALE       194.0
#define VOL_THRESH		4.0
// GPIO pin numbers
#define ADC_D0_PIN      12
#define ADC_NPINS       12
#define SMI_SOE_PIN     6
#define SMI_SWE_PIN     7
#define SMI_DREQ_PIN    24
#define TEST_PIN        25

// DMA request threshold
#define REQUEST_THRESH  4

// SMI register names for diagnostic print
char *smi_regstrs[] = {
    "CS","LEN","A","D","DSR0","DSW0","DSR1","DSW1",
    "DSR2","DSW2","DSR3","DSW3","DMC","DCS","DCA","DCD",""
};

// SMI CS register field names for diagnostic print
#define STRS(x)     STRS_(x) ","
#define STRS_(...)  #__VA_ARGS__
char *smi_cs_regstrs = STRS(SMI_CS_FIELDS);

// Structures for mapped I/O devices, and non-volatile memory
extern MEM_MAP gpio_regs, dma_regs, clk_regs;
MEM_MAP vc_mem, smi_regs;

// Pointers to SMI registers
volatile SMI_CS_REG  *smi_cs;
volatile SMI_L_REG   *smi_l;
volatile SMI_A_REG   *smi_a;
volatile SMI_D_REG   *smi_d;
volatile SMI_DMC_REG *smi_dmc;
volatile SMI_DSR_REG *smi_dsr;
volatile SMI_DSW_REG *smi_dsw;
volatile SMI_DCS_REG *smi_dcs;
volatile SMI_DCA_REG *smi_dca;
volatile SMI_DCD_REG *smi_dcd;

// Buffer for captured samples
uint16_t sample_data[2*NSAMPLES];

// Non-volatile memory size
#define VC_MEM_SIZE(nsamp) (PAGE_SIZE + ((nsamp)+4)*SAMPLE_SIZE)

void map_devices(void);
void fail(char *s);
void terminate(int sig);
void smi_start(int nsamples, int packed);
uint32_t *adc_dma_start(MEM_MAP *mp, int nsamp, uint32_t *data, DMA_CB *cbs, uint32_t *modes);
int get_adc_data(void *buff, uint16_t *data, int nsamp);
void init_smi(int width, int ns, int setup, int hold, int strobe);
void disp_smi(void);
void mode_word(uint32_t *wp, int n, uint32_t mode);
float val_volts(int val);
int adc_gpio_val(void);
void disp_reg_fields(char *regstrs, char *name, uint32_t val);
void dma_wait(int chan);
int detect_trigger(void *buff, int nsamps, float threshold);
void last_block(MEM_MAP *mp,DMA_CB *cbs, uint32_t *modes, volatile uint32_t *p);
void plot_data(int order, DataBundle *data);  
void save_to_csv(DataBundle *data); 


int main(int argc, char *argv[]) {
    void *rxbuff;
    int i, flag = 0, order = 0;

    signal(SIGINT, terminate);
    map_devices();
    for (i = 0; i < ADC_NPINS; i++)
        gpio_mode(ADC_D0_PIN + i, GPIO_IN);
    gpio_mode(SMI_SOE_PIN, GPIO_ALT1);
    init_smi(SMI_NUM_BITS, SMI_TIMING);
    map_uncached_mem(&vc_mem, VC_MEM_SIZE(2 * NSAMPLES + PRE_SAMP));
    DMA_CB *cbs = (&vc_mem)->virt;
    uint32_t *data = (uint32_t *)(cbs + 5), *modes = data + 0x10;
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    rxbuff = adc_dma_start(&vc_mem, NSAMPLES, data, cbs, modes);
    volatile uint32_t *p = REG32(dma_regs, DMA_REG(DMA_CHAN_A, DMA_CS));

    smi_start(NCYCLES * NSAMPLES + PRE_SAMP, 1);
    while (dma_active(DMA_CHAN_A) && flag != 1) {
        if (*p & (1 << 2)) {
            *REG32(dma_regs, DMA_REG(DMA_CHAN_A, DMA_CS)) = 5; // Clear 'INT' flag
            flag = detect_trigger(rxbuff, NSAMPLES, VOL_THRESH);
            if (flag) {
                last_block(&vc_mem, cbs, modes, p);
                if (MEM_BUS_ADDR((&vc_mem), &cbs[3]) == *REG32(dma_regs, DMA_REG(DMA_CHAN_A, DMA_CONBLK_AD))) {
                    order = 1;
                }
            }
        }
    }
    usleep(1000);
    get_adc_data(rxbuff, sample_data, 2 * NSAMPLES);
    smi_cs->enable = smi_dcs->enable = 0;

	DataBundle data_bundle;
	data_bundle.time_data = sample_data;
	data_bundle.nsamples = 2 * NSAMPLES;

	plot_data(order, &data_bundle);
	save_to_csv(&data_bundle);

    // Clean up dynamically allocated memory
    free(data_bundle.frequency);
    free(data_bundle.magnitude);

    terminate(0);
    return 0;
}


void plot_data(int order, DataBundle *data) {
    uint16_t *time_data = data->time_data;
    FILE *gp1, *gp2; // GNUplot pipes for two separate plots
    int i;
    double *in;
    fftw_complex *out;
    fftw_plan p;
    double Fs = 25e6; // Sampling frequency in Hz

    // Prepare for FFT
    in = (double*) fftw_malloc(sizeof(double) * FFT_NSAMPLES);
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (FFT_NSAMPLES/2 + 1));
    p = fftw_plan_dft_r2c_1d(FFT_NSAMPLES, in, out, FFTW_ESTIMATE);

    // Populate input array and perform FFT
    for (i = 0; i < data->nsamples; i++) {
        in[i] = val_volts(time_data[i]);
    }
    fftw_execute(p);

    // Allocate and store FFT results in DataBundle
    data->frequency = (double*) malloc(sizeof(double) * (FFT_NSAMPLES/2 + 1));
    data->magnitude = (double*) malloc(sizeof(double) * (FFT_NSAMPLES/2 + 1));
    for (i = 0; i <= FFT_NSAMPLES / 2; i++) {
        data->frequency[i] = (double)i * Fs / FFT_NSAMPLES;
        data->magnitude[i] = 20 * log10(sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1])/FFT_NSAMPLES);
    }

    // Plotting Time-domain data
    gp1 = popen("gnuplot -persistent", "w");
    fprintf(gp1, "set title 'ADC Voltage Plot'\n");
    fprintf(gp1, "set xlabel 'Sample Number'\n");
    fprintf(gp1, "set ylabel 'Voltage (V)'\n");
    fprintf(gp1, "set yrange [-2:7]\n");
    fprintf(gp1, "set xrange [0:%d]\n", FFT_NSAMPLES-1);
    fprintf(gp1, "set grid\n");
    fprintf(gp1, "plot '-' with lines notitle\n");
    for (i = 0; i < FFT_NSAMPLES; i++) {
        fprintf(gp1, "%d %1.3f\n", i, in[i]);
    }
    fprintf(gp1, "e\n");
    pclose(gp1);

    // Plotting FFT data
    gp2 = popen("gnuplot -persistent", "w");
    fprintf(gp2, "set title 'FFT of ADC Voltage'\n");
    fprintf(gp2, "set xlabel 'Frequency (MHz)'\n");
    fprintf(gp2, "set ylabel 'Magnitude (dB)'\n");
    fprintf(gp2, "set yrange [-150:0]\n");
    fprintf(gp2, "set xrange [0:%.1f]\n", (Fs / 2) / 1e6);
    fprintf(gp2, "set grid\n");
    fprintf(gp2, "plot '-' using 1:2 with lines notitle\n");
    for (i = 0; i <= FFT_NSAMPLES / 2; i++) {
        fprintf(gp2, "%f %f\n", data->frequency[i] / 1e6, data->magnitude[i]);
    }
    fprintf(gp2, "e\n");
    pclose(gp2);

    // Clean up FFT resources
    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);
}


int detect_trigger(void *buff, int nsamps, float threshold) {
    uint16_t *array = (uint16_t *)buff;  // Cast void* to uint16_t*
    uint16_t t = (uint16_t)threshold;    // Convert float threshold to uint16_t, assuming threshold is within uint16_t range
	t = (t*ADC_SCALE)+ADC_ZERO;
	t = t<<4;
    for (int i = 0; i < nsamps; i++) {
        if (array[i] >= t) {
            return 1;  // Return 1 if any value is greater than or equal to the threshold
        }
    }
    return 0;  // Return 0 if no value meets the threshold
}
void last_block(MEM_MAP *mp,DMA_CB *cbs, uint32_t *modes, volatile uint32_t *p)
{
	// Control block 5: disable SMI I/P pins
    cbs[5].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC;
    cbs[5].tfr_len = 3 * 4;
    cbs[5].srce_ad = MEM_BUS_ADDR(mp, &modes[3]);
    cbs[5].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0);
    cbs[5].next_cb = 0x00000000;
    *REG32(dma_regs, DMA_REG(DMA_CHAN_A, DMA_NEXTCONBK)) = MEM_BUS_ADDR(mp, &cbs[5]);
}


// Map GPIO, DMA and SMI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&smi_regs, (void *)SMI_BASE, PAGE_SIZE);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory segments and exit
void terminate(int sig)
{
    int i;

    printf("Closing\n");
    if (gpio_regs.virt)
    {
        for (i=0; i<ADC_NPINS; i++)
            gpio_mode(ADC_D0_PIN+i, GPIO_IN);
    }
    if (smi_regs.virt)
        *REG32(smi_regs, SMI_CS) = 0;
    stop_dma(DMA_CHAN_A);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&smi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    exit(0);
}

// Start SMI, given number of samples, optionally pack bytes into words
void smi_start(int nsamples, int packed)
{
    smi_l->len = nsamples;
    smi_cs->pxldat = (packed != 0);
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->start = 1;
}


// Start DMA for SMI ADC, return Rx data buffer
uint32_t *adc_dma_start(MEM_MAP *mp, int nsamp, uint32_t *data, DMA_CB *cbs, uint32_t *modes)
{
    uint32_t *modep1=data+0x18, *modep2=modep1+1,i;
	uint32_t *rxdata=data+0x20;	
	
    // Get current mode register values
    for (i=0; i<3; i++)
        modes[i] = modes[i+3] = *REG32(gpio_regs, GPIO_MODE0 + i*4);
    // Get mode values with ADC pins set to SMI
    for (i=ADC_D0_PIN; i<ADC_D0_PIN+ADC_NPINS; i++)
        mode_word(&modes[i/10], i%10, GPIO_ALT1);
    // Copy mode values into 32-bit words
    *modep1 = modes[1];
    *modep2 = modes[2];
    enable_dma(DMA_CHAN_A);
    // Control blocks 0 and 1: enable SMI I/P pins
    cbs[0].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16) | DMA_WAIT_RESP;
    cbs[0].tfr_len = 4;
    cbs[0].srce_ad = MEM_BUS_ADDR(mp, modep1);
    cbs[0].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0+4);
    cbs[0].next_cb = MEM_BUS_ADDR(mp, &cbs[1]);
    cbs[1].tfr_len = 4;
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, modep2);
    cbs[1].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0+8);
    cbs[1].next_cb = MEM_BUS_ADDR(mp, &cbs[2]);
    // Control block 2: Pre-Sampling
    cbs[2].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16)| DMA_CB_DEST_INC;
    cbs[2].tfr_len = PRE_SAMP * SAMPLE_SIZE;
    cbs[2].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[2].dest_ad = MEM_BUS_ADDR(mp, rxdata);
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[3]);
    // Control block 3: #1st Data-Buffer
    cbs[3].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16)| DMA_CB_DEST_INC| DMA_TI_INTEN;
    cbs[3].tfr_len = nsamp * SAMPLE_SIZE;
    cbs[3].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[3].dest_ad = MEM_BUS_ADDR(mp, rxdata+2*PRE_SAMP);
    cbs[3].next_cb = MEM_BUS_ADDR(mp, &cbs[4]);
	// Control block 4: #2nd Data-Buffer
	cbs[4].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16)| DMA_CB_DEST_INC| DMA_TI_INTEN;
    cbs[4].tfr_len = nsamp * SAMPLE_SIZE;
    cbs[4].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[4].dest_ad = MEM_BUS_ADDR(mp, rxdata+2*(nsamp+PRE_SAMP));
    cbs[4].next_cb = MEM_BUS_ADDR(mp, &cbs[3]); 
	rxdata+=PRE_SAMP/2;
    start_dma(mp, DMA_CHAN_A, &cbs[0], 0);
    return(rxdata);
}


// ADC DMA is complete, get data
int get_adc_data(void *buff, uint16_t *data, int nsamp)
{
    uint16_t *bp = (uint16_t *)buff;
    int i;

    for (i=0; i<nsamp; i++)
    {
		*data++ = bp[i] >> 4;
    }
    return(nsamp);
}

// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
void init_smi(int width, int ns, int setup, int strobe, int hold)
{
    int divi = ns / 2;

    smi_cs  = (SMI_CS_REG *) REG32(smi_regs, SMI_CS);
    smi_l   = (SMI_L_REG *)  REG32(smi_regs, SMI_L);
    smi_a   = (SMI_A_REG *)  REG32(smi_regs, SMI_A);
    smi_d   = (SMI_D_REG *)  REG32(smi_regs, SMI_D);
    smi_dmc = (SMI_DMC_REG *)REG32(smi_regs, SMI_DMC);
    smi_dsr = (SMI_DSR_REG *)REG32(smi_regs, SMI_DSR0);
    smi_dsw = (SMI_DSW_REG *)REG32(smi_regs, SMI_DSW0);
    smi_dcs = (SMI_DCS_REG *)REG32(smi_regs, SMI_DCS);
    smi_dca = (SMI_DCA_REG *)REG32(smi_regs, SMI_DCA);
    smi_dcd = (SMI_DCD_REG *)REG32(smi_regs, SMI_DCD);
    smi_cs->value = smi_l->value = smi_a->value = 0;
    smi_dsr->value = smi_dsw->value = smi_dcs->value = smi_dca->value = 0;
    if (*REG32(clk_regs, CLK_SMI_DIV) != divi << 12)
    {
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | (1 << 5);
        usleep(10);
        while (*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) ;
        usleep(10);
        *REG32(clk_regs, CLK_SMI_DIV) = CLK_PASSWD | (divi << 12);
        usleep(10);
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | 6 | (1 << 4);
        usleep(10);
        while ((*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) == 0) ;
        usleep(100);
    }
    if (smi_cs->seterr)
        smi_cs->seterr = 1;
    smi_dsr->rsetup = smi_dsw->wsetup = setup;
    smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    smi_dsr->rhold = smi_dsw->whold = hold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = REQUEST_THRESH;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
}

// Display SMI registers
void disp_smi(void)
{
    volatile uint32_t *p=REG32(smi_regs, SMI_CS);
    int i=0;

    while (smi_regstrs[i][0])
    {
        printf("%4s=%08X ", smi_regstrs[i++], *p++);
        if (i%8==0 || smi_regstrs[i][0]==0)
            printf("\n");
    }
}

// Get GPIO mode value into 32-bit word
void mode_word(uint32_t *wp, int n, uint32_t mode)
{
    uint32_t mask = 7 << (n * 3);

    *wp = (*wp & ~mask) | (mode << (n * 3));
}

// Convert ADC value to voltage
float val_volts(int val)
{
    return((-1*ADC_ZERO + val) / ADC_SCALE);
}

// Return ADC value, using GPIO inputs
int adc_gpio_val(void)
{
    int v = *REG32(gpio_regs, GPIO_LEV0);

    return((v>>ADC_D0_PIN) & ((1 << ADC_NPINS)-1));
}

// Display bit values in register
void disp_reg_fields(char *regstrs, char *name, uint32_t val)
{
    char *p=regstrs, *q, *r=regstrs;
    uint32_t nbits, v;

    printf("%s %08X", name, val);
    while ((q = strchr(p, ':')) != 0)
    {
        p = q + 1;
        nbits = 0;
        while (*p>='0' && *p<='9')
            nbits = nbits * 10 + *p++ - '0';
        v = val & ((1 << nbits) - 1);
        val >>= nbits;
        if (v && *r!='_')
            printf(" %.*s=%X", q-r, r, v);
        while (*p==',' || *p==' ')
            p = r = p + 1;
    }
    printf("\n");
}

void save_to_csv(DataBundle *data) {
    FILE *fp = fopen("adc_data.csv", "w");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file for writing.\n");
        return;
    }

    fprintf(fp, "Sample Index,Voltage,Frequency (Hz),Magnitude\n");
    for (int i = 0; i < data->nsamples; i++) {
        fprintf(fp, "%d,%.3f,%.3f,%.3f\n", i, val_volts(data->time_data[i]), data->frequency[i], data->magnitude[i]);
    }

    fclose(fp);
}


// EOF
