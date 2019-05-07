#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
//#include<conio.h>


//#define  DRDY  RPI_V2_GPIO_P1_11         //P0
#define  DRDY  17
//#define  RST  RPI_V2_GPIO_P1_12     //P1
#define  RST  18
//#define	SPICS	RPI_V2_GPIO_P1_15	//P3
#define  SPICS  22

#define CS_1() bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)

#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))

#define RST_1() 	bcm2835_gpio_write(RST,HIGH)
#define RST_0() 	bcm2835_gpio_write(RST,LOW)
#define Single_ended 0
#define Differential 1


/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long


uint8_t int_on = 0 ;
uint32_t Data_ready_count ;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

//---------------------------------------------------------------------------------------------------------
/* gain channel */
typedef enum{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/*GAIN   2 */
	ADS1256_GAIN_4			= (2),	/*GAIN   4 */
	ADS1256_GAIN_8			= (3),	/*GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN  16 */
	ADS1256_GAIN_32			= (5),	/*GAIN    32 */
	ADS1256_GAIN_64			= (6),	/*GAIN    64 */
}ADS1256_GAIN_E;
//---------------------------------------------------------------------------------------------------------
typedef enum{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;
#define ADS1256_DRAE_COUNT = 15;
//---------------------------------------------------------------------------------------------------------
typedef struct{
	ADS1256_GAIN_E Gain;		/* GAIN  */
	ADS1256_DRATE_E DataRate;	/* DATA output  speed*/
	int32_t AdcNow[8];			/* ADC  Conversion value */
	uint8_t Channel;			/* The current channel*/
	uint8_t ScanMode;	/*Scanning mode,   0  Single-ended input  8 channel 1 Differential input  4 channel*/
}ADS1256_VAR_T;
//---------------------------------------------------------------------------------------------------------
/*Register definition Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};
//---------------------------------------------------------------------------------------------------------
/* Command definition Table 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};
//---------------------------------------------------------------------------------------------------------
ADS1256_VAR_T g_tADS1256;
//Sapling rate conf----------------------------------------------------------------------------------------
static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] ={
	0xF0,		/*reset the default values  */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};
//---------------------------------------------------------------------------------------------------------
uint8_t ADS1256_ReadChipID(void); //ok
static void ADS1256_WaitDRDY(void);
static uint8_t ADS1256_ReadReg(uint8_t _RegID); //ok
static void ADS1256_Send8Bit(uint8_t _data);//ok
static uint8_t ADS1256_Recive8Bit(void);//ok
static void ADS1256_DelayDATA(void);//ok
void  bsp_DelayUS(uint64_t micros);//ok
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);//ok
static void ADS1256_WriteCmd(uint8_t _cmd);
void ADS1256_ISR(void);
static void ADS1256_SetChannal(uint8_t _ch);
static int32_t ADS1256_ReadData(void);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);//ok




//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_ReadChipID
//	function: Read the chip ID
//	parameter: _cmd : NULL
//	The return value: four high status register
//---------------------------------------------------------------------------------------------------------
uint8_t ADS1256_ReadChipID(void){
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_WaitDRDY
//	function: delay time  wait for automatic calibration
//	parameter:  NULL
//	The return value:  NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_WaitDRDY(void){
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");
	}
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_ReadReg
//	function: Read  the corresponding register
//	parameter: _RegID: register  ID
//	The return value: read register value
//---------------------------------------------------------------------------------------------------------
static uint8_t ADS1256_ReadReg(uint8_t _RegID){
	uint8_t read;

	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
	ADS1256_Send8Bit(0x00);	/* Write the register number */

	ADS1256_DelayDATA();	/*delay time */

	read = ADS1256_Recive8Bit();	/* Read the register values */
	CS_1();	/* SPI   cs  = 1 */

	return read;
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_Send8Bit
//	function: SPI bus to send 8 bit data
//	parameter: _data:  data
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_Send8Bit(uint8_t _data){

	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_Recive8Bit
//	function: SPI bus receive function
//	parameter: NULL
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static uint8_t ADS1256_Recive8Bit(void){
	uint8_t read = 0;
	//read = bcm2835_spi_transfer(0xff);
	read = bcm2835_spi_transfer(0xff);
	return read;
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_WriteReg
//	function: Write the corresponding register
//	parameter: _RegID: register  ID
//			 _RegValue: register Value
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue){
	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/*Write command register */
	ADS1256_Send8Bit(0x00);		/*Write the register number */

	ADS1256_Send8Bit(_RegValue);	/*send register value */
	CS_1();	/* SPI   cs = 1 */
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_DelayDATA
//	function: delay
//	parameter: NULL
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_DelayDATA(void){
	bsp_DelayUS(6.5);	/* The minimum time delay 6.5us */
}
//---------------------------------------------------------------------------------------------------------
void  bsp_DelayUS(uint64_t micros){
		bcm2835_delayMicroseconds (micros);
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_CfgADC
//	function: The configuration parameters of ADC, gain and data rate
//	parameter:	_gain:gain 1-64
//				_drate:  data  rate
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate){
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* Storage ads1256 register configuration parameters */
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
        buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited
        //ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));
        //buf[1] = ((0 << 4) | (1 << 0)) ; // AIN0 PSEL0 and AIN1 NSEL0
		buf[1] = 0x08;
		//ADCON
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		//DRATE
		buf[3] = s_tabDataRate[_drate];	// DRATE -> SPS;

		CS_0();	/* SPIƬѡ = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* Write command register, send the register address */
		ADS1256_Send8Bit(0x03);			/* Register number 4,Initialize the number  -1*/

		ADS1256_Send8Bit(buf[0]);	/* Set the status register */
		ADS1256_Send8Bit(buf[1]);	/* Set the input channel parameters */
		ADS1256_Send8Bit(buf[2]);	/* Set the ADCON control register,gain */
		ADS1256_Send8Bit(buf[3]);	/* Set the output rate */

		CS_1();	/* SPI  cs = 1 */
	}

	bsp_DelayUS(50);
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_StartScan
//	function: Configuration DRDY PIN for external interrupt is triggered
//	parameter: _ucDiffMode : 0  Single-ended input  8 channel 1 Differential input  4 channel
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
void ADS1256_StartScan(uint8_t _ucScanMode){
	g_tADS1256.ScanMode = _ucScanMode;
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_Scan
//	function:
//	parameter:NULL
//	The return value:  1
//---------------------------------------------------------------------------------------------------------
uint8_t ADS1256_Scan(void){
	if (DRDY_IS_LOW())
	{
		ADS1256_ISR();
		return 1;
	}

	return 0;
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_ISR
//	function: Collection procedures
//	parameter: NULL
//	The return value:  NULL
//---------------------------------------------------------------------------------------------------------
void ADS1256_ISR(void){
	//0  Single-ended input  8 channel
	Data_ready_count++ ;
	if (g_tADS1256.ScanMode == 0)	 
	{

		ADS1256_SetChannal(g_tADS1256.Channel);	/*Switch channel mode */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);//CMD_SYNC=0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);//CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}
		int_on = 1 ;
	}
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_SetChannal
//	function: Configuration channel number
//	parameter:  _ch:  channel number  0--7
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_SetChannal(uint8_t _ch){
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_WriteCmd
//	function: Sending a single byte order
//	parameter: _cmd : command
//	The return value: NULL
//---------------------------------------------------------------------------------------------------------
static void ADS1256_WriteCmd(uint8_t _cmd){
	CS_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI  cs  = 1 */
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_GetAdc
//	function: read ADC value
//	parameter:  channel number 0--7
//	The return value:  ADC vaule (signed number)
//---------------------------------------------------------------------------------------------------------
int32_t ADS1256_GetAdc(uint8_t _ch){
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	iTemp = g_tADS1256.AdcNow[_ch];

	return iTemp;
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_ReadData
//	function: read ADC value
//	parameter: NULL
//	The return value:  NULL
//---------------------------------------------------------------------------------------------------------
static int32_t ADS1256_ReadData(void){
	uint32_t read = 0;
    static uint8_t buf[3];

	CS_0();	/* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */

	ADS1256_DelayDATA();	/*delay time  */

	/*Read the sample results 24bit*/
    buf[0] = ADS1256_Recive8Bit();
    buf[1] = ADS1256_Recive8Bit();
    buf[2] = ADS1256_Recive8Bit();

    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];

	CS_1();	/* SPIƬѡ = 1 */

	/* Extend a signed number*/
    if (read & 0x800000)
    {
	    read |= 0xFF000000;
    }

	return (int32_t)read;
}
//---------------------------------------------------------------------------------------------------------
//	name: ADS1256_SaveData
//	function:  Take iTemp and tranfer to a txt file
//	parameter: udata
//	The return value:  NULL*/
//---------------------------------------------------------------------------------------------------------
static void ADS1256_SaveData (int32_t col0, int32_t col1, int32_t col2){
//FILE 0---------------------------------------------------------------------------------------------------
	FILE *datos0 ;					 // necesary to work with txt files
	datos0 = fopen("ADXL354_data.txt", "a+") ; //open the txt file in writing mode and write after the last line
	if (col0 < 0){
		col0 = -col0 ;
		fprintf(datos0,"-%ld.%03ld%03ld\t", col0 /1000000, (col0%1000000)/1000, col0%1000) ;
	}		
	else{
		fprintf(datos0," %ld.%03ld%03ld\t", col0 /1000000, (col0%1000000)/1000, col0%1000) ;	
	}
//---------------------------------------------------------------------------------------------------------	
	if (col1 < 0){
		col1 = -col1 ;
		fprintf(datos0,"-%ld.%03ld%03ld\t", col1 /1000000, (col1%1000000)/1000, col1%1000) ;
	}		
	else{
		fprintf(datos0," %ld.%03ld%03ld\t", col1 /1000000, (col1%1000000)/1000, col1%1000) ;	
	}
//---------------------------------------------------------------------------------------------------------	
	if (col2 < 0){
		col2 = -col2 ;
		fprintf(datos0,"-%ld.%03ld%03ld\n", col2 /1000000, (col2%1000000)/1000, col2%1000) ;
		fflush(stdout) ;
	}		
	else{
		fprintf(datos0," %ld.%03ld%03ld\n", col2 /1000000, (col2%1000000)/1000, col2%1000) ;	
		fflush(stdout) ;
	}
	fclose(datos0) ;
}

//MAIN Program---------------------------------------------------------------------------------------------
int  main(){
    uint8_t id;
    int32_t adc[8];
    uint8_t ch_num = 8 ;
	uint32_t i;
//BUFFER---------------------------------------------------------------------------------------------------
	int32_t buf[8] ;
	uint32_t size_ch0 = 0 ;
	uint32_t size_ch1 = 0 ;
	uint32_t size_ch2 = 0 ;
	uint32_t datacount ;
	uint32_t datatime ;
	uint32_t sample_rate ; 
	float sample_rate_per_channel ;
	uint8_t select_sps ;
	uint8_t case_sps = 0 ;
	struct{
		unsigned int adc_count : 3 ;	
	}Target_sample ;
	
	Target_sample.adc_count = 7 ;
//SAMPLE RATE MENU---------------------------------------------------------------------------------------------------
	do{
		system("clear");
		printf("\t Samples rate for the ADC\n") ;
		printf("\t*** Choose the sample rate for the acquisition: *** \n") ;
		printf("\t 30K sps   (0)\n");
		printf("\t 15K sps   (1)\n");
    	printf("\t 7K5 sps   (2)\n");
    	printf("\t 3.75K sps (3)\n");
    	printf("\t 2K sps    (4)\n");
    	printf("\t 800 sps    (5)\n");
    	printf("\t 500 sps   (6)\n");
    	printf("\t 100 sps   (7)\n");
    	printf("\t 60 sps    (8)\n");
    	printf("\t 50 sps    (9)\n");
    	printf("\t 30 sps    (10)\n");
    	printf("\t 25 sps    (11)\n");
    	printf("\t 15 sps    (12)\n");
    	printf("\t 10 sps    (13)\n");
    	printf("\t 5 sps     (14)\n");
    	printf("\t 2.5 sps   (15)\n");
    	printf("\t EXIT      (16)\n");
    	printf("\t------------------------------\n");
		printf("\t *** Enter Your Choice *** \n");
		printf("\t------------------------------\n\n");
		printf("\tOption for the sample rate: ") ;
		scanf("%d", &select_sps) ;
		fflush(stdout);

		switch (select_sps){
			case 0:
				printf("\n\tYou Selected 30.000 sps!");
				sample_rate = 30000 ;
				case_sps = 1 ;
				break ;
			case 1:
				printf("\n\tYou Selected 15.000 sps!");
				sample_rate = 15000 ;
				case_sps = 1 ;
				break ;
			case 2:
				printf("\n\tYou Selected 7500 sps!");			
				sample_rate = 7500 ;
				case_sps = 1 ;
				break ;
			case 3:
				printf("\n\tYou Selected 3.750 sps!");
				sample_rate = 3750 ;
				case_sps = 1 ;
				break ;
			case 4:
				printf("\n\tYou Selected 2.000 sps!");
				sample_rate = 2000 ;
				case_sps = 1 ;
				break ;
			case 5:
				printf("\n\tYou Selected 800 sps!");
				sample_rate = 800 ;
				case_sps = 1 ;
				break ;
			case 6:
				printf("\n\tYou Selected 500 sps!");
				sample_rate = 500 ;
				case_sps = 1 ;
				break ;
			case 7:
				printf("\n\tYou Selected 100 sps!");
				sample_rate = 100 ;
				case_sps = 1 ;
				break ;
			case 8:
				printf("\n\tYou Selected 60 sps!");
				sample_rate = 60 ;
				case_sps = 1 ;
				break ;
			case 9:
				printf("\n\tYou Selected 50 sps!");
				sample_rate = 50 ;
				case_sps = 1 ;
				break ;
			case 10:
				printf("\n\tYou Selected 30 sps!");
				sample_rate = 30 ;
				case_sps = 1 ;
				break ;
			case 11:
				printf("\n\tYou Selected 25 sps!");
				sample_rate = 25 ;
				case_sps = 1 ;
				break ;
			case 12:
				printf("\n\tYou Selected 15 sps!");
				sample_rate = 15 ;
				case_sps = 1 ;
				break ;
			case 13:
				printf("\n\tYou Selected 10 sps!");
				sample_rate = 10 ;
				case_sps = 1 ;
				break ;
			case 14:
				printf("\n\tYou Selected 5 sps!");
				sample_rate = 5 ;
				case_sps = 1 ;
				break ;
			case 15:
				printf("\n\tYou Selected 2.5 sps!");
				sample_rate = 2.5 ;
				case_sps = 1 ;
				break ;	
			case 16:
				printf("\n\tclosing the program....");
				abort();
			default:
				printf("\n\t\n\nINVALID SELECTION...Please try again\n");													
				case_sps = 0 ;
		}
	}while(case_sps!=1) ;
//TIME MENU---------------------------------------------------------------------------------------------------	
	printf("\n\tEnter the time in secons for the acquisition: ") ;
	scanf("%ld", &datatime) ;
	sample_rate_per_channel = 101.25 ;
	datacount = datatime * (sample_rate_per_channel) ; 
	fflush(stdin) ;
	//pointer for each analog input
	int32_t *ch0 ; int32_t *ch1 ; int32_t *ch2 ;
//ch0 memory block-----------------------------------------------------------------------------------------
  	ch0 = malloc(sizeof(int32_t) * datacount); /* allocate memory for datacount int's */
 	if (!ch0) { /* If data == 0 after the call to malloc, allocation failed for some reason */
    	perror("Error allocating memory for channel 0");
    	abort();
	}
  	/* ch points to a valid block of memory.
     clearing block. */
  	memset(ch0, 0.0, sizeof(int32_t)*datacount);

    if (!bcm2835_init())
    	return 1;
//ch1 memory block-----------------------------------------------------------------------------------------
  	ch1 = malloc(sizeof(int32_t) * datacount); /* allocate memory for datacount int's */
 	if (!ch1) { /* If data == 0 after the call to malloc, allocation failed for some reason */
    	perror("Error allocating memoryfor channel 1");
    	abort();
	}
  	/* ch points to a valid block of memory.
     clearing block. */
  	memset(ch1, 0.0, sizeof(int32_t)*datacount);

    if (!bcm2835_init())
    	return 1;
//ch2 memory block-----------------------------------------------------------------------------------------
  	ch2 = malloc(sizeof(int32_t) * datacount); /* allocate memory for datacount int's */
 	if (!ch2) { /* If data == 0 after the call to malloc, allocation failed for some reason */
    	perror("Error allocating memory for channel 2");
    	abort();
	}
  	/* ch points to a valid block of memory.
     clearing block. */
  	memset(ch2, 0.0, sizeof(int32_t)*datacount);

    if (!bcm2835_init())
    	return 1;	
//TXT file open--------------------------------------------------------------------------------------------
	FILE *datos0 = NULL;	
	datos0 = fopen("ADXL354_data.txt", "w");
	if (datos0 == NULL){
    	printf("Error opening file 0!\n") ;
    	exit(1) ;
	}
//SPI setup------------------------------------------------------------------------------------------------

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);   //default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                //default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);//default
//GPIO and Interrupt setup---------------------------------------------------------------------------------
    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
    bcm2835_gpio_write(SPICS, HIGH);
    bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);
//ADS1256 ID-----------------------------------------------------------------------------------------------
	id = ADS1256_ReadChipID();
	printf("\n\n\tID request....");
	if (id != 3){
		printf("\n\tError, ASD1256 Chip ID = 0x%d", (int)id);
	}
	else{
		printf("\n\tOk, ASD1256 Chip ID = 0x%d", (int)id);
	}
//ADS1256 config-------------------------------------------------------------------------------------------
  	
  	switch(select_sps){
  		case 0:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_30000SPS);
			break ;
		case 1:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15000SPS);
			break ;
		case 2:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_7500SPS);
			break ;
		case 3:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_3750SPS);
			break ;
		case 4:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_2000SPS);
			break ;
		case 5:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);
			break ;
		case 6:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_500SPS);
			break ;
		case 7:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_100SPS);
			break ;
		case 8:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_60SPS);
			break ;
		case 9:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_50SPS);
			break ;
		case 10:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_30SPS);
			break ;
		case 11:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_25SPS);
			break ;
		case 12:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15SPS);
			break ;
		case 13:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_10SPS);
			break ;
		case 14:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_5SPS);
			break ;
		case 15:
			ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_2d5SPS);
			break ;
		case 16:
			printf("\n\tCan't configure the sample rate");
			abort();
		default:
			printf("\n\t\n\nConfiguring the default sample rate = 1000 sps");													
  	}
    printf("\n\tADS1256 Ready");
    //Single_ended or Differential
    //start with channel = 0
    ADS1256_StartScan(Single_ended); 
    printf("\n\tAcquiring %ld samples at %f SPS per channel...", datacount, sample_rate_per_channel);
    fflush(stdout) ;
//LOOP-----------------------------------------------------------------------------------------------------
		while(1){
	    	while((ADS1256_Scan() == 0)) ;
	    		/*if (DRDY_IS_LOW()){
	    			Data_ready_count++ ;
	    		}*/
								
				if (int_on = 1){
					//printf("AdcNow: %ld \n", g_tADS1256.AdcNow[Target_sample.adc_count]) ;
					//printf("Target_sample: %d \n", Target_sample.adc_count) ;
					
					//printf("----- \n ") ;
					if(Target_sample.adc_count==0){
						ch0[size_ch0] = g_tADS1256.AdcNow[Target_sample.adc_count] ;
							size_ch0++;
					}
					if(Target_sample.adc_count==1){
						ch1[size_ch1] = g_tADS1256.AdcNow[Target_sample.adc_count] ;
							size_ch1++;
					}
					if(Target_sample.adc_count==2){
						ch2[size_ch2] = g_tADS1256.AdcNow[Target_sample.adc_count] ;
							size_ch2++;
					}
					Target_sample.adc_count++ ;
					int_on = 0 ;
				}//if(int_on)
				
				if(size_ch0 == datacount && size_ch1 == datacount && size_ch2 == datacount) {
					printf("\n\tData ready is low: %ld ", Data_ready_count) ;
	            	printf ("\n\tbuffer is full ") ;
	            	bcm2835_spi_end() ;
					bsp_DelayUS(100000) ;
	            	break ;
	        	}
		}//while(1)
		printf("\n\tSPI off") ;
		printf("\n\tSaving data...") ;
		fflush(stdout) ;
		for (i=1; i < size_ch0; i++){
		//	printf("data to buffer %d \n", i);
			ADS1256_SaveData(ch0[i]*100/167, ch1[i]*100/167, ch2[i]*100/167) ;
		}
		fclose(datos0) ;
		free(ch0) ;
		free(ch1) ;
		free(ch2) ;
    	bcm2835_close() ;
    	printf("\n\tdone\n");
    return 0 ;
}//int main











































