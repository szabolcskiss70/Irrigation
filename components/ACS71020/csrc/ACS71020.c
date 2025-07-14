#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"


/*
//Register 0x0B/0x1B
8:0 qvo_fine //Offset fine trimming on current channel 	9	1ff
17:9 sns_fine //Fine gain trimming on the current channel	9 3fe00
20:18 crs_sns //Coarse gain setting	3 1c0000
21 iavgselen //Current Averaging selection	1 200000
25:22 unused //Unused
31:26 ecc – //Error Code Correctio	6 fc000000


//Register 0x0C/0x1C
6:0 rms_avg_1 0 //Average of the rms voltage or current – stage 1	7 7F
16:7 rms_avg_2 0 //Average of the rms voltage or current – stage 2	10 0x1ff80
25:17 – 0 //Reserved
31:26 ecc – //Error Code Correctio	6

//Register 0x0D/0x1D

6:0 pacc_trim – //Trims the active power	7 0x7f
7 ichan_del_en 0 //Enable phase delay on voltage or current channel 1 0x80
8 unused 0 //unused
11:9 chan_del_sel 0 //Sets phase delay on voltage or current channel 3 0xe00
12 unused 0 //unused
20:13 fault 255 //Sets the overcurrent fault threshold	8 0x1fe000
23:21 fltdly 0 //Sets the overcurrent fault delay	3	e00000
24 halfcycle_en 0 //Outputs pulses at every zero crossing when enabled, and every rising edge when disabled 	1 0x1000000
25 squarewave_en 0 //Selects pulse or square wave output for the zero crossing reporting 1 0x2000000
31:26 ecc – //Error Code Correction 6

//Register 0x0E/0x1E
5:0 vevent_cycs 0 //Sets the number of qualifying cycles needed to flag overvoltage or undervoltage	6 0x3f
6 vadc_rate_set 0 //Sample Frequency Selection 1 0x40
7 – 0 //Reserved 
13:8 overvreg 32 //Sets the overvoltage fault threshold 6 0x3f00
19:14 undervreg 32 //Sets the undervoltage fault threshold 6 0xfc000
20 delaycnt_sel 0 //Sets the width of the voltage zero-crossing output pulse 1 0x100000
25:21 unused 0 //Unused
31:26 ecc – //Error Code Correctio 6

Register 0x0F/0x1F
1:0 unused 0 //Unused
8:2 i2c_slv_addr 0 //I2C slave address selection 7 0x1fc
9 i2c_dis_slv_addr 0 //Disable I2C slave address selection circuit 1 0x200
15:10 unused 0 //Unused
17:16 dio_0_sel 0 //Digital output 0 multiplexor selection bits 2 0x30000
19:18 dio_1_sel 0 //Digital output 1 multiplexor selection bits 2 0xc0000
25:20 unused 0 //Unused
31:26 ecc – //Error Code Correctio 6


*/

static const char *TAG = "ACS71020";

static gpio_num_t i2c_gpio_sda = 4;
static gpio_num_t i2c_gpio_scl = 15;
#define ACS71020_address_default 0x66
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)

static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;


i2c_master_bus_handle_t tool_bus_handle=NULL;
i2c_master_dev_handle_t dev_handle;


int init_ACS71020(i2c_master_bus_handle_t in_tool_bus_handle,int chip_addr)
{
	
	if (in_tool_bus_handle==NULL)
	{	
	 i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
     };

     if (i2c_new_master_bus(&i2c_bus_config, &tool_bus_handle) != ESP_OK) {
        return 0;
     }
	}
	else tool_bus_handle=in_tool_bus_handle; //already initialized by other device, just use the handle for ACS71020

	
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };
  
    if (i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) return 0;
 
   return 1;
}



int read_ACS71020(int chip_addr, int data_addr, int *X0, int *X1,int *X2,int *X3)
{
    int len = 4;
	uint8_t *data = malloc(len);	
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, (uint8_t*)&data_addr, 1, data, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        *X0=data[0];
		*X1=data[1];
		*X2=data[2];
		*X3=data[3];
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
    free(data);
    return 0;
}


int write_ACS71020(int chip_addr, int data_addr, int regValue)
{
    int len = 4;
	uint8_t *data = malloc(len + 1);
    data[0] = data_addr;
	data[1] = regValue/0xffffff;
	data[2] = regValue/0xffff;
	data[3] = regValue/0xffff;
	data[4] = regValue%0xff;
 
    esp_err_t ret = i2c_master_transmit(dev_handle, data, len + 1, I2C_TOOL_TIMEOUT_VALUE_MS);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write OK");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Write Failed");
    }
	free(data);
    return 0;
}



long read_ACS71020_register(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright)
{
 int X0,X1,X2,X3;
  long longval;
  //int retval= 
  read_ACS71020(ACS71020_address, reg_address,&X0,&X1,&X2,&X3);
    
  longval=X3;
  longval<<=8;
  longval+=X2;
  longval<<=8;
  longval+=X1;
  longval<<=8;
  longval+=X0;

  longval&=mask;
  longval<<=shiftleft;     //shift sign bit into MSB
  longval/=1<<shiftleft;   //devide the value back and keep the sign 
  longval>>=shiftright;	
return longval;
}






double MeasuredValue(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright,int fractional, float fullscale )
{
  long longval=read_ACS71020_register( ACS71020_address,  reg_address,  mask,  shiftleft, shiftright);

return(fullscale*longval/(1<<fractional));
}
void readEeprom(int ACS71020_address)
{
 double qvo_fine=   MeasuredValue(ACS71020_address, 0x0B, 0x000001ff, 0,0,0,1);
 double sns_fine=   MeasuredValue(ACS71020_address, 0x0B, 0x0003fe00, 0,9,0,1);
 double crs_sns=    MeasuredValue(ACS71020_address, 0x0B, 0x001C0000, 0,18,0,1);
 double iavgselen=  MeasuredValue(ACS71020_address, 0x0B, 0x00200000, 0,21,0,1);
 double ECC_0B=	    MeasuredValue(ACS71020_address, 0x0B, 0xfc000000, 0,26,0,1);
 
 double rms_avg_1=  MeasuredValue(ACS71020_address, 0x0C, 0x0000007f, 0,0,0,1);
 double rms_avg_2=  MeasuredValue(ACS71020_address, 0x0C, 0x0001ff80, 0,7,0,1);
 double ECC_0C=	    MeasuredValue(ACS71020_address, 0x0C, 0xfc000000, 0,26,0,1);
 
 double pacc_trim=     	MeasuredValue(ACS71020_address, 0x0D, 0x0000007f, 0,0,0,1);
 double ichan_del_en=  	MeasuredValue(ACS71020_address, 0x0D, 0x00000080, 0,7,0,1);
 double chan_del_sel=  	MeasuredValue(ACS71020_address, 0x0D, 0x00000e00, 0,9,0,1);
 double fault=  	   	MeasuredValue(ACS71020_address, 0x0D, 0x001FE000, 0,13,0,1);
 double fltdly=  		MeasuredValue(ACS71020_address, 0x0D, 0x00E00000, 0,21,0,1);
 double halfcycle_en=  	MeasuredValue(ACS71020_address, 0x0D, 0x01000000, 0,24,0,1);
 double squarewave_en=  MeasuredValue(ACS71020_address, 0x0D, 0x02000000, 0,25,0,1);
 double ECC_0D=	    	MeasuredValue(ACS71020_address, 0x0D, 0xfc000000, 0,26,0,1);


 double vevent_cycs=     	MeasuredValue(ACS71020_address, 0x0E, 0x0000003f, 0,0,0,1);
 double vadc_rate_set=  	MeasuredValue(ACS71020_address, 0x0E, 0x00000040, 0,6,0,1);
 double overvreg=  			MeasuredValue(ACS71020_address, 0x0E, 0x00003f00, 0,8,0,1);
 double undervreg=  		MeasuredValue(ACS71020_address, 0x0E, 0x000fc000, 0,14,0,1);
 double delaycnt_sel=  	   	MeasuredValue(ACS71020_address, 0x0E, 0x00100000, 0,20,0,1);
 double ECC_0E=	    		MeasuredValue(ACS71020_address, 0x0E, 0xfc000000, 0,26,0,1);

 double i2c_slv_addr=  		MeasuredValue(ACS71020_address, 0x0F, 0x000001fc, 0,2,0,1);
 double i2c_dis_slv_addr=  	MeasuredValue(ACS71020_address, 0x0F, 0x00000200, 0,9,0,1);
 double dio_0_sel=  		MeasuredValue(ACS71020_address, 0x0F, 0x0030000, 0,16,0,1);
 double dio_1_sel=  	   	MeasuredValue(ACS71020_address, 0x0F, 0x00c0000, 0,18,0,1);
 double ECC_0F=	    		MeasuredValue(ACS71020_address, 0x0F, 0xfc000000, 0,26,0,1);


 ESP_LOGI(TAG, "qvo_fine= %lf",qvo_fine);
 ESP_LOGI(TAG, "sns_fine= %lf",sns_fine);
 ESP_LOGI(TAG, "crs_sns= %lf",crs_sns);
 ESP_LOGI(TAG, "iavgselen= %lf",iavgselen);
 ESP_LOGI(TAG, "ECC_0B= %lf",ECC_0B);
 
 ESP_LOGI(TAG, "rms_avg_1= %lf",rms_avg_1);
 ESP_LOGI(TAG, "rms_avg_2= %lf",rms_avg_2);
 ESP_LOGI(TAG, "ECC_0C= %lf",ECC_0C);
 
 ESP_LOGI(TAG, "pacc_trim= %lf",pacc_trim);
 ESP_LOGI(TAG, "ichan_del_en= %lf",ichan_del_en);
 ESP_LOGI(TAG, "chan_del_sel= %lf",chan_del_sel);
 ESP_LOGI(TAG, "fault= %lf",fault);
 ESP_LOGI(TAG, "fltdly= %lf",fltdly);
 ESP_LOGI(TAG, "halfcycle_en= %lf",halfcycle_en);
 ESP_LOGI(TAG, "squarewave_en= %lf",squarewave_en);
 ESP_LOGI(TAG, "ECC_0D= %lf",ECC_0D);
	
			
 ESP_LOGI(TAG, "vevent_cycs= %lf",vevent_cycs);
 ESP_LOGI(TAG, "vadc_rate_set= %lf",vadc_rate_set);
 ESP_LOGI(TAG, "overvreg= %lf",overvreg);
 ESP_LOGI(TAG, "undervreg= %lf",undervreg);
 ESP_LOGI(TAG, "delaycnt_sel= %lf",delaycnt_sel);
 ESP_LOGI(TAG, "ECC_0E= %lf",ECC_0E);
	 
 ESP_LOGI(TAG, "i2c_slv_addr= %lf",i2c_slv_addr);
 ESP_LOGI(TAG, "i2c_dis_slv_addr= %lf",i2c_dis_slv_addr);
 ESP_LOGI(TAG, "dio_0_sel= %lf",dio_0_sel);
 ESP_LOGI(TAG, "dio_1_sel= %lf",dio_1_sel);
 ESP_LOGI(TAG, "ECC_0F= %lf",ECC_0F);
	 


}


void readShadow(int ACS71020_address)
{
 double qvo_fine=   MeasuredValue(ACS71020_address, 0x1B, 0x000001ff, 0,0,0,1);
 double sns_fine=   MeasuredValue(ACS71020_address, 0x1B, 0x0003fe00, 0,9,0,1);
 double crs_sns=    MeasuredValue(ACS71020_address, 0x1B, 0x001C0000, 0,18,0,1);
 double iavgselen=  MeasuredValue(ACS71020_address, 0x1B, 0x00200000, 0,21,0,1);
 double ECC_0B=	    MeasuredValue(ACS71020_address, 0x1B, 0xfc000000, 0,26,0,1);
 
 double rms_avg_1=  MeasuredValue(ACS71020_address, 0x1C, 0x0000007f, 0,0,0,1);
 double rms_avg_2=  MeasuredValue(ACS71020_address, 0x1C, 0x0001ff80, 0,7,0,1);
 double ECC_0C=	    MeasuredValue(ACS71020_address, 0x1C, 0xfc000000, 0,26,0,1);
 
 double pacc_trim=     	MeasuredValue(ACS71020_address, 0x1D, 0x0000007f, 0,0,0,1);
 double ichan_del_en=  	MeasuredValue(ACS71020_address, 0x1D, 0x00000080, 0,7,0,1);
 double chan_del_sel=  	MeasuredValue(ACS71020_address, 0x1D, 0x00000e00, 0,9,0,1);
 double fault=  	   	MeasuredValue(ACS71020_address, 0x1D, 0x001FE000, 0,13,0,1);
 double fltdly=  		MeasuredValue(ACS71020_address, 0x1D, 0x00E00000, 0,21,0,1);
 double halfcycle_en=  	MeasuredValue(ACS71020_address, 0x1D, 0x01000000, 0,24,0,1);
 double squarewave_en=  MeasuredValue(ACS71020_address, 0x1D, 0x02000000, 0,25,0,1);
 double ECC_0D=	    	MeasuredValue(ACS71020_address, 0x1D, 0xfc000000, 0,26,0,1);


 double vevent_cycs=     	MeasuredValue(ACS71020_address, 0x1E, 0x0000003f, 0,0,0,1);
 double vadc_rate_set=  	MeasuredValue(ACS71020_address, 0x1E, 0x00000040, 0,6,0,1);
 double overvreg=  			MeasuredValue(ACS71020_address, 0x1E, 0x00003f00, 0,8,0,1);
 double undervreg=  		MeasuredValue(ACS71020_address, 0x1E, 0x000fc000, 0,14,0,1);
 double delaycnt_sel=  	   	MeasuredValue(ACS71020_address, 0x1E, 0x00100000, 0,20,0,1);
 double ECC_0E=	    		MeasuredValue(ACS71020_address, 0x1E, 0xfc000000, 0,26,0,1);

 double i2c_slv_addr=  		MeasuredValue(ACS71020_address, 0x1E, 0x000001fc, 0,2,0,1);
 double i2c_dis_slv_addr=  	MeasuredValue(ACS71020_address, 0x1E, 0x00000200, 0,9,0,1);
 double dio_0_sel=  		MeasuredValue(ACS71020_address, 0x1E, 0x0030000, 0,16,0,1);
 double dio_1_sel=  	   	MeasuredValue(ACS71020_address, 0x1E, 0x00c0000, 0,18,0,1);
 double ECC_0F=	    		MeasuredValue(ACS71020_address, 0x1E, 0xfc000000, 0,26,0,1);


 ESP_LOGI(TAG, "qvo_fine= %lf",qvo_fine);
 ESP_LOGI(TAG, "sns_fine= %lf",sns_fine);
 ESP_LOGI(TAG, "crs_sns= %lf",crs_sns);
 ESP_LOGI(TAG, "iavgselen= %lf",iavgselen);
 ESP_LOGI(TAG, "ECC_0B= %lf",ECC_0B);
 
 ESP_LOGI(TAG, "rms_avg_1= %lf",rms_avg_1);
 ESP_LOGI(TAG, "rms_avg_2= %lf",rms_avg_2);
 ESP_LOGI(TAG, "ECC_0C= %lf",ECC_0C);
 
 ESP_LOGI(TAG, "pacc_trim= %lf",pacc_trim);
 ESP_LOGI(TAG, "ichan_del_en= %lf",ichan_del_en);
 ESP_LOGI(TAG, "chan_del_sel= %lf",chan_del_sel);
 ESP_LOGI(TAG, "fault= %lf",fault);
 ESP_LOGI(TAG, "fltdly= %lf",fltdly);
 ESP_LOGI(TAG, "halfcycle_en= %lf",halfcycle_en);
 ESP_LOGI(TAG, "squarewave_en= %lf",squarewave_en);
 ESP_LOGI(TAG, "ECC_0D= %lf",ECC_0D);
	
			
 ESP_LOGI(TAG, "vevent_cycs= %lf",vevent_cycs);
 ESP_LOGI(TAG, "vadc_rate_set= %lf",vadc_rate_set);
 ESP_LOGI(TAG, "overvreg= %lf",overvreg);
 ESP_LOGI(TAG, "undervreg= %lf",undervreg);
 ESP_LOGI(TAG, "delaycnt_sel= %lf",delaycnt_sel);
 ESP_LOGI(TAG, "ECC_0E= %lf",ECC_0E);
	 
 ESP_LOGI(TAG, "i2c_slv_addr= %lf",i2c_slv_addr);
 ESP_LOGI(TAG, "i2c_dis_slv_addr= %lf",i2c_dis_slv_addr);
 ESP_LOGI(TAG, "dio_0_sel= %lf",dio_0_sel);
 ESP_LOGI(TAG, "dio_1_sel= %lf",dio_1_sel);
 ESP_LOGI(TAG, "ECC_0F= %lf",ECC_0F);
	 


}



void readMeasValues()
{
/* double i=    MeasuredValue(ACS71020_address, 0x2B, 0x0001ffff,15, 0,15,30.0);
 double u=    MeasuredValue(ACS71020_address, 0x2A, 0x0001ffff,15, 0,16,0.275*(R1_4+Rs)/Rs); 
 double irms= MeasuredValue(ACS71020_address, 0x20, 0x7fff0000, 0,16,14,30.0);
 double urms= MeasuredValue(ACS71020_address, 0x20, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs); 
 double num=  MeasuredValue(ACS71020_address, 0x25, 0x000001ff, 0, 0,0,1); 
 double p=    MeasuredValue(ACS71020_address, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs);
*/
}
