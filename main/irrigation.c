#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <time.h>
#include <sys/time.h>
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

//#include "driver/gpio.h"

#include <driver/spi_master.h>
#include <stdio.h>

#include "sdkconfig.h"

#include "ds18b20.h" 

#include <stdint.h>
#include <stddef.h>
#include "esp_wifi.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "lora.h"
#include "math.h"
#include "esp_timer.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include "ssd1306.h"
#include "font8x8_basic.h"

#include "driver/pulse_cnt.h"


#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"


#include "host/util/util.h"
#include "esp_mac.h"
#include "esp_crt_bundle.h"




#include "pump_control.h"
#include "DIO.h"
#include "log2file.h"


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
//#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
//#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_SSID      "DIGI-KISS2"
#define EXAMPLE_ESP_WIFI_PASS      "cll471ipg381ivw7342"


#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif








typedef bool T_MQTT_Sub_Callback(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32]);





#include "ACS71020.h"





/*#define WIFI_SSID "ZONG MBB-E8372-6714"
#define WIFI_PASS "15625888"*/


#define BROKER_URL "mqtt://szabolcskiss.ddns.net:1883"
char *maintopic="IRRIGATION";


const esp_app_desc_t *app_desc;
char new_Firmware_version[16];

//extern int read_ACS71020(int chip_addr, int data_addr, int *X0, int *X1,int *X2,int *X3);
extern double MeasuredValue(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright,int fractional, float fullscale );
//extern void readEeprom(int ACS71020_address_default);
//extern void readShadow(int ACS71020_address_default);
extern int init_ACS71020(i2c_master_bus_handle_t in_tool_bus_handle,int chip_addr);
extern int write_ACS71020(int chip_addr, int data_addr, int regValue);
extern long read_ACS71020_register(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright);
void read_ACS71020_register2(int reg_addr,long value);


#define ACS71020_address_default 0x66
#define Rs 1000.0
#define R1_4 2000000.0

#define num_states 15
typedef enum {MAN_ON,PROG_STARTED,MAN_RESUMED,PROG_RESUMED,INIT,ENABLED,DISABLED,MAN_OFF,SUSPENDED,DELAY,PROG_FINISHED,PROG_END,IDLE,REBOOTED,NOREQUEST} T_states;
char* str_states[num_states]={"MAN_ON","PROG_STARTED","MAN_RESUMED","PROG_RESUMED","INIT","ENABLED","DISABLED","MAN_OFF","SUSPENDED","DELAY","PROG_FINISHED","PROG_END","IDLE","REBOOTED","NOREQUEST"};
char* str_short_states[num_states]={"ON_M","START_P","RES_M","RES_P","INIT","ENAB","DIS","OFF_M","SUSP","DELAY","FIN","END_P","IDLE","REBO","NO_REQ"};
typedef enum {OFF,LEVEL,POWER,CT,STACK,LOG,VOLUME} T_measure_mode;
typedef enum {USE_BLE,USE_WIFI,MAIN_TASK,POWERMETER_TASK,TEMPSENSOR,CURRENTSENSOR,MEASURE_LEVEL,MEASURE_POWER} T_run_mode_bits;
int run_mode=(1<<USE_BLE) | (1<<USE_WIFI);
bool USE_MCP=false;



static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static const int MQTT_CONNECTED_BIT = BIT3;
static EventGroupHandle_t s_wifi_event_group;


T_measure_mode measure_mode=OFF;
int pump_restart_delay=10;


int SNTP_synchronized=0;
bool FW_update_available=false;
char* url_buf="";
char* url_buf_szabolcskiss="http://szabolcskiss.ddns.net/irrigation.bin";
char* url_buf_git="https://github.com/szabolcskiss70/irrigation/raw/main/release/irrigation.bin";
char* OTA_SOURCE_URL="https://github.com/szabolcskiss70/irrigation/raw/main/release/irrigation.bin";
void ota_update_task(void *pvParameter);
void switch_channel(int ch, T_states status);
void Save_data_to_NVS();

char MQTT_BLE_answer[2048]="";




void mainTask(void *pvParameters);
static SemaphoreHandle_t mutex;
static SemaphoreHandle_t mutex_mqtt_ble;
static SemaphoreHandle_t MAIN_TASK_mutex;

static const char *TAG = "irrigation";
static int prevhour=0;
float water_level=0;

int wifi_retry_count=0;
time_t now_life_received=0;
time_t now_life_sent=0;

const int DS_PIN = 17; //GPIO where you connected ds18b20

#define GPIO_OUTPUT_PUMP_1   23
#define GPIO_OUTPUT_OUT_2    12
#define GPIO_OUTPUT_OUT_3    2
#define GPIO_OUTPUT_OUT_4    25
#define GPIO_Vext            21

#define ISOLATED_INPUT_PUMP_1 36
#define ISOLATED_INPUT_2      39

#define PRG_BUTTON 0


#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_PUMP_1 | 1ULL<<GPIO_OUTPUT_OUT_2 | 1ULL<<GPIO_OUTPUT_OUT_3 | 1ULL<<GPIO_OUTPUT_OUT_4 | 1ULL<<GPIO_Vext)
//#define GPIO_INPUT_PIN_SEL (1ULL<<ISOLATED_INPUT_PUMP_1 | 1ULL<<ISOLATED_INPUT_2)


float temperature=0;

double powerconsumptionWs=0;

esp_mqtt_client_handle_t mqtt_client;
int mqtt_connected = 0;

SSD1306_t Display;
#define PIN_SDA 4
#define PIN_SCL 15

#define PIN_OLED_RESET 16

//#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
//#include "esp_adc_cal.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2            1
#endif

#if EXAMPLE_USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#else
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if EXAMPLE_USE_ADC2

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

bool do_calibration1_chan0=false;
bool do_calibration1_chan1=false;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;



static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//static esp_adc_cal_characteristics_t *adc_chars;
//static const adc_channel_t channel = ADC_CHANNEL_2;  
//static const adc_atten_t atten = ADC_ATTEN_DB_11;
//static const adc_unit_t unit = ADC_UNIT_1;

    adc_oneshot_unit_handle_t adc1_handle;

/*
static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}*/


typedef struct{
	int32_t on_time;
	int32_t off_time;
	char weekdays[8];
	int32_t duration; //[MIN]
	int32_t volume;	  //[l]
} T_chedule_data;


#define CHANNEL_NUM 3
#define PERIODS 10
typedef struct{
	char Name[8];
	int Valve_GPIO_OUTPUT;
	int channel_disabled; //=0;
	bool ontime_of_period_added_to_daily;
	int prev_daily_period_ontimes; //sum of finished period ontimes
	int period_ontime;			//sum of on times in period (end-resume+suspend-resume+suspend-start)
	time_t last_switch_on_time; //timestamp of pump starts
    int period_volume; //=0;
	int daily_volume; //=0;
	bool Channel_pump_ON; //=false;
	T_chedule_data Chedule_array[PERIODS]; 
	T_states channel_states_before_suspend;// =INIT;
    T_states channel_state; //=INIT;
	time_t status_change_time[num_states];
    T_states manual_change_request;  //=NOREQUEST;	
	int requested_ontime;
	bool fix_preassure;
} T_channel;
T_channel channels[CHANNEL_NUM];


void init_channel(int ch, char* name, int GPIO, bool fix_preassure)
{
	//strcpy(channels[ch].Name,name); //from NVS
	channels[ch].Valve_GPIO_OUTPUT=GPIO;
	channels[ch].channel_disabled=0;
	channels[ch].ontime_of_period_added_to_daily=false;
	channels[ch].prev_daily_period_ontimes=0;
	channels[ch].last_switch_on_time=0;
    channels[ch].period_volume=0;
	channels[ch].period_ontime=0;
	channels[ch].daily_volume=0;
	channels[ch].Channel_pump_ON=false;
	memset(channels[ch].Chedule_array,0,sizeof(channels[ch].Chedule_array)); 
	channels[ch].channel_states_before_suspend=INIT;
    channels[ch].channel_state=INIT;
    channels[ch].manual_change_request=NOREQUEST;	
	channels[ch].fix_preassure=fix_preassure;
	set_DIO_direction(GPIO,GPIO_MODE_OUTPUT);
	for(int i=0; i<num_states;i++) channels[ch].status_change_time[i]=-1;
	channels[ch].requested_ontime=300; //[s]
}





/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
 //   ESP_LOGI(TAG, "Notification of a time synchronization event");
	SNTP_synchronized=1;
	ESP_LOGI(TAG, "SNTP_synchronized");
}


void to_upper(const char *str, char *out_str)
{
  while(*str != 0) {
    *out_str = toupper(*str);
    ++str;
    ++out_str;
  }
  *out_str = 0;
}

void to_lower(const char *str, char *out_str)
{
  while(*str != 0) {
    *out_str = tolower(*str);
    ++str;
    ++out_str;
  }
  *out_str = 0;
}




 int my_esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, char* subtopic,int par)
 {
   int msg_id;
   char* full_topicname=calloc(1,strlen(maintopic)+1+strlen(subtopic)+1);
   sprintf(full_topicname,"%s/%s",maintopic,subtopic);
   printf("Full_topic_name=%s\n",full_topicname);
   msg_id=esp_mqtt_client_subscribe(client, full_topicname, par); 
   /*to_lower(subtopic,subtopic);
   sprintf(full_topicname,"%s/%s",maintopic,subtopic);
   printf("Full_topic_name=%s\n",full_topicname);
   msg_id=esp_mqtt_client_subscribe(client, full_topicname, par); */
   free(full_topicname);
   return (msg_id);
 }
 
 int my_esp_mqtt_client_publish(esp_mqtt_client_handle_t client, char* subtopic,const char* message,int par1, int par2, int par3)
 {
   if (mqtt_connected)
   {	
   int msg_id;
   char* full_topicname;
   bool dynamic=false;
   if (strncmp(subtopic,maintopic,strlen(maintopic))==0) full_topicname=subtopic;
   else
   {
    full_topicname=calloc(1,strlen(maintopic)+1+strlen(subtopic)+1);
	dynamic=true;
    sprintf(full_topicname,"%s/%s",maintopic,subtopic);
   }

   msg_id = esp_mqtt_client_publish(mqtt_client,  full_topicname, message, par1, par2, par3);   
   ESP_LOGI("PUBLISH", "ID:%d, Topic:%s, Message:%s ", msg_id,full_topicname,message);
   if (dynamic) free(full_topicname);
   return (msg_id);
   }
   else return 0;
 }  
 
 void Publish_file(char* filename)
{
    char buf[1000];
	if (mqtt_connected)
	{
		FILE *ptr_file=fopen(filename,"r");
		if (ptr_file!=NULL)
		{
			while (fgets(buf,1000, ptr_file)!=NULL) 	
			{
					int msg_id;
					char topicname[32];	
					
					strcpy(topicname, "FILE");	
					msg_id = my_esp_mqtt_client_publish(mqtt_client, topicname, buf, 0, 0, 0);   //Qos=1; retain=0
			}
			fclose(ptr_file);
	    }
	}
}

void app_main_SNTP()
{
    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
 //   if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
 //   }

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Hungary is: %s", strftime_buf);


    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long int)outdelta.tv_sec,
                        (long int)outdelta.tv_usec/1000,
                        (long int)outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }


}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    esp_sntp_init();
}

void Write_Msg_toDisplay(int line, char *Msg)
{
	char message[33];
	strcpy(message,Msg);
	while (strlen(message)<16) strcat(message," ");

	xSemaphoreTake(mutex, portMAX_DELAY);
     ssd1306_display_text(&Display, line, message, strlen(message), false);	
	xSemaphoreGive(mutex);
}


void init_display()
{
    ESP_LOGI(TAG, "INTERFACE is i2c");
	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&Display, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
 	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&Display, 128, 64);
	ssd1306_clear_screen(&Display, false);
	ssd1306_contrast(&Display, 0xff);
	ssd1306_display_text_x3(&Display, 0, "Hello", 5, false);	
	
}





bool compare_topic(char* par1, char* par2, char wilcarded_topic[5][32])
{
	int wildcardcnt=0;
	char *ptr1=par1;
	char *ptr2=par2;
	
	char *ptr1_slash;
	char *ptr2_slash; 
	int top_length1,top_length2;
    memset(wilcarded_topic,0,sizeof(wilcarded_topic)*32);	
	do
	{
	 if (strcmp(ptr2,"#")==0) 
	 {
		strcpy(wilcarded_topic[0],ptr1);
		return true; 
	 }  	
	 top_length1= strlen(ptr1);
	 if ((ptr1_slash=strchr(ptr1,'/'))!=NULL) top_length1=ptr1_slash-ptr1;  	
	 top_length2= strlen(ptr2);
	 if ((ptr2_slash=strchr(ptr2,'/'))!=NULL) top_length2=ptr2_slash-ptr2;  	
	 if (strncmp(ptr2,"+",1)!=0)
	 {
	  if (top_length1!=top_length2) return false;
	  if  (strncmp(ptr1,ptr2,top_length1)!=0) return false;
	 }
	 else if (top_length2!=1) return false;
	      else 
		  {
			wildcardcnt++;
			strncpy((char *)wilcarded_topic[wildcardcnt],ptr1,top_length1);
			wilcarded_topic[wildcardcnt][top_length1]=0;
		  }
	 
	 if ((ptr1_slash==NULL) && (ptr2_slash==NULL)) return true; 
	 if ((ptr1_slash==NULL) || (ptr2_slash==NULL)) return false;
	 
	 ptr1=ptr1_slash+1;
	 ptr2=ptr2_slash+1; 
	}
	while (true);
}





bool is_topic_equal(char* arrived_topic,char* subtopic,bool MQTT, char wilcarded_topic[5][32])
{
 if (MQTT)
 {
  if (strncmp(arrived_topic,maintopic,strlen(maintopic)!=0)) return false;
  return (compare_topic(arrived_topic+strlen(maintopic)+1,subtopic,wilcarded_topic));
 }
 else return (compare_topic(arrived_topic,subtopic,wilcarded_topic));
}



void report_scheduling(int ch)
{
 int period;
 char message[128];
 
 for(period=0;period<PERIODS;period++)
 {
	if(channels[ch].Chedule_array[period].off_time>channels[ch].Chedule_array[period].on_time)
	{		
	sprintf(message,"P%d:%2.2d:%2.2d-%2.2d:%2.2d [%c%c%c%c%c%c%c] %ldmin %ldl", 
	 period+1,
	 (int)(channels[ch].Chedule_array[period].on_time/3600),
	 (int)(channels[ch].Chedule_array[period].on_time%3600/60),
	 (int)(channels[ch].Chedule_array[period].off_time/3600),
	 (int)(channels[ch].Chedule_array[period].off_time%3600/60),
	 channels[ch].Chedule_array[period].weekdays[0],
	 channels[ch].Chedule_array[period].weekdays[1],
	 channels[ch].Chedule_array[period].weekdays[2],
	 channels[ch].Chedule_array[period].weekdays[3],
	 channels[ch].Chedule_array[period].weekdays[4],
	 channels[ch].Chedule_array[period].weekdays[5],
	 channels[ch].Chedule_array[period].weekdays[6],
	 channels[ch].Chedule_array[period].duration,
	 channels[ch].Chedule_array[period].volume);
	 
	 if(mqtt_connected)
     {	
      char topicname[32];	
      int msg_id;
      sprintf(topicname, "CHANNEL/%s/cheduled",channels[ch].Name); 
	  msg_id = my_esp_mqtt_client_publish(mqtt_client,  topicname, message, 0, 0, 0);   //Qos=0; retain=0
	  ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
	 }
    }
 }	 
	
}



int is_channel_active(int ch)
{
	switch (channels[ch].channel_state)
	{
	 case MAN_ON:
	 case PROG_STARTED:
	 case MAN_RESUMED:
	 case PROG_RESUMED: return 1;
     default: return 0;	 
	}
}


time_t sec_in_day()
{
 time_t now;
 time(&now);
 now-=1658700000;
 return (now%86400); 
}


void append_ontimes2string(int ch) 
{
	sprintf(MQTT_BLE_answer+strlen(MQTT_BLE_answer),"%s: Daily ontime: %llds  %1.0fl\n",channels[ch].Name,channels[ch].prev_daily_period_ontimes+is_channel_active(ch)?(sec_in_day()-channels[ch].last_switch_on_time):0,1.0*channels[ch].daily_volume/YF_DN32_PULSE_PER_LITER);
	sprintf(MQTT_BLE_answer+strlen(MQTT_BLE_answer),"%s:last period ontime: %llds  %1.0fl\n",channels[ch].Name,channels[ch].period_ontime+is_channel_active(ch)?(sec_in_day()-channels[ch].last_switch_on_time):0,1.0*channels[ch].period_volume/YF_DN32_PULSE_PER_LITER);
}





void reboot_WIFI_STICK()
{
	ESP_LOGI(TAG, "reboot wifi stick");
	writeDO(GPIO_Vext, 1);
	vTaskDelay(5*1000 / portTICK_PERIOD_MS);	
	writeDO(GPIO_Vext, 0);
}


 


int update_item(char * ldata,char* item,int data_addr)
{
	char formatstring[128];
	int regValue;
	strcpy(formatstring,item);
	strcat(formatstring,"=%d");
	if(sscanf(ldata,formatstring,&regValue)==1)
			 {
				 eeprom_reg_t reg; 
				 xSemaphoreTake(mutex, portMAX_DELAY);
	     		  reg.frame.value = read_ACS71020_register(ACS71020_address_default, data_addr, 0xffffffff, 0,0);
				 xSemaphoreGive(mutex); 
				 
				 switch(data_addr)
				 {
					case 0x0B:  
								{
									eeprom_0x0B_t addr_0x0B;
									addr_0x0B.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"qvo_fine")==0) addr_0x0B.fields.qvo_fine=regValue;	
									else if (strcmp(item,"sns_fine")==0) addr_0x0B.fields.sns_fine=regValue;	
									else if (strcmp(item,"crs_sns")==0) addr_0x0B.fields.crs_sns=regValue;	
									else if (strcmp(item,"iavgselen")==0) addr_0x0B.fields.iavgselen=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0B.eeprom_data;
								}
								break;
					case 0x0C:  
								{
									eeprom_0x0C_t addr_0x0C;
									addr_0x0C.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"rms_avg_1")==0) addr_0x0C.fields.rms_avg_1=regValue;	
									else if (strcmp(item,"rms_avg_2")==0) addr_0x0C.fields.rms_avg_2=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0C.eeprom_data;
								}
								break;
					case 0x0D:  
								{
									eeprom_0x0D_t addr_0x0D;
									addr_0x0D.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"pacc_trim")==0) addr_0x0D.fields.pacc_trim=regValue;	
									else if (strcmp(item,"ichan_del_en")==0) addr_0x0D.fields.ichan_del_en=regValue;	
									else if (strcmp(item,"chan_del_sel")==0) addr_0x0D.fields.chan_del_sel=regValue;	
									else if (strcmp(item,"fault")==0) addr_0x0D.fields.fault=regValue;	
									else if (strcmp(item,"fltdly")==0) addr_0x0D.fields.fltdly=regValue;	
									else if (strcmp(item,"halfcycle_en")==0) addr_0x0D.fields.halfcycle_en=regValue;	
									else if (strcmp(item,"squarewave_en")==0) addr_0x0D.fields.squarewave_en=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0D.eeprom_data;
								}
								break;
					case 0x0E:  
								{
									eeprom_0x0E_t addr_0x0E;
									addr_0x0E.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"delaycnt_sel")==0) addr_0x0E.fields.delaycnt_sel=regValue;	
									else if (strcmp(item,"undervreg")==0) addr_0x0E.fields.undervreg=regValue;	
									else if (strcmp(item,"overvreg")==0) addr_0x0E.fields.overvreg=regValue;	
									else if (strcmp(item,"vadc_rate_set")==0) addr_0x0E.fields.vadc_rate_set=regValue;	
									else if (strcmp(item,"vevent_cycs")==0) addr_0x0E.fields.vevent_cycs=regValue;	
									reg.frame.fields.eeprom_data=addr_0x0E.eeprom_data;
								}
								break;
					case 0x0F:  
								{
									eeprom_0x0F_t addr_0x0F;
									addr_0x0F.eeprom_data = reg.frame.fields.eeprom_data; 
									if (strcmp(item,"dio_1_sel")==0) addr_0x0F.fields.dio_1_sel=regValue;	
									else if (strcmp(item,"dio_0_sel")==0) addr_0x0F.fields.dio_0_sel=regValue;	
									else if (strcmp(item,"i2c_dis_slv_addr")==0) addr_0x0F.fields.i2c_dis_slv_addr=regValue;	
									else if (strcmp(item,"i2c_slv_addr")==0) addr_0x0F.fields.i2c_slv_addr=regValue;										
									reg.frame.fields.eeprom_data=addr_0x0F.eeprom_data;
								}
					default:	break;			
									
				 }
				 
				 xSemaphoreTake(mutex, portMAX_DELAY);
				  write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
				  write_ACS71020(ACS71020_address_default, data_addr+0x10, reg.frame.fields.eeprom_data); //write shadow
				  write_ACS71020(ACS71020_address_default, data_addr, reg.frame.fields.eeprom_data); //write EEPROM	
				 xSemaphoreGive(mutex); 
				 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", "shadow + eeprom writen", 0, 0, 0);   //Qos=0; retain=0	 
				 return 1;
			 }
	else return 0;		 

}


bool LIFE_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
	sscanf(ldata,"%lld",&now_life_received);
				
				//if(measure_mode==LOG)
				{
				 char message[16];
				 sprintf(message,"%lld",now_life_received);
				 my_esp_mqtt_client_publish(mqtt_client, "LIFE_LOOP", message, 0, 0, 0);   //Qos=0; retain=1
				 if (now_life_sent==now_life_received)
				 {
				   static bool firstrun=true;
				   if (firstrun)	
				   {
					esp_ota_mark_app_valid_cancel_rollback(); //validate the last OTA update
					strcpy(MQTT_BLE_answer,"FIRMWARE/ROLLBACK CANCELLED AUTOMATICALLY"); 
					my_esp_mqtt_client_publish(mqtt_client, "LIFE_LOOP", MQTT_BLE_answer, 0, 0, 0);   //Qos=0; retain=1
				    firstrun=false;
				   }

				 }
				}
				return true;
}
bool FIRMWARE_URL_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
                ESP_LOGI(TAG, "Topic:%s",ltopic);
				if (strlen(url_buf)) free(url_buf);
				url_buf = calloc(1, strlen(ldata)+1);
				strcpy(url_buf,ldata);
                OTA_SOURCE_URL=url_buf;
				sprintf(MQTT_BLE_answer,"FIRMWARE/SOURCE: %s", OTA_SOURCE_URL);
				return true;
}
bool FIRMWARE_SELECT_URL_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
 char charval;
				
				 if(sscanf(ldata,"%c",&charval)==1) 
				 {
				   switch (charval)
					 {
						case '?': break; // query actual value		
						case 'S': OTA_SOURCE_URL=url_buf_szabolcskiss;
								break;	
						case 'G': OTA_SOURCE_URL=url_buf_git;
								break;		
						case 'N':OTA_SOURCE_URL=url_buf;
								break;		
						default: OTA_SOURCE_URL="Invalid source";
								break;
						 
					 }
				  
	
				  sprintf(MQTT_BLE_answer,"FIRMWARE/SOURCE: %s", OTA_SOURCE_URL);  
				  return true;
				 }
				 else sprintf(MQTT_BLE_answer,"FIRMWARE/SOURCE: %s", "Invalid format");  
				 return false;

}
bool FIRMWARE_VERSION_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
			{
				ESP_LOGI(TAG, "FIRMWARE/VERSION");
				if(strcmp(ldata,"?")==0) sprintf(MQTT_BLE_answer,"Running version:%s", app_desc->version);
				else if(strcmp(ldata,app_desc->version)!=0) 
				{
				  strcpy(new_Firmware_version,ldata);
				  ESP_LOGI(TAG, "NEW FIRMWARE Name:%s",new_Firmware_version);
				  sprintf(MQTT_BLE_answer,"FIRMWARE/UPDATE new version available: %s", new_Firmware_version); 
		    	  FW_update_available=true;
                  Write_Msg_toDisplay(0,"OTA update started.");	
  				  xTaskCreate(&ota_update_task, "ota_update_task", 8192, NULL, 5, NULL);	
				}
				else sprintf(MQTT_BLE_answer,"Version %s already running.", ldata); 
				return true;
			}
bool FIRMWARE_ROLLBACK_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
			{
				 if(strcmp(ldata,"CANCEL_ROLLBACK")==0)  
				 {
					 esp_ota_mark_app_valid_cancel_rollback(); //validate the last OTA update
					 strcpy(MQTT_BLE_answer,"FIRMWARE/ROLLBACK CANCELLED"); 
				 }
				 else if(strcmp(ldata,"ROLLBACK")==0) 
				 {
					strcpy(MQTT_BLE_answer,"FIRMWARE/ROLLBACK STARTED"); 
					my_esp_mqtt_client_publish(mqtt_client, "FIRMWARE/ROLLBACK", "STARTED", 0, 0, 0);   //Qos=1; retain=1
					esp_ota_mark_app_invalid_rollback_and_reboot(); //rollback to previous FW
					 
				 }
				 return true;
}
bool LEVEL___CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
			{
				get_LEVEL_string(MQTT_BLE_answer);
				return true;
			}
bool TIME___CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
				char strftime_buf[64];
				time_t now;
				struct tm timeinfo;
				time(&now);
				localtime_r(&now, &timeinfo);
				strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
				if(SNTP_synchronized) strcat(strftime_buf," SYNC");
                sprintf(MQTT_BLE_answer,"%s {%s}", "TIME",strftime_buf); 
				return true;
}
bool PUMP___CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
 sprintf(MQTT_BLE_answer,"%s ",GetPumpStatusString(0)); 
 if (pump_num==2) 
  {
   strcat(MQTT_BLE_answer,",");	
   strcat(MQTT_BLE_answer,GetPumpStatusString(1));
 }
 return true;
}
bool PUMP_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
	ESP_LOGI("PUMP","Callback: topic:%s\n ldata:%s \n wilcarded:%s",ltopic,ldata,wilcarded_topic[1]);
    int ch;
    if (strcmp(wilcarded_topic[1],"PRIO")==0)
	{
 
				 if(strcmp(ldata,"ON")==0) switch_pump(true);
                 else switch_pump(false);
				 PUMP___CB(ltopic,  ldata,  MQTT,wilcarded_topic);
	}
    else if ((strlen(wilcarded_topic[1])==1) && (sscanf(wilcarded_topic[1],"%d",&ch)==1) && (ch>=1) && (ch<=2))
	{
		ch--;
		if(strcmp(ldata,"ON")==0) switch_pump_id_to_state(ch,P_ON);
		else if (strcmp(ldata,"DISABLE")==0) enable_pump(ch,false);
		else if (strcmp(ldata,"ENABLE")==0) enable_pump(ch,true);
		else if (strcmp(ldata,"SET_PRIO")==0) setPUMP_prio(ch,true);
		else if (strcmp(ldata,"SET_SWITCHBACK")==0) setPUMP_switchbackifavailable(ch,true);
		else if (strcmp(ldata,"DEL_SWITCHBACK")==0) setPUMP_switchbackifavailable(ch,false);
        else switch_pump_id_to_state(ch,P_OFF);

		PUMP___CB(ltopic,  ldata,  MQTT,wilcarded_topic);
    }
    return true;
   
}
bool ACS71020_read_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])			

			{
			  long val=0;	
			 char strval[32];
			 int reg,mask,shift_left,shift_right,fields;
			 fields=sscanf(ldata,"0x%x,0x%x,%d,%d",&reg,&mask,&shift_left,&shift_right);
			 switch (fields)
			 {
				
				 case 1: mask=0xffffffff; [[fallthrough]];
				 case 2: shift_left=0;    [[fallthrough]];
				 case 3: shift_right=0;   [[fallthrough]];
				 case 4: 
						xSemaphoreTake(mutex, portMAX_DELAY);
							val= read_ACS71020_register(ACS71020_address_default, reg, mask, shift_left,shift_right);
						xSemaphoreGive(mutex); 		
						 sprintf(strval,"%x:%lx",reg,val);
						 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
						 read_ACS71020_register2(reg,val);
						 if(!MQTT) sprintf(MQTT_BLE_answer,"%s {%s}", "ACS71020",strval); 
				 break;
			 }
			 
			   
			 
			 
	        return true;
			}
bool ACS71020_write_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])				
{	
		     int data_addr,regValue;
			 if(sscanf(ldata,"0x%x=0x%x",&data_addr,&regValue)==2)
			 {
				if ((data_addr>=0x1B) && (data_addr<=0x1F)) 
				{//write SHADOW
				    xSemaphoreTake(mutex, portMAX_DELAY);
					 write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
					 write_ACS71020(ACS71020_address_default, data_addr, regValue); //write shadow
					 xSemaphoreGive(mutex); 
					my_esp_mqtt_client_publish(mqtt_client, "ACS71020", "shadow writen", 0, 0, 0);   //Qos=0; retain=0	
				    if(!MQTT) sprintf(MQTT_BLE_answer,"%s {%s}", "ACS71020","shadow writen"); 
				}	
				else if ((data_addr>=0x0B) && (data_addr<=0x0F)) 
				{//write SHADOW + EEPROM
					xSemaphoreTake(mutex, portMAX_DELAY);
					 write_ACS71020(ACS71020_address_default, 0x2F, 0x4f70656E); //enter to customer mode
					 write_ACS71020(ACS71020_address_default, data_addr+0x10, regValue); //write shadow
				     write_ACS71020(ACS71020_address_default, data_addr, regValue); //write EEPROM*/	
					xSemaphoreGive(mutex); 
					my_esp_mqtt_client_publish(mqtt_client, "ACS71020", "shadow + eeprom writen", 0, 0, 0);   //Qos=0; retain=0	
				    if(!MQTT) sprintf(MQTT_BLE_answer,"%s {%s}", "ACS71020","shadow + eeprom writen"); 
				}					
			 }

/*
   qvo_fine
   sns_fine
   crs_sns
   iavgselen
   
   rms_avg_2
   rms_avg_1
   
   pacc_trim
   ichan_del_en
   chan_del_sel
   fault
   fltdly
   halfcycle_en
   squarewave_en
   
   delaycnt_sel
   undervreg
   overvreg
   vadc_rate_set
   vevent_cycs
   
   dio_1_sel
   dio_0_sel
   i2c_dis_slv_addr
   i2c_slv_addr
*/






	 else if(update_item(ldata,"qvo_fine",0x0B)) {}	
	 else if(update_item(ldata,"sns_fine",0x0B)) {}			
	 else if(update_item(ldata,"crs_sns",0x0B))	{}		
	 else if(update_item(ldata,"iavgselen",0x0B)){}		

	 else if(update_item(ldata,"rms_avg_2",0x0C)){}		
	 else if(update_item(ldata,"rms_avg_1",0x0C)){}			 

else if(update_item(ldata,"pacc_trim",0x0D)){}		
else if(update_item(ldata,"ichan_del_en",0x0D)){}		
else if(update_item(ldata,"chan_del_sel",0x0D)){}		
else if(update_item(ldata,"fault",0x0D)){}		
else if(update_item(ldata,"fltdly",0x0D)){}		
else if(update_item(ldata,"halfcycle_en",0x0D)){}		
else if(update_item(ldata,"squarewave_en",0x0D)){}		
			 
			
else if(update_item(ldata,"delaycnt_sel",0x0E)){}		
else if(update_item(ldata,"undervreg",0x0E)){}		
else if(update_item(ldata,"overvreg",0x0E)){}		
else if(update_item(ldata,"vadc_rate_set",0x0E)){}		
else if(update_item(ldata,"vevent_cycs",0x0E)){}		
			
			
else if(update_item(ldata,"dio_1_sel",0x0F)){}			
else if(update_item(ldata,"dio_0_sel",0x0F)){}		
else if(update_item(ldata,"i2c_dis_slv_addr",0x0F)){}		
else if(update_item(ldata,"i2c_slv_addr",0x0F)){}	
else {}			
			
			 
return true;			 		
}

int get_Channel_from_wildcarded(char wilcarded_topic[5][32])
{
	int ch;
    if ((strlen(wilcarded_topic[1])==1) && (sscanf(wilcarded_topic[1],"%d",&ch)==1) && (ch>0) && (ch<=CHANNEL_NUM)) return (ch-1);
     for (ch=0;ch<CHANNEL_NUM;ch++)
	 {
	  if(strcmp(channels[ch].Name,wilcarded_topic[1])==0) return ch;
     }
	return -1;
}

bool CHANNEL_schedule_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{	
	int period;	
	int ch=get_Channel_from_wildcarded(wilcarded_topic);	
	if (ch==-1) 
	 {
		sprintf(MQTT_BLE_answer,"Invalid channel in topic: %s ", ltopic);
		return false;
	 }
			if (strstr(ltopic,"/SCHEDULE/?")!=NULL) report_scheduling(ch);	
			else if (sscanf(wilcarded_topic[0],"PERIOD%d",&period)==1)
			{
			 int HH_on,MM_on,HH_off,MM_off;
			 char weekdays[7];
			 int duration=0;
			 int volume=0;
			 period--;
			 ESP_LOGI(TAG, "CH:%d,period:%d",ch,period);
			 if((ch>=0) && (ch<CHANNEL_NUM) && (period>=0) && (period<PERIODS))
			 {
			  if(sscanf(ldata,"%2d:%2d-%2d:%2d [%c%c%c%c%c%c%c] %d %d",&HH_on,&MM_on,&HH_off,&MM_off,&weekdays[0],&weekdays[1],&weekdays[2],&weekdays[3],&weekdays[4],&weekdays[5],&weekdays[6],&duration,&volume)==13)
			  {
				channels[ch].Chedule_array[period].on_time=HH_on*3600+MM_on*60;
				channels[ch].Chedule_array[period].off_time=HH_off*3600+MM_off*60;
				strncpy(channels[ch].Chedule_array[period].weekdays,weekdays,7);
				channels[ch].Chedule_array[period].weekdays[7]=0;
				channels[ch].Chedule_array[period].duration=duration;
				channels[ch].Chedule_array[period].volume=volume;
				ESP_LOGI(TAG, "on_time:%d,off_time:%d",(int)channels[ch].Chedule_array[period].on_time,(int)channels[ch].Chedule_array[period].off_time);
				ESP_LOGI(TAG,"%s",weekdays);	
				if(channels[ch].Chedule_array[period].on_time>=channels[ch].Chedule_array[period].off_time)
				{
					channels[ch].Chedule_array[period].on_time=0;
					channels[ch].Chedule_array[period].off_time=0;
					channels[ch].Chedule_array[period].weekdays[0]=0;
					channels[ch].Chedule_array[period].duration=0;
					channels[ch].Chedule_array[period].volume=0;
				}					
			  }
			  else
			  { //delete invalid schedule 
					channels[ch].Chedule_array[period].on_time=0;
					channels[ch].Chedule_array[period].off_time=0;
					channels[ch].Chedule_array[period].weekdays[0]=0;
					channels[ch].Chedule_array[period].duration=0;
					channels[ch].Chedule_array[period].volume=0;
			  }
			  Save_data_to_NVS();
			 }
			 report_scheduling(ch);
			}
	        *MQTT_BLE_answer=0; 
			return true;
		}

bool CHANNEL_statistic_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])	
{	
	int ch=get_Channel_from_wildcarded(wilcarded_topic);
    if (ch==-1) 
	 {
		sprintf(MQTT_BLE_answer,"Invalid channel in topic: %s ", ltopic);
		return false;
	 }
	 MQTT_BLE_answer[0]=0;
	 append_ontimes2string(ch);
	 return true;
}

bool CHANNEL_TIMES_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])	
{	
	int ch=get_Channel_from_wildcarded(wilcarded_topic);
    if (ch==-1) 
	 {
		sprintf(MQTT_BLE_answer,"Invalid channel in topic: %s ", ltopic);
		return false;
	 }
	 MQTT_BLE_answer[0]=0;
			for (int i=0;i<num_states;i++)
			{
				sprintf(MQTT_BLE_answer+strlen(MQTT_BLE_answer),"CH:%s %s: %lld\n",channels[ch].Name,str_short_states[i],channels[ch].status_change_time[i]);
			}

	 return true;
}


bool restart_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])	
			{
                if(strcmp(ldata,"ESP")==0) 	
				{
					prevhour=0;
					for(int ch=0;ch<CHANNEL_NUM;ch++) 
					{
						switch_channel(ch,DISABLED);
						vTaskDelay(1*1000 / portTICK_PERIOD_MS);
					}
					esp_restart();   
				}
				else if(strcmp(ldata,"WIFI")==0) 	 reboot_WIFI_STICK();	
			return true;					
			}
bool pump_restart_delay_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])			
			{
				int intval;
				 if(sscanf(ldata,"%d",&intval)==1) 
				 {
					 if ((intval>=0) && (intval<=60)) 
					 {
						 pump_restart_delay=intval;
						 Save_data_to_NVS();
						 sprintf(MQTT_BLE_answer,"%s {%d}", "pump_restart_delay",pump_restart_delay); 
					 }
					 else sprintf(MQTT_BLE_answer,"%s %s", "pump_restart_delay","Out of range"); 
				 }
				 else sprintf(MQTT_BLE_answer,"%s %s", "pump_restart_delay","invalid format!"); 
			return true;	 
			}
bool measure_mode_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])			
			{
				 if(strcmp(ldata,"LEVEL")==0)  measure_mode=LEVEL;
				 else if(strcmp(ldata,"POWER")==0)  measure_mode=POWER;
				 else if(strcmp(ldata,"CT")==0)  measure_mode=CT;
				 else if(strcmp(ldata,"STACK")==0)  measure_mode=STACK;
				 else if(strcmp(ldata,"LOG")==0)  measure_mode=LOG;
				 else if(strcmp(ldata,"VOLUME")==0)  measure_mode=VOLUME;
				 else measure_mode=OFF;
				 sprintf(MQTT_BLE_answer,"%s {%d}", "measure_mode",(int) measure_mode); 
				 return true;
			}
bool temp___CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])				
			{
				sprintf(MQTT_BLE_answer,"%0.2f°C",temperature);
				return true;
			}
bool run_mode_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])			
			{
				int intval;
				 if(sscanf(ldata,"%d",&intval)==1) 
				 {
					 if ((intval>=0) && (intval<=65535)) 
					 {
						char msg[16];
						 run_mode=intval;
						 Save_data_to_NVS();
				         sprintf(msg,"%d",run_mode);
			             sprintf(MQTT_BLE_answer,"%s {%d}", "run_mode_value",run_mode); 

						 if (run_mode & (1<<MAIN_TASK)) 
						 {
						  if (xSemaphoreTake(MAIN_TASK_mutex, 0)==pdFALSE)  xTaskCreatePinnedToCore(&mainTask, "mainTask", 4096, NULL, 5, NULL, 0);
						 }
					 }
				 }
			return true;
			}
bool run_mode___CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])			
{
				sprintf(MQTT_BLE_answer,"run_mode:%d",run_mode);
				return true;
}



bool CHANNEL_request_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])	
{
	int ch=get_Channel_from_wildcarded(wilcarded_topic);
    if (ch==-1) 
	 {
		sprintf(MQTT_BLE_answer,"Invalid channel in topic: %s ", ltopic);
		return false;
	 }	 
	else
	{
				int i;
				ESP_LOGI(TAG, "CHANNEL/%s/request",channels[ch].Name);
				if((ch>=0) && (ch<CHANNEL_NUM))
				{
				 for(i=0;i<num_states;i++)
				 {
				  if(strcmp(ldata,str_states[i])==0) break;
				 }
				 if (i<num_states) channels[ch].manual_change_request=i; 
				 else if(strcmp(ldata,"ON")==0) channels[ch].manual_change_request=MAN_ON;
				 else if(strcmp(ldata,"OFF")==0) channels[ch].manual_change_request=MAN_OFF;
			     else channels[ch].manual_change_request=NOREQUEST;
				 if(channels[ch].manual_change_request==ENABLED) {channels[ch].channel_disabled=0;Save_data_to_NVS();}
				 else if (channels[ch].manual_change_request==DISABLED) {channels[ch].channel_disabled=1;Save_data_to_NVS();}
				 else if(strcmp(ldata,"VALVE ON")==0) writeDO(channels[ch].Valve_GPIO_OUTPUT, true);
				 else if(strcmp(ldata,"VALVE OFF")==0) writeDO(channels[ch].Valve_GPIO_OUTPUT, false);
				 
				}
				sprintf(MQTT_BLE_answer,"%s {%s}", ltopic,str_states[channels[ch].channel_state]);  					 
			}
			return true;
}

bool LIST_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
	if (strcmp(ldata,"NAMES")==0)
	{
	 	
	 strcpy(MQTT_BLE_answer,"Channel names:\n");
	 for(int ch=0;ch<CHANNEL_NUM;ch++)
	 {
      append_ontimes2string(ch);		
	 }
    } 
    else if (strcmp(ldata,"LOG")==0)
	{
	 Publish_file(LOG_FILE);
	 MQTT_BLE_answer[0]=0;
	}
	else if (strcmp(ldata,"IRR")==0)
	{
	 Publish_file(IRR_FILE);
	 MQTT_BLE_answer[0]=0;
	}

	return true;
}

bool CHANNEL_SET_NAME_CB(char* ltopic, char* ldata, bool MQTT,char wilcarded_topic[5][32])
{
	printf("%s",wilcarded_topic[1]);
	int ch=get_Channel_from_wildcarded(wilcarded_topic);
    if (ch==-1) 
	 {
		sprintf(MQTT_BLE_answer,"Invalid channel in topic: %s ", ltopic);
		return false;
	 }

	if ((strlen(ldata)>0) && (strlen(ldata)<=sizeof(channels[ch].Name)-1)) 
	{
		strcpy(channels[ch].Name,ldata);
		Save_data_to_NVS();
		sprintf(MQTT_BLE_answer,"Channel name (%s) for CH%d saved.}", channels[ch].Name,ch);

	}
	else sprintf(MQTT_BLE_answer,"Invalid length (1..7) {%s}", ldata);
	return true;
}	



bool help_CB(char* ltopic, char* ldata, bool MQTT,char *wilcarded_topic)
{
				char *message="IRRIGATION/HELP show the available commands -Topic {Message}\n\
IRRIGATION/FIRMWARE/URL {URL} -set new URL for OTA\n\
IRRIGATION/FIRMWARE/SELECT_URL {?:G:S:N} - ?: query, S:szabolcskiss; G:github; N:new given by FIRMWARE/URL \n\
IRRIGATION/FIRMWARE/VERSION  {version:?} -set new version for OTA:query\n\
IRRIGATION/FIRMWARE/ROLLBACK {ROLLBACK:CANCEL_ROLLBACK} -keep or rollback OTA update\n\
IRRIGATION/CHANNEL/x/REQUEST {MAN_ON,PROG_STARTED,MAN_RESUMED,PROG_RESUMED,INIT,ENABLED,DISABLED,MAN_OFF,SUSPENDED,DELAY,PROG_FINISHED,PROG_END,IDLE,REBOOTED,NOREQUEST} -set new state\n\
IRRIGATION/CHANNEL/x/SCHEDULE/PERIODx {10:00-12:00 [+++++++] 30 2000} -add new schedule period 30min 2000l\n\
IRRIGATION/CHANNEL/x/SCHEDULE/? {}   -list all programmed periods\n\
IRRIGATION/CHANNEL/x/STATISTIC {} -get statistic\n\
IRRIGATION/PUMP/+/REQUEST +:PRIO|1|2 {ON|OFF|DISABLE|ENABLE|SET_PRIO|SET_SWITCHBACK|DEL_SWITCHBACK} -switch PUMP ON|OFF\n\
IRRIGATION/PUMP/? {} -query pump status\n\
";		
		      	
				  my_esp_mqtt_client_publish(mqtt_client, "MEASURE/commands", message, 0, 0, 0);   //Qos=0; retain=0	
				 
				 vTaskDelay(3*1000 / portTICK_PERIOD_MS);	
message="IRRIGATION/PUMP_RESTART_DELAY {10min} -set pump restart delay\n\
IRRIGATION/LEVEL/? {} -query water level\n\
IRRIGATION/TIME/? {} -query TIME\n\
IRRIGATION/TEMP/? {} -query temperature sensor\n\
IRRIGATION/MEASURE_MODE {POWER:LEVEL:STACK:CT:LOG:VOLUME:OFF}\n\
IRRIGATION/RESTART {ESP:WIFI} -force restart of ESP32 or WIFI dongle\n\
IRRIGATION/ACS71020/READ {0xhex_address} \n\
IRRIGATION/ACS71020/WRITE {0xhex_address=0xhex_value}";	
			
				  my_esp_mqtt_client_publish(mqtt_client, "MEASURE/commands", message, 0, 0, 0);   //Qos=0; retain=0	
					
				 if(!MQTT) 
				 {

					message="help show the available commands -Topic {Message}\n\
SSID {new SSID} - store new SSID \n\
PWD  {new PASSWORD} -sore new PAssword\n\
SAVE_NVS {} - save permanent data to NVS\n\
RUN_MODE {} -new runmode:{USE_BLE,USE_WIFI,MAIN_TASK,POWERMETER_TASK,TEMPSENSOR,CURRENTSENSOR,MEASURE_LEVEL,MEASURE_POWER} \n\
FIRMWARE/URL {URL} -set new URL for OTA\n\
FIRMWARE/SELECT_URL {?:G:S:N} - ?: query, S:szabolcskiss; G:github; N:new given by FIRMWARE/URL \n\
FIRMWARE/VERSION  {version:?} -set new version for OTA:query\n\
FIRMWARE/ROLLBACK {ROLLBACK:CANCEL_ROLLBACK} -keep or rollback OTA update\n\
CHANNEL/x/REQUEST {MAN_ON,PROG_STARTED,MAN_RESUMED,PROG_RESUMED,INIT,ENABLED,DISABLED,MAN_OFF,SUSPENDED,DELAY,PROG_FINISHED,PROG_END,IDLE,REBOOTED,NOREQUEST} -set new state\n\
CHANNEL/x/SCHEDULE/PERIODx {10:00-12:00 [+++++++] 30 2000} -add new schedule period 30min 2000l\n\
CHANNEL/x/SCHEDULE/? {}   -list all programmed periods\n\
CHANNEL/x/STATISTIC {} -get statistic\n\
PUMP/+/REQUEST +:PRIO|1|2 {ON|OFF|DISABLE|ENABLE|SET_PRIO|SET_SWITCHBACK|DEL_SWITCHBACK} -switch PUMP ON|OFF\n\
PUMP/? {} -query pump status\n\
pump_restart_delay {10min} -set pump restart delay\n\
LEVEL/? {} -query water level\n\
TIME/? {} -query TIME\n\
TEMP/? {} -query temperature sensor\n\
MEASURE_MODE {POWER:LEVEL:STACK:CT:LOG:VOLUME:OFF}\n\
RESTART {ESP:WIFI} -force restart of ESP32 or WIFI dongle\n\
ACS71020/READ {0xhex_address} \n\
ACS71020/WRITE {0xhex_address=0xhex_value}";	
					sprintf(MQTT_BLE_answer,"%s", message); 
				 }
				
return true;
}

char* subscribe_topics[]=                   {"LIFE" ,"FIRMWARE/URL" ,"FIRMWARE/SELECT_URL" ,"FIRMWARE/VERSION" ,"FIRMWARE/ROLLBACK" ,"CHANNEL/+/REQUEST"      ,"CHANNEL/+/STATISTIC"      ,"CHANNEL/+/SCHEDULE/#"    ,"RESTART" ,"PUMP_RESTART_DELAY","MEASURE_MODE" ,"LEVEL/?" ,"TIME/?"  ,"ACS71020/READ" ,"ACS71020/WRITE" ,"TEMP/?" ,"HELP" ,"PUMP/+/REQUEST" ,"PUMP/?" ,"RUN_MODE" ,"RUN_MODE/?","LIST","CHANNEL/+/SET_NAME","CHANNEL/+/TIMES"};
T_MQTT_Sub_Callback *MQTT_Sub_Callbacks[]=  {LIFE_CB,FIRMWARE_URL_CB,FIRMWARE_SELECT_URL_CB,FIRMWARE_VERSION_CB,FIRMWARE_ROLLBACK_CB,CHANNEL_request_CB,CHANNEL_statistic_CB,CHANNEL_schedule_CB,restart_CB,pump_restart_delay_CB,measure_mode_CB,LEVEL___CB,TIME___CB,ACS71020_read_CB,ACS71020_write_CB,temp___CB,help_CB,PUMP_CB,PUMP___CB,run_mode_CB,run_mode___CB,LIST_CB,CHANNEL_SET_NAME_CB,CHANNEL_TIMES_CB}; 


bool Process_EVENT_DATA(char* ltopic, char* ldata, bool MQTT)
{
	int msg_id;
	char publish_topic[128];
	char wilcarded_topic[5][32];
 

            for (int i=0;i<sizeof(subscribe_topics)/4;i++)
			{
              if (is_topic_equal(ltopic,subscribe_topics[i],MQTT,wilcarded_topic)) 
			   {
				bool ret=MQTT_Sub_Callbacks[i](ltopic,  ldata,  MQTT,wilcarded_topic);
				if (subscribe_topics[i][strlen(subscribe_topics[i])-1]=='#') sprintf(publish_topic,"%s","ANSWER");
				else sprintf(publish_topic,"%s/answer",ltopic);
                if (strlen(MQTT_BLE_answer)>0) my_esp_mqtt_client_publish(mqtt_client, publish_topic, MQTT_BLE_answer, 0, 0, 0);   //Qos=0; retain=0	
				return true;
			   }
			}

			my_esp_mqtt_client_publish(mqtt_client, publish_topic, "Unknown command", 0, 0, 0);   //Qos=1; retain=1
			if(!MQTT) sprintf(MQTT_BLE_answer,"%s {%s}", "Unknown command",ltopic);
			return false;
}
	
  
 

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    //esp_mqtt_client_handle_t client = event->client;
	int msg_id;
    char ltopic[128];
	char ldata[128];
     ESP_LOGI(TAG, "mqtt_event_handler called");
	    // your_context_t *context = event->context;
    switch ((esp_mqtt_event_id_t)event_id) {
		case MQTT_EVENT_BEFORE_CONNECT:
			ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
		
		break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
             mqtt_connected = 1;
			 xEventGroupSetBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
			 
			// Write_Msg_toDisplay(0,"MQTT connected.");
			 
		     for (int i=0;i<sizeof(subscribe_topics)/4;i++)
			 {
                msg_id = my_esp_mqtt_client_subscribe(mqtt_client, subscribe_topics[i], 1); 
				ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d,topic=%s", msg_id,subscribe_topics[i]);
			 }


			 if(mqtt_connected)
			 {		 
			  sprintf(MQTT_BLE_answer,"%s run_mode=%d",app_desc->version,run_mode);
			  msg_id = my_esp_mqtt_client_publish(mqtt_client, "FIRMWARE/RUNNING_VERSION",MQTT_BLE_answer, 0, 0, 1);   //Qos=1; retain=1
			  ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			  
			   msg_id = my_esp_mqtt_client_publish(mqtt_client, "FIRMWARE/ROLLBACK", "", 0, 0, 0);   //Qos=1; retain=0
			   ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 }
			 		 
	/* if(mqtt_connected)
     {	
			 msg_id = my_esp_mqtt_client_publish(mqtt_client, "CHANNEL/1/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 
			 msg_id = my_esp_mqtt_client_publish(mqtt_client, "CHANNEL/2/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
			 
			 msg_id = my_esp_mqtt_client_publish(mqtt_client, "CHANNEL/3/status", "INIT", 0, 0, 1);   //Qos=1; retain=1
			 ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);
	 }*/
			 
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		//	Write_Msg_toDisplay(0,"MQTT disconnected.");
            mqtt_connected = 0;
			xEventGroupClearBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
			strncpy(ltopic,event->topic,event->topic_len);
			ltopic[event->topic_len]=0;
			to_upper(ltopic,ltopic);
			strncpy(ldata,event->data,event->data_len);
			ldata[event->data_len]=0;
			ESP_LOGI(TAG, "%s",ltopic);
			ESP_LOGI(TAG, "%s",ldata);
	xSemaphoreTake(mutex_mqtt_ble, portMAX_DELAY);
	 if(Process_EVENT_DATA(ltopic,ldata, true)) printf("Process_EVENT_DATA returned true");
	xSemaphoreGive(mutex_mqtt_ble);


      
	       
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    //return ESP_OK;
}




static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URL,
        // .user_context = (void *)your_context
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}









void switch_pump_for_channel(int channel,int status)
{
 if (status==1)	
 {
	 switch_pump(true);
	 channels[channel].Channel_pump_ON=true; 
 }
 else
 {
	channels[channel].Channel_pump_ON=false;  
	for(int i=0; i<CHANNEL_NUM; i++)
	 if (channels[i].Channel_pump_ON) return; // there is active channel, return without switch pump off
    switch_pump(false);		
 }
}


void switch_channel_relays(int channel, int status)
{ 
  switch_pump_for_channel(channel,status);
  writeDO(channels[channel].Valve_GPIO_OUTPUT, status);
}



void switch_channel(int ch, T_states status)
{
	int new_relay_status=0;
	
	char topicname[64];
	
	int msg_id;
	time_t now=sec_in_day();
   	
	if((status==MAN_ON) ||  (status==PROG_STARTED)  ||  (status==MAN_RESUMED) ||  (status==PROG_RESUMED))  new_relay_status=1; 
	
	if(channels[ch].channel_state!=status)
	{//statuschange			
	 sprintf(topicname,"%s/CHANNEL/%s/status",maintopic,channels[ch].Name);
	 if(channels[ch].channel_state==DISABLED)
	 {
		new_relay_status=0;
		if((status!=ENABLED) && (status!=DISABLED)) return;
	 }
	 
	 else if(((channels[ch].channel_state==SUSPENDED) || (channels[ch].channel_state==DELAY)) && ((status==MAN_ON) || (status==PROG_STARTED)))
	 {
	 	channels[ch].channel_states_before_suspend=status; //new turn ON request will be handled after resume
		if(mqtt_connected)
				{	
			     char message[64]; 
			     sprintf(message,"%s->%s",str_states[channels[ch].channel_state],str_states[channels[ch].channel_states_before_suspend]);
				 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,message , 0, 0, 1);
				 ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				}
		return;		
	 }

    if (new_relay_status && isPUMP_disabled_or_suspended()) 
	{
		if(mqtt_connected)
				{	
				 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,"REFUSED_ON"  , 0, 0, 1);
				 ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				}
		return; //not allowed to switch pump on 
	}

    if (status==PROG_STARTED)
	 {
		ESP_LOGI(TAG,"PROG START"); 
		channels[ch].period_ontime=0;
		channels[ch].period_volume=0;
		channels[ch].ontime_of_period_added_to_daily=false;
	 }
	 else if ((status==PROG_END) || (status==PROG_FINISHED))
	 {
		ESP_LOGI(TAG,"PROG FINISH or END"); 
		if (!channels[ch].ontime_of_period_added_to_daily)
					{
			         append_log(IRR_FILE,"%s %ds  %1.0fl",channels[ch].Name,channels[ch].period_ontime,1.0*channels[ch].period_volume/YF_DN32_PULSE_PER_LITER);
				 	 channels[ch].prev_daily_period_ontimes+=channels[ch].period_ontime;
				 	 channels[ch].ontime_of_period_added_to_daily=true;
					}
	 }


	 
	 if(new_relay_status) {if (!is_channel_active(ch)) {channels[ch].last_switch_on_time=now;channels[ch].period_volume=0;}} // from OFF to ON
	else if (is_channel_active(ch)) if (channels[ch].last_switch_on_time!=-1) {channels[ch].period_ontime+=now-channels[ch].last_switch_on_time;} //from ON to OFF
	


	if(measure_mode==LOG)
				{
					if(mqtt_connected)
					{
					MQTT_BLE_answer[0]=0;	
					append_ontimes2string(ch);	
					msg_id = my_esp_mqtt_client_publish(mqtt_client, "LOG", MQTT_BLE_answer, 0, 0, 0);   //Qos=0; retain=0
					}
	  
				}

	channels[ch].status_change_time[status]=now;	
	channels[ch].channel_state=status;		   
    switch_channel_relays(ch,new_relay_status);
				
				if(mqtt_connected)
				{	
			     if((channels[ch].channel_state==SUSPENDED) || (channels[ch].channel_state==DELAY))
				 {
				  char message[64]; 
			      sprintf(message,"%s->%s",str_states[channels[ch].channel_state],str_states[channels[ch].channel_states_before_suspend]);
				  msg_id = esp_mqtt_client_publish(mqtt_client,topicname,message , 0, 0, 1);
				 }
		         else
				 {			 
			    	 msg_id = esp_mqtt_client_publish(mqtt_client,topicname,str_states[status]  , 0, 0, 1);
				     ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
				 }
				}
	}			
}


static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
	//	Write_Msg_toDisplay(0,"Start Wifi connection.");
		esp_wifi_connect();
        
/*	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {	
	   Write_Msg_toDisplay(0,"Wifi connected");
	*/	
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
	//	Write_Msg_toDisplay(0,"Wifi disconnected");
		ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
       // if(!smartconfig_started)
		{
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			if (mqtt_connected) 
			{
				esp_mqtt_client_stop(mqtt_client);
				mqtt_connected=0;
			}
			esp_sntp_stop();
			ESP_LOGI(TAG, "WIFI reconnecting");
			wifi_retry_count++;
			ESP_LOGI(TAG, "wifi_retry_count:%d",wifi_retry_count);
			vTaskDelay(10*1000 / portTICK_PERIOD_MS);	
			//SmartConfigIflongpresssed();
			if(wifi_retry_count==100)
			{
			 wifi_retry_count=0;
			 reboot_WIFI_STICK();
			 vTaskDelay(10*1000 / portTICK_PERIOD_MS);	
			}				
			else if(wifi_retry_count==150) 
			{
				wifi_retry_count=0;
				esp_restart();
			}
			esp_wifi_connect();
		}
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        
		app_main_SNTP();
		mqtt_app_start();
		wifi_retry_count=0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		
    }/* else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
		Write_Msg_toDisplay(0,"Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
		Write_Msg_toDisplay(0,"Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");
		Write_Msg_toDisplay(0,"Got SSID and password:");
        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s",(char*)ssid);
        ESP_LOGI(TAG, "PASSWORD:%s",(char*)password);
		Write_Msg_toDisplay(1,(char*)ssid);
		Write_Msg_toDisplay(2,(char*)password);
		vTaskDelay(3*1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH)); 
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }*/
}
/*
static void initialise_wifi(void)
{
	ESP_LOGI(TAG, "initialise_wifi");
	EventBits_t uxBits;
    const TickType_t xTicksToWait = 300000 / portTICK_PERIOD_MS;
    esp_netif_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
	
	uxBits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, false, xTicksToWait); 
    if(uxBits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
    }
	else{
		//timeout
		 ESP_LOGI(TAG, "WiFi timeout");
		 reboot_WIFI_STICK();
		 ESP_LOGI(TAG, "ESP restart");
		 esp_restart();
	}
	
}*/

 wifi_config_t wifi_config;
void initialise_wifi(void)
{
	Write_Msg_toDisplay(0,"connect to WIFI...");
	const TickType_t xTicksToWait = 300000 / portTICK_PERIOD_MS;
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
//	esp_event_handler_instance_t instance_scan;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL,&instance_got_ip));
//	ESP_ERROR_CHECK(esp_event_handler_instance_register(SC_EVENT, ESP_EVENT_ANY_ID,  &event_handler,NULL,&instance_scan));													


/*
    memcpy(wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
    wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;
	memcpy(wifi_config.sta.sae_h2e_identifier, EXAMPLE_H2E_IDENTIFIER, sizeof(wifi_config.sta.sae_h2e_identifier));
*/

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
   // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            xTicksToWait /*portMAX_DELAY*/);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", wifi_config.sta.ssid, wifi_config.sta.password);
    } 	
	else{
		//timeout
		 ESP_LOGI(TAG, "WiFi timeout");
		 reboot_WIFI_STICK();
		 ESP_LOGI(TAG, "ESP restart");
		 esp_restart();
	}
}


void read_ACS71020_register2(int reg_addr,long value)
{
    char strval[128];
	eeprom_reg_t reg;
    reg.address = reg_addr;
    reg.frame.value =  value;
    eeprom_0x0B_t addr_0x0B;
    eeprom_0x0C_t addr_0x0C;   
    eeprom_0x0D_t addr_0x0D;
    eeprom_0x0E_t addr_0x0E;
    eeprom_0x0F_t addr_0x0F;
   
   acs_0x20_t addr_0x20;
   acs_0x21_t addr_0x21;
   acs_0x22_t addr_0x22;
   acs_0x23_t addr_0x23;
   acs_0x24_t addr_0x24;
   acs_0x25_t addr_0x25;
   acs_0x26_t addr_0x26;
   acs_0x27_t addr_0x27;
   acs_0x28_t addr_0x28;
   acs_0x29_t addr_0x29;
   acs_0x2A_t addr_0x2A;
   acs_0x2B_t addr_0x2B;
   acs_0x2C_t addr_0x2C;
   acs_0x2D_t addr_0x2D;
  // acs_0x2E_t addr_0x2E;
   acs_0x2F_t addr_0x2F;
   acs_0x30_t addr_0x30;
   
  
   
   
   acs_reg_t req_volatile;
   req_volatile.address = reg_addr;
   req_volatile.register_value =  value;
    
    switch (reg_addr)
	{
	 case 0x0B:
	 case 0x1B:
	
     addr_0x0B.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"qvo_fine: %d", addr_0x0B.fields.qvo_fine);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"sns_fine: %d", addr_0x0B.fields.sns_fine);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"crs_sns: %d", addr_0x0B.fields.crs_sns);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"iavgselen: %d", addr_0x0B.fields.iavgselen);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 
	 case 0x0C:
	 case 0x1C:
	 
     addr_0x0C.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"rms_avg_2: %d", addr_0x0C.fields.rms_avg_2);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"rms_avg_1: %d", addr_0x0C.fields.rms_avg_1);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 	 
	 break;
	 
	 case 0x0D:
	 case 0x1D:
	
     addr_0x0D.eeprom_data = reg.frame.fields.eeprom_data;
     sprintf(strval,"pacc_trim: %d", addr_0x0D.fields.pacc_trim);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
     sprintf(strval,"ichan_del_en: %d", addr_0x0D.fields.ichan_del_en);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"chan_del_sel: %d", addr_0x0D.fields.chan_del_sel);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"fault: %d", addr_0x0D.fields.fault);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"fltdly: %d", addr_0x0D.fields.fltdly);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"halfcycle_en: %d", addr_0x0D.fields.halfcycle_en);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     sprintf(strval,"squarewave_en: %d", addr_0x0D.fields.squarewave_en);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
     break;
	 case 0x0E:
	 case 0x1E:
	 
     addr_0x0E.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"delaycnt_sel: %d", addr_0x0E.fields.delaycnt_sel);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"undervreg: %d", addr_0x0E.fields.undervreg);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"overvreg: %d", addr_0x0E.fields.overvreg);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"vadc_rate_set: %d", addr_0x0E.fields.vadc_rate_set);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"vevent_cycs: %d", addr_0x0E.fields.vevent_cycs);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break; 
	 case 0x0F:
	 case 0x1F:
	
     addr_0x0F.eeprom_data = reg.frame.fields.eeprom_data; 
	 sprintf(strval,"dio_1_sel: %d", addr_0x0F.fields.dio_1_sel);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"dio_0_sel: %d", addr_0x0F.fields.dio_0_sel);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 sprintf(strval,"i2c_dis_slv_addr: %d", addr_0x0F.fields.i2c_dis_slv_addr);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"i2c_slv_addr: %d", addr_0x0F.fields.i2c_slv_addr);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 
	 case 0x20:
     addr_0x20.register_value = req_volatile.register_value; 
	 sprintf(strval,"irms: %d, %2.2fA", addr_0x20.fields.irms,addr_0x20.fields.irms*30/pow(2,14));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"vrms: %d, %2.2fV", addr_0x20.fields.vrms,addr_0x20.fields.vrms*550/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x21:
	 addr_0x21.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactive: %d, %2.2fW", addr_0x21.fields.pactive,addr_0x21.fields.pactive*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x22:
	 addr_0x22.register_value = req_volatile.register_value; 
	 sprintf(strval,"papparent: %d, %2.2fVA", addr_0x22.fields.papparent,addr_0x22.fields.papparent*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x23:
	 addr_0x23.register_value = req_volatile.register_value; 
	 sprintf(strval,"pimag: %d, %2.2fVA", addr_0x23.fields.pimag,addr_0x23.fields.pimag*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x24:
	 addr_0x24.register_value = req_volatile.register_value; 
	 sprintf(strval,"pfactor: %d, %2.2f", addr_0x24.fields.pfactor,addr_0x24.fields.pfactor/pow(2,9));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x25:
	 addr_0x25.register_value = req_volatile.register_value; 
	 sprintf(strval,"numptsout: %d", addr_0x25.fields.numptsout);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	   case 0x26:
	 addr_0x26.register_value = req_volatile.register_value; 
	 sprintf(strval,"irmsavgonesec: %d, %2.2fA", addr_0x26.fields.irmsavgonesec,addr_0x26.fields.irmsavgonesec*30/pow(2,14));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	  sprintf(strval,"vrmsavgonesec: %d, %2.2fV", addr_0x26.fields.vrmsavgonesec,addr_0x26.fields.vrmsavgonesec*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	    case 0x27:
	 addr_0x27.register_value = req_volatile.register_value; 
	 sprintf(strval,"irmsavgonemin: %d, %2.2fA", addr_0x27.fields.irmsavgonemin,addr_0x27.fields.irmsavgonemin*30/pow(2,14));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	  sprintf(strval,"vrmsavgonemin: %d, %2.2fV", addr_0x27.fields.vrmsavgonemin,addr_0x27.fields.vrmsavgonemin*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	     case 0x28:
	 addr_0x28.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactavgonesec: %d, %2.2fW", addr_0x28.fields.pactavgonesec,addr_0x28.fields.pactavgonesec*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	      case 0x29:
	 addr_0x29.register_value = req_volatile.register_value; 
	 sprintf(strval,"pactavgonemin: %d, %2.2fW", addr_0x29.fields.pactavgonemin,addr_0x29.fields.pactavgonemin*30*0.275*(R1_4+Rs)/Rs/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x2A:
	 addr_0x2A.register_value = req_volatile.register_value; 
	 sprintf(strval,"vcodes: %d, %2.2FV", addr_0x2A.fields.vcodes,addr_0x2A.fields.vcodes*0.275*(R1_4+Rs)/Rs/pow(2,16));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	
	 break;
	 case 0x2B:
	 addr_0x2B.register_value = req_volatile.register_value; 
	 sprintf(strval,"icodes: %d, %2.2FA", addr_0x2B.fields.icodes,addr_0x2B.fields.icodes*30/pow(2,15));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 case 0x2C:
	 addr_0x2C.register_value = req_volatile.register_value; 
	 sprintf(strval,"pinstant: %ld, %2.2fW", addr_0x2C.fields.pinstant,addr_0x2C.fields.pinstant*30*0.275*(R1_4+Rs)/Rs/pow(2,29));
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x2D:
	 addr_0x2D.register_value = req_volatile.register_value; 
	 sprintf(strval,"vzerocrossout: %d", addr_0x2D.fields.vzerocrossout);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"faultout: %d", addr_0x2D.fields.faultout);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"faultlatched: %d", addr_0x2D.fields.faultlatched);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"overvoltage: %d", addr_0x2D.fields.overvoltage);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"undervoltage: %d", addr_0x2D.fields.undervoltage);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"posangle: %d", addr_0x2D.fields.posangle);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 sprintf(strval,"pospf: %d", addr_0x2D.fields.pospf);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	  case 0x2F:
	 addr_0x2F.register_value = req_volatile.register_value; 
	 sprintf(strval,"access_code: %ld", addr_0x2F.fields.access_code);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	   case 0x30:
	 addr_0x30.register_value = req_volatile.register_value; 
	 sprintf(strval,"customer_access: %d", addr_0x30.fields.customer_access);
	 my_esp_mqtt_client_publish(mqtt_client, "ACS71020", strval, 0, 0, 0);   //Qos=0; retain=0	 
	 break;
	 
	default: break;
	}
}


bool isSingleChannelTurnedON(int ch)
{
 for (int i=0; i<CHANNEL_NUM; i++)
 {
  if ((ch!=i) && channels[i].Channel_pump_ON)	return false; 
 }	 
 return true;
}


void mainTask(void *pvParameters){
  ds18b20_init(DS_PIN);
 
  int msg_id;
  int ch,i;
  time_t time_at_start;
  time_t runtime=0;
  
  xSemaphoreTake(MAIN_TASK_mutex, portMAX_DELAY);


  time(&time_at_start);
  int      TimeToPublish = 600000000; //in uS
  
  xEventGroupWaitBits(s_wifi_event_group, MQTT_CONNECTED_BIT, false, false, 30*1000 / portTICK_PERIOD_MS); 
  

   for(ch=0;ch<CHANNEL_NUM;ch++) 
   {
	if(channels[ch].channel_disabled) switch_channel(ch,DISABLED);
	else  switch_channel(ch,REBOOTED);
   }

 uint64_t TimePastPublish = esp_timer_get_time();

 

  while (!(FW_update_available) && strlen(OTA_SOURCE_URL) && ((run_mode & (1<<MAIN_TASK))>0)) {
	  if(measure_mode==STACK)
	  { char message[64];
	    int unused_stack=  uxTaskGetStackHighWaterMark(NULL); 
		if(mqtt_connected)
		{	
		wifi_ap_record_t ap;
		esp_wifi_sta_get_ap_info(&ap);
		sprintf(message,"unused_stack:%dbytes, runtime:%llds, RSSSI:%d",unused_stack, runtime, ap.rssi);
		my_esp_mqtt_client_publish(mqtt_client, "MEASURE/STACK", message, 0, 0, 0);   //Qos=0; retain=0
	
		}
		
	  }
 			
    for(ch=0;ch<CHANNEL_NUM;ch++) //manual switches
	{		
     if(mqtt_connected && (channels[ch].manual_change_request!=NOREQUEST))
	 {
		switch_channel(ch,channels[ch].manual_change_request);
		channels[ch].manual_change_request=NOREQUEST;
	 }
	}	


	     
	
	if((FW_update_available) && strlen(OTA_SOURCE_URL))
	{
	 my_esp_mqtt_client_publish(mqtt_client, "FIRMWARE/UPDATE", "started", 0, 0, 0);   //Qos=0; retain=0	
	 
	 for(ch=0;ch<CHANNEL_NUM;ch++) switch_channel(ch,DISABLED);
			
	 break;
	}
	
	  vTaskDelay(1*1000 / portTICK_PERIOD_MS);
	 
	
	 {
	 int ch;
	 char timeStr[32];
	 int dayofweek;
	 time_t now;
	 time(&now);
	 runtime=now-time_at_start;
	 
		 
	 now-=1658700000;
	 static int prev_dayofweek=0;
	 
	
	dayofweek=(int)(now/86400)%7+1;
	if(prev_dayofweek!=dayofweek) //midnight
	{
		time_t now2;
	    time(&now2);
		
		for(ch=0;ch<CHANNEL_NUM;ch++) 
		{
			channels[ch].prev_daily_period_ontimes=0; //erase ontime at daychange
			channels[ch].period_volume=0; //erase volume at daychange
			channels[ch].daily_volume=0;
		}
   
		if(((prev_dayofweek>0) && ((now2-now_life_received)>1800))) // reboot at every midnight if mqtt broken more than 1/2 hour
		{
			reboot_WIFI_STICK();
			esp_restart();
		}
	}
		
	
	prev_dayofweek=dayofweek;
	
	now%=86400; 
	int hour=(int)now/3600;
	if(prevhour!=hour)
	{
	 MQTT_BLE_answer[0]=0;	
	 for(i=0;i<CHANNEL_NUM;i++) append_ontimes2string(i);
	 my_esp_mqtt_client_publish(mqtt_client, "REPORT/ONTIME", MQTT_BLE_answer, 0, 0, 0);   //Qos=0; retain=0

	 
	 sprintf(MQTT_BLE_answer,"%0.1fcm",water_level);
	
	  my_esp_mqtt_client_publish(mqtt_client, "REPORT/LEVEL", MQTT_BLE_answer, 0, 0, 0);   //Qos=0; retain=0
	
	 prevhour=hour;
	}
	
	for(ch=0;ch<CHANNEL_NUM;ch++) //manual switches
	{		
     if(channels[ch].channel_state==MAN_ON)
	 {
		if (now-channels[ch].last_switch_on_time>channels[ch].requested_ontime)  switch_channel(ch,MAN_OFF);
	 }
	}



	
    if (run_mode & (1<<TEMPSENSOR)) temperature=ds18b20_get_temp();
   
	if(SNTP_synchronized)
	{	
	 char* days[7]={"M","Tu","W","Th","F","Sa","Su"};	
     sprintf(timeStr,"%2.2d:%2.2d:%2.2d %s %0.2f°C",(int)((now)/3600),(int)(((now)%3600)/60),(int)((now)%60),days[dayofweek-1],temperature); 
	 Write_Msg_toDisplay(1,timeStr);	
	
	if ( (esp_timer_get_time() - TimePastPublish) >= TimeToPublish )
    {
       if(mqtt_connected) 
	   {	
			  char message[64];             
			  time(&now_life_sent);
			  sprintf(message,"%lld:%s",now_life_sent,timeStr);   
			  my_esp_mqtt_client_publish(mqtt_client, "LIFE", message, 0, 0, 1);   //Qos=0; retain=1
	   }
      TimePastPublish = esp_timer_get_time(); // get next publish time
    }
	
	
	int delta_volume_cnt=measure_flowrate();
	
    if(measure_mode==VOLUME)
		{
		 if(mqtt_connected)
		 {	
		  char message[128];
          GetVolumeString(message);
		  msg_id = my_esp_mqtt_client_publish(mqtt_client, "MEASURE/VOLUME", message, 0, 0, 0);   //Qos=0; retain=0
	     }
		}
		 
	if (delta_volume_cnt>0) //waterflow
	{
	 for(ch=0; ch<CHANNEL_NUM;ch++) 
	 {
		 if (isSingleChannelTurnedON(ch)) {channels[ch].daily_volume+=delta_volume_cnt;channels[ch].period_volume+=delta_volume_cnt;} 
		 else
		 {
	      if (channels[ch].fix_preassure) 
		  {
			  
		  }
			 
		 }
	 }
	 
	 
/*
	 else if ( Channel_pump_ON[0] && !Channel_pump_ON[1] &&  Channel_pump_ON[2]) {daily_volume[0]+=delta_volume_cnt*ratio02;daily_volume[2]+=delta_volume_cnt*(1.0-ratio02);
	 else if ( Channel_pump_ON[0] &&  Channel_pump_ON[1] &&  !Channel_pump_ON[2]){daily_volume[0]+=delta_volume_cnt*ratio01;daily_volume[1]+=delta_volume_cnt*(1.0-ratio01);
	 else if (!Channel_pump_ON[0] &&  Channel_pump_ON[1] &&  Channel_pump_ON[2]) {daily_volume[1]+=delta_volume_cnt*ratio12;daily_volume[2]+=delta_volume_cnt*(1.0-ratio12);
	 else if ( Channel_pump_ON[0] &&  Channel_pump_ON[1] &&  Channel_pump_ON[2]) {daily_volume[0]+=delta_volume_cnt*ratio0;daily_volume[1]+=delta_volume_cnt*ratio1;daily_volume[1]+=delta_volume_cnt*(1.0-ratio0-ratio1);
	*/}
	
	
	
	for(ch=0;ch<CHANNEL_NUM;ch++)
	{
	 if (channels[ch].channel_disabled) continue; 	
	 for(i=0;i<PERIODS;i++)
	 {
	   if(channels[ch].Chedule_array[i].on_time<channels[ch].Chedule_array[i].off_time)
	   {	
			//ESP_LOGI(TAG,"ch=%d,period=%d,day=%d,time:%d,start%d,stop%d,day:%c",ch,i,dayofweek,(int) now,(int)channels[ch].Chedule_array[i].on_time,(int)channels[ch].Chedule_array[i].off_time,channels[ch].Chedule_array[i].weekdays[dayofweek]);
		if((channels[ch].Chedule_array[i].on_time<now) && (channels[ch].Chedule_array[i].off_time>now) && ((channels[ch].Chedule_array[i].weekdays[dayofweek-1]=='+') || (channels[ch].Chedule_array[i].weekdays[dayofweek-1]=='x') || (channels[ch].Chedule_array[i].weekdays[dayofweek-1]=='X')|| (channels[ch].Chedule_array[i].weekdays[dayofweek-1]=='1')))
		{
			ESP_LOGI(TAG,"times:%d<%d<%d",(int)channels[ch].Chedule_array[i].on_time,(int)now,(int)channels[ch].Chedule_array[i].off_time);
			
			
			if(now-channels[ch].Chedule_array[i].on_time<30) 
			{ 
				if(channels[ch].channel_state!=PROG_STARTED) switch_channel(ch,PROG_STARTED);		
			}
			else if(channels[ch].Chedule_array[i].off_time-now<30) 
			{
				if(channels[ch].channel_state!=PROG_END) switch_channel(ch,PROG_END);			
			}
			else if(is_channel_active(ch))
			{
     		 if ((channels[ch].Chedule_array[i].duration<=(channels[ch].period_ontime + now-channels[ch].last_switch_on_time)/60) ||
			    (channels[ch].Chedule_array[i].volume<=channels[ch].period_volume/YF_DN32_PULSE_PER_LITER))	
				 switch_channel(ch,PROG_FINISHED);		 
			}	
		}	
	   }		
	 }// for periods
	}//for channel
	}
	else Write_Msg_toDisplay(1,"wait for SNTP sync.");
    
	switch (check_pump_protection(now))
	{
	 case P_SUSPENDED:
	      for(ch=0;ch<CHANNEL_NUM;ch++) 
		  { 
	        switch(channels[ch].channel_state)
			{ //switch active channels to SUSPENDED
				case MAN_ON:
				case PROG_STARTED:
				case MAN_RESUMED:
				case PROG_RESUMED:
				  channels[ch].channel_states_before_suspend=channels[ch].channel_state;
				  switch_channel(ch,SUSPENDED);
				default: break;
			}  
		  }
		  Write_Msg_toDisplay(2,"pump suspended");
		  break;
     case P_DELAY:
	      for(ch=0;ch<CHANNEL_NUM;ch++)
			 { 
			  if(channels[ch].channel_state==SUSPENDED) 
			   {				   
				     switch_channel(ch,DELAY);
				}
			 } 
	      break;
     case P_RESUMED:
	 		for(ch=0;ch<CHANNEL_NUM;ch++)
			{ 
			 switch(channels[ch].channel_state)
			 {
				case SUSPENDED: [[fallthrough]];
				case DELAY: 
			                 switch (channels[ch].channel_states_before_suspend)
							 {
								case MAN_ON: switch_channel(ch,MAN_RESUMED); break;
								case PROG_STARTED: switch_channel(ch,PROG_RESUMED); break;
								default:switch_channel(ch,channels[ch].channel_states_before_suspend);
							 }	
							 break;
				default:	break;
			 } 
			}
	      break;
     default:
	         break;		  

	}
	 
	{
	
		 char message[32]="";
		 for(ch=0;ch<CHANNEL_NUM;ch++)
		 {
		  strcat(message,str_short_states[channels[ch].channel_state]);
	      if (ch<CHANNEL_NUM-1) strcat(message," ");
		 }
		 Write_Msg_toDisplay(0,message);
		  
	}		

if (run_mode & (1<<MEASURE_POWER)) 
{
char message[128];
xSemaphoreTake(mutex, portMAX_DELAY);
		
/*	
double i=    MeasuredValue(ACS71020_address_default, 0x2B, 0x0001ffff,15, 0,15,30.0);
// ESP_LOGI(TAG, "i= %lf", i);
double u=    MeasuredValue(ACS71020_address_default, 0x2A, 0x0001ffff,15, 0,16,0.275*(R1_4+Rs)/Rs; 
//ESP_LOGI(TAG, "u= %lf", u);
*/
double irms= MeasuredValue(ACS71020_address_default, 0x20, 0x7fff0000, 0,16,14,30.0);
//ESP_LOGI(TAG, "irms= %lf", irms);
double urms= MeasuredValue(ACS71020_address_default, 0x20, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs); 

/*
double urms_1s= MeasuredValue(ACS71020_address_default, 0x26, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs; 

double urms_1m= MeasuredValue(ACS71020_address_default, 0x27, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs; 


//ESP_LOGI(TAG, "urms= %lf", urms);
*/
double num=  MeasuredValue(ACS71020_address_default, 0x25, 0x000001ff, 0, 0,0,1); 
//ESP_LOGI(TAG, "num= %lf", num);


double p=    MeasuredValue(ACS71020_address_default, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs);
//ESP_LOGI(TAG, "p= %lf", p);


xSemaphoreGive(mutex);

 sprintf(message,"%0.1lfV %0.1lfA %0.1lfW",urms,irms,p);
 Write_Msg_toDisplay(3,message);
 
 if(measure_mode==POWER)
 {
  sprintf(message,"Urms=%0.1lfV  Irms=%0.2lfA  P=%0.1lfW num=%0.0lf consumption:%lfkWh\n",urms,irms,p,num,powerconsumptionWs/3600/1000);
  	 if(mqtt_connected)
     {	
		msg_id = my_esp_mqtt_client_publish(mqtt_client, "MEASURE/POWER", message, 0, 0, 0);   //Qos=0; retain=0
	    ESP_LOGI(TAG, "publish successful, msg_id=%d", msg_id);	
	 }
 }


}

if(run_mode & (1<<MEASURE_LEVEL)) //measure water level
{
	char message[128];
 
        /*
        uint32_t adc_reading = 0;
        //Multisampling
		
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		//ESP_LOGI(TAG, "Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
		water_level=((voltage-142)/44.13*3/2);
		sprintf(message,"%2.1fcm",water_level);		
		Write_Msg_toDisplay(2,message);*/
		
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
			water_level=((voltage[0][0]-142)/44.13*3/2);
			sprintf(message,"%2.1fcm",water_level);		
			Write_Msg_toDisplay(2,message);
        }
		
		
		
		
		if(measure_mode==LEVEL)
		{
		 //sprintf(message,"Raw: %d,Voltage: %dmV level:%2.1fcm", adc_reading, voltage, (voltage-142)/44.13*3/2);
		 sprintf(message,"Level:%2.1fcm Temp:%0.2f°C", (voltage[0][0]-142)/44.13*3/2, temperature);
		
		  msg_id = my_esp_mqtt_client_publish(mqtt_client, "MEASURE/LEVEL", message, 0, 0, 0);   //Qos=0; retain=0
	   
		}
		if(measure_mode==CT)
		{
			/*TODO
		uint32_t current_CT[100]={};
		float avg_current_CT=0;
		float rms_current_CT=0;
		int zerocross=0;
		int first_zerocross=0;
		int last_zerocross=0;
		for(i=0;i<100;i++)
		{	
		 
		 current_CT[i] = esp_adc_cal_raw_to_voltage(adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1), adc_chars)*10;
		 avg_current_CT+=current_CT[i];
		 vTaskDelay(0.1 / portTICK_PERIOD_MS);
		}
		avg_current_CT/=100;
		for(i=0;i<100;i++)
		{
			if((i<99) && (current_CT[i]<=avg_current_CT) && (current_CT[i+1]>avg_current_CT)) {zerocross++; if(zerocross==1) first_zerocross=i;}
			else if ((i<99) && (current_CT[i]>avg_current_CT) && (current_CT[i+1]<=avg_current_CT)) {zerocross++; last_zerocross=i;}
		}
		for(i=first_zerocross;i<last_zerocross;i++)
		{
			rms_current_CT+= pow(current_CT[i]-avg_current_CT,2);
		}
				
		rms_current_CT=sqrt(rms_current_CT/(last_zerocross-first_zerocross));
		
        if(mqtt_connected)
		 {
		  sprintf(message,"CT_rms=%2.1fA first:%d,last:%d,zerocross=%d, offset=%f",rms_current_CT,first_zerocross,last_zerocross,zerocross,avg_current_CT);				 
		  msg_id = my_esp_mqtt_client_publish(mqtt_client, "MEASURE/CT", message, 0, 0, 0);   //Qos=0; retain=0
	     }
		
		}
		*/
		
		
		// ESP_LOGI(TAG, "CT voltage: %dmV\n", current_CT);
}	 
  
  
  
  
 
  
  
  }
  }
  } //while(1)

  xSemaphoreGive(MAIN_TASK_mutex);

}

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

esp_err_t _http_event_handler_OTA(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
	case HTTP_EVENT_REDIRECT:
	     ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
         break;
	   
    }
    return ESP_OK;
}

esp_err_t my_esp_https_ota(const esp_http_client_config_t *config)
{
	char message[128];
	esp_app_desc_t new_app_info;
	
    if (!config) {
        ESP_LOGE(TAG, "esp_http_client config not found");
		sprintf(MQTT_BLE_answer,"Error:%s","esp_http_client config not found"); 
	    if(mqtt_connected)
		{
	   	 my_esp_mqtt_client_publish(mqtt_client, "OTA/ERROR", message, 0, 0, 0);   //Qos=1; retain=0
       	 vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
		return ESP_ERR_INVALID_ARG;
    }    

    esp_https_ota_config_t ota_config = {
        .http_config = config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (https_ota_handle == NULL) {
		sprintf(MQTT_BLE_answer,"Error:%s","esp_https_ota_begin failed"); 
		if(mqtt_connected)
		{
	     my_esp_mqtt_client_publish(mqtt_client, "OTA/ERROR", message, 0, 0, 0);   //Qos=1; retain=0
         vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
        return ESP_FAIL;
    }
	
	
    esp_https_ota_get_img_desc(https_ota_handle, &new_app_info);

        sprintf(MQTT_BLE_answer,"Image project name:%s",new_app_info.project_name); 
		if(mqtt_connected)
		{
	     my_esp_mqtt_client_publish(mqtt_client, "OTA/INFO", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=0
         vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
   

	
	 ESP_LOGE(TAG,"FW project:Running:%s, received:%s",app_desc->project_name,new_app_info.project_name); 
	 if(strcmp(app_desc->project_name,new_app_info.project_name)!=0)
	 {
		ESP_LOGE(TAG, "wrong FIRMWARE project. Running:%s, received:%s",app_desc->project_name,new_app_info.project_name); 
		sprintf(MQTT_BLE_answer,"wrong FIRMWARE project. Running:%s, received:%s",app_desc->project_name,new_app_info.project_name); 
		if(mqtt_connected)
		{
	     my_esp_mqtt_client_publish(mqtt_client, "OTA/ERROR", message, 0, 0, 0);   //Qos=1; retain=0
         vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
		return ESP_FAIL;
	 }
	
	
	 ESP_LOGE(TAG, "FIRMARE version. Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
	  sprintf(MQTT_BLE_answer,"Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
	 if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/RUNNING_INFO", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
	  if(strcmp(new_Firmware_version,new_app_info.version)!=0) 
	 {
		 ESP_LOGE(TAG, "wrong FIRMWARE version. Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
		 sprintf(MQTT_BLE_answer,"wrong FIRMWARE version. Running:%s, Expected:%s, received:%s",app_desc->version,new_Firmware_version,new_app_info.version); 
		 if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/RUNNING_INFO", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
		 return ESP_FAIL;
	 }
 

    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
    }

    
	     sprintf(MQTT_BLE_answer,"%s finished","esp_https_ota_perform"); 
		 if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/INFO", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}

    esp_err_t ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if (err != ESP_OK) {
        /* If there was an error in esp_https_ota_perform(),
           then it is given more precedence than error in esp_https_ota_finish()
         */
		sprintf(MQTT_BLE_answer,"%s failed","esp_https_ota_perform"); 
		 if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/ERROR", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
		
        return err;
    } else if (ota_finish_err != ESP_OK) {
		sprintf(MQTT_BLE_answer,"%s failed","esp_https_ota_finish"); 
        if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/ERROR", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		}
        return ota_finish_err;
    }
    return ESP_OK;
}




void ota_update_task(void *pvParameter)
{	
 xSemaphoreTake(MAIN_TASK_mutex, portMAX_DELAY);	
 ESP_LOGI(TAG, "Starting OTA update");
 //int ptr=strlen(OTA_SOURCE_URL)-1;
//while((ptr>OTA_SOURCE_URL) && (strcmp(OTA_SOURCE_URL[ptr-3],".bin"!=0))) OTA_SOURCE_URL[ptr--]=0;

 ESP_LOGI(TAG, "OTA URL:%s",OTA_SOURCE_URL);
//ESP_LOGI(TAG, "OTA CERT:%s",server_cert_pem_start);




    esp_http_client_config_t config = {
		.url = OTA_SOURCE_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = _http_event_handler_OTA,
    };
	
	
#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif


 //   esp_err_t ret = esp_https_ota(&config);
 esp_err_t ret = my_esp_https_ota(&config);
    if (ret == ESP_OK) {
       sprintf(MQTT_BLE_answer,"OTA update finished.%s","Restarting..."); 
        if(mqtt_connected)
		{
	       my_esp_mqtt_client_publish(mqtt_client, "OTA/DONE", MQTT_BLE_answer, 0, 0, 0);   //Qos=1; retain=1
	       vTaskDelay(3*1000 / portTICK_PERIOD_MS);
		   esp_mqtt_client_stop(mqtt_client); 
		} 

        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
   
    FW_update_available=false;
	xSemaphoreGive(MAIN_TASK_mutex);
}


void Load_data_from_NVS()
{
    nvs_handle_t nvs_handle;
	 int i,ch;
	 size_t buf_len;
	 int ret;
	 long int intval;
    if((ret=nvs_open("my_NVS", NVS_READWRITE, &nvs_handle))!=ESP_OK) ESP_LOGI(TAG, "NVS open failed. %d",ret);
	
	if(nvs_get_i32(nvs_handle, "pump_delay",&intval)==ESP_OK) pump_restart_delay=(int)intval;
	ESP_LOGI(TAG, "pump_restart_delay:%dmin",pump_restart_delay);
	
	if(nvs_get_i32(nvs_handle, "run_mode",&intval)==ESP_OK) run_mode=(int)intval | (1<<USE_BLE) | (1<<USE_WIFI);;
	ESP_LOGI(TAG, "run_mode:%d",run_mode);
	
	nvs_set_i32(nvs_handle, "PUMP1_DISABLED", isPUMP_disabled(0)?1:0);  
    nvs_set_i32(nvs_handle, "PUMP2_DISABLED", isPUMP_disabled(1)?1:0); 

    nvs_get_i32(nvs_handle, "PUMP1_DISABLED", &intval);	
	enable_pump(0, intval?false:true);
    nvs_get_i32(nvs_handle, "PUMP2_DISABLED", &intval);	
	enable_pump(1, intval?false:true);

   nvs_get_i32(nvs_handle, "PUMP1_PRIO", &intval);
   setPUMP_prio(0,intval==1);
   nvs_get_i32(nvs_handle, "PUMP2_PRIO", &intval);
   setPUMP_prio(1,intval==1);  

   nvs_get_i32(nvs_handle, "PUMP1_SWITCHBACK", &intval); 
   setPUMP_switchbackifavailable(0,intval==1); 
   nvs_get_i32(nvs_handle, "PUMP2_SWITCHBACK", &intval); 
   setPUMP_switchbackifavailable(1,intval==1); 
	
	for (ch=0;ch<CHANNEL_NUM;ch++)
    {
	  char keyName[32];   //CHx_x 
	  long int keyvalue;
	  size_t length=sizeof(channels[ch].Name);
	  sprintf(keyName,"CH%1.1d_NAME",ch);
	  nvs_get_str(nvs_handle, keyName, channels[ch].Name,&length);
	  sprintf(keyName,"CH%1.1d_DISABLED",ch);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &keyvalue);	
	  channels[ch].channel_disabled=(int)keyvalue;
		
	 
     for(i=0;i<PERIODS;i++)
     {
	  sprintf(keyName,"CH%1.1d_ON_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &channels[ch].Chedule_array[i].on_time);
	  ESP_LOGI(TAG, "VALUE:%d",(int)channels[ch].Chedule_array[i].on_time);
	  sprintf(keyName,"CH%1.1d_OFF_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &channels[ch].Chedule_array[i].off_time);
	  ESP_LOGI(TAG, "VALUE:%d",(int)channels[ch].Chedule_array[i].off_time);
	  sprintf(keyName,"CH%1.1d_DAY_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  buf_len = sizeof(channels[ch].Chedule_array[i].weekdays);
	  nvs_get_str(nvs_handle, keyName, channels[ch].Chedule_array[i].weekdays,&buf_len);
      ESP_LOGI(TAG, "VALUE:%s",channels[ch].Chedule_array[i].weekdays);
		
	  sprintf(keyName,"CH%1.1d_DUR_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &channels[ch].Chedule_array[i].duration);
	  ESP_LOGI(TAG, "VALUE:%d",(int)channels[ch].Chedule_array[i].duration);	
	  sprintf(keyName,"CH%1.1d_VOL_%1.1d",ch,i);
	  ESP_LOGI(TAG, "KEY:%s",keyName);
	  nvs_get_i32(nvs_handle, keyName, &channels[ch].Chedule_array[i].volume);
	  ESP_LOGI(TAG, "VALUE:%d",(int)channels[ch].Chedule_array[i].volume);
    } 
   }
	
    nvs_close(nvs_handle);
}


void Save_data_to_NVS()
{
    nvs_handle_t  nvs_handle;
	 int i,ch;
    if(nvs_open("my_NVS", NVS_READWRITE, &nvs_handle)!=ESP_OK) ESP_LOGI(TAG, "NVS OPEN FAILED");
		
   // TEST_ESP_OK(nvs_erase_all(nvs_handle));
   
    nvs_set_i32(nvs_handle, "pump_delay",pump_restart_delay);
	nvs_set_i32(nvs_handle, "run_mode",run_mode);
	
   nvs_set_i32(nvs_handle, "PUMP1_DISABLED", isPUMP_disabled(0)?1:0);  
   nvs_set_i32(nvs_handle, "PUMP2_DISABLED", isPUMP_disabled(1)?1:0);   

   nvs_set_i32(nvs_handle, "PUMP1_PRIO", getPUMP_prio(0)?1:0);  
   nvs_set_i32(nvs_handle, "PUMP2_PRIO", getPUMP_prio(1)?1:0);  

   nvs_set_i32(nvs_handle, "PUMP1_SWITCHBACK", getPUMP_switchbackifavailable(0)?1:0);  
   nvs_set_i32(nvs_handle, "PUMP2_SWITCHBACK", getPUMP_switchbackifavailable(1)?1:0); 

   for (ch=0;ch<CHANNEL_NUM;ch++)
   {
	 char keyName[32];   //CHx_x 
	 sprintf(keyName,"CH%1.1d_NAME",ch);
	 nvs_set_str(nvs_handle, keyName, channels[ch].Name);   
	 sprintf(keyName,"CH%1.1d_DISABLED",ch);
	 nvs_set_i32(nvs_handle, keyName, channels[ch].channel_disabled);   
    for(i=0;i<PERIODS;i++)
    {
 	
	 sprintf(keyName,"CH%1.1d_ON_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, channels[ch].Chedule_array[i].on_time);
	 sprintf(keyName,"CH%1.1d_OFF_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, channels[ch].Chedule_array[i].off_time);
	 sprintf(keyName,"CH%1.1d_DAY_%1.1d",ch,i);
	 channels[ch].Chedule_array[i].weekdays[7]=0;
	 nvs_set_str(nvs_handle, keyName, channels[ch].Chedule_array[i].weekdays);
	 sprintf(keyName,"CH%1.1d_DUR_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, channels[ch].Chedule_array[i].duration); 
	 sprintf(keyName,"CH%1.1d_VOL_%1.1d",ch,i);
	 nvs_set_i32(nvs_handle, keyName, channels[ch].Chedule_array[i].volume); 
    } 
   }
      
   if(nvs_commit(nvs_handle)!=ESP_OK) ESP_LOGI(TAG, "NVS COMMIT FAILED");; 

    nvs_close(nvs_handle);
	ESP_LOGI(TAG, "NVS STORED");
}



void delete_all_chedules()
{
 int i;
 for(i=0;i<CHANNEL_NUM;i++) memset(channels[i].Chedule_array,0,sizeof(channels[i].Chedule_array));
 Save_data_to_NVS();
}


uint8_t buf[32];

void task_rx(void *p)
{
   int x;
   for(;;) {
      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet(buf, sizeof(buf));
         buf[x] = 0;
         printf("Received: %s\n", buf);
		 ESP_LOGI(TAG, "Received: %s\n", buf);
         lora_receive();
      }
      vTaskDelay(1);
   }
}

 



void PowerMeterTask(void *pvParameters){
    
	TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = (60*1000 / portTICK_PERIOD_MS);

    while(1){

        vTaskDelayUntil( &xLastWakeTime, xFrequency);
		xSemaphoreTake(mutex, portMAX_DELAY);
		  powerconsumptionWs+=    MeasuredValue(ACS71020_address_default, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs)*60.0;
		xSemaphoreGive(mutex); 
  
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}



/*
static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}*/




/*
#define SERVICE_UUID        0x00FF
#define CHAR1_UUID          0xA111
#define CHAR2_UUID          0xA222

static uint16_t conn_handle;
static uint16_t chr1_val_handle;
static uint16_t chr2_val_handle;

static const char *char1_desc = "Char1: R/W & notify";
static const char *char2_desc = "Char2: Read only";*/

/*
static int
gatt_svr_chr_access(const struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid == CHAR1_UUID) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint32_t v = 0xDEADBEEF;
            return os_mbuf_append(ctxt->om, &v, sizeof(v));
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint32_t v;
            os_mbuf_copydata(ctxt->om, 0, sizeof(v), &v);
            //ESP_LOGI(TAG, "Char1 written: 0x%08X", v);
            ble_gatts_notify_custom(conn_handle, ctxt->chr->val_handle, (uint8_t*)&v, sizeof(v));
            return 0;
        }
    } else if (uuid == CHAR2_UUID) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint16_t v = 0x1234;
            return os_mbuf_append(ctxt->om, &v, sizeof(v));
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}


#define BLE_GATT_DSC_UUID_CHAR_USER_DESC 0x2901
#define BLE_GATT_DSC_UUID_CHAR_FORMAT 0x2904
*/
/*
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(CHAR1_UUID),
                .access_cb = gatt_svr_chr_access,
                .val_handle = &chr1_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_UUID_CHAR_USER_DESC),
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .value = (uint8_t*)char1_desc,
                        .len = 0, .max_len = 0,
                    },
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_UUID_CHAR_FORMAT),
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .value = (uint8_t[]){ 0x07, 0, 0,0,0,0,0 },
                        .len = 7, .max_len = 7,
                    },
                    { 0 }
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(CHAR2_UUID),
                .access_cb = gatt_svr_chr_access,
                .val_handle = &chr2_val_handle,
                .flags = BLE_GATT_CHR_F_READ,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_UUID_CHAR_USER_DESC),
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .value = (uint8_t*)char2_desc,
                        .len = 0, .max_len = 0,
                    },
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_UUID_CHAR_FORMAT),
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .value = (uint8_t[]){ 0x06, 0,0,0,0,0,0 },
                        .len = 7, .max_len = 7,
                    },
                    { 0 }
                }
            },
            { 0 }
        }
    },
    { 0 }
};*/

/*
static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, NULL);
    ble_hs_adv_fields adv = {0};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.tx_pwr_lvl_is_present = 1;
    ble_gap_adv_set_fields(&adv);
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      BLE_GAP_ADV_TYPE_ADV_IND, NULL, NULL);
    ESP_LOGI(TAG, "Advertising started");
}*/

static void ble_app_on_reset(int reason) {
    ESP_LOGE(TAG, "Reset; reason=%d", reason);
}





uint8_t ble_addr_type;
void ble_app_advertise(void);

// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
  
    //char * data = (char *)ctxt->om->om_data;
	int intvalue=0;
	char strvalue[64];
	char ltopic[128];
	char ldata[128];
	char* strdata;
	strdata = calloc(1, ctxt->om->om_len+1);
	strncpy(strdata,(char *)ctxt->om->om_data,ctxt->om->om_len);
	strdata[ctxt->om->om_len]=0;
	memset(ltopic,0,sizeof(ltopic));
	memset(ldata,0,sizeof(ldata));
    sscanf(strdata,"%[^=]=%[^\n]",ltopic,ldata);
	to_upper(ltopic,ltopic);
	printf("strdata: %s\n", strdata);
    printf("Topic: %s Data: %s\n", ltopic, ldata);
    //printf("%d\n",strcmp(data, (char *)"LIGHT ON")==0);
    xSemaphoreTake(mutex_mqtt_ble, portMAX_DELAY);

    if (sscanf(strdata, "RUN_MODE=%d",&intvalue)==1)
    {
	   run_mode=intvalue;	
	  /* if (mqtt_connected) 
			{
				esp_mqtt_client_stop(mqtt_client);
				mqtt_connected=0;
			}
	   esp_sntp_stop();
	   esp_wifi_disconnect();
	   Save_data_to_NVS();
	   esp_restart();  */
	   sprintf(MQTT_BLE_answer,"run_mode=%d",run_mode);
    }
	else if (strcmp(ltopic,"SSID")==0)
    {
	   memcpy(wifi_config.sta.ssid, ldata, sizeof(wifi_config.sta.ssid));
	   printf("SSID:%s stored.\n",wifi_config.sta.ssid);
	   sprintf(MQTT_BLE_answer,"SSID:%s stored.\n",wifi_config.sta.ssid);
    }
	else if (strcmp(ltopic,"PWD")==0)
    {
	   printf("Password:%s\n",ldata);
	    memcpy(wifi_config.sta.password, ldata, sizeof(wifi_config.sta.password));
		Write_Msg_toDisplay(1,(char*)wifi_config.sta.ssid);
		Write_Msg_toDisplay(2,(char*)wifi_config.sta.password);
		vTaskDelay(3*1000 / portTICK_PERIOD_MS);
        esp_wifi_disconnect() ;
        esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
		esp_wifi_set_storage(WIFI_STORAGE_FLASH); 
        esp_wifi_connect();
		sprintf(MQTT_BLE_answer,"PWD:%s",wifi_config.sta.password);
    }
    else if (strcmp(strdata, "SAVE_NVS")==0)
    {
        Save_data_to_NVS();
		sprintf(MQTT_BLE_answer,"SAVE_NVS:%s","Done");
    }
	else if (Process_EVENT_DATA(ltopic,ldata, false)) {ble_gatts_notify_custom(conn_handle, ctxt->chr->val_handle, "Done");}
    else{
        printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
		sprintf(MQTT_BLE_answer,"Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    }
    

	xSemaphoreGive(mutex_mqtt_ble);



    free(strdata);
    return 0;
}

// Read data from ESP32 defined as server

static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{	
    os_mbuf_append(ctxt->om, MQTT_BLE_answer, strlen(MQTT_BLE_answer));
    return 0;
}


static int device_read_write(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
 if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
  {
	return (device_read(con_handle, attr_handle, ctxt, arg));
  }
  else if (ctxt->op ==   BLE_GATT_ACCESS_OP_WRITE_CHR)
  {
    return (device_write(con_handle, attr_handle, ctxt, arg));
  }
  else printf("unidentified Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);

  return 0;
}


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
       /*  {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},*/
		 {.uuid = BLE_UUID16_DECLARE(0xFEF5),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |  BLE_GATT_CHR_F_NOTIFY,
          .access_cb = device_read_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
		if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void init_BLE()
{
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set(maintopic); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread

/*
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
    nimble_port_init();
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    nimble_port_freertos_init(NULL);*/
}



void app_main()
{

	//int center, top, bottom;
	//char lineChar[20];

	

	app_desc = esp_app_get_description();
	ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Project name:     %s", app_desc->project_name);
    ESP_LOGI(TAG, "App version:      %s", app_desc->version);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
	
        ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_LOGI(TAG, "nvs_flash_erased");
        err = nvs_flash_init();
    }
	
   if (err!=ESP_OK) ESP_LOGI(TAG, "NVS INIT ERROR %d",err);
	ESP_ERROR_CHECK(err);   
 


	init_pump(0,GPIO_OUTPUT_PUMP_1,ISOLATED_INPUT_PUMP_1,ISOLATED_INPUT_2,true,true); 
	init_channel(0,"CH0",GPIO_OUTPUT_OUT_2,true);
	init_channel(1,"CH1",GPIO_OUTPUT_OUT_3,false);
	init_channel(2,"CH2",GPIO_OUTPUT_OUT_4,true);

//	delete_all_chedules();
    mutex = xSemaphoreCreateMutex();
	mutex_mqtt_ble = xSemaphoreCreateMutex();
	MAIN_TASK_mutex = xSemaphoreCreateMutex();


    if (run_mode & (1<<USE_BLE)) init_BLE();

	Load_data_from_NVS();
    Mount_Filesystem("user_fs");
	
	 gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	
	
	/*
	
	//interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	gpio_set_direction(PRG_BUTTON ,GPIO_MODE_INPUT);
	gpio_set_pull_mode(PRG_BUTTON,GPIO_PULLUP_ONLY);*/

    set_DIO_direction(PRG_BUTTON,GPIO_MODE_INPUT);
    gpio_set_pull_mode(PRG_BUTTON,GPIO_PULLUP_ONLY);

    set_DIO_direction(GPIO_Vext,GPIO_MODE_OUTPUT);
    writeDO(GPIO_Vext, 0);	
	

    init_display();
	if (USE_MCP) Init_DIO(Display._i2c_bus_handle);
    if(readDI(PRG_BUTTON)==0) {run_mode=(1<<USE_BLE);Save_data_to_NVS();esp_restart();}

	if (run_mode & (1<<MEASURE_POWER))  init_ACS71020(Display._i2c_bus_handle,ACS71020_address_default);


	   //Check if Two Point or Vref are burned into eFuse
    //check_efuse();

    //Configure ADC
    /*if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }*/

    //Characterize ADC
    /*TODO
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);*/



    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//

     do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
     do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

	
	if (run_mode & (1<<USE_WIFI)) initialise_wifi();

	if (getfilesize(LOG_FILE)>1E6) remove(LOG_FILE);
	if (getfilesize(IRR_FILE)>0.5E6) remove(IRR_FILE);

    if (run_mode & (1<<MAIN_TASK))          xTaskCreatePinnedToCore(&mainTask, "mainTask", 4096, NULL, 5, NULL, 0);
    if (run_mode & (1<<POWERMETER_TASK))	xTaskCreatePinnedToCore(&PowerMeterTask, "PowerMeterTask", 4096, NULL, 5, NULL, 0);
	


	//append_log(LOG_FILE,"Rebooted. run_mode=%d\n",run_mode); 
	//append_log(IRR_FILE,"append test. run_mode=%d\n",run_mode); 
    //read_log(IRR_FILE);


   
	if (true)
	{
		char buf[1000];
		FILE *ptr_file=fopen(IRR_FILE,"r");
		if (ptr_file!=NULL)
		{
			while (fgets(buf,sizeof(buf)-1, ptr_file)!=NULL) 	
			{
			 ESP_LOGI("FILE","%s",buf);
			}
			fclose(ptr_file);
		}
	}



//	readEeprom(ACS71020_address_default);
//    readShadow(ACS71020_address_default);
	
 /*  lora_init();
   ESP_LOGI(TAG, "Done");
    ESP_LOGI(TAG, "lora_set_frequency");
   lora_set_frequency(433e6);
    ESP_LOGI(TAG, "Done");
   lora_enable_crc();
   xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);*/
	ESP_LOGI("TEST_DI","ISOLATED_INPUT_PUMP_1=%d",readDI(ISOLATED_INPUT_PUMP_1));
	Write_Msg_toDisplay(3,"Done!");
	
	
	if(false)
	{
     ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
     if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
     }
     if (do_calibration1_chan1) {
        example_adc_calibration_deinit(adc1_cali_chan1_handle);
     }
	}



    while(USE_MCP) {
		bool value;
        ESP_LOGI("MCP_TEST","clear bit1");
        writeDO(107,0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        value=readDI(100);
        ESP_LOGI("MCP_TEST","bit1 value: %d",value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);


        ESP_LOGI("MCP_TEST","set bit1");
        writeDO(107,1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        value=readDI(100);
        ESP_LOGI("MCP_TEST","bit1 value: %d",value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }




	if(mqtt_connected) 
	   {	
			  char message[64];  
			  char timeStr[32];           
			  time(&now_life_sent);
			  sprintf(timeStr,"%2.2d:%2.2d:%2.2d",(int)((now_life_sent)/3600),(int)(((now_life_sent)%3600)/60),(int)((now_life_sent)%60)); 
	
			  sprintf(message,"%lld:%s",now_life_sent,timeStr);   
			  my_esp_mqtt_client_publish(mqtt_client, "LIFE", message, 0, 0, 1);   //Qos=0; retain=1
	   }
}
