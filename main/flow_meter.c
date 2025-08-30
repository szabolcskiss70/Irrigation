#include "flow_meter.h"
#include "esp_log.h"

static const char *TAG = "FLOW_METER";

void install_pcnt(int id,pcnt_unit_handle_t *pcnt_unit,int GPIO_CNT)
{
	ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
		.flags.accum_count = true,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit,&filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = GPIO_CNT,
		.level_gpio_num=-1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a));
    

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_HOLD));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
  

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, watch_points[i]));
    }
    /*pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));*/

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));
}

