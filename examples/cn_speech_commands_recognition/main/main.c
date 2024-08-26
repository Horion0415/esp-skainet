/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
#include "esp_log.h"
#include "speech_commands_action.h"
#include "model_path.h"

#define LOG_MEMORY_SYSTEM_INFO         (1)
#define LOG_TASK_SYSTEM_INFO           (1)
#define LOG_TIME_INTERVAL_MS           (2000)
#define SYS_TASKS_ELAPSED_TIME_MS      (2000)   // Period of stats measurement

int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t *afe_data = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;

static const char *TAG = "main";

esp_err_t print_real_time_mem_stats(void);

void play_music(void *arg)
{
    while (task_flag) {
        switch (play_voice) {
        case -2:
            vTaskDelay(10);
            break;
        case -1:
            wake_up_action();
            play_voice = -2;
            break;
        default:
            speech_commands_action(play_voice);
            play_voice = -2;
            break;
        }
    }
    vTaskDelete(NULL);
}

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch <= feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        esp_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);

    //print active speech commands
    multinet->print_active_speech_commands(model_data);
    printf("------------detect start------------\n");
    // FILE *fp = fopen("/sdcard/out1", "w");
    // if (fp == NULL) printf("can not open file\n");
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }
#if CONFIG_IDF_TARGET_ESP32
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("wakeword detected\n");
            play_voice = -1;
            detect_flag = 1;
            afe_handle->disable_wakenet(afe_data);
            printf("-----------listening-----------\n");
        }
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
	    multinet->clean(model_data);  // clean all status of multinet
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            play_voice = -1;
            detect_flag = 1;
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
        }
#endif

        if (detect_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f\n", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                printf("\n-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void app_main()
{
    models = esp_srmodel_init("model");
    ESP_ERROR_CHECK(esp_board_init(16000, 2, 32));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));


    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);;
#if CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_P4_FUNCTION_EV_BOARD
    afe_config.pcm_config.total_ch_num = 3;
    afe_config.pcm_config.mic_num = 2;
    afe_config.pcm_config.ref_num = 1;
    afe_config.wakenet_mode = DET_MODE_2CH_90;
#endif
    afe_data = afe_handle->create_from_config(&afe_config);

    task_flag = 1;
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);

// #if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD || defined CONFIG_ESP32_KORVO_V1_1_BOARD
//     xTaskCreatePinnedToCore(&led_Task, "led", 3 * 1024, NULL, 5, NULL, 0);
// #endif
// #if defined  CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD || CONFIG_ESP32_S3_KORVO_2_V3_0_BOARD || CONFIG_ESP32_KORVO_V1_1_BOARD
//     xTaskCreatePinnedToCore(&play_music, "play", 2 * 1024, NULL, 5, NULL, 1);
// #endif

    // You can call afe_handle->destroy to destroy AFE.
//    vTaskDelay(2000 / portTICK_PERIOD_MS);
//    task_flag = 0;
//
//    printf("destroy\n");
//    afe_handle->destroy(afe_data);
//    afe_data = NULL;
//    printf("successful\n");

#if LOG_MEMORY_SYSTEM_INFO
    static char buffer[2048];
    while (1) {
        sprintf(buffer, "\t  Biggest /     Free /    Total\n"
                " SRAM : [%8d / %8d / %8d]\n"
                "PSRAM : [%8d / %8d / %8d]\n",
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        printf("------------ Memory ------------\n");
        printf("%s\n", buffer);

        ESP_ERROR_CHECK(print_real_time_mem_stats());
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(LOG_TIME_INTERVAL_MS));
    }
#endif

}

#if LOG_TASK_SYSTEM_INFO
#define ARRAY_SIZE_OFFSET                   8   // Increase this if audio_sys_get_real_time_stats returns ESP_ERR_INVALID_SIZE

#define audio_malloc    malloc
#define audio_calloc    calloc
#define audio_free      free
#define AUDIO_MEM_CHECK(tag, x, action) if (x == NULL) { \
        ESP_LOGE(tag, "Memory exhausted (%s:%d)", __FILE__, __LINE__); \
        action; \
    }

const char *task_state[] = {
    "Running",
    "Ready",
    "Blocked",
    "Suspended",
    "Deleted"
};

/** @brief
 * "Extr": Allocated task stack from psram, "Intr": Allocated task stack from internel
 */
const char *task_stack[] = {"Extr", "Intr"};

esp_err_t print_real_time_mem_stats(void)
{
// #if (CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
//     TaskStatus_t *start_array = NULL, *end_array = NULL;
//     UBaseType_t start_array_size, end_array_size;
//     uint32_t start_run_time, end_run_time;
//     uint32_t total_elapsed_time;
//     uint32_t task_elapsed_time, percentage_time;
//     esp_err_t ret;

//     // Allocate array to store current task states
//     start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
//     start_array = (TaskStatus_t *)audio_malloc(sizeof(TaskStatus_t) * start_array_size);
//     AUDIO_MEM_CHECK(TAG, start_array, {
//         ret = ESP_FAIL;
//         goto exit;
//     });
//     // Get current task states
//     start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
//     if (start_array_size == 0) {
//         ESP_LOGE(TAG, "Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET");
//         ret = ESP_FAIL;
//         goto exit;
//     }

//     vTaskDelay(pdMS_TO_TICKS(SYS_TASKS_ELAPSED_TIME_MS));

//     // Allocate array to store tasks states post delay
//     end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
//     end_array = (TaskStatus_t *)audio_malloc(sizeof(TaskStatus_t) * end_array_size);
//     AUDIO_MEM_CHECK(TAG, start_array, {
//         ret = ESP_FAIL;
//         goto exit;
//     });

//     // Get post delay task states
//     end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
//     if (end_array_size == 0) {
//         ESP_LOGE(TAG, "Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET");
//         ret = ESP_FAIL;
//         goto exit;
//     }

//     // Calculate total_elapsed_time in units of run time stats clock period.
//     total_elapsed_time = (end_run_time - start_run_time);
//     if (total_elapsed_time == 0) {
//         ESP_LOGE(TAG, "Delay duration too short. Trying increasing SYS_TASKS_ELAPSED_TIME_MS");
//         ret = ESP_FAIL;
//         goto exit;
//     }

//     ESP_LOGI(TAG, "| Task              | Run Time    | Per | Prio | HWM       | State   | CoreId   | Stack ");

//     // Match each task in start_array to those in the end_array
//     for (int i = 0; i < start_array_size; i++) {
//         for (int j = 0; j < end_array_size; j++) {
//             if (start_array[i].xHandle == end_array[j].xHandle) {

//                 task_elapsed_time = end_array[j].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
//                 percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
//                 ESP_LOGI(TAG, "| %-17s | %-11d |%2d%%  | %-4u | %-9u | %-7s | %-8x | %s",
//                                 start_array[i].pcTaskName, (int)task_elapsed_time, (int)percentage_time, start_array[i].uxCurrentPriority,
//                                 (int)start_array[i].usStackHighWaterMark, task_state[(start_array[i].eCurrentState)],
//                                 start_array[i].xCoreID, 0);

//                 // Mark that task have been matched by overwriting their handles
//                 start_array[i].xHandle = NULL;
//                 end_array[j].xHandle = NULL;
//                 break;
//             }
//         }
//     }

//     // Print unmatched tasks
//     for (int i = 0; i < start_array_size; i++) {
//         if (start_array[i].xHandle != NULL) {
//             ESP_LOGI(TAG, "| %s | Deleted", start_array[i].pcTaskName);
//         }
//     }
//     for (int i = 0; i < end_array_size; i++) {
//         if (end_array[i].xHandle != NULL) {
//             ESP_LOGI(TAG, "| %s | Created", end_array[i].pcTaskName);
//         }
//     }
//     printf("\n");
//     ret = ESP_OK;

// exit:    // Common return path
//     if (start_array) {
//         audio_free(start_array);
//         start_array = NULL;
//     }
//     if (end_array) {
//         audio_free(end_array);
//         end_array = NULL;
//     }
//     return ret;
// #else
//     ESP_LOGW(TAG, "Please enbale `CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID` and `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` in menuconfig");
//     return ESP_FAIL;
// #endif
    return ESP_OK;
}
#endif
