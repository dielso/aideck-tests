/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck GAP8
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * WiFi image streamer example
 * Refactored and snipped for personal use. 2022, dielsonsoaresojr@gmail.com.
 */

#include "pmsis.h"
#include "cpx.h"
#include "bsp/bsp.h"
#include "bsp/buffer.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"

#define IMG_ORIENTATION 0x0101
#define WIDTH    324
#define HEIGHT   324
#define ASYNC

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

PI_L2 unsigned char *buff;
PI_L2 unsigned char *buff_resize;

static struct pi_device camera;
static pi_buffer_t buffer;

// time ef. measuring
static uint32_t start = 0;
static uint32_t captureTime = 0;
static uint32_t transferTime = 0;
static uint32_t encodingTime = 0;

struct pi_device cluster_dev = {0};
struct pi_cluster_conf cl_conf = {0};

static pi_task_t task1;

static int open_pi_camera_himax(struct pi_device *device)
{
  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  pi_open_from_conf(device, &cam_conf);
  if (pi_camera_open(device))
    return -1;

  // rotate image
  pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
  uint8_t set_value = 3;
  uint8_t reg_value;
  pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
  pi_time_wait_us(1000000);
  pi_camera_reg_get(&camera, IMG_ORIENTATION, &reg_value);
  if (set_value != reg_value)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
    return -1;
  }
  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

  pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

  return 0;
}

static void capture_done_cb(void *arg)
{
  xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

int cam_task(void *parameters)
{
  vTaskDelay(2000);
  cpxPrintToConsole(LOG_TO_CRTP, "Starting camera task...\n");
  uint32_t resolution = WIDTH * HEIGHT;
  buff = (unsigned char *)pmsis_l2_malloc(resolution * sizeof(unsigned char));
  if (buff == NULL)
  {
      cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image \n");
      return;
  }
  buff_resize = (unsigned char *)pmsis_l2_malloc(81 * 81 * sizeof(unsigned char));
  if (buff_resize == NULL)
  {
      cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Resized Image \n");
      return;
  }

  if (open_pi_camera_himax(&camera))
  {
      cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
      return;
  }

  pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, buff);
  pi_buffer_set_format(&buffer, WIDTH, HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

  cpxPrintToConsole(LOG_TO_CRTP, "Buffers initialized and camera opened\n");

  while(1){
      vTaskDelay(20);
      #ifdef ASYNC_CAP
      pi_camera_capture_async(&camera, buff, resolution, pi_task_callback(&task1, capture_done_cb, NULL));
      pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
      cpxPrintToConsole(LOG_TO_CRTP, "Starting async capture\n");
      xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);
      #else
      pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
      cpxPrintToConsole(LOG_TO_CRTP, "Starting sync capture\n");
      pi_camera_capture(&camera, buff, resolution);
      #endif
      pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

      captureTime = xTaskGetTickCount() - start;
      cpxPrintToConsole(LOG_TO_CRTP, "Finished capture w %d ticks\n", captureTime);
  }

  // Stop the camera and immediately close it // shouldnt reach this
  pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
  pi_camera_close(&camera);
  pmsis_exit(0);
}

void start_example(void)
{
  struct pi_device device;

  cpxInit();
  cpxPrintToConsole(LOG_TO_CRTP, "-- Deck System IN --\n");

  evGroup = xEventGroupCreate();

  BaseType_t xTask;

  xTask = xTaskCreate(cam_task, "cam_task", configMINIMAL_STACK_SIZE * 4,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
    pmsis_exit(-1);
  }

  while (1)
  {
    pi_yield();
  }
}

int main(void)
{
    pi_bsp_init();
    // Increase the FC freq to 250 MHz
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    return pmsis_kickoff((void *)start_example);
}