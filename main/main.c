/*
 * M5Stack Tab5 — UVC host: FireBeetle MJPEG 320x240@15fps → display
 *
 * FireBeetle ESP32-P4 streams MJPEG 320x240 @15fps, VID=0x303A PID=0x8000.
 * Tab5 BSP portrait: H_RES=720 (width), V_RES=1280 (height).
 *
 * sdkconfig.defaults must have:
 *   # CONFIG_UVC_CHECK_PAYLOAD_HEADER_EOH is not set
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_ops.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#include "jpeg_decoder.h"   // ESP32-P4 hardware JPEG decoder

static const char *TAG = "tab5_uvc";

// ---- FireBeetle camera ----
#define CAM_W    320
#define CAM_H    240
#define CAM_FPS   15
#define JPEG_BUF  (64 * 1024)   // 64KB — safe ceiling for 320x240 MJPEG

// ---- Display ----
// BSP portrait: H_RES=720 (x-axis), V_RES=1280 (y-axis)
// BUT: BSP has swap_xy active, so drawing (x, y, x+w, y+h) renders
// with width along y-axis and height along x-axis.
// To get a 320-wide × 240-tall rectangle appearing landscape:
//   pass draw region as 240 in x-dimension, 320 in y-dimension.
// Effective center: x=(720-240)/2=240  y=(1280-320)/2=480
#define DISP_W   BSP_LCD_H_RES   // 720
#define DISP_H   BSP_LCD_V_RES   // 1280
#define DRAW_X   ((DISP_W - CAM_H) / 2)   // (720-240)/2 = 240
#define DRAW_Y   ((DISP_H - CAM_W) / 2)   // (1280-320)/2 = 480

// ---- USB ----
// FireBeetle connects at FS (MPS=1023). 64 URBs × 1023 = 65KB buffer.
#define NUM_URBS   16
#define URB_SZ     1023 * 16

#define USB_HOST_PRIO   15
#define UVC_DRV_PRIO    (USB_HOST_PRIO + 1)
#define FRAME_PROC_PRIO 5

static QueueHandle_t         frame_q  = NULL;
static uvc_host_stream_hdl_t g_stream = NULL;
static bool                  g_running = false;

// -------------------------------------------------------------------
// UVC frame callback — just queue the JPEG frame
// -------------------------------------------------------------------
static bool frame_cb(const uvc_host_frame_t *frame, void *ctx)
{
    static uint32_t n = 0;
    n++;
    if (n <= 5 || n % 30 == 0) {
        ESP_LOGI(TAG, "frame_cb #%lu: %d B", (unsigned long)n, (int)frame->data_len);
    }
    if (xQueueSendToBack(frame_q, &frame, 0) != pdPASS) {
        ESP_LOGW(TAG, "queue full — drop");
        return true;
    }
    return false;
}

// -------------------------------------------------------------------
// Stream event callback
// -------------------------------------------------------------------
static void stream_event_cb(const uvc_host_stream_event_data_t *event, void *ctx)
{
    switch (event->type) {
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Camera disconnected");
        g_running = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        break;
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGD(TAG, "Transfer error %d", event->transfer_error.error);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame overflow — increase JPEG_BUF");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame underflow");
        break;
    default:
        break;
    }
}

// -------------------------------------------------------------------
// Frame processing task — JPEG decode → draw_bitmap
// -------------------------------------------------------------------
static void frame_proc_task(void *arg)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)arg;

    // RGB565 output buffer: 320×240×2 = 153600 bytes
    uint16_t *rgb = heap_caps_malloc(CAM_W * CAM_H * 2, MALLOC_CAP_SPIRAM);
    assert(rgb);

    uint32_t count = 0;
    int64_t  t_fps = 0;

    ESP_LOGI(TAG, "frame_proc ready. DRAW=(%d,%d)+(%d,%d)",
             DRAW_X, DRAW_Y, CAM_H, CAM_W);

    while (1) {
        uvc_host_frame_t *frame;
        if (xQueueReceive(frame_q, &frame, pdMS_TO_TICKS(3000)) != pdPASS) {
            ESP_LOGW(TAG, "No frame for 3s (drawn=%lu)", (unsigned long)count);
            continue;
        }

        count++;
        if (count == 1) {
            ESP_LOGI(TAG, "FIRST FRAME: %d bytes", (int)frame->data_len);
        }

        // Hardware JPEG decode → RGB565
        esp_jpeg_image_cfg_t cfg = {
            .indata       = frame->data,
            .indata_size  = frame->data_len,
            .outbuf       = (uint8_t *)rgb,
            .outbuf_size  = CAM_W * CAM_H * 2,
            .out_format   = JPEG_IMAGE_FORMAT_RGB565,
            .flags        = { .swap_color_bytes = 1 },  // RGB565 byte swap for LCD
        };
        esp_jpeg_image_output_t out = {0};
        esp_err_t ret = esp_jpeg_decode(&cfg, &out);

        if (ret == ESP_OK) {
            // swap_xy is active in BSP: pass (CAM_H wide, CAM_W tall)
            // so the image renders landscape on the portrait display
            esp_lcd_panel_draw_bitmap(panel,
                DRAW_X, DRAW_Y,
                DRAW_X + CAM_H,   // 240
                DRAW_Y + CAM_W,   // 320
                rgb);
        } else {
            ESP_LOGW(TAG, "JPEG decode failed: %d (frame #%lu, %d B)",
                     ret, (unsigned long)count, (int)frame->data_len);
        }

        uvc_host_frame_return(g_stream, frame);

        if (count % 30 == 0) {
            int64_t now = esp_timer_get_time();
            float fps = (t_fps == 0) ? 0.0f
                      : (30.0f * 1e6f / (float)(now - t_fps));
            t_fps = now;
            ESP_LOGI(TAG, "drawn=%lu  fps=%.1f  last=%dB",
                     (unsigned long)count, fps, (int)frame->data_len);
        }
    }
}

// -------------------------------------------------------------------
// USB library task
// -------------------------------------------------------------------
static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }
}

// -------------------------------------------------------------------
// app_main
// -------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Tab5 UVC host — FireBeetle MJPEG 320x240@15fps");

    // ---- Display ----
    esp_lcd_panel_handle_t    panel = NULL;
    esp_lcd_panel_io_handle_t io    = NULL;
    bsp_display_config_t disp_cfg = {
        .dsi_bus = {
            .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
            .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
        }
    };
    ESP_ERROR_CHECK(bsp_display_new(&disp_cfg, &panel, &io));
    bsp_display_brightness_set(80);

    // Black fill + red box at expected video location
    {
        size_t sz = (size_t)DISP_W * DISP_H * 2;
        uint16_t *fb = heap_caps_calloc(1, sz, MALLOC_CAP_SPIRAM);
        if (fb) {
            // Red box: DRAW_X..DRAW_X+CAM_H wide, DRAW_Y..DRAW_Y+CAM_W tall
            for (int y = DRAW_Y; y < DRAW_Y + CAM_W; y++)
                for (int x = DRAW_X; x < DRAW_X + CAM_H; x++)
                    fb[y * DISP_W + x] = 0xF800;
            esp_lcd_panel_draw_bitmap(panel, 0, 0, DISP_W, DISP_H, fb);
            free(fb);
        }
    }
    ESP_LOGI(TAG, "Display %dx%d. Red box at (%d,%d)+(%d,%d)",
             DISP_W, DISP_H, DRAW_X, DRAW_Y, CAM_H, CAM_W);

    // ---- USB-A power ----
    ESP_LOGI(TAG, "USB-A VBUS on...");
    ESP_ERROR_CHECK(bsp_feature_enable(BSP_FEATURE_USB, true));
    vTaskDelay(pdMS_TO_TICKS(500));

    // ---- Resources ----
    frame_q = xQueueCreate(4, sizeof(uvc_host_frame_t *));
    assert(frame_q);

    // ---- USB host ----
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, USB_HOST_PRIO, NULL);

    // ---- UVC driver ----
    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority   = UVC_DRV_PRIO,
        .xCoreID                = tskNO_AFFINITY,
        .create_background_task = true,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_cfg));

    // ---- Frame task ----
    xTaskCreate(frame_proc_task, "frame_proc", 8 * 1024, (void *)panel,
                FRAME_PROC_PRIO, NULL);

    // ---- Stream config ----
    // FireBeetle: VID=0x303A, PID=0x8000, MJPEG 320x240 @15fps FS (MPS=1023)
    const uvc_host_stream_config_t stream_cfg = {
        .event_cb  = stream_event_cb,
        .frame_cb  = frame_cb,
        .user_ctx  = NULL,
        .usb = {
            .vid              = 0x303A,
            .pid              = 0x8000,
            .uvc_stream_index = 0,
        },
        .vs_format = {
            .h_res  = CAM_W,   // 320
            .v_res  = CAM_H,   // 240
            .fps    = CAM_FPS, // 15
            .format = UVC_VS_FORMAT_MJPEG,
        },
        .advanced = {
            .number_of_frame_buffers = 3,
            .frame_size              = JPEG_BUF,   // 64KB — safe for any 320x240 JPEG
            .number_of_urbs          = NUM_URBS,   // 64 × 1023 = 65KB buffer
            .urb_size                = URB_SZ,     // 1023 — FS ISOC MPS
            .frame_heap_caps         = MALLOC_CAP_SPIRAM,
        },
    };

    while (1) {
        ESP_LOGI(TAG, "Waiting for FireBeetle (0x303A:0x8000)...");
        esp_err_t err = uvc_host_stream_open(&stream_cfg,
                                              pdMS_TO_TICKS(10000), &g_stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Open failed 0x%x — retry in 2s", err);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        g_running = true;
        ESP_LOGI(TAG, "Opened MJPEG %dx%d @%dfps. Starting...",
                 CAM_W, CAM_H, CAM_FPS);
        esp_err_t start_err = uvc_host_stream_start(g_stream);
        if (start_err != ESP_OK) {
            ESP_LOGE(TAG, "stream_start failed 0x%x", start_err);
            uvc_host_stream_close(g_stream);
            g_running = false;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "Stream running. Watch for 'frame_cb #1'...");

        while (g_running)
            vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGW(TAG, "Disconnected — re-searching");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}