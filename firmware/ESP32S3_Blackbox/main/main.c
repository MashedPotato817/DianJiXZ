#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "blackbox_config.h"

static const char *TAG = "edc_blackbox";

static SemaphoreHandle_t g_data_mutex;
static SemaphoreHandle_t g_log_mutex;
static char g_latest_line[BLACKBOX_LINE_MAX];
static uint32_t g_line_count;
static uint32_t g_line_sequence;
static uint64_t g_last_line_ms;
static size_t g_log_bytes;

static const char INDEX_HTML[] =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>EDC2026 Blackbox</title>"
"<style>"
"body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:0;background:#101418;color:#eef3f8}"
"main{max-width:920px;margin:0 auto;padding:20px}"
"h1{font-size:22px;margin:0 0 12px}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin:14px 0}"
".card{background:#18212b;border:1px solid #2b3a47;border-radius:8px;padding:12px}"
".label{color:#8fa3b5;font-size:12px}.value{font-size:24px;margin-top:4px}"
"pre{white-space:pre-wrap;word-break:break-word;background:#0b0f14;border:1px solid #2b3a47;border-radius:8px;padding:12px;min-height:120px}"
"button,a.button{display:inline-block;background:#3b82f6;color:white;border:0;border-radius:6px;padding:9px 12px;text-decoration:none;margin-right:8px;cursor:pointer}"
"</style></head><body><main>"
"<h1>EDC2026 Car Blackbox</h1>"
"<div class='grid'>"
"<div class='card'><div class='label'>Lines</div><div id='lines' class='value'>0</div></div>"
"<div class='card'><div class='label'>Log bytes</div><div id='bytes' class='value'>0</div></div>"
"<div class='card'><div class='label'>Last age ms</div><div id='age' class='value'>-</div></div>"
"</div>"
"<pre id='latest'>waiting telemetry...</pre>"
"<a class='button' href='/download'>Download log</a>"
"<button onclick='clearLog()'>Clear log</button>"
"<script>"
"async function tick(){"
" const r=await fetch('/api/latest');"
" const line=await r.text();"
" document.getElementById('latest').textContent=line||'waiting telemetry...';"
" const s=await (await fetch('/api/stats')).json();"
" document.getElementById('lines').textContent=s.lines;"
" document.getElementById('bytes').textContent=s.bytes;"
" document.getElementById('age').textContent=s.age_ms;"
"}"
"async function clearLog(){await fetch('/clear');tick();}"
"setInterval(tick,500);tick();"
"</script></main></body></html>";

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void update_log_size(void)
{
    struct stat st;
    if (stat(BLACKBOX_LOG_PATH, &st) == 0) {
        g_log_bytes = (size_t)st.st_size;
    } else {
        g_log_bytes = 0;
    }
}

static void rotate_log_if_needed(void)
{
    if (g_log_bytes < BLACKBOX_LOG_MAX_BYTES) {
        return;
    }

    unlink(BLACKBOX_LOG_PATH);
    g_log_bytes = 0;
    FILE *f = fopen(BLACKBOX_LOG_PATH, "a");
    if (f) {
        fprintf(f, "# log rotated at %llu ms\n", (unsigned long long)now_ms());
        fclose(f);
        update_log_size();
    }
}

static void append_log_line(const char *line)
{
    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    rotate_log_if_needed();

    FILE *f = fopen(BLACKBOX_LOG_PATH, "a");
    if (f) {
        fprintf(f, "%llu,%s\n", (unsigned long long)now_ms(), line);
        fclose(f);
        update_log_size();
    }

    xSemaphoreGive(g_log_mutex);
}

static void publish_line(const char *line)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strlcpy(g_latest_line, line, sizeof(g_latest_line));
        g_line_count++;
        g_line_sequence++;
        g_last_line_ms = now_ms();
        xSemaphoreGive(g_data_mutex);
    }

    append_log_line(line);
}

static void uart_rx_task(void *arg)
{
    uint8_t byte;
    char line[BLACKBOX_LINE_MAX];
    size_t pos = 0;

    while (1) {
        int len = uart_read_bytes(BLACKBOX_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            if (pos > 0) {
                line[pos] = '\0';
                publish_line(line);
                pos = 0;
            }
            continue;
        }

        if (pos < (sizeof(line) - 1U)) {
            line[pos++] = (char)byte;
        } else {
            line[sizeof(line) - 1U] = '\0';
            publish_line(line);
            pos = 0;
        }
    }
}

static void udp_broadcast_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGW(TAG, "UDP socket create failed");
        vTaskDelete(NULL);
        return;
    }

    int enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(BLACKBOX_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    uint32_t last_seq = 0;
    char line[BLACKBOX_LINE_MAX];

    while (1) {
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (g_line_sequence != last_seq) {
                last_seq = g_line_sequence;
                strlcpy(line, g_latest_line, sizeof(line));
                xSemaphoreGive(g_data_mutex);
                sendto(sock, line, strlen(line), 0, (struct sockaddr *)&dest, sizeof(dest));
            } else {
                xSemaphoreGive(g_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t latest_handler(httpd_req_t *req)
{
    char line[BLACKBOX_LINE_MAX];
    line[0] = '\0';

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strlcpy(line, g_latest_line, sizeof(line));
        xSemaphoreGive(g_data_mutex);
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, line, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stats_handler(httpd_req_t *req)
{
    uint32_t lines = 0;
    size_t bytes = 0;
    uint64_t age = 0;
    char response[128];

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lines = g_line_count;
        age = g_last_line_ms ? (now_ms() - g_last_line_ms) : 0;
        xSemaphoreGive(g_data_mutex);
    }

    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bytes = g_log_bytes;
        xSemaphoreGive(g_log_mutex);
    }

    snprintf(response, sizeof(response),
             "{\"lines\":%lu,\"bytes\":%u,\"age_ms\":%llu}",
             (unsigned long)lines,
             (unsigned int)bytes,
             (unsigned long long)age);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t download_handler(httpd_req_t *req)
{
    char buf[256];
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=telemetry.log");

    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "log busy");
    }

    FILE *f = fopen(BLACKBOX_LOG_PATH, "r");
    if (!f) {
        xSemaphoreGive(g_log_mutex);
        return httpd_resp_send(req, "", 0);
    }

    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            fclose(f);
            xSemaphoreGive(g_log_mutex);
            return ESP_FAIL;
        }
    }

    fclose(f);
    xSemaphoreGive(g_log_mutex);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t clear_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        unlink(BLACKBOX_LOG_PATH);
        update_log_size();
        xSemaphoreGive(g_log_mutex);
    }

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "OK");
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    const httpd_uri_t latest = {.uri = "/api/latest", .method = HTTP_GET, .handler = latest_handler};
    const httpd_uri_t stats = {.uri = "/api/stats", .method = HTTP_GET, .handler = stats_handler};
    const httpd_uri_t download = {.uri = "/download", .method = HTTP_GET, .handler = download_handler};
    const httpd_uri_t clear = {.uri = "/clear", .method = HTTP_GET, .handler = clear_handler};

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &latest));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &stats));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &download));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &clear));
}

static void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BLACKBOX_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BLACKBOX_UART_PORT, BLACKBOX_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BLACKBOX_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BLACKBOX_UART_PORT,
                                 BLACKBOX_UART_TX_PIN,
                                 BLACKBOX_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static void init_spiffs(void)
{
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 4,
        .format_if_mount_failed = true,
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    update_log_size();
}

static void init_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = BLACKBOX_WIFI_SSID,
            .ssid_len = strlen(BLACKBOX_WIFI_SSID),
            .channel = BLACKBOX_WIFI_CHANNEL,
            .password = BLACKBOX_WIFI_PASSWORD,
            .max_connection = BLACKBOX_WIFI_MAX_STA,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(BLACKBOX_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: %s, open http://192.168.4.1/", BLACKBOX_WIFI_SSID);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_data_mutex = xSemaphoreCreateMutex();
    g_log_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_data_mutex == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(g_log_mutex == NULL ? ESP_FAIL : ESP_OK);

    strlcpy(g_latest_line, "", sizeof(g_latest_line));

    init_spiffs();
    init_uart();
    init_wifi_ap();
    start_webserver();

    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    xTaskCreate(udp_broadcast_task, "udp_broadcast", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Blackbox ready. UART%d RX=%d TX=%d baud=%d",
             BLACKBOX_UART_PORT,
             BLACKBOX_UART_RX_PIN,
             BLACKBOX_UART_TX_PIN,
             BLACKBOX_UART_BAUD);
}
