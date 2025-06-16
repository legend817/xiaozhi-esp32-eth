#include "ethernet_board.h"
#include "display.h"
#include "application.h"
#include "system_info.h"
#include "settings.h"
#include "font_awesome_symbols.h"
#include "assets/lang_config.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_eth.h>
#include <esp_eth_mac.h>      // 包含 eth_mac_t 定义
#include <esp_eth_phy.h>  
#include <esp_eth_mac_spi.h>
#include <esp_netif.h>
#include <esp_http.h>
#include <esp_mqtt.h>
#include <esp_udp.h>
#include <esp_log.h>
#include <tcp_transport.h>
#include <tls_transport.h>
#include <web_socket.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "EthernetBoard";

// 定义 GPIO 和 PHY 地址（根据W5500规范调整）
#define MOSI_GPIO 11
#define MISO_GPIO 13
#define SCLK_GPIO 12
#define CS_GPIO   10
#define RST_GPIO  14
#define INT_GPIO  17
#define PHY_ADDR  1  // W5500的PHY地址固定为1
#define SPI_HOST  SPI2_HOST

EthernetBoard::EthernetBoard() {
    // 创建以太网网络接口
    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg = {
        .base = &base_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    eth_netif_ = esp_netif_new(&cfg);
    
    if (!eth_netif_) {
        ESP_LOGE(TAG, "创建以太网接口失败");
    }

    // 停止 DHCP 客户端
    esp_err_t ret = esp_netif_dhcpc_stop(eth_netif_);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止 DHCP 客户端失败: %s", esp_err_to_name(ret));
    }
        // 设置静态 IP 地址
    esp_netif_ip_info_t ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(192, 168, 0, 109) }, // 静态 IP
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 255, 0) }, // 子网掩码
        .gw = { .addr = ESP_IP4TOADDR(192, 168, 0, 1) } // 网关
    };

    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif_, &ip_info));

    ESP_ERROR_CHECK(esp_netif_set_hostname(eth_netif_, "xiaozhi-esp32"));
}

std::string EthernetBoard::GetBoardType() {
    return "ethernet";
}

void EthernetBoard::InitEthernet() {
    if (eth_initialized_) return;
    
    ESP_LOGI(TAG, "初始化以太网 (W5500)");
    
    // 安装 GPIO 中断服务
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOSI_GPIO) | (1ULL << MISO_GPIO) | (1ULL << SCLK_GPIO) | 
                        (1ULL << CS_GPIO) | (1ULL << RST_GPIO) | (1ULL << INT_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1));

    // 初始化 TCP/IP 栈
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 1. 配置 SPI 总线
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_GPIO,
        .miso_io_num = MISO_GPIO,
        .sclk_io_num = SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    
    // 尝试初始化SPI总线（如果已初始化则忽略错误）
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI 总线初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    
    // 2. 配置 SPI 设备 (W5500)
    spi_device_interface_config_t devcfg = {
        .command_bits = 16,  // 必须为16位（地址相位）
        .address_bits = 8,   // 必须为8位（命令相位）
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000, // 20 MHz
        .spics_io_num = CS_GPIO,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle_));
    
    // 3. 配置以太网 MAC
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = INT_GPIO; // 设置中断引脚
    
    // 4. 配置以太网 PHY
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = PHY_ADDR;      // W5500 PHY地址固定为1
    phy_config.reset_gpio_num = RST_GPIO;
    
    // 5. 创建 W5500 MAC 和 PHY 实例
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "创建 W5500 MAC 实例失败");
        return;
    }
    
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    if (!phy) {
        ESP_LOGE(TAG, "创建 W5500 PHY 实例失败");
        mac->del(mac); // 清理已创建的MAC
        return;
    }
    
    // 6. 安装以太网驱动
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_config, &eth_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "以太网驱动安装失败: %s", esp_err_to_name(ret));
        mac->del(mac);
        phy->del(phy);
        return;
    }

    // 设置MAC地址
    // 6. 设置 MAC 地址
    uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
    ret = esp_eth_ioctl(eth_handle_, ETH_CMD_S_MAC_ADDR, mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 MAC 地址失败: %s", esp_err_to_name(ret));
        esp_eth_driver_uninstall(eth_handle_);
        mac->del(mac);
        phy->del(phy);
        spi_bus_remove_device(spi_handle_);
        return;
    }
    
    // 7. 将以太网接口附加到网络接口
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle_);
    ret = esp_netif_attach(eth_netif_, glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "附加网络接口失败: %s", esp_err_to_name(ret));
        esp_eth_driver_uninstall(eth_handle_);
        return;
    }
    
    // 8. 注册以太网事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, 
        [](void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
            auto self = static_cast<EthernetBoard*>(arg);
            
            switch (event_id) {
                case ETHERNET_EVENT_CONNECTED:
                    ESP_LOGI(TAG, "以太网连接成功");
                    self->eth_connected_ = true;
                    break;
                case ETHERNET_EVENT_DISCONNECTED:
                    ESP_LOGW(TAG, "以太网断开连接");
                    self->eth_connected_ = false;
                    break;
                case ETHERNET_EVENT_START:
                    ESP_LOGI(TAG, "以太网启动");
                    break;
                case ETHERNET_EVENT_STOP:
                    ESP_LOGI(TAG, "以太网停止");
                    break;
            }
        }, this));
    
    // 9. 注册IP事件处理
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, 
        [](void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "获取到IP: " IPSTR, IP2STR(&event->ip_info.ip));
        }, this));
    
    // 10. 启动以太网
    ret = esp_eth_start(eth_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动以太网失败: %s", esp_err_to_name(ret));
    } else {
        eth_initialized_ = true;
        ESP_LOGI(TAG, "以太网初始化完成");
        
        // 等待以太网连接
        int retry_count = 0;
        while (!eth_connected_ && retry_count < 20) {
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
        }
        if (!eth_connected_) {
            ESP_LOGW(TAG, "以太网连接超时");
        }
    }
}

bool EthernetBoard::IsEthernetConnected() {
    return eth_connected_;
}

std::string EthernetBoard::GetIpAddress() {
    if (!eth_connected_) return "0.0.0.0";
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(eth_netif_, &ip_info) == ESP_OK) {
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        return std::string(ip_str);
    }
    return "0.0.0.0";
}

void EthernetBoard::StartNetwork() {
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification(Lang::Strings::CONNECTING_ETHERNET, 30000);
    
    // 初始化以太网
    InitEthernet();
    
    // 等待以太网连接和 IP 地址分配
    ESP_LOGI(TAG, "等待以太网连接...");
    const int max_retries = 30; // 30秒超时
    int retry_count = 0;
    bool got_ip = false;
    
    while (retry_count < max_retries) {
        if (IsEthernetConnected()) {
            std::string ip = GetIpAddress();
            if (ip != "0.0.0.0") {
                ESP_LOGI(TAG, "以太网连接成功！IP: %s", ip.c_str());
                std::string notification = Lang::Strings::ETHERNET_CONNECTED;
                notification += ip;
                display->ShowNotification(notification.c_str(), 5000);
                got_ip = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
        ESP_LOGI(TAG, "等待以太网连接和IP分配 %d/%d", retry_count, max_retries);
    }
    
    if (!got_ip) {
        ESP_LOGE(TAG, "以太网连接或IP分配失败");
        display->ShowNotification(Lang::Strings::ETHERNET_CONNECT_FAILED, 5000);
        ESP_LOGE(TAG, "网络连接失败，请检查物理连接");
        return;
    }

    // 等待DNS就绪
    ESP_LOGI(TAG, "等待DNS就绪...");
    retry_count = 0;
    while (retry_count < 10) { // 额外等待10秒确保DNS就绪
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }
    
    // 设置DNS服务器
    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = ESP_IP4TOADDR(8, 8, 8, 8); // 使用Google DNS
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    ESP_ERROR_CHECK(esp_netif_set_dns_info(eth_netif_, ESP_NETIF_DNS_MAIN, &dns_info));
    
    // 设置备用DNS服务器
    dns_info.ip.u_addr.ip4.addr = ESP_IP4TOADDR(114, 114, 114, 114); // 使用114DNS
    ESP_ERROR_CHECK(esp_netif_set_dns_info(eth_netif_, ESP_NETIF_DNS_BACKUP, &dns_info));
    
    ESP_LOGI(TAG, "网络初始化完成");
}

// 以下其他函数保持不变...

// 以下网络客户端创建函数保持不变，因为底层协议栈相同
Http* EthernetBoard::CreateHttp() {
    return new EspHttp();
}

WebSocket* EthernetBoard::CreateWebSocket() {
    Settings settings("websocket", false);
    std::string url = settings.GetString("url");
    if (url.find("wss://") == 0) {
        return new WebSocket(new TlsTransport());
    } else {
        return new WebSocket(new TcpTransport());
    }
    return nullptr;
}

Mqtt* EthernetBoard::CreateMqtt() {
    return new EspMqtt();
}

Udp* EthernetBoard::CreateUdp() {
    return new EspUdp();
}

const char* EthernetBoard::GetNetworkStateIcon() {
    if (IsEthernetConnected()) {
        return FONT_AWESOME_ETHERNET; // 使用以太网图标
    }
    return FONT_AWESOME_ETHERNET_OFF; // 使用断开图标
}

std::string EthernetBoard::GetBoardJson() {
    std::string board_json = std::string("{\"type\":\"ethernet\",");
    board_json += "\"name\":\"" BOARD_NAME "\",";
    
    if (IsEthernetConnected()) {
        board_json += "\"ip\":\"" + GetIpAddress() + "\",";
    }
    
    board_json += "\"mac\":\"" + SystemInfo::GetMacAddress() + "\"}";
    return board_json;
}

void EthernetBoard::SetPowerSaveMode(bool enabled) {
    // 以太网通常没有电源管理模式
    ESP_LOGW(TAG, "Power save mode not supported for Ethernet");
}

std::string EthernetBoard::GetDeviceStatusJson() {
    auto& board = Board::GetInstance();
    auto root = cJSON_CreateObject();

    // ... (其他部分保持不变)

    // Audio speaker
    auto audio_speaker = cJSON_CreateObject();
    auto audio_codec = board.GetAudioCodec();
    if (audio_codec) {
        cJSON_AddNumberToObject(audio_speaker, "volume", audio_codec->output_volume());
    }
    cJSON_AddItemToObject(root, "audio_speaker", audio_speaker);

    // Screen brightness
    auto backlight = board.GetBacklight();
    auto screen = cJSON_CreateObject();
    if (backlight) {
        cJSON_AddNumberToObject(screen, "brightness", backlight->brightness());
    }
    auto display = board.GetDisplay();
    if (display && display->height() > 64) { // For LCD display only
        cJSON_AddStringToObject(screen, "theme", display->GetTheme().c_str());
    }
    cJSON_AddItemToObject(root, "screen", screen);

    // Battery
    int battery_level = 0;
    bool charging = false;
    bool discharging = false;
    if (board.GetBatteryLevel(battery_level, charging, discharging)) {
        cJSON* battery = cJSON_CreateObject();
        cJSON_AddNumberToObject(battery, "level", battery_level);
        cJSON_AddBoolToObject(battery, "charging", charging);
        cJSON_AddItemToObject(root, "battery", battery);
    }

    // 网络部分修改为以太网
    auto network = cJSON_CreateObject();
    cJSON_AddStringToObject(network, "type", "ethernet");
    
    if (IsEthernetConnected()) {
        cJSON_AddStringToObject(network, "ip", GetIpAddress().c_str());
        cJSON_AddStringToObject(network, "status", "connected");
    } else {
        cJSON_AddStringToObject(network, "status", "disconnected");
    }
    
    cJSON_AddItemToObject(root, "network", network);

    // ... (其他部分保持不变)
    // Chip
    float esp32temp = 0.0f;
    if (board.GetTemperature(esp32temp)) {
        auto chip = cJSON_CreateObject();
        cJSON_AddNumberToObject(chip, "temperature", esp32temp);
        cJSON_AddItemToObject(root, "chip", chip);
    }

    auto json_str = cJSON_PrintUnformatted(root);
    std::string json(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    return json;
}