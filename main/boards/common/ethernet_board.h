#ifndef ETHERNET_BOARD_H
#define ETHERNET_BOARD_H

#include "board.h"
#include <string>

// 替换 WiFi 相关头文件为以太网
#include <driver/spi_master.h>
#include <esp_netif.h>
#include <esp_eth.h>

class EthernetBoard : public Board {
public:
    EthernetBoard();
    std::string GetBoardType() override;
    
    void StartNetwork() override;
    Http* CreateHttp() override;
    WebSocket* CreateWebSocket() override;
    Mqtt* CreateMqtt() override;
    Udp* CreateUdp() override;
    
    const char* GetNetworkStateIcon() override;
    std::string GetBoardJson() override;
    void SetPowerSaveMode(bool enabled) override;
    std::string GetDeviceStatusJson() override;
    
    // 移除 WiFi 配置相关功能
    void EnterWifiConfigMode() = delete;
    void ResetWifiConfiguration() = delete;

private:
    void InitEthernet();
    bool IsEthernetConnected();
    std::string GetIpAddress();
    
    esp_eth_handle_t eth_handle_;
    esp_netif_t *eth_netif_;
    bool eth_connected_ = false;
    bool eth_initialized_ = false;
    spi_device_handle_t spi_handle_;
    
};

#endif // WIFI_BOARD_H