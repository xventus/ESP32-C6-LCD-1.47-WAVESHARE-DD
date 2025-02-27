//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   sd_card.h
/// @author Petr Vanek

#pragma once

#include <iostream>
#include <dirent.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include "hardware.h"
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdspi_host.h>
#include <freertos/semphr.h>
#include "spi_manager.h"

class SdCard
{
private:
    static constexpr const char *TAG = "SdCard";
    std::string mountPoint_;
    sdmmc_card_t *sdCard_;
    int mosi_, miso_, clk_, cs_;

public:
    SdCard(const std::string &mountPoint, int mosi, int miso, int clk, int cs)
        : mountPoint_(mountPoint), sdCard_(nullptr), mosi_(mosi), miso_(miso), clk_(clk), cs_(cs)
    {
    }

    ~SdCard()
    {
        unmount();
    }

    esp_err_t initBus(bool needInit)
    {
        esp_err_t ret = ESP_OK;
        if (needInit) {
          

            const spi_bus_config_t busCfg = {
                .mosi_io_num = mosi_,
                .miso_io_num = miso_,
                .sclk_io_num = clk_,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .max_transfer_sz = 4000};

            SPILockGuard lock;

            ret = spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to initialize SPI bus.");
                return ret;
            }
        }

        return ret;
    }

    esp_err_t mount(bool failOnFormat)
    {
        esp_err_t ret;
        SPILockGuard lock;
        sdspi_device_config_t slotConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
        slotConfig.gpio_cs = static_cast<gpio_num_t>(cs_);
        slotConfig.host_id = SPI2_HOST;

        const esp_vfs_fat_mount_config_t mountConfig = {
            .format_if_mount_failed = failOnFormat,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
            .disk_status_check_enable = false,
            .use_one_fat = false};
            

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();

        ret = esp_vfs_fat_sdspi_mount(mountPoint_.c_str(), &host, &slotConfig, &mountConfig, &sdCard_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
            spi_bus_free(SPI2_HOST);
        }

        return ret;
    }

    esp_err_t unmount()
    {
        if (sdCard_)
        {
            SPILockGuard lock;
            esp_err_t err = esp_vfs_fat_sdcard_unmount(mountPoint_.c_str(), sdCard_);
            sdCard_ = nullptr;
            return err;
        }
        return ESP_OK;
    }

    std::vector<std::string> listDirectory(const std::string &path) const
    {
        std::vector<std::string> files;

        SPILockGuard lock;

        DIR *dir = opendir((mountPoint_ + path).c_str());
        if (dir)
        {
            struct dirent *entry;
            while ((entry = readdir(dir)) != nullptr)
            {
                files.emplace_back(entry->d_name);
            }
            closedir(dir);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to open directory: %s", path.c_str());
        }
        return files;
    }

    std::string readFile(const std::string &path) const
    {
        std::string content;
        
        SPILockGuard lock;

        FILE *file = fopen((mountPoint_ + path).c_str(), "r");
        if (file)
        {
            char buffer[1024];
            while (fgets(buffer, sizeof(buffer), file))
            {
                content += buffer;
            }
            fclose(file);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to open file for reading: %s", path.c_str());
        }
        return content;
    }

    bool writeFile(const std::string &path, const std::string &data) const
    {
        SPILockGuard lock;

        FILE *file = fopen((mountPoint_ + path).c_str(), "w");
        if (file)
        {
            fwrite(data.c_str(), 1, data.size(), file);
            fclose(file);
            return true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to open file for writing: %s", path.c_str());
            return false;
        }
    }

    bool deleteFile(const std::string &path) const
    {
        SPILockGuard lock;

        std::string fullPath = mountPoint_ + path;
        if (remove(fullPath.c_str()) == 0)
        {
            ESP_LOGI(TAG, "File deleted: %s", fullPath.c_str());
            return true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to delete file: %s", fullPath.c_str());
            return false;
        }
    }
};