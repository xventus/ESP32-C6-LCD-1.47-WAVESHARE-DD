//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   spi_manager.h
/// @author Petr Vanek

#pragma once

#include <freertos/semphr.h>
#include <esp_log.h>

/**
 * @brief Singleton class to manage SPI bus access synchronization.
 */
class SpiManager {
private:
    static constexpr const char *TAG = "SpiManager"; ///< Logging tag.
    SemaphoreHandle_t _mutex;                        ///< Mutex for SPI synchronization.

    /**
     * @brief Private constructor for singleton.
     */
    SpiManager() {
        ESP_LOGI(TAG, "SpiManager ctor");
        _mutex = xSemaphoreCreateMutex();
        if (!_mutex) {
            ESP_LOGE(TAG, "Failed to create SPI mutex");
        }
    }

public:
    /**
     * @brief Destructor to clean up resources.
     */
    ~SpiManager() {
        if (_mutex) {
            vSemaphoreDelete(_mutex);
        }
    }

    // Delete copy constructor and assignment operator.
    SpiManager(const SpiManager &) = delete;
    SpiManager &operator=(const SpiManager &) = delete;

    /**
     * @brief Get the singleton instance of SpiManager.
     * @return Pointer to the SpiManager instance.
     */
    static SpiManager *getInstance() {
        static SpiManager instance;  
        return &instance;
    }

    /**
     * @brief Overloaded operator to access singleton methods directly.
     * @return Pointer to the SpiManager instance.
     */
    SpiManager *operator->() { return this; }

    /**
     * @brief Lock the SPI bus to ensure exclusive access.
     */
    void lock() {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to lock SPI mutex");
        }
    }

    /**
     * @brief Unlock the SPI bus after completing operations.
     */
    void unlock() {
        xSemaphoreGive(_mutex);
    }
};

/**
 * @brief Guard class to automatically lock and unlock SPI access.
 */
class SPILockGuard {
public:
    /**
     * @brief Locks SPI on creation.
     */
    explicit SPILockGuard() {
        //ESP_LOGI("SPILockGuard", "-----> SPI LOCK");
        SpiManager::getInstance()->lock();
    }

    /**
     * @brief Unlocks SPI on destruction.
     */
    ~SPILockGuard() {
        //ESP_LOGI("SPILockGuard", "<----- SPI UNLOCK");
        SpiManager::getInstance()->unlock();
    }
};
