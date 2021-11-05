/*
 * @Author       : yfwang
 * @Date         : 2021-08-25 16:59:43
 * @LastEditTime : 2021-11-05 15:25:50
 * @LastEditors  : yfwang
 * @FilePath     : \powergw\app\src\main.c
 */

#include "main.h"

global_t global;

/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void *arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
#if NRF_LOG_ENABLED
    ret_code_t err_code = NRF_LOG_INIT(xTaskGetTickCount, configTICK_RATE_HZ);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &global.task_handle_log))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    /*Startup sysview */
    SEGGER_SYSVIEW_Conf();
#endif
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook(void)
{
#if NRF_LOG_ENABLED
    vTaskResume(global.task_handle_log);
#endif
}

void vApplicationMallocFailedHook(void)
{
    NRF_LOG_INFO("vApplicationMallocFailedHook");
}

void dfu_buttonless_init(void)
{
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
}
/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

static void gpiote_init(void)
{
    ret_code_t err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

static void sleep_mode_init(void)
{
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

void wdt_event_handler(void)
{
}

void wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;

    /* 一个周期内 */
    config.reload_value = 5000;
    err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&global.m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void wdt_feed(void)
{
    nrf_drv_wdt_channel_feed(global.m_channel_id);
}

void task_watch_dog(void *arg)
{
    uint32_t current_tick = xTaskGetTickCount();
    int32_t next_delay_tick = usticks(1000 * 1000);
    nrf_gpio_cfg_output(WDT_PIN);
    nrf_gpio_pin_clear(WDT_PIN);
    for (;;)
    {
        vTaskDelayUntil(&current_tick, next_delay_tick);
        nrf_gpio_pin_set(WDT_PIN);
        nrf_gpio_pin_clear(WDT_PIN);
        // wdt_feed();
    }
}

static void global_init(void)
{
    global.freertos_on = true;
    global.lteState = true;
}

static void semaphore_init(void)
{
    global.semaphore_spi1 = xSemaphoreCreateBinary();
    ASSERT(global.semaphore_spi1);

    global.semaphore_spi1_xfer_complete = xSemaphoreCreateBinary();
    ASSERT(global.semaphore_spi1_xfer_complete);

    global.semaphore_spi2 = xSemaphoreCreateBinary();
    ASSERT(global.semaphore_spi2)

    global.semaphore_spi2_xfer_complete = xSemaphoreCreateBinary();
    ASSERT(global.semaphore_spi2_xfer_complete)

    global.semphore_sx1268_rx_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1268_rx_done)

    global.semphore_sx1268_tx_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1268_tx_done)

    global.semphore_sx1268_dio_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1268_dio_done)

    global.semphore_sx1280_1_dio_process = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1280_1_dio_process);

    global.semphore_sx1280_2_dio_process = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1280_2_dio_process);

    global.semphore_sx1280_1_dio1_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1280_1_dio1_done);

    global.semphore_sx1280_2_dio1_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1280_2_dio1_done);

    global.semphore_sx1280_tx_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_sx1280_tx_done);

    global.semphore_uarte_lte_tx_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_uarte_lte_tx_done)

    global.semphore_saadc_sample_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_saadc_sample_done)

    global.semphore_lte_connect_done = xSemaphoreCreateBinary();
    ASSERT(global.semphore_lte_connect_done)

    global.semphore_tdma_start = xSemaphoreCreateBinary();
    ASSERT(global.semphore_tdma_start)
}

static void queue_init(void)
{
    global.queue_uarte_lte_tx_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_uarte_lte_tx_msg)

    global.queue_uarte_lte_rx_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_uarte_lte_rx_msg)

    global.queue_sensor_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_sensor_msg)

    global.queue_mpower_sensor_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_mpower_sensor_msg)

    global.queue_mpower_send_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_mpower_send_msg)

    global.queue_sx1268_send_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_sx1268_send_msg)

    global.queue_sx1268_receive_msg = xQueueCreate(20, sizeof(msg_t));
    ASSERT(global.queue_sx1268_receive_msg)
}

static void task_init(void)
{
    UNUSED_VARIABLE(xTaskCreate(task_lte, "lte", 1024, NULL, 2,
                                &global.task_handle_lte));
    UNUSED_VARIABLE(xTaskCreate(task_uarte_lte_tx, "uarte_lte_tx", 1024, NULL, 2,
                                &global.task_handle_lte_uarte_tx));
    UNUSED_VARIABLE(xTaskCreate(task_uarte_lte_rx, "uarte_lte_rx", 1024, NULL, 2,
                                &global.task_handle_lte_uarte_rx));

    UNUSED_VARIABLE(xTaskCreate(task_sx1268, "sx1268_task", 1024, NULL, 2,
                                &global.task_handle_sx1268));
    UNUSED_VARIABLE(xTaskCreate(task_low_power, "low_power_task", 1024, NULL, 2,
                                &global.task_handle_low_power));
    UNUSED_VARIABLE(xTaskCreate(task_tdma, "tdma_task", 1024, NULL, 2,
                                &global.task_handle_tdma));

    UNUSED_VARIABLE(xTaskCreate(task_sx1280, "sx1280_task", 1024, NULL, 2,
                                &global.task_handle_sx1280));
    UNUSED_VARIABLE(xTaskCreate(task_micro_power, "micro_power_task", 512, NULL, 2,
                                &global.task_handle_micro_power));
    UNUSED_VARIABLE(xTaskCreate(task_sx1280_1_dio_process, "sx1280_1_dio_process", 1024, NULL, 2,
                                &global.task_handle_sx1280_1_dio_process));
    UNUSED_VARIABLE(xTaskCreate(task_sx1280_2_dio_process, "sx1280_2_dio_process", 1024, NULL, 2,
                                &global.task_handle_sx1280_2_dio_process));
                                
    UNUSED_VARIABLE(xTaskCreate(task_watch_dog, "watch_dog_task", 128, NULL, 1,
                                &global.task_handle_watch_dog));
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize modules.
    log_init();
    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.
    // dfu_buttonless_init();
    clock_init();
    gpiote_init();
    sleep_mode_init();

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // Initialize modules.
    timers_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();
    // wdt_init();
    global_init();
    semaphore_init();
    queue_init();
    task_init();
    NRF_LOG_INFO("Initialization complete");

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, false);

    NRF_LOG_INFO("powergw start");
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}
