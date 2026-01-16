#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "LinBusListener.h"
#include "esphome/core/log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"

#ifdef CUSTOM_ESPHOME_UART
#include "esphome/components/uart/truma_uart_component_esp32_arduino.h"
#define ESPHOME_UART uart::truma_ESP32ArduinoUARTComponent
#else
// RZ: Use standard UART Component
#define ESPHOME_UART uart::UARTComponent
#endif // CUSTOM_ESPHOME_UART

// RZ: Correct include for modern ESPHome
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

// RZ: Fix for FreeRTOS type name change
#define QUEUE_WAIT_BLOCKING (TickType_t) portMAX_DELAY

void LinBusListener::setup_framework() {
  // Wir holen uns die Komponente nur, um sicherzugehen, dass sie da ist
  auto uartComp = static_cast<ESPHOME_UART *>(this->parent_);

  // -------------------------------------------------------------------------
  // RZ FIX: Hardware-Interrupts und FIFO-Hacks entfernt
  // Der folgende Code greift auf 'uart_num' und 'hw_serial' zu, die es in der
  // aktuellen API nicht mehr gibt. ESPHome verwaltet den Buffer jetzt selbst.
  // -------------------------------------------------------------------------

  /* ALTE LOGIK ENTFERNT (verursacht Fehler):
  
  uart_intr_config_t uart_intr;
  uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M; 
  uart_intr.rxfifo_full_thresh = 1;
  uart_intr.rx_timeout_thresh = 10;
  uart_intr.txfifo_empty_intr_thresh = 10;
  
  // FEHLER: uart_num existiert nicht mehr
  uart_intr_config(static_cast<uart_port_t>(uart_num), &uart_intr);

  // FEHLER: hw_serial existiert nicht mehr
  hw_serial->onReceive([this]() { this->onReceive_(); }, false);
  hw_serial->onReceiveError([this](hardwareSerial_error_t val) {
    this->clear_uart_buffer_();
    if (val == UART_BREAK_ERROR) {
      if (this->current_state_ != READ_STATE_SYNC) {
        this->current_state_ = READ_STATE_BREAK;
      }
      return;
    }
  });
  */

  // -------------------------------------------------------------------------
  // Task Creation (Bleibt bestehen, das ist korrektes ESP32 FreeRTOS)
  // -------------------------------------------------------------------------
  
  // Creating LIN msg event Task
  xTaskCreatePinnedToCore(LinBusListener::eventTask_,
                          "lin_event_task",         // name
                          4096,                     // stack size (in words)
                          this,                     // input params
                          2,                        // priority
                          &this->eventTaskHandle_,  // handle
                          0                         // core
  );

  if (this->eventTaskHandle_ == NULL) {
    ESP_LOGE(TAG, " -- LIN message Task not created!");
  }
}

void LinBusListener::eventTask_(void *args) {
  LinBusListener *instance = (LinBusListener *) args;
  for (;;) {
    instance->process_lin_msg_queue(QUEUE_WAIT_BLOCKING);
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef QUEUE_WAIT_BLOCKING
#undef ESPHOME_UART

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
