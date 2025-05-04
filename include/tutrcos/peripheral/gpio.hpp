#pragma once

#include "main.h"

#include <array>
#include <cstdint>
#include <functional>

#include "tutrcos/utility.hpp"

namespace tutrcos {
namespace peripheral {

/**
 * `GPIO::set_callback` を使う場合は System Core -> GPIO -> NVIC -> EXTI
 * line x interrupts を有効化してください。
 *
 * @code{.cpp}
 * #include <tutrcos.hpp>
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *
 *   GPIO led(LD2_GPIO_Port, LD2_Pin);
 *   GPIO button(B1_GPIO_Port, B1_Pin);
 *
 *   button.set_callback([&] {
 *     // GPIO EXTI 割り込み
 *   });
 *
 *   while (true) {
 *     led.write(!button.read()); // ボタンを押している間LED点灯
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class GPIO {
public:
  GPIO(GPIO_TypeDef *port, uint16_t pin) : port_{port}, pin_{pin} {
    auto gpio =
        std::find(get_instances().begin(), get_instances().end(), nullptr);
    TUTRCOS_VERIFY(gpio != get_instances().end());
    *gpio = this;
  }

  ~GPIO() {
    auto gpio = std::find(get_instances().begin(), get_instances().end(), this);
    TUTRCOS_VERIFY(gpio != get_instances().end());
    *gpio = nullptr;
  }

  void write(bool state) {
    HAL_GPIO_WritePin(port_, pin_, static_cast<GPIO_PinState>(state));
  }

  bool read() { return HAL_GPIO_ReadPin(port_, pin_); }

  void toggle() { HAL_GPIO_TogglePin(port_, pin_); }

  void set_callback(std::function<void()> &&callback) {
    callback_ = std::move(callback);
  }

  GPIO_TypeDef *get_hal_port() { return port_; }

  uint16_t get_hal_pin() { return pin_; }

private:
  GPIO_TypeDef *port_;
  uint16_t pin_;
  std::function<void()> callback_;

  static inline std::array<GPIO *, 21> &get_instances() {
    static std::array<GPIO *, 21> instances{};
    return instances;
  }

  friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
};

} // namespace peripheral
} // namespace tutrcos
