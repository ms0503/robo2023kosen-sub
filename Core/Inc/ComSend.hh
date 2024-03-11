#pragma once

#ifdef USE_FULL_LL_DRIVER

#include <cstdint>
#ifdef STM32F0
    #include <stm32f0xx_ll_usart.h>
#endif
#ifdef STM32F3
    #include <stm32f3xx_ll_usart.h>
#endif
#ifdef STM32F4
    #include <stm32f4xx_ll_usart.h>
#endif
#include <vector>

struct SendBuffer {
    std::uint8_t *ptr;
    std::uint16_t size;
    std::uint16_t count;
};

class ComSend {
public:
    explicit ComSend(USART_TypeDef *usart, SendBuffer &buf);

    template<std::size_t SIZE>
    inline void send(const std::uint8_t (&data)[SIZE]) {
        std::vector<std::uint8_t> data1(data, data + SIZE);
        send(data1);
    }

    void send(std::vector<std::uint8_t> data);

    void handler();

private:
    USART_TypeDef *const _usart;
    SendBuffer &_buf;
};

#endif // USE_FULL_LL_DRIVER
