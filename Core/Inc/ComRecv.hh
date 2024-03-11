#pragma once

#ifdef USE_FULL_LL_DRIVER

#include <cstdint>
#include <functional>
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

class ComRecv {
public:
    explicit ComRecv(USART_TypeDef *usart, bool &recvComplete, std::function<bool(const std::vector<std::uint8_t>)> callback);

    ~ComRecv() = default;

    template<std::size_t SIZE>
    inline void receive(const std::uint8_t (&data)[SIZE]) {
        const std::vector<std::uint8_t> data1(data, data + SIZE);
        receive(data1);
    }

    void receive(const std::vector<std::uint8_t> &recvData) const;

    void handler();

private:
    USART_TypeDef *const _usart;
    const std::function<bool(const std::vector<std::uint8_t>)> _callback;
    std::vector<std::uint8_t> _rxBuf;
    std::uint8_t _count = 0;
    std::uint8_t _len = 0;
    bool &_recvComplete;
};

#endif // USE_FULL_LL_DRIVER
