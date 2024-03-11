#ifdef USE_FULL_LL_DRIVER

#include "ComRecv.hh"
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
#include <utility>
#include <vector>

ComRecv::ComRecv(USART_TypeDef *const usart, bool &recvComplete, std::function<bool(const std::vector<std::uint8_t>)> callback):
    _usart(usart),
    _recvComplete(recvComplete),
    _rxBuf(256),
    _callback(std::move(callback)) {
}

void ComRecv::receive(const std::vector<std::uint8_t> &recvData) const {
    if(recvData.at(0) != 'S') return;
    std::uint8_t checksum = 0;
    std::for_each(recvData.begin(), recvData.begin() + recvData.at(1) - 1, [&](std::uint8_t data) -> void {
        checksum = checksum + data & 0xff;
    });
    if(checksum != recvData.at(recvData.end() - recvData.begin() - 1)) return;
    std::vector<std::uint8_t> data(recvData.begin() + 2, recvData.end() - 1);
    _callback(data);
}

void ComRecv::handler() {
    if(LL_USART_IsActiveFlag_ORE(_usart)) {
        LL_USART_ReceiveData8(_usart);
        _count = 0;
        LL_USART_DisableIT_RXNE(_usart);
        LL_USART_ClearFlag_ORE(_usart);
        LL_USART_ClearFlag_NE(_usart);
        LL_USART_EnableIT_RXNE(_usart);
    } else if(LL_USART_IsActiveFlag_PE(_usart)) {
        LL_USART_ReceiveData8(_usart);
        _count = 0;
    } else if(LL_USART_IsActiveFlag_FE(_usart)) {
        LL_USART_ReceiveData8(_usart);
        _count = 0;
    } else if(LL_USART_IsActiveFlag_RXNE(_usart)) {
        if(_count == 0) _recvComplete = false;
        _rxBuf.at(_count++) = LL_USART_ReceiveData8(_usart);
        if(_rxBuf.at(0) != 'S') _count = 0;
        if(_count == 2) _len = _rxBuf.at(1);
        if(2 < _count && _count == _len) {
            _count = 0;
            receive(_rxBuf);
            _recvComplete = true;
        }
    }
}

#endif // USE_FULL_LL_DRIVER
