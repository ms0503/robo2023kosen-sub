#include "ComSend.hh"
#include <algorithm>
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

ComSend::ComSend(USART_TypeDef *const usart, SendBuffer &buf):
    _usart(usart),
    _buf(buf) {
}

void ComSend::send(std::vector<std::uint8_t> sendData) {
    sendData.insert(sendData.begin(), 'S');
    sendData.insert(sendData.begin() + 1, sendData.size() + 2);
    std::uint8_t checksum = 0;
    std::for_each(sendData.begin(), sendData.end(), [&](std::uint8_t data) -> void {
        checksum = checksum + data & 0xff;
    });
    sendData.push_back(checksum);
    _buf.ptr = sendData.data() + 1;
    _buf.size = sendData.size();
    _buf.count = 0;
    LL_USART_TransmitData8(_usart, sendData.at(0));
    LL_USART_EnableIT_TXE(_usart);
}

void ComSend::handler() {
    if(LL_USART_IsActiveFlag_TXE(_usart)) {
        _buf.count++;
        if(_buf.size <= _buf.count) {
            LL_USART_DisableIT_TXE(_usart);
            return;
        }
        LL_USART_TransmitData8(_usart, *(_buf.ptr++));
    }
}
