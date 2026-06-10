#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define F_CPU 8000000UL
#define BAUD 9600
#define UBRR_VAL ((F_CPU / 16 / BAUD) - 1)

class UART {
public:
    void init() {
        UBRR0H = static_cast<uint8_t>(UBRR_VAL >> 8);
        UBRR0L = static_cast<uint8_t>(UBRR_VAL);
        UCSR0B = (1 << RXEN0) | (1 << TXEN0);
        UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    }

    void sendChar(char c) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;
    }

    void sendString(const char *str) {
        while (*str) {
            sendChar(*str++);
        }
    }

    char recvChar() {
        while (!(UCSR0A & (1 << RXC0)));
        return UDR0;
    }

    void recvLine(char *buf, uint8_t maxLen) {
        uint8_t i = 0;
        while (i < maxLen - 1) {
            char c = recvChar();
            if (c == '\n' || c == '\r') break;
            buf[i++] = c;
        }
        buf[i] = '\0';
    }
};

int main() {
    char recvBuf[32];
    UART uart;

    uart.init();

    while (true) {
        uart.recvLine(recvBuf, sizeof(recvBuf));

        if (strlen(recvBuf) > 0) {
            uart.sendString("Hello RaspberryPi\n");
        }
    }

    return 0;
}