#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

#define LED_PORT PORTA
#define LED_DDR  DDRA
#define LED_PIN  PA0

/* ---------- UART0: 38400bps @ 8MHz ----------
   UBRR = F_CPU/(16*BAUD) - 1
   = 8,000,000/(16*38,400) - 1
   = 12 (실효 약 38,461bps, 오차 ~0.16%) */
static void uart0_init_38400(void) {
    // 더블스피드 OFF
    UCSR0A &= ~(1 << U2X0);

    // Baud 설정
    UBRR0H = 0;
    UBRR0L = 12;

    // 8N1
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // TX Enable (RX 필요시 RXEN0 추가)
    UCSR0B = (1 << TXEN0);
}

static void uart0_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart0_print(const char* s) {
    while (*s) uart0_putc(*s++);
}

int main(void) {
    // LED PA0: 출력, 초기 OFF
    LED_DDR  |= (1 << LED_PIN);
    LED_PORT &= ~(1 << LED_PIN);

    // UART 초기화
    uart0_init_38400();
    uart0_print("\r\n=== ATmega128 8MHz / 38400bps ===\r\n");

    while (1) {
        // LED ON (5초)
        LED_PORT |= (1 << LED_PIN);
        uart0_print("LED ON\r\n");
        _delay_ms(5000);

        // LED OFF (5초)
        LED_PORT &= ~(1 << LED_PIN);
        uart0_print("LED OFF\r\n");
        _delay_ms(5000);
    }
}