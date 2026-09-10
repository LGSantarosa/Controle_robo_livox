// Escuta pura: qual dos dois fios carrega o TX da placa de hover?
//
// O firmware da placa (EFeru) manda o frame de feedback periodicamente mesmo
// sem receber comando. Então, com a MEGA calada, o fio que mexer é o TX dela.
//
// A cada segundo imprime:
//
//   p18: bordas=0 alto=100%   p19: bordas=3120 alto=62%   rx19=180 cdab=10  primeiros: CD AB ...
//
//   bordas  transições do nível do pino (independe de baud)
//   alto    fração das amostras em HIGH (linha parada com pull-up = 100%)
//   rx19    bytes que a UART do 19 decodificou a 115200 (só o 19 tem UART RX)
//   cdab    pares CD AB (início de frame da placa) entre eles
//
// Leitura: bordas>0 num pino = alguém transmite nele. Se for no 18, os fios
// estão cruzados em relação a esta placa (o azul não é o TX dela).
//
// O TX1 fica DESLIGADO (TXEN1 limpo): o pino 18 volta a ser GPIO de entrada e
// a MEGA não põe nada na linha. Pull-up interno nos dois — sem ele um pino solto
// capta ruído e parece tráfego.

#include <Arduino.h>

static const uint32_t JANELA_MS = 1000;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial1.begin(115200);
    UCSR1B &= ~_BV(TXEN1);        // cala o TX1: só RX
    pinMode(18, INPUT_PULLUP);    // PD3
    pinMode(19, INPUT_PULLUP);    // PD2 (a UART lê por cima, o pull-up fica)

    Serial.println(F("\n== ESCUTA PURA  18 e 19 como entrada  MEGA NAO TRANSMITE =="));
}

void loop() {
    uint32_t b18 = 0, b19 = 0, h18 = 0, h19 = 0, n = 0, rx = 0, cdab = 0;
    uint8_t prim[16], n_prim = 0, prev = 0;
    uint8_t last = PIND & (_BV(PD3) | _BV(PD2));

    const uint32_t t0 = millis();
    while (millis() - t0 < JANELA_MS) {
        const uint8_t v = PIND & (_BV(PD3) | _BV(PD2));
        const uint8_t mudou = v ^ last;
        if (mudou & _BV(PD3)) b18++;
        if (mudou & _BV(PD2)) b19++;
        if (v & _BV(PD3)) h18++;
        if (v & _BV(PD2)) h19++;
        n++;
        last = v;
        while (Serial1.available()) {
            const uint8_t b = (uint8_t)Serial1.read();
            rx++;
            if (n_prim < sizeof(prim)) prim[n_prim++] = b;
            if (prev == 0xCD && b == 0xAB) cdab++;
            prev = b;
        }
    }

    Serial.print(F("p18: bordas=")); Serial.print(b18);
    Serial.print(F(" alto="));       Serial.print(100UL * h18 / n);
    Serial.print(F("%   p19: bordas=")); Serial.print(b19);
    Serial.print(F(" alto="));       Serial.print(100UL * h19 / n);
    Serial.print(F("%   rx19="));    Serial.print(rx);
    Serial.print(F(" cdab="));       Serial.print(cdab);
    Serial.print(F("  primeiros:"));
    for (uint8_t i = 0; i < n_prim; i++) {
        Serial.print(' ');
        if (prim[i] < 0x10) Serial.print('0');
        Serial.print(prim[i], HEX);
    }
    Serial.println();
}
