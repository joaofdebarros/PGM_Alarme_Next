#include "XMC1100.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"
#include "xmc_usic.h"
#include <stdint.h>

#define SIZE 0
#define IDT 1
#define UNIQUEID 2
//#define ADRS0 2
//#define ADRS1 3
//#define ADRS2 4
//#define ADRS3 5
#define FUNCTION 3
#define DATA 4

#define TAMANHO_BUFFER 8
#define TAMANHO_BUFFER_ACK 11
uint8_t buffer_size = 8;
// Buffer de envio e recepção de dados
volatile uint8_t Rx_buffer[TAMANHO_BUFFER_ACK];
volatile uint8_t Rx_buffer_index = 0;
volatile uint8_t Buffer_TX[TAMANHO_BUFFER_ACK] = {0};

typedef enum {
  PGM_ID = 0x01,
  PGM_BROADCAST_ID = 0x02,
  PGM_ID_RESPONSE = 0x03,
} PGM_DEVICE_ID_t;

uint32_t UniqueChipID[4] = {0, 0, 0, 0};
uint8_t uid = 0;

// Flags de envio e recepção de dados
volatile bool pacote_completo = false;
volatile bool recebendo = false;
volatile bool pacote_obsoleto = false;
volatile bool cadastrado = false;

// Variáveis do pacote
#define start_byte 0x7E
uint8_t UID0 = 0;
uint8_t UID1 = 0;
uint8_t UID2 = 0;
uint8_t UID3 = 0;
uint8_t checksum = 0;
#define stop_byte 0x81

#define ACK 0x06
// Máquina de estados
uint8_t estado = 0;

// Variáveis de tempo/delays
uint16_t delay_aleatorio = 0;
uint8_t Blinking_gap = 10;
uint16_t Blink_wait = 1000;
uint8_t piscadas = 0;
uint8_t numero_modulo = 0;
volatile uint32_t systick = 0;
volatile uint32_t delay_tx = 0;
volatile bool aguardando_envio = false;
volatile bool piscando = false;

// União bitflag
typedef union {
  uint8_t Byte;
  struct {
    bool rele_1 : 1;
    bool rele_2 : 1;
    bool rele_3 : 1;
    bool rele_4 : 1;
    bool rele_5 : 1;
    bool bitflag5 : 1;
    bool bitflag6 : 1;
    bool bitflag7 : 1;
  } Bits;
} FLAG0;

FLAG0 status_rele, ligar_rele;

typedef struct {
  XMC_GPIO_PORT_t *port;
  uint8_t pin;
  uint8_t blink_count;
  uint8_t blink_target;
  bool piscando;
} LED_t;

LED_t leds[1] = {{LED_ST_PORT, LED_ST_PIN, 0, 0, false}};

XMC_GPIO_PORT_t *const rele_ports[5] = {RL1_PORT, RL2_PORT, RL3_PORT, RL4_PORT,
                                        RL5_PORT};

const uint8_t rele_pins[5] = {RL1_PIN, RL2_PIN, RL3_PIN, RL4_PIN, RL5_PIN};

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Rotina para salvar o UID do micro
void get_UID() {

  volatile uint32_t *UCIDptr = (volatile uint32_t *)0x10000FF0;

  for (uint8_t Count = 0; Count < 4; Count++) {
    UniqueChipID[Count] = UCIDptr[Count];
  }

  UID0 = (UniqueChipID[0] >> 0) & 0xFF;
  UID1 = (UniqueChipID[0] >> 8) & 0xFF;
  UID2 = (UniqueChipID[0] >> 16) & 0xFF;
  UID3 = (UniqueChipID[0] >> 24) & 0xFF;
}

// Rotina para salvar o UID do micro
// void get_UID() {
//  uint32_t UniqueChipID[3] = {0, 0, 0};
//
//  volatile uint32_t *UCIDptr = (volatile uint32_t *)0x10000FF0;
//
//  for (uint8_t Count = 0; Count < 3; Count++) {
//    UniqueChipID[Count] = UCIDptr[Count];
//  }
//
//  UID0 = (UniqueChipID[0] >> 0) & 0xFF;
//  UID1 = (UniqueChipID[0] >> 8) & 0xFF;
//  UID2 = (UniqueChipID[0] >> 16) & 0xFF;
//  UID3 = (UniqueChipID[0] >> 24) & 0xFF;
//}

uint16_t gerar_intervalo(uint8_t UID0, uint8_t UID1, uint8_t UID2, uint8_t UID3,
                         uint32_t tempo) {
  // Combina os valores do UID e tempo para criar uma semente pseudoaleatória
  uint32_t semente = UID0;
  semente ^= ((uint32_t)UID1 << 8);
  semente ^= ((uint32_t)UID2 << 16);
  semente ^= ((uint32_t)UID3 << 24);
  semente ^= tempo; // misturando com o tempo para aleatoriedade

  // Um pequeno hash para embaralhar a semente
  semente ^= (semente >> 13);
  semente *= 0x85ebca6b;
  semente ^= (semente >> 16);

  // Reduz o valor para o intervalo [20, 500]
  uint16_t intervalo = 20 + (semente % (450 - 2 + 1));
  return intervalo;
}

void montar_pacote(uint8_t size, uint8_t ID, uint8_t Addrs, uint8_t function,
                   uint8_t data, volatile uint8_t *destino) {
  destino[0] = start_byte;
  destino[1] = size;
  destino[2] = ID;
  destino[3] = Addrs;
  destino[4] = function;
  destino[5] = data;

  destino[6] = ~(destino[0] ^ destino[1] ^ destino[2] ^ destino[3] ^
                 destino[4] ^ destino[5]);

  destino[7] = stop_byte;
}

void montar_pacote_cadastro(uint8_t size, uint8_t ID, uint8_t Addrs_1,
                            uint8_t Addrs_2, uint8_t Addrs_3, uint8_t Addrs_4,
                            uint8_t function, uint8_t data,
                            volatile uint8_t *destino) {
  destino[0] = start_byte;
  destino[1] = size;
  destino[2] = ID;
  destino[3] = Addrs_1; // UID0
  destino[4] = Addrs_2; // UID1
  destino[5] = Addrs_3; // UID2
  destino[6] = Addrs_4; // UID3
  destino[7] = function;
  destino[8] = data; // uid CRC-8
  destino[9] =
      ~(destino[0] ^ destino[1] ^ destino[2] ^ destino[3] ^ destino[4] ^
        destino[5] ^ destino[6] ^ destino[7] ^ destino[8]);

  destino[10] = stop_byte;
}

// Rotina para receber dados
void USIC0_1_IRQHandler(void) {
  if (pacote_completo == false) {
    while (!XMC_USIC_CH_RXFIFO_IsEmpty(UART1_HW)) {

      uint8_t rx = XMC_UART_CH_GetReceivedData(UART1_HW);
      if (!recebendo) {
        if (rx == 0x7E) {
          recebendo = true;
          Rx_buffer_index = 0;
        }

      } else {
        if (rx == 0x81) {
          recebendo = false;
          if (Rx_buffer[IDT] == PGM_ID || Rx_buffer[IDT] == PGM_BROADCAST_ID) {
            pacote_completo = true;
          } else {
            Rx_buffer_index = 0;
          }

          break;
        } else {
          if (Rx_buffer_index < 12) {
            Rx_buffer[Rx_buffer_index++] = rx;
          } else {
            recebendo = false;
            Rx_buffer_index = 0;
          }
        }
      }
    }

    if ((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT) {
      XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
          UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
          (TAMANHO_BUFFER - Rx_buffer_index) - 1);
    }
  }
}

void blink_led_ST(uint8_t n) {

  leds[0].blink_target = n * 2; // *2 porque liga/desliga conta 2 vezes
  leds[0].blink_count = 0;
  leds[0].piscando = true;
}

// Rotina para controle dos tempos
void SysTick_Handler(void) {

  if (--Blinking_gap == 0) {

    if (cadastrado) {
      Blinking_gap = 200;
    } else {
      Blinking_gap = 100;
    }

    if (leds[0].piscando) {
      if (leds[0].blink_count < leds[0].blink_target) {
        XMC_GPIO_ToggleOutput(leds[0].port, leds[0].pin);
        leds[0].blink_count++;
      } else {
        if (cadastrado) {
          Blink_wait = 1000;
        }
        if (!cadastrado) {
          Blink_wait = 100;
        }
        leds[0].piscando = false;
        XMC_GPIO_SetOutputHigh(leds[0].port, leds[0].pin); // garante desligado
      }
    }
  }

  if (--Blink_wait == 0) {

    if (!cadastrado) {
      blink_led_ST(4);
    } else {
      blink_led_ST(numero_modulo);
    }
  }

  systick++;
}
// Máquina de estados
void Controle() {

#define RECEIVE 0
#define GET_UID 1
#define STATUS_RL 2
#define RL_CONTROL 3
#define DELETE 4
#define TRANSMIT 5
#define DELAY_ENVIO 6
#define LIMPAR 7

  switch (estado) {
  case RECEIVE: {

    XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
    if (pacote_completo) {

      if (Rx_buffer[FUNCTION] == 'A' && !cadastrado) {

        if (Rx_buffer[UNIQUEID] == uid) {
          numero_modulo = Rx_buffer[DATA];
          cadastrado = true;
        } else {
          estado = GET_UID;
        }

      } else if (Rx_buffer[FUNCTION] == 'S' && Rx_buffer[UNIQUEID] == uid) {
        cadastrado = true;
        numero_modulo = Rx_buffer[DATA];
        estado = STATUS_RL;
      } else if (Rx_buffer[FUNCTION] == 'T' && Rx_buffer[UNIQUEID] == uid) {
        ligar_rele.Byte = Rx_buffer[DATA];
        estado = RL_CONTROL;
      } else if (Rx_buffer[FUNCTION] == 'D') {
        if (Rx_buffer[IDT] == PGM_BROADCAST_ID) {
          ligar_rele.Byte = 0x00;
          cadastrado = false;
          pacote_obsoleto = true;
          estado = RL_CONTROL;
        }

        if (Rx_buffer[IDT] == PGM_ID && Rx_buffer[UNIQUEID] == uid) {
          ligar_rele.Byte = 0x00;
          cadastrado = false;
          pacote_obsoleto = true;
          estado = RL_CONTROL;
        }

      } else {
        pacote_obsoleto = true;
        estado = LIMPAR;
      }
    }

  } break;

  case GET_UID: {
    // Enviar o UID do dispositivo
    buffer_size = 11;
    montar_pacote_cadastro(buffer_size, PGM_ID_RESPONSE, UID0, UID1, UID2, UID3, 'A',
                           uid, Buffer_TX);
	
    estado = TRANSMIT;

  } break;

  case STATUS_RL: {
    if (Rx_buffer[UNIQUEID] == uid) {
      // Enviar o status de cada rele
	  buffer_size = 8;
      montar_pacote(buffer_size, PGM_ID_RESPONSE, uid, 'S', status_rele.Byte, Buffer_TX);

      delay_aleatorio = gerar_intervalo(UID0, UID1, UID2, UID3, systick);

      estado = TRANSMIT;
    } else {
      estado = LIMPAR;
    }

  } break;

  case RL_CONTROL: {
    // Ligar cada rele conforme solicitado
    for (int i = 0; i < 5; i++) {
      uint8_t mask =
          (1 << i); // cria uma máscara para cada bit (rele_1 até rele_5)

      if (ligar_rele.Byte & mask) {
        XMC_GPIO_SetOutputHigh(rele_ports[i], rele_pins[i]);
        status_rele.Byte |= mask;
      } else {
        XMC_GPIO_SetOutputLow(rele_ports[i], rele_pins[i]);
        status_rele.Byte &= ~mask;
      }
    }
	
	buffer_size = 8;
    montar_pacote(buffer_size, PGM_ID_RESPONSE, uid, 'T', ACK, Buffer_TX);
    estado = TRANSMIT;

  } break;

  case DELETE: {
	buffer_size = 8;
    montar_pacote(buffer_size, PGM_ID_RESPONSE, uid, 'D', ACK, Buffer_TX);
    estado = TRANSMIT;
  } break;

  case TRANSMIT: {

    if (!cadastrado) {
      delay_tx = systick + gerar_intervalo(UID0, UID1, UID2, UID3, systick);
    } else {
      delay_tx = systick + 2;
    }
    aguardando_envio = true;
    estado = DELAY_ENVIO;

  } break;

  case DELAY_ENVIO: {
    if (systick >= delay_tx) {
      XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
      for (int i = 0; i < buffer_size; i++) {
        XMC_UART_CH_Transmit(UART1_HW, Buffer_TX[i]);
      }
      while (!XMC_USIC_CH_TXFIFO_IsEmpty(UART1_HW))
        ;

      XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);

      pacote_obsoleto = true;
      aguardando_envio = false;
      estado = LIMPAR;
    }

  } break;

  case LIMPAR: {
    if (pacote_obsoleto) {
      for (uint8_t i = 0; i < sizeof(Buffer_TX); i++) {
        Buffer_TX[i] = 0;
      }
      pacote_completo = false;
      pacote_obsoleto = false;
    }

    for (uint8_t i = 0; i < sizeof(Rx_buffer); i++) {
      Rx_buffer[i] = 0;
    }

    pacote_completo = false;
    estado = RECEIVE;

  } break;
  }
}

int main(void) {
  cy_rslt_t result;

  /* Inicializa a placa */
  result = cybsp_init();
  if (result != CY_RSLT_SUCCESS) {
    CY_ASSERT(0);
  }

  SysTick_Config(SystemCoreClock / 1000);

  NVIC_EnableIRQ(USIC0_1_IRQn);
  NVIC_SetPriority(USIC0_1_IRQn, 2);

  XMC_UART_CH_Start(UART1_HW);

  // Inverte a saída de dados da UART (nível lógico de TX)
  UART1_HW->SCTR = (UART1_HW->SCTR & ~(0X3 << 6)) | (0X1 << 6);

  while (XMC_USIC_CH_TXFIFO_IsFull(UART1_HW))
    ;

  if ((TAMANHO_BUFFER - Rx_buffer_index) < UART1_RXFIFO_LIMIT) {
    XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
        UART1_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
        (TAMANHO_BUFFER - Rx_buffer_index) - 1);
  }

  XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
  XMC_GPIO_SetOutputLow(LED_ST_PORT, LED_ST_PIN);

  get_UID();
  uid = crc8((uint8_t *)UniqueChipID, 16);

  while (1) {
    Controle();
  }
}