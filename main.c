/*
 * main.c
 *
 *  Created on: 22 de ago. de 2025
 *      Author: joao.victor
 */

#include "main.h"
#include "xmc_common.h"
#include "xmc_gpio.h"
#include <stdint.h>

pgm_packet_error_e pgm_alarm_packet_demount(uint8_t *datain, uint16_t len,
                                            pgm_alarm_packet_t *packet) {
  uint8_t checksum_validate;
  uint16_t i, size, lHold;

  if (datain == NULL || packet == NULL) {
    return PGM_PACKET_FAIL_UNKNOWN;
  }

  checksum_validate = 0x7E;
  // Transport all bytes to the struct
  size = 0;
  memset(packet, 0, sizeof(pgm_alarm_packet_t));

  for (i = 0; i < PGM_PACKET_LENGTH_LEN; i++) {
    packet->len = (datain[size++]);
  }
  lHold = (len - 7);
  checksum_validate ^= packet->len;

  for (i = 0; i < PGM_PACKET_ID_LEN; i++) {
    packet->id = (datain[size++]);
  }
  checksum_validate ^= packet->id;

  for (i = 0; i < PGM_PACKET_ADDRESS_LEN; i++) {
    packet->address = datain[size++];
  }
  checksum_validate ^= packet->address;

  for (i = 0; i < PGM_PACKET_FUNCTION_LEN; i++) {
    packet->function = datain[size++];
  }
  checksum_validate ^= packet->function;

  if (lHold > sizeof(packet->data)) {
    return PGM_PACKET_FAIL_UNKNOWN;  // pacote inválido
  }

  memcpy(packet->data, &datain[size], lHold);
  size += lHold;

  for (i = 0; i < lHold; i++) {
    checksum_validate ^= packet->data[i];
  }

  for (i = 0; i < PGM_PACKET_CHECKSUM_LEN; i++) {
    packet->checksum = (datain[size++]);
  }
  checksum_validate = ~checksum_validate;

  for (i = 0; i < PGM_PACKET_TAIL_LEN; i++) {
    packet->tail = (datain[size++]);
  }

  if (packet->tail != 0x81) {
    return PGM_PACKET_FAIL_TAIL;
  }

  if (checksum_validate != packet->checksum) {
    return PGM_PACKET_FAIL_CHECKSUM;
  }

  return PGM_PACKET_OK;
}

pgm_packet_error_e pgm_gate_packet_demount(uint8_t *datain, uint16_t len,
                                           pgm_gate_packet_t *packet) {
  uint8_t checksum_validate;
  uint16_t i, size, lHold;

  if (datain == NULL || packet == NULL) {
    return PGM_PACKET_FAIL_UNKNOWN;
  }

  checksum_validate = 0x7E;
  // Transport all bytes to the struct
  size = 0;
  memset(packet, 0, sizeof(pgm_gate_packet_t));

  for (i = 0; i < PGM_PACKET_LENGTH_LEN; i++) {
    packet->len = (datain[size++]);
  }
  lHold = (len - 6);
  checksum_validate ^= packet->len;

  for (i = 0; i < PGM_PACKET_ID_LEN; i++) {
    packet->id = (datain[size++]);
  }
  checksum_validate ^= packet->id;

  for (i = 0; i < PGM_PACKET_FUNCTION_LEN; i++) {
    packet->function = datain[size++];
  }
  checksum_validate ^= packet->function;

  if (lHold > sizeof(packet->data)) {
    return PGM_PACKET_FAIL_UNKNOWN;  // pacote inválido
  }
  
  memcpy(packet->data, &datain[size], lHold);
  size += lHold;

  for (i = 0; i < lHold; i++) {
    checksum_validate ^= packet->data[i];
  }

  for (i = 0; i < PGM_PACKET_CHECKSUM_LEN; i++) {
    packet->checksum = (datain[size++]);
  }
  checksum_validate = ~checksum_validate;

  for (i = 0; i < PGM_PACKET_TAIL_LEN; i++) {
    packet->tail = (datain[size++]);
  }

//  if (packet->tail != 0x81) {
//    return PGM_PACKET_FAIL_TAIL;
//  }

  if (checksum_validate != packet->checksum) {
    return PGM_PACKET_FAIL_CHECKSUM;
  }

  return PGM_PACKET_OK;
}

void reset_uart() {
  XMC_UART_CH_InitEx(UART_Bus_HW, &UART_Bus_config, false);
  XMC_UART_CH_SetInputSource(UART_Bus_HW,
                             (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0,
                             UART_Bus_DX0_INPUT);
  XMC_UART_CH_SetSamplePoint(UART_Bus_HW, 8U);
  XMC_USIC_CH_SetFractionalDivider(
      UART_Bus_HW, XMC_USIC_CH_BRG_CLOCK_DIVIDER_MODE_FRACTIONAL, 289U);
  XMC_USIC_CH_SetBaudrateDivider(UART_Bus_HW,
                                 XMC_USIC_CH_BRG_CLOCK_SOURCE_DIVIDER, false,
                                 58U, XMC_USIC_CH_BRG_CTQSEL_PDIV, 0U, 15U);
  XMC_USIC_CH_RXFIFO_Configure(UART_Bus_HW, UART_Bus_RXFIFO_DPTR,
                               UART_Bus_RXFIFO_SIZE, UART_Bus_RXFIFO_LIMIT);
  XMC_USIC_CH_TXFIFO_Configure(UART_Bus_HW, UART_Bus_TXFIFO_DPTR,
                               UART_Bus_TXFIFO_SIZE, UART_Bus_TXFIFO_LIMIT);
  XMC_USIC_CH_SetInterruptNodePointer(
      UART_Bus_HW, XMC_USIC_CH_INTERRUPT_NODE_POINTER_RECEIVE, 1U);
  XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(
      UART_Bus_HW, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 0U);
  XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(
      UART_Bus_HW, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 1U);
  XMC_UART_CH_EnableEvent(UART_Bus_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
  XMC_USIC_CH_TXFIFO_EnableEvent(
      UART_Bus_HW, (uint32_t)XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
  XMC_USIC_CH_RXFIFO_EnableEvent(
      UART_Bus_HW, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);
  XMC_UART_CH_Start(UART_Bus_HW);

  XMC_GPIO_Init(Bus_TX_PORT, Bus_TX_PIN, &Bus_TX_config);
  XMC_GPIO_SetHardwareControl(Bus_TX_PORT, Bus_TX_PIN, Bus_TX_HWO);
}

void reset_gate_uart() {
 XMC_UART_CH_InitEx(UART_Prog_HW, &UART_Prog_config, false);
    XMC_UART_CH_SetInputSource(UART_Prog_HW, (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0, UART_Prog_DX0_INPUT);
    XMC_UART_CH_SetSamplePoint(UART_Prog_HW, 8U);
    XMC_USIC_CH_SetFractionalDivider(UART_Prog_HW, XMC_USIC_CH_BRG_CLOCK_DIVIDER_MODE_FRACTIONAL, 289U);
    XMC_USIC_CH_SetBaudrateDivider(UART_Prog_HW, XMC_USIC_CH_BRG_CLOCK_SOURCE_DIVIDER, false, 58U, XMC_USIC_CH_BRG_CTQSEL_PDIV, 0U, 15U);
    XMC_USIC_CH_RXFIFO_Configure(UART_Prog_HW, UART_Prog_RXFIFO_DPTR, UART_Prog_RXFIFO_SIZE, UART_Prog_RXFIFO_LIMIT);
    XMC_USIC_CH_TXFIFO_Configure(UART_Prog_HW, UART_Prog_TXFIFO_DPTR, UART_Prog_TXFIFO_SIZE, UART_Prog_TXFIFO_LIMIT);
    XMC_USIC_CH_SetInterruptNodePointer(UART_Prog_HW, XMC_USIC_CH_INTERRUPT_NODE_POINTER_RECEIVE, 2U);
    XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(UART_Prog_HW, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 3U);
    XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(UART_Prog_HW, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 2U);
    XMC_UART_CH_EnableEvent(UART_Prog_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
    XMC_USIC_CH_TXFIFO_EnableEvent(UART_Prog_HW, (uint32_t)XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
    XMC_USIC_CH_RXFIFO_EnableEvent(UART_Prog_HW, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);
    XMC_UART_CH_Start(UART_Prog_HW);

  XMC_GPIO_Init(PROG_TX_PORT, PROG_TX_PIN, &PROG_TX_config);
  XMC_GPIO_SetHardwareControl(PROG_TX_PORT, PROG_TX_PIN, PROG_TX_HWO);
}

void switch_to_gpio() {
  XMC_GPIO_SetMode(Bus_TX_PORT, Bus_TX_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
  XMC_GPIO_SetOutputLow(Bus_TX_PORT, Bus_TX_PIN);
}

void switch_gate_to_gpio() {
  XMC_GPIO_SetMode(PROG_TX_PORT, PROG_TX_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
  XMC_GPIO_SetOutputLow(PROG_TX_PORT, PROG_TX_PIN);
}

void switch_to_uart() {
  XMC_GPIO_SetMode(Bus_TX_PORT, Bus_TX_PIN, XMC_GPIO_MODE_OUTPUT_ALT2);
  reset_uart();
}

void switch_gate_to_uart() {
  XMC_GPIO_SetMode(PROG_TX_PORT, PROG_TX_PIN, XMC_GPIO_MODE_OUTPUT_ALT2);
  reset_gate_uart();
}

uint8_t calculate_checksum(uint8_t *buffer, uint8_t payload_size,
                           bool automatizador) {
  uint8_t sum = 0;
  uint8_t header_size = 0;

  if (automatizador) {
    header_size = 4;
  } else {
    header_size = 5;
  }

  for (uint8_t i = 0; i < (header_size + payload_size); i++) {
    sum ^= buffer[i];
  }

  return ~sum;
}

uint8_t crc8(const uint8_t *data, uint8_t len, uint8_t inc) {
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

  // Aplica o inc no final
  crc ^= inc;
  for (uint8_t j = 0; j < 8; j++) {
    if (crc & 0x80)
      crc = (crc << 1) ^ 0x07;
    else
      crc <<= 1;
  }

  return crc;
}

void update_flash(uint8_t valor) {
  uint32_t dado = 0xFFFFFFFF;
  dado &= 0xFFFFFF00; // limpa os 8 bits inferiores
  dado |= valor;      // insere o valor

  while (XMC_FLASH_IsBusy())
    ;
  XMC_FLASH_ErasePage((uint32_t *)FLASH_SECTOR_ADDR);
  while (XMC_FLASH_IsBusy())
    ;
  XMC_FLASH_ProgramPage((uint32_t *)FLASH_SECTOR_ADDR, &dado);
  while (XMC_FLASH_IsBusy())
    ;
}

uint8_t read_flash() {
  const uint32_t *ptr = (uint32_t *)FLASH_SECTOR_ADDR;

  // Verifica se está em branco (0xFF)
  if (*ptr == 0xFFFFFFFF) {
    return 0; // Valor padrão se ainda não foi escrito
  } else {
    return *ptr;
  }
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

  // Reduz o valor para o intervalo
  uint16_t intervalo = 20 + (semente % (1100 - 10 + 1));
  return intervalo;
}

uint8_t montar_pacote(uint8_t *tx, uint8_t size, uint8_t id, uint8_t adrs,
                      uint8_t fnct, uint8_t *data, uint8_t payload_size,
                      bool automatizador) {
  uint8_t total_size = 0;

  if (automatizador) {
    total_size = payload_size + 6;

    tx[0] = start_byte;
    tx[1] = size;
    tx[2] = id;
    tx[3] = fnct;

    for (int i = 0; i < payload_size; i++) {
      tx[4 + i] = data[i];
    }

    tx[4 + payload_size] = calculate_checksum(tx, payload_size, automatizador);
    tx[5 + payload_size] = stop_byte;
  } else {
    total_size = payload_size + 7;

    tx[0] = start_byte;
    tx[1] = size;
    tx[2] = id;
    tx[3] = adrs;
    tx[4] = fnct;

    for (int i = 0; i < payload_size; i++) {
      tx[5 + i] = data[i];
    }

    tx[5 + payload_size] = calculate_checksum(tx, payload_size, automatizador);
    tx[6 + payload_size] = stop_byte;
  }

  return total_size;
}

void receive_alarm_packet() {
  if (alarm_packet_completed == false && new_alarm_packet) {
    while (!XMC_USIC_CH_RXFIFO_IsEmpty(UART_Bus_HW)) {
	  new_alarm_packet = false;
      uint8_t rx = XMC_UART_CH_GetReceivedData(UART_Bus_HW);
      if (!recebendo) {
        if (rx == start_byte) {
          recebendo = true;
          Rx_buffer_index_alarm = 0;
        }

      } else {

        if (rx == stop_byte && (Rx_buffer_index_alarm == Rx_buffer[0] - 2)) {
          recebendo = false;
          Rx_buffer[Rx_buffer_index_alarm] = rx;
          pgm.pgm_error = pgm_alarm_packet_demount(Rx_buffer, Rx_buffer[0],
                                                   &pgm.alarm_packet);

          if (pgm.pgm_error == PGM_PACKET_OK &&
              (pgm.alarm_packet.id == PGM_ID ||
               pgm.alarm_packet.id == PGM_BROADCAST_ID)) {
            alarm_packet_completed = true;
          } else {
            alarm_packet_completed = false;
            Rx_buffer_index_alarm = 0;
          }

          break;
        } else {
          if (Rx_buffer_index_alarm < 12) {
            Rx_buffer[Rx_buffer_index_alarm++] = rx;
          } else {
            recebendo = false;
            Rx_buffer_index_alarm = 0;
          }
        }
      }
    }

    if ((TAMANHO_BUFFER - Rx_buffer_index_alarm) < UART_Bus_RXFIFO_LIMIT) {
      XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
          UART_Bus_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
          (TAMANHO_BUFFER - Rx_buffer_index_alarm) - 1);
    }
  }
}

void receive_gate_packet() {
  if (gate_packet_completed == false && new_gate_packet) {
    while (!XMC_USIC_CH_RXFIFO_IsEmpty(UART_Prog_HW)) {
	  new_gate_packet = false;
      uint8_t rx = XMC_UART_CH_GetReceivedData(UART_Prog_HW);
      if (!recebendo) {
        if (rx == start_byte) {
          recebendo = true;
          Rx_buffer_index_gate = 0;
        }

      } else {

        if (Rx_buffer_index_gate == Rx_buffer_gate[0] - 2) {
          recebendo = false;
          Rx_buffer_gate[Rx_buffer_index_gate] = rx;
          pgm.pgm_error = pgm_gate_packet_demount(Rx_buffer_gate, Rx_buffer_gate[0],
                                                   &pgm.gate_packet);

          if (pgm.pgm_error == PGM_PACKET_OK) {
            gate_packet_completed = true;
          } else {
            gate_packet_completed = false;
            Rx_buffer_index_gate = 0;
          }

          break;
        } else {
          if (Rx_buffer_index_gate < 40) {
            Rx_buffer_gate[Rx_buffer_index_gate++] = rx;
          } else {
            recebendo = false;
            Rx_buffer_index_gate = 0;
          }
        }
      }
    }

    if ((TAMANHO_BUFFER - Rx_buffer_index_gate) < UART_Bus_RXFIFO_LIMIT) {
      XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
          UART_Bus_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
          (TAMANHO_BUFFER - Rx_buffer_index_gate) - 1);
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

  if (--num_aleatorio == 0) {
    num_aleatorio = 200;
  }

  if (--Blinking_gap == 0) {

    if (cadastrado) {
      Blinking_gap = 200;
    } else {
      if(gate){
		Blinking_gap = 200;
	  }else{
		Blinking_gap = 100;	
	  }
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
  
  if(--gate_packet_delay == 0){
	send_packet = true;
	gate_packet_delay = 1000;
  }

  if (--cont_rele[0] == 0) {
    if (pgm.run_rele[0].function == PGM_PULSED) {
      cont_rele[0] = pgm.run_rele[0].time;
      time_rele_flag[0] = true;
    }else if(pgm.run_rele[0].function == PGM_RETENTION){
	  time_rele_flag[0] = true;
	}else if(pgm.run_rele[0].function == PGM_DELAYED_TOGGLE){
	  time_rele_flag[0] = true;
	}else {
		time_rele_flag[0] = false;
	}

    
  }

  if (--cont_rele[1] == 0) {
    if (pgm.run_rele[1].function == PGM_PULSED) {
      cont_rele[1] = pgm.run_rele[1].time;
      time_rele_flag[1] = true;
    }else if(pgm.run_rele[1].function == PGM_RETENTION){
	  time_rele_flag[1] = true;
	}else if(pgm.run_rele[1].function == PGM_DELAYED_TOGGLE){
	  time_rele_flag[1] = true;
	}else{
		time_rele_flag[1] = false;
	}

    
  }

  if (--cont_rele[2] == 0) {
    if (pgm.run_rele[2].function == PGM_PULSED) {
      cont_rele[2] = pgm.run_rele[2].time;
      time_rele_flag[2] = true;
    }else if(pgm.run_rele[2].function == PGM_RETENTION){
	  time_rele_flag[2] = true;
	}else if(pgm.run_rele[2].function == PGM_DELAYED_TOGGLE){
	  time_rele_flag[2] = true;
	}else{
		time_rele_flag[2] = false;
	}

    
  }

  if (--cont_rele[3] == 0) {
    if (pgm.run_rele[3].function == PGM_PULSED) {
      cont_rele[3] = pgm.run_rele[3].time;
      time_rele_flag[3] = true;
    }else if(pgm.run_rele[3].function == PGM_RETENTION){
	  time_rele_flag[3] = true;
	}else if(pgm.run_rele[3].function == PGM_DELAYED_TOGGLE){
	  time_rele_flag[3] = true;
	}else{
		time_rele_flag[3] = false;
	}

    
  }

  systick++;
}

void rele_Control() {
    
  if (time_rele_flag[0]) {
    if (pgm.run_rele[0].function == PGM_PULSED && pulsing_rl[0] == true) {
      new_state_rl[0] = !(pgm.run_rele[0].previous_state);
      time_rele_flag[0] = false;
      if (new_state_rl[0]) {
        XMC_GPIO_SetOutputHigh(rele_ports[0], rele_pins[0]);
      } else {
        XMC_GPIO_SetOutputLow(rele_ports[0], rele_pins[0]);
      }

      pgm.run_rele[0].previous_state = new_state_rl[0];
    }else{
	  XMC_GPIO_SetOutputLow(rele_ports[0], rele_pins[0]);
	}

    if (pgm.run_rele[0].function == PGM_RETENTION) {
      time_rele_flag[0] = false;
      XMC_GPIO_SetOutputLow(rele_ports[0],
                              rele_pins[0]);
    }
    
    if (pgm.run_rele[0].function == PGM_DELAYED_TOGGLE) {
      time_rele_flag[0] = false;
      XMC_GPIO_SetOutputHigh(rele_ports[0], rele_pins[0]);
    }
  }

  if (time_rele_flag[1]) {
    if (pgm.run_rele[1].function == PGM_PULSED) {
		if(pulsing_rl[1] == true){
		  new_state_rl[1] = !(pgm.run_rele[1].previous_state);
	      time_rele_flag[1] = false;
	      if (new_state_rl[1]) {
	        XMC_GPIO_SetOutputHigh(rele_ports[1], rele_pins[1]);
	      } else {
	        XMC_GPIO_SetOutputLow(rele_ports[1], rele_pins[1]);
	      }
	
	      pgm.run_rele[1].previous_state = new_state_rl[1];
		}else{
	  	  XMC_GPIO_SetOutputLow(rele_ports[1], rele_pins[1]);
	   }
    }

    if (pgm.run_rele[1].function == PGM_RETENTION) {
      time_rele_flag[1] = false;
      XMC_GPIO_SetOutputLow(rele_ports[1],
                              rele_pins[1]);
    }
    
    if (pgm.run_rele[1].function == PGM_DELAYED_TOGGLE) {
      time_rele_flag[1] = false;
      XMC_GPIO_SetOutputHigh(rele_ports[1], rele_pins[1]);
    }
  }

  if (time_rele_flag[2]) {
    if (pgm.run_rele[2].function == PGM_PULSED) {
		if(pulsing_rl[2] == true){
		  new_state_rl[2] = !(pgm.run_rele[2].previous_state);
	      time_rele_flag[2] = false;
	      if (new_state_rl[2]) {
	        XMC_GPIO_SetOutputHigh(rele_ports[2], rele_pins[2]);
	      } else {
	        XMC_GPIO_SetOutputLow(rele_ports[2], rele_pins[2]);
	      }
	
	      pgm.run_rele[2].previous_state = new_state_rl[2];
		}else{
	  	  XMC_GPIO_SetOutputLow(rele_ports[2], rele_pins[2]);
	   }
    }

    if (pgm.run_rele[2].function == PGM_RETENTION) {
      time_rele_flag[2] = false;
      XMC_GPIO_SetOutputLow(rele_ports[2],
                              rele_pins[2]);
    }
    
    if (pgm.run_rele[2].function == PGM_DELAYED_TOGGLE) {
      time_rele_flag[2] = false;
      XMC_GPIO_SetOutputHigh(rele_ports[2], rele_pins[2]);
    }
  }

  if (time_rele_flag[3]) {
    if (pgm.run_rele[3].function == PGM_PULSED) {
		if(pulsing_rl[3] == true){
		  new_state_rl[3] = !(pgm.run_rele[3].previous_state);
	      time_rele_flag[3] = false;
	      if (new_state_rl[3]) {
	        XMC_GPIO_SetOutputHigh(rele_ports[3], rele_pins[3]);
	      } else {
	        XMC_GPIO_SetOutputLow(rele_ports[3], rele_pins[3]);
	      }
	
	      pgm.run_rele[3].previous_state = new_state_rl[3];
		}else{
	  	  XMC_GPIO_SetOutputLow(rele_ports[3], rele_pins[3]);
	   }
    }

    if (pgm.run_rele[3].function == PGM_RETENTION) {
      time_rele_flag[3] = false;
      XMC_GPIO_SetOutputLow(rele_ports[3],
                              rele_pins[3]);
    }
    
    if (pgm.run_rele[3].function == PGM_DELAYED_TOGGLE) {
      time_rele_flag[3] = false;
      XMC_GPIO_SetOutputHigh(rele_ports[3], rele_pins[3]);
    }
  }
}

void rele_stop(uint8_t rele_index){
	time_rele_flag[rele_index] = false;
	pulsing_rl[rele_index] = false;
	rele_started[rele_index] = false;
	XMC_GPIO_SetOutputLow(rele_ports[rele_index], rele_pins[rele_index]);
}

void rele_start(uint8_t rele_index, uint8_t function, uint8_t state, uint16_t time){
	
	if(state == false){
		rele_stop(rele_index);
	}else{
		if(!rele_started[rele_index]){
			rele_started[rele_index] = true;
			
			pgm.run_rele[rele_index].function = function;
		    pgm.run_rele[rele_index].state = state;
		    pgm.run_rele[rele_index].time = time;
		
			// Ligar cada rele conforme solicitado
			if (pgm.run_rele[rele_index].function == PGM_TOGGLE) {
		      if(pgm.run_rele[rele_index].state == true){
				  XMC_GPIO_SetOutputHigh(rele_ports[rele_index], rele_pins[rele_index]);
			  }
		    }
		    
		    if (pgm.run_rele[rele_index].function == PGM_PULSED) {
		      pgm.run_rele[rele_index].previous_state = pgm.run_rele[rele_index].state;
		      cont_rele[rele_index] = pgm.run_rele[rele_index].time;
		      
		      if(pgm.run_rele[rele_index].state == true){
				  pulsing_rl[rele_index] = true;
			  }else{
				  pulsing_rl[rele_index] = false;
			  }
		    }
		    
		    if (pgm.run_rele[rele_index].function == PGM_RETENTION) {
			  XMC_GPIO_SetOutputHigh(rele_ports[rele_index], rele_pins[rele_index]);
		      pgm.run_rele[rele_index].previous_state = pgm.run_rele[rele_index].state;
		      cont_rele[rele_index] = (pgm.run_rele[rele_index].time);
		    }
		    
		    if(pgm.run_rele[rele_index].function == PGM_DELAYED_TOGGLE){
				pgm.run_rele[rele_index].previous_state = false;
		      	cont_rele[rele_index] = (pgm.run_rele[rele_index].time);
			}
		}
	}
	
	
	
}

// Rotina para receber dados
void USIC0_1_IRQHandler(void) {
  new_alarm_packet = true;
}

void USIC0_2_IRQHandler(void) {
  new_gate_packet = true;
}


// Máquina de estado_alarms
void Control_alarm() {

  switch (estado_alarm) {
  case RECEIVE: {

    XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
    if (alarm_packet_completed) {

      if (pgm.alarm_packet.function == PGM_REGISTER && !cadastrado) {
        estado_alarm = GET_UID;

      } else if (pgm.alarm_packet.function == PGM_RETRY_CRC && Rx_buffer[8] == NAK) {
        if (pgm.alarm_packet.id == PGM_ID && Rx_buffer[4] == UID0 && Rx_buffer[5] == UID1 && Rx_buffer[6] == UID2 && Rx_buffer[7] == UID3) {
          pgm.ligar_rele.Byte = 0x00;
          numero_modulo = 0;
          cadastrado = false;
          incremento = num_aleatorio;
          update_flash(incremento);
          crc_address = crc8((uint8_t *)UniqueChipID, 16, incremento);
          estado_alarm = RL_CONTROL;
        } else if (pgm.alarm_packet.id == PGM_BROADCAST_ID) {
          pgm.ligar_rele.Byte = 0x00;
          numero_modulo = 0;
          cadastrado = false;
          incremento = num_aleatorio;
          update_flash(incremento);
          crc_address = crc8((uint8_t *)UniqueChipID, 16, incremento);
          estado_alarm = RL_CONTROL;
        } else {
          // Se o ID não for válido, ignora o pacote
          pacote_obsoleto = true;
          estado_alarm = LIMPAR;
        }

      } else if (pgm.alarm_packet.function == PGM_STATUS && pgm.alarm_packet.address == crc_address) {
        cadastrado = true;
        numero_modulo = pgm.alarm_packet.data[0] + 1;
        estado_alarm = STATUS_RL;
        
      } else if (pgm.alarm_packet.function == PGM_TOGGLE || pgm.alarm_packet.function == PGM_PULSED || pgm.alarm_packet.function == PGM_RETENTION) {
		
        if (pgm.alarm_packet.id == PGM_BROADCAST_ID) {
          //          pgm.ligar_rele.Byte = pgm.alarm_packet.data;
          estado_alarm = RL_CONTROL;
        } else if (pgm.alarm_packet.id == PGM_ID && pgm.alarm_packet.address == crc_address) {

          rele_start(pgm.alarm_packet.data[0], pgm.alarm_packet.function, pgm.alarm_packet.data[1], (((uint16_t)pgm.alarm_packet.data[2] << 8) | pgm.alarm_packet.data[3]));
		  
          estado_alarm = RL_CONTROL;
        } else {
          pacote_obsoleto = true;
          estado_alarm = LIMPAR;
        }

      } else if (pgm.alarm_packet.function == PGM_DELETE) {
        if (pgm.alarm_packet.id == PGM_BROADCAST_ID) {
          pgm.ligar_rele.Byte = 0x00;
          cadastrado = false;
          pacote_obsoleto = true;
          estado_alarm = RL_CONTROL;
        } else if (pgm.alarm_packet.id == PGM_ID && pgm.alarm_packet.address == crc_address) {
          pgm.ligar_rele.Byte = 0x00;
          cadastrado = false;
          pacote_obsoleto = true;
          estado_alarm = RL_CONTROL;
        } else {
          pacote_obsoleto = true;
          estado_alarm = LIMPAR;
        }

      } else if(pgm.alarm_packet.function == PGM_GATE_STATUS && pgm.alarm_packet.address == crc_address){
		estado_alarm = GATE_STATUS;
	  } else if(pgm.alarm_packet.function == PGM_GATE_CMD && pgm.alarm_packet.address == crc_address){
		pgm.gate_info.action = pgm.alarm_packet.data[0];
		send_gate_cmd = true;
		
		buffer_size = 8;
    	montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, crc_address,
                  PGM_GATE_CMD, &ACK, 1, false);
    	estado_alarm = TRANSMIT;
	  } else{
        pacote_obsoleto = true;
        estado_alarm = LIMPAR;
      }
    }

  } break;

  case GET_UID: {
    // Enviar o UID do dispositivo
    buffer_size = 12;
    uint8_t data[5] = {UID0, UID1, UID2, UID3, crc_address};
    montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, 0x00, PGM_REGISTER,
                  data, 5, false);

    estado_alarm = TRANSMIT;

  } break;

  case STATUS_RL: {
    if (pgm.alarm_packet.address == crc_address) {
      // Enviar o status de cada rele
      buffer_size = 8;
      montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, crc_address,
                    PGM_STATUS, &pgm.status_rele.Byte, 1, false);

      delay_aleatorio = gerar_intervalo(UID0, UID1, UID2, UID3, systick);

      estado_alarm = TRANSMIT;
    } else {
      estado_alarm = LIMPAR;
    }

  } break;
  
  case GATE_STATUS: {
    // Enviar o UID do dispositivo
    if(gate_info_ready){
		gate_info_ready = false;
		
		buffer_size = 8;
    	
    	montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, crc_address, PGM_GATE_STATUS,
                  &pgm.gate_info.state, 1, false);

    	estado_alarm = TRANSMIT;
	}
    

  } break;

  case RL_CONTROL: {
    
    //    rele_Control();

    buffer_size = 8;
    montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, crc_address,
                  pgm.alarm_packet.function, &ACK, 1, false);
    estado_alarm = TRANSMIT;

  } break;

  case DELETE: {
    buffer_size = 8;
    montar_pacote(Buffer_TX, buffer_size, PGM_ID_RESPONSE, crc_address,
                  PGM_DELETE, &ACK, 1, false);
    estado_alarm = TRANSMIT;
  } break;

  case TRANSMIT: {

    if (!cadastrado) {
      delay_tx = systick + gerar_intervalo(UID0, UID1, UID2, UID3, systick);
    } else {
      delay_tx = systick + 15;
    }
    aguardando_envio = true;
    estado_alarm = DELAY_ENVIO;

  } break;

  case DELAY_ENVIO: {
    if (systick >= delay_tx) {
      switch_to_uart();
      XMC_Delay(1);
      XMC_GPIO_SetOutputLow(Bus_Controle_PORT, Bus_Controle_PIN);
      for (int i = 0; i < buffer_size; i++) {
        XMC_UART_CH_Transmit(UART_Bus_HW, Buffer_TX[i]);
      }
      while (!XMC_USIC_CH_TXFIFO_IsEmpty(UART_Bus_HW))
        ;

      XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);

      XMC_Delay(2);
      switch_to_gpio();

      pacote_obsoleto = true;
      aguardando_envio = false;
      estado_alarm = LIMPAR;
    }

  } break;

  case LIMPAR: {
    if (pacote_obsoleto) {
      for (uint8_t i = 0; i < sizeof(Buffer_TX); i++) {
        Buffer_TX[i] = 0;
      }
      alarm_packet_completed = false;
      pacote_obsoleto = false;
    }

    for (uint8_t i = 0; i < sizeof(Rx_buffer); i++) {
      Rx_buffer[i] = 0;
    }

    alarm_packet_completed = false;
    estado_alarm = RECEIVE;

  } break;
  }
}

void Control_gate() {
  
  switch (estado_gate) {
  case RECEIVE: {

    if (gate_packet_completed) {
		gate_packet_completed = false;
//      cadastrado = true;
//      numero_modulo = 1;
	  gate = true;
      gate_info_ready = true;
      
      if(pgm.gate_packet.id == 1 && pgm.gate_packet.len != 38){
		prog_connected = true;
	  }else if(pgm.gate_packet.id == 2){
		if(pgm.gate_packet.function == 'H'){
			pgm.gate_info.state = pgm.gate_packet.data[5];
			
			uint8_t preset = 1;
			bool luz = (pgm.gate_packet.data[7]) & 1;
			bool trava = (pgm.gate_packet.data[7] >> 1) & 1;
			bool abre = (pgm.gate_packet.data[7] >> 2) & 1;
			bool fecha = (pgm.gate_packet.data[7] >> 3) & 1;
			bool foto_fecha = (pgm.gate_packet.data[7] >> 4) & 1;
			bool foto_abre = (pgm.gate_packet.data[7] >> 5) & 1;
			bool reed_abre = !((pgm.gate_packet.data[7] >> 6) & 1);
			bool reed_fecha = !((pgm.gate_packet.data[7] >> 7) & 1);
			bool bot = (pgm.gate_packet.data[6]) & 1;
			
			if(preset == 0){
				// Status
				if(foto_fecha){
					XMC_GPIO_SetOutputHigh(rele_ports[0], rele_pins[0]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[0], rele_pins[0]);
				}
				
				if(foto_abre){
					XMC_GPIO_SetOutputHigh(rele_ports[1], rele_pins[1]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[1], rele_pins[1]);
				}
				
				if(reed_fecha){
					XMC_GPIO_SetOutputHigh(rele_ports[2], rele_pins[2]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[2], rele_pins[2]);
				}
				
				if(reed_abre){
					XMC_GPIO_SetOutputHigh(rele_ports[3], rele_pins[3]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[3], rele_pins[3]);
				}
			}
			
			if(preset == 1){
				//Acessórios
				if(pgm.gate_info.state == ABRINDO || pgm.gate_info.state == FECHANDO){
					XMC_GPIO_SetOutputHigh(rele_ports[0], rele_pins[0]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[0], rele_pins[0]);
				}
				
				if(pgm.gate_info.state == ABRINDO || pgm.gate_info.state == ABERTO){
					XMC_GPIO_SetOutputHigh(rele_ports[1], rele_pins[1]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[1], rele_pins[1]);
				}
				
				if(trava){
					XMC_GPIO_SetOutputHigh(rele_ports[2], rele_pins[2]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[2], rele_pins[2]);
				}
				
				if(luz){
					XMC_GPIO_SetOutputHigh(rele_ports[3], rele_pins[3]);
				}else{
					XMC_GPIO_SetOutputLow(rele_ports[3], rele_pins[3]);
				}
			}
			
			if(preset == 2){
				if(pgm.gate_info.state == ABERTO){
					rele_start(0, PGM_TOGGLE, true, 300);
				}else{
					rele_stop(0);
				}
				
				if(pgm.gate_info.state == ABRINDO){
					rele_start(1, PGM_TOGGLE, true, 300);
				}else{
					rele_stop(1);
				}
				
				if(pgm.gate_info.state == FECHADO){
					rele_start(2, PGM_TOGGLE, true, 300);
				}else{
					rele_stop(2);
				}
				
				if(pgm.gate_info.state == FECHANDO){
					rele_start(3, PGM_TOGGLE, true, 600);
				}else{
					rele_stop(3);
				}
			}
			
			if(preset == 3){
				if(!abre && !foto_fecha){
					rele_start(0, PGM_DELAYED_TOGGLE, true, 1000);
					rele_start(1, PGM_DELAYED_TOGGLE, true, 5000);
					rele_start(2, PGM_DELAYED_TOGGLE, true, 10000);
					rele_start(3, PGM_DELAYED_TOGGLE, true, 15000);
				}else{
					rele_stop(0);
					rele_stop(1);
					rele_stop(2);
					rele_stop(3);
				}
			}
			
		  }
	  }else if(pgm.gate_packet.id == 3){
		ppaon_connected = true;
	  }
      
	  
    }
    //REMOVIDO PARA TESTE: && !prog_connected
    if(send_packet == true && !ppaon_connected){
    	estado_gate = TRANSMIT;
	}
	
  } break;

  case TRANSMIT: {
	
	if(send_gate_cmd == true){
		send_gate_cmd = false;
		buffer_size = 10;
		uint8_t data[4] = {pgm.gate_info.action,0,0,0};
		montar_pacote(Buffer_TX, buffer_size, 0x02, 0x00,
                  'C', data, 4, true);
		
	}else{
		send_packet = false;
		buffer_size = 6;
		montar_pacote(Buffer_TX, buffer_size, 0x02, 0x00,
                  'H', 0X00, 0, true);
	}
	
    delay_tx = systick + 15;

    aguardando_envio = true;
    estado_gate = DELAY_ENVIO;

  } break;

  case DELAY_ENVIO: {
    if (systick >= delay_tx) {
	  switch_gate_to_uart();
      XMC_Delay(1);
      for (int i = 0; i < buffer_size; i++) {
        XMC_UART_CH_Transmit(UART_Prog_HW, Buffer_TX[i]);
      }
      while (!XMC_USIC_CH_TXFIFO_IsEmpty(UART_Prog_HW))
        ;
	
      XMC_Delay(2);
      switch_gate_to_gpio();

      pacote_obsoleto = true;
      aguardando_envio = false;
      estado_gate = LIMPAR;
    }

  } break;

  case LIMPAR: {
    if (pacote_obsoleto) {
      for (uint8_t i = 0; i < sizeof(Buffer_TX); i++) {
        Buffer_TX[i] = 0;
      }
      gate_packet_completed = false;
      pacote_obsoleto = false;
    }

    for (uint8_t i = 0; i < sizeof(Rx_buffer); i++) {
      Rx_buffer[i] = 0;
    }

    gate_packet_completed = false;
    estado_gate = RECEIVE;

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
  NVIC_EnableIRQ(USIC0_2_IRQn);
  NVIC_SetPriority(USIC0_2_IRQn, 2);

  XMC_UART_CH_Start(UART_Bus_HW);

  // Inverte a saída de dados da UART (nível lógico de TX)
  //  UART_Bus_HW->SCTR = (UART_Bus_HW->SCTR & ~(0X3 << 6)) | (0X1 << 6);
  XMC_UART_CH_EnableInputInversion(UART_Bus_HW,
                                   (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0);
  XMC_UART_CH_EnableInputInversion(UART_Prog_HW,
                                   (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0);
  switch_to_gpio();
  switch_gate_to_gpio();

  while (XMC_USIC_CH_TXFIFO_IsFull(UART_Bus_HW))
    ;

  if ((TAMANHO_BUFFER - Rx_buffer_index_alarm) < UART_Bus_RXFIFO_LIMIT) {
    XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(
        UART_Bus_HW, XMC_USIC_CH_FIFO_SIZE_8WORDS,
        (TAMANHO_BUFFER - Rx_buffer_index_alarm) - 1);
  }

  XMC_GPIO_SetOutputHigh(Bus_Controle_PORT, Bus_Controle_PIN);
  XMC_GPIO_SetOutputLow(LED_ST_PORT, LED_ST_PIN);

  get_UID();

  incremento = read_flash();

  crc_address = crc8((uint8_t *)UniqueChipID, 16, incremento);

  while (1) {
    rele_Control();
    receive_alarm_packet();
    Control_alarm();
    receive_gate_packet();
    Control_gate();
  }
}
