/*
 * main.h
 *
 *  Created on: 22 de ago. de 2025
 *      Author: joao.victor
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "XMC1100.h"
//#include "cy_syslib.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "xmc1_gpio.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"
#include "xmc_usic.h"
#include <stdint.h>
#include <xmc_flash.h>
//   0x10008F00U
#define FLASH_SECTOR_ADDR (uint32_t *)0x10008F00

#define SIZE 0
#define IDT 1
#define UNIQUEID 2
#define FUNCTION 3
#define DATA 4
#define CHECKSUM 5

#define RECEIVE 0
#define GET_UID 1
#define STATUS_PGM 2
#define GATE_STATUS 3
#define RL_CONTROL 4
#define DELETE 5
#define TRANSMIT 6
#define DELAY_ENVIO 7
#define LIMPAR 8

#define PGM_PACKET_HEADER_LEN 1
#define PGM_PACKET_LENGTH_LEN 1
#define PGM_PACKET_ID_LEN 1
#define PGM_PACKET_ADDRESS_LEN 1
#define PGM_PACKET_FUNCTION_LEN 1
#define PGM_PACKET_TAIL_LEN 1
#define PGM_PACKET_CHECKSUM_LEN 1

#define TAMANHO_BUFFER 8
#define TAMANHO_BUFFER_ACK 48

uint8_t ACK = 0x06;
uint8_t NAK = 0x06;
uint8_t incremento = 0;
uint8_t buffer_size = 8;
uint8_t package_size = 0;
// Buffer de envio e recepção de dados
uint8_t Rx_buffer[TAMANHO_BUFFER_ACK];
uint8_t Buffer_TX[TAMANHO_BUFFER_ACK] = {0};
uint8_t Rx_buffer_index_alarm = 0;
uint8_t Rx_buffer_index_gate = 0;

uint8_t Rx_buffer_gate[40];
uint8_t Tx_buffer_gate[40];
uint8_t Rx_buffer_prog[40];
uint8_t Tx_buffer_prog[40];

bool click_right = false;
bool click_left = false;
bool click_add = false;
bool click_subtract = false;
bool click_ok = false;

typedef enum {
  PGM_ID = 0x01,
  PGM_BROADCAST_ID = 0x02,
  PGM_ID_RESPONSE = 0x03,
} PGM_DEVICE_ID_t;

typedef enum {
  PGM_REGISTER = 0x00,
  PGM_TOGGLE = 0x01,
  PGM_DELAYED_TOGGLE= 0x02,
  PGM_PULSED = 0x03,
  PGM_RETENTION = 0x04,
  PGM_STATUS = 0x05,
  PGM_DELETE = 0x06,
  PGM_RETRY_CRC = 0x07,
  PGM_GATE_STATUS = 0x08,
  PGM_GATE_CMD = 0x09,
} PGM_FUNCTION_t;

typedef enum{
	On_Off,
	Delayed_Toggle,
	Pulsed,
	Retention,
} rele_mode_t;

typedef enum{
	LUZ,
	TRAVA,
	BOT_ABRE,
	BOT_FECHA,
	BOT,
	FOT_FEC,
	FOT_ABR,
	REED_FEC,
	REED_ABR,	
	PORTAO_ABERTO,
	PORTAO_ABRINDO,
	PORTAO_FECHADO,
	PORTAO_FECHANDO,
} rele_trigger_t;

typedef enum {
  PGM_PACKET_OK,
  PGM_PACKET_FAIL_HEADER,
  PGM_PACKET_FAIL_TAIL,
  PGM_PACKET_FAIL_LENGHT,
  PGM_PACKET_FAIL_CHECKSUM,
  PGM_PACKET_FAIL_UNKNOWN = 0xFF
} pgm_packet_error_e;

typedef enum{
	ABERTO = 1,
	ABRINDO,
	FECHADO,
	FECHANDO,
	SEMIABERTO,
	TRAVADO,
	LENDO_ABRE,
	LENDO_FECHA,
	INICIAL,
} gate_status_t;

typedef enum{
	ABRIR = 1,
	FECHAR,
	TRAVAR,
	PARAR,
	ACIONARMOTOR = 12,
} gate_cmd_t;

typedef struct{
  uint8_t state;
  uint8_t action;
} pgm_gate_info_t;

typedef struct {
  bool state;
  bool previous_state;
  uint8_t function;
  uint16_t time;
} pgm_timebase_t;

typedef struct {
  rele_mode_t mode;
  uint16_t time;
  rele_trigger_t trigger;
  bool state;
} pgm_gate_config_t;

typedef struct {
  uint8_t len;
  uint8_t id;
  uint8_t address;
  uint8_t function;
  uint8_t data[5];
  uint8_t checksum;
  uint8_t tail;
} pgm_alarm_packet_t;

typedef struct {
  uint8_t len;
  uint8_t id;
  uint8_t function;
  uint8_t data[40];
  uint8_t checksum;
  uint8_t tail;
} pgm_connect_packet_t;

uint32_t UniqueChipID[4] = {0, 0, 0, 0};
uint8_t crc_address = 0;

// Flags de envio e recepção de dados
volatile bool alarm_packet_completed = false;
volatile bool gate_packet_completed = false;
volatile bool prog_packet_completed = false;
volatile bool boot_completed = false;
volatile bool recebendo = false;
volatile bool pacote_obsoleto = false;
volatile bool cadastrado = false;
volatile bool gate = false;
volatile bool prog_connected = false;
volatile bool ppaon_connected = false;
volatile bool channel_free = true;
volatile bool currently_sending = false;
bool new_alarm_packet = false;
bool new_gate_packet = false;
bool new_prog_packet = false;
volatile bool check_registration = false;

volatile bool uart_swaped = false;

// Variáveis do pacote
#define start_byte 0x7E
uint8_t UID0 = 0;
uint8_t UID1 = 0;
uint8_t UID2 = 0;
uint8_t UID3 = 0;
uint8_t checksum = 0;
#define stop_byte 0x81

uint8_t checksum_validate = 0;
volatile bool checksum_ok = false;

// Máquina de estado_alarms
uint8_t estado_alarm = 0;
uint8_t estado_gate = TRANSMIT;
uint8_t estado_prog_tx = 0;
uint8_t estado_prog_rx = 0;

// Variáveis de tempo/delays
uint16_t gate_packet_delay = 3000;
uint16_t prog_packet_delay = 5000;
uint16_t check_registration_timeout = 10000;
uint8_t num_aleatorio = 200;
uint16_t delay_aleatorio = 0;
uint16_t Blinking_gap = 10;
uint16_t Blink_wait = 1000;
uint8_t piscadas = 0;
uint8_t numero_modulo = 0;
volatile uint32_t systick = 0;
volatile uint32_t last_packet = 0;
volatile uint32_t delay_tx = 0;
volatile bool aguardando_envio = false;
volatile bool piscando = false;
uint16_t cont_rele[4] = {0, 0, 0, 0};
bool time_rele_flag[4] = {0, 0, 0, 0};
bool rele_started[4];

bool new_state_rl[4];
bool pulsing_rl[4];
bool send_packet;
bool send_prog_packet;
bool gate_info_ready;
bool send_gate_cmd;

bool cmd_alarm;
bool cmd_botoeira;
bool cmd_abre;
bool cmd_fecha;

bool LG_state;
bool TRAVA_state;
bool BOT_ABRE_state;
bool BOT_FECHA_state;
bool FOT_FEC_state;
bool FOT_ABR_state;
bool REED_FEC_state;
bool REED_ABR_state;
bool BOT_state;
bool ABERTO_state;
bool ABRINDO_state;
bool FECHADO_state;
bool FECHANDO_state;

bool save_to_memory;

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
} FLAG_Bits;


// União bitflag
typedef union {
  uint8_t Byte;
  struct {
    bool OK : 1;
    bool MAIS : 1;
    bool MENOS : 1;
    bool DIREITA : 1;
    bool ESQUERDA : 1;
    bool reservado_1 : 1;
    bool reservado_2 : 1;
    bool reservado_3 : 1;
  } Bits;
} Button_Bits;

typedef struct {
  XMC_GPIO_PORT_t *port;
  uint8_t pin;
  uint8_t blink_count;
  uint8_t blink_target;
  bool piscando;
} LED_t;

LED_t leds[1] = {{LED_ST_PORT, LED_ST_PIN, 0, 0, false}};

XMC_GPIO_PORT_t *const rele_ports[5] = {RL1_PORT, RL2_PORT, RL3_PORT};

const uint8_t rele_pins[5] = {RL1_PIN, RL2_PIN, RL3_PIN};

typedef struct {
  pgm_packet_error_e pgm_error;
  pgm_alarm_packet_t alarm_packet;
  pgm_connect_packet_t connect_packet;
  Button_Bits buttons;
  bool gate_inputs[13];
  pgm_gate_config_t gate_rele_config[3];
  FLAG_Bits status_rele, ligar_rele;
  pgm_timebase_t run_rele[4];
  pgm_gate_info_t gate_info;
} pgm_t;


pgm_t pgm;

#endif /* MAIN_H_ */
