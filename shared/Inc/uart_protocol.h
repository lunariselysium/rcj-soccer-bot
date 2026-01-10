


#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#pragma pack(push, 1) // Ensure no padding
typedef struct {
	uint16_t ir_sensors[16];
	uint16_t ultrasonics[4];
	uint32_t timestamp_ms;
	uint8_t sequence_num;
	uint8_t crc8;
} sensor_packet_t;
#pragma pack(pop)

typedef struct {
	uint32_t packets_received;
	uint32_t packets_lost;
	uint32_t crc_errors;
	uint32_t timeouts;
	float avg_latency_ms;
} protocol_stats_t;


#define HEADER_BYTE 0xAA
#define FOOTER_BYTE 0x55
#define PACKET_SIZE sizeof(sensor_packet_t)
#define FRAME_SIZE (PACKET_SIZE + 2)

extern UART_HandleTypeDef *protocol_huart;

typedef void (*sensor_data_callback_t)(const sensor_packet_t* packet);


// Mosaic side
void mosaic_send_init(UART_HandleTypeDef *huart);
bool mosaic_send_sensors(const uint16_t* ir_data, const uint16_t* ultrasonic_data);


// Fresco side
void fresco_receive_init(UART_HandleTypeDef *huart);
void fresco_set_callback(sensor_data_callback_t callback);
void fresco_poll(void);
bool fresco_check_for_data(uint32_t timeout_ms);



// Utils
uint8_t calculate_crc8(const uint8_t*data, size_t length);
bool validate_packet(const sensor_packet_t* packet);

protocol_stats_t fresco_get_stats(void);
void fresco_reset_stats(void);
