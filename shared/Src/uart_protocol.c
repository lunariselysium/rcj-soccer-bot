


#include "uart_protocol.h"
#include <string.h>
#include <stdbool.h>


UART_HandleTypeDef *protocol_huart = NULL;

// Fresco side
static sensor_data_callback_t data_callback = NULL;
static uint8_t expected_sequence = 0;

static protocol_stats_t stats = {0};

// Mosaic
void mosaic_send_init(UART_HandleTypeDef *huart){
	protocol_huart = huart;
}

bool mosaic_send_sensors(const uint16_t* ir_data, const uint16_t* ultrasonic_data){
	if (!protocol_huart || !ir_data || !ultrasonic_data){
		return false;
	}

	static uint8_t sequence = 0;
	sensor_packet_t packet;
	HAL_StatusTypeDef status;

	memcpy(packet.ir_sensors, ir_data, sizeof(packet.ir_sensors));
	memcpy(packet.ultrasonics, ultrasonic_data, sizeof(packet.ultrasonics));

	packet.timestamp_ms = HAL_GetTick();
	packet.sequence_num = sequence++;

	packet.crc8 = calculate_crc8((uint8_t*)&packet, sizeof(packet)-1);

	// Create frame with header and footer
	uint8_t tx_buffer[FRAME_SIZE];
	tx_buffer[0] = HEADER_BYTE;
	memcpy(&tx_buffer[1], &packet, PACKET_SIZE);
	tx_buffer[FRAME_SIZE - 1] = FOOTER_BYTE;

	status = HAL_UART_Transmit(protocol_huart, tx_buffer, FRAME_SIZE, 100);
	return (status == HAL_OK);
}




// Fresco side
void fresco_receive_init(UART_HandleTypeDef *huart){
	protocol_huart = huart;
	expected_sequence = 0;
	memset(&stats, 0, sizeof(stats));
}

void fresco_set_callback(sensor_data_callback_t callback){
	data_callback = callback;
}

void fresco_poll(void){
	sensor_packet_t packet;
	if (fresco_check_for_data(0)){
		// Packet received in callback
	}
}

bool fresco_check_for_data(uint32_t timeout_ms){
	if (!protocol_huart) return false;

	uint8_t header;
	uint32_t start_time = HAL_GetTick();

	HAL_StatusTypeDef status = HAL_UART_Receive(protocol_huart, &header, 1, timeout_ms);
	if (status != HAL_OK){
		if (status == HAL_TIMEOUT){
			stats.timeouts++;
		}
		return false;
	}

	if (header != HEADER_BYTE){
		return false;
	}

	uint8_t rx_buffer[FRAME_SIZE - 1];
	status = HAL_UART_Receive(protocol_huart, rx_buffer, FRAME_SIZE, 50);
	if (status != HAL_OK){
		if (status == HAL_TIMEOUT){
					stats.timeouts++;
		}
		return false;
	}

	if (rx_buffer[FRAME_SIZE - 2] != FOOTER_BYTE){
		return false;
	}

	sensor_packet_t packet;
	memcpy(&packet, rx_buffer, PACKET_SIZE);

	if (!validate_packet(&packet)){
		stats.crc_errors++;
		return false;
	}

	uint8_t seq_diff = packet.sequence_num - expected_sequence;
	if (seq_diff > 1){
		stats.packets_lost += (seq_diff - 1);
	}
	expected_sequence = packet.sequence_num + 1;

	stats.packets_received++;
	uint32_t latency = HAL_GetTick() - packet.timestamp_ms;
	stats.avg_latency_ms = (stats.avg_latency_ms * 0.9f) + (latency * 0.1f);

	if (data_callback){
		data_callback(&packet);
	}

	return true;
}










uint8_t calculate_crc8(const uint8_t*data, size_t length){
	uint8_t crc = 0x00;

	while (length--){
		crc ^= *data++;
		for (int i=0; i<8; i++){
			if (crc & 0x80){
				crc = (crc << 1) ^ 0x07; // Polynomial x^8 + x^2 + x +1
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

bool validate_packet(const sensor_packet_t* packet){
	if (!packet) return false;

	uint8_t calculated_crc = calculate_crc8((uint8_t*)packet, sizeof(sensor_packet_t) - 1);
	return (calculated_crc == packet->crc8);
}

protocol_stats_t fresco_get_stats(void){
	return stats;
}

void fresco_reset_stats(void){
	memset(&stats, 0, sizeof(stats));
	expected_sequence = 0;
}








