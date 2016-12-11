/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>



// Fields present in the frame_control header
#define MAC_TYPE 0b111
	#define MAC_TYPE_BEACON 0b000
	#define MAC_TYPE_DATA 0b001
	#define MAC_TYPE_ACK 0b010
	#define MAC_TYPE_CMD 0b011
#define MAC_SEC_ENBL (1 << 3)
#define MAC_PENDING (1 << 4)
#define MAC_REQ_ACK (1 << 5)
#define MAC_PAN_COMP (1 << 6)
#define MAC_DEST_ADDR (0b11 << 10)
	#define MAC_DEST_ADDR_NONE (0 << 10)
	#define MAC_DEST_ADDR_SHORT (0b10 << 10)
	#define MAC_DEST_ADDR_LONG (0b11 << 10)
#define MAC_VERSION (0b11 << 12)
	#define MAC_VERSION_2003 (0b00 << 12)
	#define MAC_VERSION_2011 (0b01 << 12)
#define MAC_SRC_ADDR (0b11 << 14)
	#define MAC_SRC_ADDR_NONE (0 << 14)
	#define MAC_SRC_ADDR_SHORT (0b10 << 14)
	#define MAC_SRC_ADDR_LONG (0b11 << 14)


typedef struct {
	// Header
	uint16_t frame_control;
	uint8_t seq_num;

	uint16_t dst_pan;
	uint64_t dst_addr;

	uint16_t src_pan;
	uint64_t src_addr;

	int size; // Size of the payload
	char payload[127]; // 1027 for non-standard long DW frames

} mac802154_message_t;


int mac802154_message_pack(mac802154_message_t *msg, char *buf);
void mac802154_message_unpack(mac802154_message_t *msg, const char *buf, int len);
