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

int
mac802154_message_pack(mac802154_message_t *msg, char *buf)
{


}



void
mac802154_message_unpack(mac802154_message_t *msg, const char *buf, int len)
{
	unsigned idx = 0;

	memcpy(&msg->frame_control, buf + idx, 2); idx += 2;
	memcpy(&msg->seq_num, buf + idx, 1); idx++;


	// If the PAN ID compression bit is set to one and both the source and destination addresses are present, the frame shall contain only the Destination PAN Identifier field, and the Source PAN Identifier field shall be assumed equal to that of the destination

	//  If the PAN ID compression bit is set to zero, then the PAN Identifier field shall be present if and only if the corresponding address is present

	// Decode destination pan
	//if ((msg->frame_control & MAC_DEST_ADDR) != MAC_DEST_ADDR_NONE && !(msg->frame_control & MAC_PAN_COMP)



	// Decode destination address
	if ((msg->frame_control & MAC_DEST_ADDR) == MAC_DEST_ADDR_SHORT) {
		memcpy(&msg->dst_addr, buf + idx, 2); idx += 2;

	} else if ((msg->frame_control & MAC_DEST_ADDR) == MAC_DEST_ADDR_LONG) {
		memcpy(&msg->dst_addr, buf + idx, 8); idx += 8;
	}

	// Decode source PAN
	if ((msg->frame_control & MAC_SRC_ADDR) != MAC_SRC_ADDR_NONE) {
		memcpy(&msg->src_pan, buf + idx, 2); idx += 2;
	}

	// Decode source address
	if ((msg->frame_control & MAC_SRC_ADDR) == MAC_SRC_ADDR_SHORT) {
		memcpy(&msg->src_addr, buf + idx, 2); idx += 2;

	} else if ((msg->frame_control & MAC_SRC_ADDR) == MAC_DEST_ADDR_LONG) {
		memcpy(&msg->src_addr, buf + idx, 8); idx += 8;
	}

	// Copy payload based on remaining length
	// ...


}
