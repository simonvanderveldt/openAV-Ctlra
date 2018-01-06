/*
 * Copyright (c) 2017, OpenAV Productions,
 * Harry van Haaren <harryhaaren@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "impl.h"
#include "midi.h"

#define CONTROLS_SIZE 512
#define LIGHTS_SIZE 512

#define CTLRA_DRIVER_VENDOR 0x2982
#define CTLRA_DRIVER_DEVICE 0x1967

static const char *ableton_push2_button_names[] = {
	"tap tempo",
	"metronome",
	"first unlabelled button below the screen (from the left)",
	"second unlabelled button below the screen",
	"third unlabelled button below the screen",
	"fourth unlabelled button below the screen",
	"fifth unlabelled button below the screen",
	"sixth unlabelled button below the screen",
	"seventh unlabelled button below the screen",
	"eighth unlabelled button below the screen",
	"master",
	"stop clip",
	"setup",
	"layout",
	"convert",
	"1/4",
	"1/4t",
	"1/8",
	"1/8t",
	"1/16",
	"1/16t",
	"1/32",
	"1/32t",
	"left",
	"right",
	"up",
	"down",
	"select",
	"shift",
	"note",
	"session",
	"add device",
	"add track",
	"octave down",
	"octave up",
	"repeat",
	"accent",
	"scale",
	"user",
	"mute",
	"solo",
	"previous page",
	"next page",
	"play",
	"record",
	"new",
	"duplicate",
	"automate",
	"fixed length",
	"first unlabelled button above the screen (from the left)",
	"second unlabelled button above the screen",
	"third unlabelled button above the screen",
	"fourth unlabelled button above the screen",
	"fifth unlabelled button above the screen",
	"sixth unlabelled button above the screen",
	"seventh unlabelled button above the screen",
	"eighth unlabelled button above the screen",
	"device",
	"browse",
	"mix",
	"clip",
	"quantize",
	"double loop",
	"delete",
	"undo",
	"third encoder touch",
	"fourth encoder touch",
	"fifth encoder touch",
	"sixth encoder touch",
	"sevent encoder touch",
	"eighth encoder touch",
	"ninth encoder touch",
	"tenth encoder touch",
	"eleventh encoder touch",
	"second encoder touch",
	"first encoder touch",
};
#define CONTROL_NAMES_BUTTONS_SIZE (sizeof(ableton_push2_button_names) /\
				    sizeof(ableton_push2_button_names[0]))

static const char *ableton_push2_encoder_names[] = {
	"first encoder (from the left, dented)",
	"second encoder",
	"third encoder",
	"fourth encoder",
	"fifth encoder",
	"sixth encoder",
	"seventh encoder",
	"eighth encoder",
	"ninth encoder",
	"tenth encoder",
	"eleventh encoder",
};
#define CONTROL_NAMES_ENCODERS_SIZE (sizeof(ableton_push2_encoder_names) /\
				    sizeof(ableton_push2_encoder_names[0]))


static const char *ableton_push2_control_get_name(enum ctlra_event_type_t type,
			       uint32_t control)
{
	switch(type) {
	// case CTLRA_EVENT_SLIDER:
	// 	if(control >= CONTROL_NAMES_SLIDERS_SIZE)
	// 		return 0;
	// 	return ni_kontrol_z1_names_sliders[control];
	case CTLRA_EVENT_BUTTON:
		if(control >= CONTROL_NAMES_BUTTONS_SIZE)
			return 0;
		return ableton_push2_button_names[control];
	case CTLRA_EVENT_ENCODER:
		if(control >= CONTROL_NAMES_ENCODERS_SIZE)
			return 0;
		return ableton_push2_encoder_names[control];
	// case CTLRA_FEEDBACK_ITEM:
	// 	if(control >= CONTROL_NAMES_FEEDBACK_SIZE)
	// 		return 0;
	// 	return ni_kontrol_z1_names_feedback[control];
	default:
		break;
	}
	return 0;
}

/* Represents the the hardware device */
struct ableton_push2_t {
	/* base handles usb i/o etc */
	struct ctlra_dev_t base;
	/* midi i/o */
	struct ctlra_midi_t *midi;
	int16_t cc_to_btn_id[127];
	int16_t cc_to_enc_id[127];
	int16_t note_to_btn_id[127];
};

static uint32_t
ableton_push2_poll(struct ctlra_dev_t *base)
{
	struct ableton_push2_t *dev = (struct ableton_push2_t *)base;
	ctlra_midi_input_poll(dev->midi);
	return 0;
}

int ableton_push2_midi_input_cb(uint8_t nbytes, uint8_t * buf, void *ud)
{
	struct ableton_push2_t *dev = (struct ableton_push2_t *)ud;
	printf("%x", buf[0]);
	printf("%x", buf[1]);
	printf("%x", buf[2]);

	switch(buf[0] & 0xf0) {
		case 0x90: /* Note On */
		case 0x80: /* Note Off */ {
			int id = dev->note_to_btn_id[buf[1]];
			struct ctlra_event_t event = {
				.type = CTLRA_EVENT_BUTTON,
				.button  = {
					.id = id,
					.pressed = buf[0] >= 0x90,
				},
			};
			struct ctlra_event_t *e = {&event};
			dev->base.event_func(&dev->base, 1, &e,
						dev->base.event_func_userdata);
		} break;

		case 0xb0: /* control change */ {
			int id = dev->cc_to_btn_id[buf[1]];
			if(id == -1) {
				id = dev->cc_to_enc_id[buf[1]];
				int clock = buf[2];
				int anticlock = 127 - buf[2];
				float delta = (buf[2] > 64 ? anticlock : clock)  / 210.f;
				struct ctlra_event_t event = {
					.type = CTLRA_EVENT_ENCODER,
					.encoder = {
						.id = id,
						.flags = CTLRA_EVENT_ENCODER_FLAG_FLOAT,
						.delta_float = delta,
					},
				};
				struct ctlra_event_t *e = {&event};
				dev->base.event_func(&dev->base, 1, &e,
					dev->base.event_func_userdata);
			} else {
				struct ctlra_event_t event = {
					.type = CTLRA_EVENT_BUTTON,
					.button = {
						.id = id,
						.pressed = buf[2] > 0
					},
				};
				struct ctlra_event_t *e = {&event};
				dev->base.event_func(&dev->base, 1, &e,
					dev->base.event_func_userdata);
			}
		} break;
	};

	return 0;
}

static void ableton_push2_light_set(struct ctlra_dev_t *base,
				    uint32_t light_id,
				    uint32_t light_status)
{
	struct ableton_push2_t *dev = (struct ableton_push2_t *)base;

	uint8_t out[3];

	if(light_id == -1) {
		out[2] = (light_status >> 16) & 0xff;
		out[1] = (light_status >>  8) & 0xff;
		out[0] = (light_status >>  0) & 0xff;
	} else {
		uint8_t b3 = 0;
		b3 |= light_status >> 24;
		b3 |= light_status >> 16;
		b3 |= light_status >>  8;

		out[0] = 0x90;
		out[1] = light_id;
		out[2] = b3;
	}

	ctlra_midi_output_write(dev->midi, 3, out);
}

void
ableton_push2_light_flush(struct ctlra_dev_t *base, uint32_t force)
{
	struct ableton_push2_t *dev = (struct ableton_push2_t *)base;
}

static int32_t
ableton_push2_disconnect(struct ctlra_dev_t *base)
{
	struct ableton_push2_t *dev = (struct ableton_push2_t *)base;
	/* Cleanup the MIDI I/O ports */
	ctlra_midi_destroy(dev->midi);
	free(dev);
	return 0;
}

struct ctlra_dev_info_t ctlra_ableton_push2_info;

struct ctlra_dev_t *
ctlra_ableton_push2_connect(ctlra_event_func event_func, void *userdata,
			   void *future)
{
	(void)future;
	struct ableton_push2_t *dev = calloc(1, sizeof(struct ableton_push2_t));
	if(!dev)
		goto fail;

	for(int i = 0; i < 127; i++) {
	   dev->cc_to_btn_id[i] = -1;
	   dev->cc_to_enc_id[i] = -1;
		 dev->note_to_btn_id[i] = -1;
	}

	int counter = 0;
  // MIDI cc buttons
	dev->cc_to_btn_id[3] = counter++; // tap tempo
	dev->cc_to_btn_id[9] = counter++; // metronome
	dev->cc_to_btn_id[20] = counter++; // first unlabelled button below the screen (from the left)
	dev->cc_to_btn_id[21] = counter++; // second unlabelled button below the screen
	dev->cc_to_btn_id[22] = counter++; // third unlabelled button below the screen
	dev->cc_to_btn_id[23] = counter++; // fourth unlabelled button below the screen
	dev->cc_to_btn_id[24] = counter++; // fifth unlabelled button below the screen
	dev->cc_to_btn_id[25] = counter++; // sixth unlabelled button below the screen
	dev->cc_to_btn_id[26] = counter++; // seventh unlabelled button below the screen
	dev->cc_to_btn_id[27] = counter++; // eighth unlabelled button below the screen
	dev->cc_to_btn_id[28] = counter++; // master
	dev->cc_to_btn_id[29] = counter++; // stop clip
	dev->cc_to_btn_id[30] = counter++; // setup
	dev->cc_to_btn_id[31] = counter++; // layout
	dev->cc_to_btn_id[35] = counter++; // convert
	dev->cc_to_btn_id[36] = counter++; // 1/4
	dev->cc_to_btn_id[37] = counter++; // 1/4t
	dev->cc_to_btn_id[38] = counter++; // 1/8
	dev->cc_to_btn_id[39] = counter++; // 1/8t
	dev->cc_to_btn_id[40] = counter++; // 1/16
	dev->cc_to_btn_id[41] = counter++; // 1/16t
	dev->cc_to_btn_id[42] = counter++; // 1/32
	dev->cc_to_btn_id[43] = counter++; // 1/32t
	dev->cc_to_btn_id[44] = counter++; // left
	dev->cc_to_btn_id[45] = counter++; // right
	dev->cc_to_btn_id[46] = counter++; // up
	dev->cc_to_btn_id[47] = counter++; // down
	dev->cc_to_btn_id[48] = counter++; // select
	dev->cc_to_btn_id[49] = counter++; // shift
	dev->cc_to_btn_id[50] = counter++; // note
	dev->cc_to_btn_id[51] = counter++; // session
	dev->cc_to_btn_id[52] = counter++; // add device
	dev->cc_to_btn_id[53] = counter++; // add track
	dev->cc_to_btn_id[54] = counter++; // octave down
	dev->cc_to_btn_id[55] = counter++; // octave up
	dev->cc_to_btn_id[56] = counter++; // repeat
	dev->cc_to_btn_id[57] = counter++; // accent
	dev->cc_to_btn_id[58] = counter++; // scale
	dev->cc_to_btn_id[59] = counter++; // user
	dev->cc_to_btn_id[60] = counter++; // mute
	dev->cc_to_btn_id[61] = counter++; // solo
	dev->cc_to_btn_id[62] = counter++; // previous page
	dev->cc_to_btn_id[63] = counter++; // next page
	dev->cc_to_btn_id[85] = counter++; // play
	dev->cc_to_btn_id[86] = counter++; // record
	dev->cc_to_btn_id[87] = counter++; // new
	dev->cc_to_btn_id[88] = counter++; // duplicate
	dev->cc_to_btn_id[89] = counter++; // automate
	dev->cc_to_btn_id[90] = counter++; // fixed length
	dev->cc_to_btn_id[102] = counter++; // first unlabelled button above the screen (from the left)
	dev->cc_to_btn_id[103] = counter++; // second unlabelled button above the screen
	dev->cc_to_btn_id[104] = counter++; // third unlabelled button above the screen
	dev->cc_to_btn_id[105] = counter++; // fourth unlabelled button above the screen
	dev->cc_to_btn_id[106] = counter++; // fifth unlabelled button above the screen
	dev->cc_to_btn_id[107] = counter++; // sixth unlabelled button above the screen
	dev->cc_to_btn_id[108] = counter++; // seventh unlabelled button above the screen
	dev->cc_to_btn_id[109] = counter++; // eighth unlabelled button above the screen
	dev->cc_to_btn_id[110] = counter++; // device
	dev->cc_to_btn_id[111] = counter++; // browse
	dev->cc_to_btn_id[112] = counter++; // mix
	dev->cc_to_btn_id[113] = counter++; // clip
	dev->cc_to_btn_id[116] = counter++; // quantize
	dev->cc_to_btn_id[117] = counter++; // double loop
	dev->cc_to_btn_id[118] = counter++; // delete
	dev->cc_to_btn_id[119] = counter++; // undo
  // MIDI note buttons
	dev->note_to_btn_id[0] = counter++; // third encoder touch
	dev->note_to_btn_id[1] = counter++; // fourth encoder touch
	dev->note_to_btn_id[2] = counter++; // fifth encoder touch
	dev->note_to_btn_id[3] = counter++; // sixth encoder touch
	dev->note_to_btn_id[4] = counter++; // sevent encoder touch
	dev->note_to_btn_id[5] = counter++; // eighth encoder touch
	dev->note_to_btn_id[6] = counter++; // ninth encoder touch
	dev->note_to_btn_id[7] = counter++; // tenth encoder touch
	dev->note_to_btn_id[8] = counter++; // eleventh encoder touch
	dev->note_to_btn_id[9] = counter++; // second encoder touch
	dev->note_to_btn_id[10] = counter++; // first encoder touch

	counter = 0;
	dev->cc_to_enc_id[14] = counter++; // first encoder (from the left, dented)
	dev->cc_to_enc_id[15] = counter++; // second encoder
	dev->cc_to_enc_id[71] = counter++; // third encoder
	dev->cc_to_enc_id[72] = counter++; // fourth encoder
	dev->cc_to_enc_id[73] = counter++; // fifth encoder
	dev->cc_to_enc_id[74] = counter++; // sixth encoder
	dev->cc_to_enc_id[75] = counter++; // seventh encoder
	dev->cc_to_enc_id[76] = counter++; // eighth encoder
	dev->cc_to_enc_id[77] = counter++; // ninth encoder
	dev->cc_to_enc_id[78] = counter++; // tenth encoder
	dev->cc_to_enc_id[79] = counter++; // eleventh encoder

	dev->midi = ctlra_midi_open("Ctlra PUSH2",
				    ableton_push2_midi_input_cb, dev);
	if(dev->midi == 0) {
		printf("Ctlra: error opening midi i/o\n");
		goto fail;
	}

	dev->base.info = ctlra_ableton_push2_info;

	dev->base.poll = ableton_push2_poll;
	dev->base.disconnect = ableton_push2_disconnect;
	dev->base.light_set = ableton_push2_light_set;
	dev->base.light_flush = ableton_push2_light_flush;

	dev->base.event_func = event_func;
	dev->base.event_func_userdata = userdata;

	return (struct ctlra_dev_t *)dev;
fail:
	free(dev);
	return NULL;
}

struct ctlra_dev_info_t ctlra_ableton_push2_info = {
	.vendor    = "Ableton",
	.device    = "Push 2",
	.vendor_id = CTLRA_DRIVER_VENDOR,
	.device_id = CTLRA_DRIVER_DEVICE,
	.size_x    = 512,
	.size_y    = 512,
	.get_name =  ableton_push2_control_get_name,
};

CTLRA_DEVICE_REGISTER(ableton_push2)
