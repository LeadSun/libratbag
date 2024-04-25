/*
 * Copyright © 2024 Red Hat, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "config.h"

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver-lamzu.h"
#include "libratbag-data.h"

#include "libratbag-private.h"
#include "libratbag-hidraw.h"

#define LAMZU_REPORT_ID 0x08
#define LAMZU_REPORT_SIZE 17
#define LAMZU_CRC_INIT 0x55
#define LAMZU_CMD_PARAMS_MAX_SIZE 10

#define LAMZU_CMD_SET_SETTING 0x07
#define LAMZU_CMD_GET_SETTING 0x08
#define LAMZU_CMD_GET_ACTIVE_PROFILE 0x0e
#define LAMZU_CMD_SET_ACTIVE_PROFILE 0x0f

/* None of these type flags are set for modifier keys */
#define LAMZU_KEY_FLAG_TYPE_MASK 0x0f
#define LAMZU_KEY_FLAG_HID AS_MASK(0)
#define LAMZU_KEY_FLAG_HID_CC AS_MASK(1)
#define LAMZU_KEY_FLAG_DIRECTION AS_MASK(2)

#define LAMZU_KEY_FLAG_RELEASE AS_MASK(6)
#define LAMZU_KEY_FLAG_PRESS AS_MASK(7)

#define LAMZU_SETTING_COMBOS_OFFSET 0x0100
#define LAMZU_SETTING_MACROS_OFFSET 0x0300


#define LAMZU_MACRO_MAX_KEYS 70

static unsigned int LAMZU_POLLING_RATES[] = { 125, 250, 500, 1000 };

/* Used for both requests and responses */
struct _lamzu_message {
	uint8_t report_id;
	uint8_t cmd;
	uint8_t padding;

	/* Big endian offset / setting ID only used for setting cmds */
	uint8_t offset[2];

	uint8_t params_length;
	uint8_t params[LAMZU_CMD_PARAMS_MAX_SIZE];
	uint8_t crc;
} __attribute__((packed));
_Static_assert(sizeof(struct _lamzu_message) == LAMZU_REPORT_SIZE, "The size of `_lamzu_message` is wrong.");

union lamzu_message {
	struct _lamzu_message data;
	uint8_t raw[LAMZU_REPORT_SIZE];
};

enum lamzu_action_type {
	LAMZU_ACTION_TYPE_DISABLED = 0x00,
	LAMZU_ACTION_TYPE_BUTTON = 0x01,
	LAMZU_ACTION_TYPE_DPI = 0x02,
	LAMZU_ACTION_TYPE_WHEEL_LR = 0x03,

	/* Parameters: interval (10-255), repeat (0-3) */
	LAMZU_ACTION_TYPE_FIRE = 0x04,

	/*
	 * Used for both single media keys and combo keys. Pretty much a short macro
	 * (up to 6 key events) without delays.
	*/
	LAMZU_ACTION_TYPE_COMBO = 0x05,

	/* Parameters: macro index (0-15), 3 */
	LAMZU_ACTION_TYPE_MACRO = 0x06,

	LAMZU_ACTION_TYPE_POLL_RATE = 0x07,

	/* Parameters: dpi value (0-23), 0 */
	LAMZU_ACTION_TYPE_DPI_LOCK = 0x0a, 

	LAMZU_ACTION_TYPE_WHEEL_UD = 0x0b,
} __attribute__((packed));

struct lamzu_dpi {
	uint8_t dpi_x;
	uint8_t dpi_y;
	uint8_t zero;
	uint8_t crc;
} __attribute__((packed));

struct lamzu_color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t crc;
} __attribute__((packed));

struct lamzu_action {
	uint8_t type;
	uint8_t params[2];
	uint8_t crc;
} __attribute__((packed));

struct lamzu_key {
	uint8_t flags;
	uint8_t data[2]; /* Little endian */
} __attribute__((packed));

struct lamzu_macro_key {
	struct lamzu_key key;
	uint8_t delay_ms[2]; /* Big endian */
} __attribute__((packed));

struct lamzu_macro {
	uint8_t name_length;
	char name[30];
	uint8_t keys_length;
	struct lamzu_macro_key keys[LAMZU_MACRO_MAX_KEYS];
	uint8_t crc;
	uint8_t padding;
} __attribute__((packed));
_Static_assert(sizeof(struct lamzu_macro) == 384, "The size of `lamzu_macro` is wrong.");

struct lamzu_combo {
	uint8_t keys_length;
	struct lamzu_key keys[6];
	uint8_t crc;
	uint8_t padding[12];
};
_Static_assert(sizeof(struct lamzu_combo) == 32, "The size of `lamzu_combo` is wrong.");

struct lamzu_settings {
  uint8_t report_rate;
  uint8_t report_rate_crc;
  uint8_t dpi_preset_count;
  uint8_t dpi_preset_count_crc;
  uint8_t current_dpi_preset;
  uint8_t current_dpi_preset_crc;
  uint8_t unknown_setting1[2];
  uint8_t unknown_setting2[2];
  uint8_t lift_off_distance;
  uint8_t lift_off_distance_crc;
	struct lamzu_dpi dpi_presets[LAMZU_NUM_DPI];
	struct lamzu_color dpi_colors[LAMZU_NUM_DPI];
  uint8_t unknown_setting3[2];
  uint8_t unknown_setting4[2];
  uint8_t unknown_setting5[2];
  uint8_t unknown_setting6[2];
  uint8_t unknown_setting7[4]; /* Possibly the charging light color */
  uint8_t unknown_setting8[2];
  uint8_t unknown_setting9[2];
  uint8_t unknown_setting10[2];
  uint8_t unknown_setting11[2];
	struct lamzu_action button_actions[LAMZU_BUTTON_MAX];
  uint8_t unknown_setting12[7];
  uint8_t unknown_setting13[2];
  uint8_t debounce_ms;
  uint8_t debounce_ms_crc;
  uint8_t motion_sync;
  uint8_t motion_sync_crc;
  uint8_t unknown_setting14[2];
  uint8_t angle_snapping;
  uint8_t angle_snapping_crc;
  uint8_t ripple_control;
  uint8_t ripple_control_crc;
  uint8_t unknown_setting15[2];
  uint8_t peak_performance;
  uint8_t peak_performance_crc;
  uint8_t peak_performance_time;
  uint8_t peak_performance_time_crc;
  uint8_t mode_select;
  uint8_t mode_select_crc;
	uint8_t unknown_setting16[2];
	uint8_t padding[3];
} __attribute__((packed));
_Static_assert(sizeof(struct lamzu_settings) == 192, "The size of `lamzu_settings` is wrong.");

struct lamzu_data {
  struct lamzu_settings settings[LAMZU_PROFILE_MAX];
	struct lamzu_combo combos[LAMZU_PROFILE_MAX][LAMZU_BUTTON_MAX];
	struct lamzu_macro macros[LAMZU_PROFILE_MAX][LAMZU_BUTTON_MAX];
};

struct lamzu_button_mapping {
	uint8_t raw[2];
  struct ratbag_button_action action;
};

/**
 * Contains all the mappings that can be performed statically. Missing:
 * Fire (TODO): Has parameters and no API to implement
 * Macro: Has parameters so is mapped in code
 * Poll rate (TODO): No API to implement
 * DPI lock (TODO): Has parameters and no API to implement
*/
static struct lamzu_button_mapping lamzu_button_mapping[] = {
	{ {LAMZU_ACTION_TYPE_DISABLED, 0x00}, BUTTON_ACTION_NONE},
	{ {LAMZU_ACTION_TYPE_BUTTON, 0x01}, BUTTON_ACTION_BUTTON(1)},
	{ {LAMZU_ACTION_TYPE_BUTTON, 0x02}, BUTTON_ACTION_BUTTON(2)},
	{ {LAMZU_ACTION_TYPE_BUTTON, 0x04}, BUTTON_ACTION_BUTTON(3)},
	{ {LAMZU_ACTION_TYPE_BUTTON, 0x08}, BUTTON_ACTION_BUTTON(4)},
	{ {LAMZU_ACTION_TYPE_BUTTON, 0x10}, BUTTON_ACTION_BUTTON(5)},
	{ {LAMZU_ACTION_TYPE_DPI, 0x01}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_CYCLE_UP)},
	{ {LAMZU_ACTION_TYPE_DPI, 0x02}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_UP)},
	{ {LAMZU_ACTION_TYPE_DPI, 0x03}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_DOWN)},
	{ {LAMZU_ACTION_TYPE_WHEEL_LR, 0x01}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_LEFT)},
	{ {LAMZU_ACTION_TYPE_WHEEL_LR, 0x02}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_RIGHT)},
	{ {LAMZU_ACTION_TYPE_COMBO, 0x00}, BUTTON_ACTION_MACRO},
	{ {LAMZU_ACTION_TYPE_WHEEL_UD, 0x01}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_UP)},
	{ {LAMZU_ACTION_TYPE_WHEEL_UD, 0x02}, BUTTON_ACTION_SPECIAL(RATBAG_BUTTON_ACTION_SPECIAL_WHEEL_DOWN)},
};

static const struct ratbag_button_action lamzu_macro_action = BUTTON_ACTION_MACRO;

static const struct ratbag_button_action*
lamzu_raw_to_button_action(const uint8_t *data)
{
	struct lamzu_button_mapping *mapping;

	/* Raw macro mappings have parameters so have to be mapped manually */
	if (data[0] == LAMZU_ACTION_TYPE_MACRO)
		return &lamzu_macro_action;

	ARRAY_FOR_EACH(lamzu_button_mapping, mapping) {
		if (mapping->raw[0] == data[0] && mapping->raw[1] == data[1])
			return &mapping->action;
	}

	return NULL;
}


static uint8_t*
lamzu_button_action_to_raw(const struct ratbag_button_action *action)
{
	struct lamzu_button_mapping *mapping;

	ARRAY_FOR_EACH(lamzu_button_mapping, mapping) {
		if (ratbag_button_action_match(&mapping->action, action))
			return mapping->raw;
	}

	return NULL;
}

uint8_t
lamzu_compute_crc(uint8_t *data, size_t len)
{
	uint8_t crc = LAMZU_CRC_INIT;
	for (unsigned int i = 0; i < len; i++)
	{
		crc -= data[i];
	}

	return crc;
}

/**
 * Makes a standard request and reads the response. Takes care of the report
 * ID and CRC calculation / check.
*/
int
lamzu_request(struct ratbag_device *device,
		union lamzu_message *request, union lamzu_message *response)
{
	int rc;

	request->data.report_id = LAMZU_REPORT_ID;
	request->data.crc = lamzu_compute_crc(request->raw, LAMZU_REPORT_SIZE - 1);

	rc = ratbag_hidraw_output_report(device, request->raw, LAMZU_REPORT_SIZE);
	if (rc < 0)
		return rc;

	/* Some requests generate multiple responses so skip the unwanted ones */
	for (int tries = 0; tries < 3; tries++)
	{
		memset(response, 0, sizeof(union lamzu_message));
		rc = ratbag_hidraw_read_input_report(device, response->raw, LAMZU_REPORT_SIZE, NULL);
		if (rc < 0)
			return rc;

		if (response->data.cmd == request->data.cmd)
			break;
	}

	if (rc != LAMZU_REPORT_SIZE || response->data.cmd != request->data.cmd)
		return -EIO;

	// TODO: Could crc the whole message, should result in 0 for a good message
	uint8_t crc = lamzu_compute_crc(response->raw, LAMZU_REPORT_SIZE - 1);
	if (response->data.crc != crc)
	{
		log_error(device->ratbag,
			"CRC check failed. Expected 0x%02x, received 0x%02x\n",
			crc, response->data.crc);
		return -EIO;
	}

	return 0;
}

/** Read a block of settings data using multiple requests if necessary */
int lamzu_read_settings_raw(struct ratbag_device *device, uint8_t *buf, size_t start_address, size_t len)
{
	int rc;
	size_t offset = 0;
	size_t address;
	uint8_t params_length;
	union lamzu_message request =
	{
		.data.cmd = LAMZU_CMD_GET_SETTING,
	};
	union lamzu_message response;

	while (offset < len)
	{
		/* Can only retrieve up to LAMZU_CMD_PARAMS_MAX_SIZE bytes at a time */
		params_length = len - offset;
		if (params_length > LAMZU_CMD_PARAMS_MAX_SIZE)
			params_length = LAMZU_CMD_PARAMS_MAX_SIZE;
		address = start_address + offset;

		set_unaligned_be_u16(request.data.offset, address);
		request.data.params_length = params_length;
		
		rc = lamzu_request(device, &request, &response);
		if (rc < 0 )
			return rc;

		if (response.data.params_length != params_length)
		  return -EIO;

		memcpy(&buf[offset], response.data.params, params_length);

		offset += params_length;
	}
}

/** Write a block of settings data using multiple requests if necessary */
int lamzu_write_settings_raw(struct ratbag_device *device, uint8_t *buf, size_t start_address, size_t len)
{
	int rc;
	size_t offset = 0;
	size_t address;
	uint8_t params_length;
	union lamzu_message request =
	{
		.data.cmd = LAMZU_CMD_SET_SETTING,
	};
	union lamzu_message response;

	while (offset < len)
	{
		/* Can only write up to LAMZU_CMD_PARAMS_MAX_SIZE bytes at a time */
		params_length = len - offset;
		if (params_length > LAMZU_CMD_PARAMS_MAX_SIZE)
			params_length = LAMZU_CMD_PARAMS_MAX_SIZE;
		else
			memset(request.data.params, 0, LAMZU_CMD_PARAMS_MAX_SIZE);
		address = start_address + offset;

		memcpy(request.data.params, &buf[offset], params_length);
		set_unaligned_be_u16(request.data.offset, address);
		request.data.params_length = params_length;

		/* DEBUG: Testing */
		printf("Write: ");
		for (int i =0; i < 17; i++)
		{
			printf("%02x ", request.raw[i]);
		}
		printf("\n");
		
		/*rc = lamzu_request(device, &request, &response);
		if (rc < 0 )
			return rc;*/

		/* Response should be identical to request */
		/*if (response.data.params_length != params_length)
		  return -1;*/

		offset += params_length;
	}
}

/* TODO: These are standard HID modifier codes. Is there already something we
can use? */
enum lamzu_modifier {
	LAMZU_MODIFIER_CTRL = AS_MASK(0),
	LAMZU_MODIFIER_SHIFT = AS_MASK(1),
	LAMZU_MODIFIER_ALT = AS_MASK(2),
	LAMZU_MODIFIER_META = AS_MASK(3),
};

struct lamzu_modifier_mapping {
	enum lamzu_modifier modifier;
	unsigned int keycode;
};

static const struct lamzu_modifier_mapping lamzu_modifier_map[] = {
	{ LAMZU_MODIFIER_CTRL, KEY_LEFTCTRL },
	{ LAMZU_MODIFIER_SHIFT, KEY_LEFTSHIFT },
	{ LAMZU_MODIFIER_ALT, KEY_LEFTALT },
	{ LAMZU_MODIFIER_META, KEY_LEFTMETA },
	{ LAMZU_MODIFIER_CTRL, KEY_RIGHTCTRL },
	{ LAMZU_MODIFIER_SHIFT, KEY_RIGHTSHIFT },
	{ LAMZU_MODIFIER_ALT, KEY_RIGHTALT },
	{ LAMZU_MODIFIER_META, KEY_RIGHTMETA },
};

enum lamzu_directions {
	LAMZU_DIRECTION_LEFT = AS_MASK(0),
	LAMZU_DIRECTION_RIGHT = AS_MASK(1),
	LAMZU_DIRECTION_MIDDLE = AS_MASK(2),
	LAMZU_DIRECTION_BACK = AS_MASK(3),
	LAMZU_DIRECTION_FORWARD = AS_MASK(4),
};

struct lamzu_direction_mapping {
	enum lamzu_directions direction;
	unsigned int keycode;
};

static const struct lamzu_direction_mapping lamzu_direction_map[] = {
	{ LAMZU_DIRECTION_LEFT, KEY_LEFT },
	{ LAMZU_DIRECTION_RIGHT, KEY_RIGHT },
	{ LAMZU_DIRECTION_MIDDLE, 0 },
	{ LAMZU_DIRECTION_BACK, KEY_DOWN },
	{ LAMZU_DIRECTION_FORWARD, KEY_UP },
};

unsigned int lamzu_key_to_keycode(const struct ratbag_device *device, const struct lamzu_key *key)
{
	uint16_t key_data = get_unaligned_le_u16(key->data);
	const struct lamzu_modifier_mapping *mod_mapping;
	const struct lamzu_direction_mapping *dir_mapping;

	switch (key->flags & LAMZU_KEY_FLAG_TYPE_MASK)
	{
		/* Modifiers */
		case 0:
			ARRAY_FOR_EACH(lamzu_modifier_map, mod_mapping) {
				if (mod_mapping->modifier == key_data)
					return mod_mapping->keycode;
			}
			return 0;

		case LAMZU_KEY_FLAG_HID:
			return ratbag_hidraw_get_keycode_from_keyboard_usage(device, key_data);

		case LAMZU_KEY_FLAG_HID_CC:
			return ratbag_hidraw_get_keycode_from_consumer_usage(device, key_data);

		case LAMZU_KEY_FLAG_DIRECTION:
			ARRAY_FOR_EACH(lamzu_direction_map, dir_mapping) {
				if (dir_mapping->direction == key_data)
					return dir_mapping->keycode;
			}
			return 0;
	}

	return 0;


	/*switch (key->flags & LAMZU_KEY_FLAG_TYPE_MASK)
	{*/
		/* Modifiers */
		/*case 0:
			switch (key_data)
			{
				case AS_MASK(0):
					return KEY_LEFTCTRL;

				case AS_MASK(1):
					return KEY_LEFTSHIFT;

				case AS_MASK(2):
					return KEY_LEFTALT;

				case AS_MASK(3):
					return KEY_LEFTMETA;

				default:
					return 0;
			}
		
		case LAMZU_KEY_FLAG_HID:
			return ratbag_hidraw_get_keycode_from_keyboard_usage(device, key_data);

		case LAMZU_KEY_FLAG_HID_CC:
			return ratbag_hidraw_get_keycode_from_consumer_usage(device, key_data);

		case LAMZU_KEY_FLAG_DIRECTION:
			switch (key_data)
			{
				case LAMZU_DIRECTION_LEFT:
					return KEY_LEFT;

				case LAMZU_DIRECTION_RIGHT:
					return KEY_RIGHT;

				case LAMZU_DIRECTION_MIDDLE:
					return 0;

				case LAMZU_DIRECTION_BACK:
					return KEY_DOWN;

				case LAMZU_DIRECTION_FORWARD:
					return KEY_UP;

				default:
					return 0;
			}
	}*/
}

void
lamzu_keycode_to_key(const struct ratbag_device *device,
	unsigned int keycode, struct lamzu_key *key)
{
	unsigned int cc_keycode =
		ratbag_hidraw_get_consumer_usage_from_keycode(device, keycode);
	const struct lamzu_modifier_mapping *mod_mapping;
	memset(key, 0, sizeof(struct lamzu_key));

	if (cc_keycode == 0) { /* Not a consumer code */
		/* Attempt to map as a modifier */
		ARRAY_FOR_EACH(lamzu_modifier_map, mod_mapping) {
			if (mod_mapping->keycode == keycode)
			{
				key->data[0] = (uint8_t)mod_mapping->modifier;
				return;
			}
		}

		/* Map as a HID key */
		key->flags = LAMZU_KEY_FLAG_HID;
		set_unaligned_le_u16(key->data,
			ratbag_hidraw_get_keyboard_usage_from_keycode(device, keycode));

		/*switch (keycode) {
			case KEY_LEFTCTRL:
				key->data[0] = AS_MASK(0);
				break;

			case KEY_LEFTSHIFT:
				key->data[0] = AS_MASK(1);
				break;

			case KEY_LEFTALT:
				key->data[0] = AS_MASK(2);
				break;

			case KEY_LEFTMETA:
				key->data[0] = AS_MASK(3);
				break;

			default:
				key->flags = LAMZU_KEY_FLAG_HID;
				set_unaligned_le_u16(key->data,
					ratbag_hidraw_get_keyboard_usage_from_keycode(device, keycode));
		}*/
	} else {
		key->flags = LAMZU_KEY_FLAG_HID_CC;
		set_unaligned_le_u16(key->data, cc_keycode);
	}
}

void
lamzu_read_macro(struct ratbag_button *button)
{
	struct ratbag_device *device;
	struct lamzu_macro *macro;
	struct lamzu_combo *combo;
	struct lamzu_data *drv_data;
	struct ratbag_button_macro *m = NULL;
	uint8_t *buf;
	size_t start_address;
	int rc;
	uint8_t i;

	device = button->profile->device;
	drv_data = ratbag_get_drv_data(device);

	if (drv_data->settings[button->profile->index].button_actions[button->index].type
		== LAMZU_ACTION_TYPE_COMBO)
	{
		/* Read combo data from mouse */
		combo = &drv_data->combos[button->profile->index][button->index];
		buf = (uint8_t*)combo;
		start_address = LAMZU_SETTING_COMBOS_OFFSET + (button->index * sizeof(struct lamzu_combo));
		rc = lamzu_read_settings_raw(device, buf, start_address, sizeof(struct lamzu_combo));
		if (rc < 0)
			return;

		m = ratbag_button_macro_new("combo-key");

		log_raw(device->ratbag,
			"macro on button %d of profile %d is named 'combo-key', and contains %d events:\n",
			button->index, button->profile->index, combo->keys_length);

		/* Convert lamzu combo to ratbag macro */
		for (i = 0; i < combo->keys_length; i++)
		{
			unsigned int keycode = lamzu_key_to_keycode(device, &combo->keys[i]);
			ratbag_button_macro_set_event(m,
				i,
				combo->keys[i].flags & LAMZU_KEY_FLAG_PRESS ? RATBAG_MACRO_EVENT_KEY_PRESSED : RATBAG_MACRO_EVENT_KEY_RELEASED,
				keycode);
		}
	}
	else
	{
		/* Read macro data from mouse */
		macro = &drv_data->macros[button->profile->index][button->index];
		buf = (uint8_t*)macro;
		start_address = LAMZU_SETTING_MACROS_OFFSET + (button->index * sizeof(struct lamzu_macro));
		rc = lamzu_read_settings_raw(device, buf, start_address, sizeof(struct lamzu_macro));
		if (rc < 0)
			return;

		/* Convert macro name to null-terminated string */
		char macro_name[30];
		uint8_t macro_name_length = max(macro->name_length, 30);
		memcpy(&macro_name, macro->name, macro_name_length);
		macro_name[macro_name_length] = 0;

		m = ratbag_button_macro_new(macro_name);

		log_raw(device->ratbag,
			"macro on button %d of profile %d is named '%s', and contains %d events:\n",
			button->index, button->profile->index,
			macro_name, macro->keys_length);

		/* Convert lamzu macro to ratbag macro */
		for (i = 0; i < macro->keys_length; i++) {
			unsigned int keycode = lamzu_key_to_keycode(device, &macro->keys[i].key);
			ratbag_button_macro_set_event(m,
						      i * 2,
						      macro->keys[i].key.flags & LAMZU_KEY_FLAG_PRESS ? RATBAG_MACRO_EVENT_KEY_PRESSED : RATBAG_MACRO_EVENT_KEY_RELEASED,
						      keycode);
			uint16_t delay_ms = get_unaligned_be_u16(macro->keys[i].delay_ms);
			ratbag_button_macro_set_event(m,
						      i * 2 + 1,
						      RATBAG_MACRO_EVENT_WAIT,
						      delay_ms);
		}
	}
	ratbag_button_copy_macro(button, m);
}

/**
 * Read a button mapping from the stored settings, making requests for any
 * macros. Before calling, make sure that the active profile on the mouse is
 * correct.
*/
static void
lamzu_read_button(struct ratbag_button *button)
{
	const struct ratbag_button_action *action;
	const struct ratbag_device *device;
	const struct lamzu_data *drv_data;
	const uint8_t *buf;
	int rc;
	size_t start_address;

	device = button->profile->device;
	drv_data = ratbag_get_drv_data(device);

	buf = &drv_data->settings[button->profile->index].button_actions[button->index].type;
	action = lamzu_raw_to_button_action(buf);

	if (action)
		ratbag_button_set_action(button, action);

	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_BUTTON);
	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_SPECIAL);
	ratbag_button_enable_action_type(button, RATBAG_BUTTON_ACTION_TYPE_MACRO);

	if (action && action->type == RATBAG_BUTTON_ACTION_TYPE_MACRO)
		lamzu_read_macro(button);
}

/**
 * Write button mapping to mouse, including a macro if necessary. Before
 * calling, make sure that the active profile on the mouse is correct.
*/
static int
lamzu_write_button(struct ratbag_button *button)
{
	struct ratbag_device *device;
	struct ratbag_button_action *action = &button->action;
	struct lamzu_data *drv_data;
	struct lamzu_settings *settings;
	struct lamzu_macro *macro;
	uint8_t *raw_action;
	unsigned int i, macro_key_count;
	size_t name_length;
	int rc;

	device = button->profile->device;
	drv_data = ratbag_get_drv_data(device);
	settings = &drv_data->settings[button->profile->index];

	if (button->action.type == RATBAG_BUTTON_ACTION_TYPE_MACRO) {
		macro = &drv_data->macros[button->profile->index][button->index];
		memset(macro->name, 0xff, 30);
		memset(macro->keys, 0, sizeof(struct lamzu_macro_key) *
			LAMZU_MACRO_MAX_KEYS);

		for (i = 0; i < MAX_MACRO_EVENTS && macro_key_count < LAMZU_MACRO_MAX_KEYS; i++) {
			if (action->macro->events[i].type == RATBAG_MACRO_EVENT_INVALID)
				return -EINVAL;

			if (action->macro->events[i].type == RATBAG_MACRO_EVENT_NONE)
				break;

			/* Ignore the first wait */
			if (action->macro->events[i].type == RATBAG_MACRO_EVENT_WAIT &&
			    !macro_key_count)
				continue;

			if (action->macro->events[i].type == RATBAG_MACRO_EVENT_KEY_PRESSED ||
			    action->macro->events[i].type == RATBAG_MACRO_EVENT_KEY_RELEASED) {
				lamzu_keycode_to_key(device, action->macro->events[i].event.key,
					&macro->keys[macro_key_count].key);
			}

			switch (action->macro->events[i].type) {
			case RATBAG_MACRO_EVENT_KEY_PRESSED:
				macro->keys[macro_key_count].key.flags |= LAMZU_KEY_FLAG_PRESS;
				break;
			case RATBAG_MACRO_EVENT_KEY_RELEASED:
				macro->keys[macro_key_count].key.flags |= LAMZU_KEY_FLAG_RELEASE;
				break;
			case RATBAG_MACRO_EVENT_WAIT:
				set_unaligned_be_u16(macro->keys[--macro_key_count].delay_ms,
					action->macro->events[i].event.timeout);
				break;
			case RATBAG_MACRO_EVENT_INVALID:
			case RATBAG_MACRO_EVENT_NONE:
				/* Should not happen */
				log_error(device->ratbag,
					  "Something went wrong while writing a macro.\n");
			}
			macro_key_count++;
		}

		/* Null termination is unnecessary */
		name_length = min(strlen(action->macro->name), 30);
		strncpy(macro->name, action->macro->name, name_length);
		macro->name_length = name_length;

		macro->keys_length = macro_key_count;
		macro->crc = lamzu_compute_crc((uint8_t*)macro,
			sizeof(struct lamzu_macro) - 1);

		settings->button_actions[button->index].type = LAMZU_ACTION_TYPE_MACRO;
		settings->button_actions[button->index].params[0] = button->index;
		settings->button_actions[button->index].params[1] = 3;
		settings->button_actions[button->index].crc = lamzu_compute_crc(
			(uint8_t*)&settings->button_actions[button->index], 3);

		rc = lamzu_write_settings_raw(device, (uint8_t*)&settings->button_actions[button->index],
			offsetof(struct lamzu_settings, button_actions) +
			(button->index * sizeof(struct lamzu_action)), sizeof(struct lamzu_action));
		if (rc < 0)
			return rc;
		
		rc = lamzu_write_settings_raw(device, (uint8_t*)&drv_data->macros[button->index],
			LAMZU_SETTING_MACROS_OFFSET +
			(button->index * sizeof(struct lamzu_macro)), sizeof(struct lamzu_macro));
		if (rc < 0)
			return rc;
	} else {
		raw_action = lamzu_button_action_to_raw(&button->action);
		if (raw_action == NULL)
			return -EINVAL;

		settings->button_actions[button->index].type = raw_action[0];
		settings->button_actions[button->index].params[0] = raw_action[1];
		settings->button_actions[button->index].params[1] = 0;
		settings->button_actions[button->index].crc = lamzu_compute_crc(
			(uint8_t*)&settings->button_actions[button->index], 3);

		rc = lamzu_write_settings_raw(device, (uint8_t*)&settings->button_actions[button->index],
			offsetof(struct lamzu_settings, button_actions) +
			(button->index * sizeof(struct lamzu_action)), sizeof(struct lamzu_action));
		if (rc < 0)
			return rc;
	}
}

static int
lamzu_set_active_profile(struct ratbag_device *device, unsigned int index)
{
	union lamzu_message request = {
		.data.cmd = LAMZU_CMD_SET_ACTIVE_PROFILE,
		.data.params_length = 1,
		.data.params[0] = index,
	};
	union lamzu_message response;
	uint8_t rc;

	if (index >= LAMZU_PROFILE_MAX)
		return -EINVAL;

	rc = lamzu_request(device, &request, &response);

	msleep(100);
	
	return rc;
}

/**
 * Read entire profile from mouse and set supported values in the ratbag
 * profile. Will only fetch combo and macro mappings if they are mapped to
 * buttons. Has a side effect of setting the active profile on the mouse,
 * since only the active profile can be read.
*/
static void
lamzu_read_profile(struct ratbag_profile *profile)
{
	struct ratbag_device *device = profile->device;
	struct ratbag_resolution *resolution;
	struct ratbag_button *button;
	struct lamzu_data *drv_data;
	struct lamzu_settings *settings;
	int rc;
	int dpi_x, dpi_y;
	unsigned int report_rate;

	assert(profile->index <= LAMZU_PROFILE_MAX);

	drv_data = ratbag_get_drv_data(device);
	settings = &drv_data->settings[profile->index];

	/* We can only read settings from the active profile */
	rc = lamzu_set_active_profile(device, profile->index);
	if (rc < 0)
		return;

	rc = lamzu_read_settings_raw(device, (uint8_t*)settings, 0, sizeof(struct lamzu_settings));
	if (rc < 0)
		return;

	/* FIXME: This is wrong. report rates are a mask not an index. */
	/* Report rates are stored in descending order indexed from 1 */
	if (settings->report_rate > 0 && settings->report_rate <= ARRAY_LENGTH(LAMZU_POLLING_RATES))
	{
		report_rate = LAMZU_POLLING_RATES[4 - settings->report_rate];
	} else {
		log_error(device->ratbag,
			  "Report rate read from mouse is out of range (0x%02x)\n",
			  settings->report_rate);
		report_rate = 0;
	}

	ratbag_profile_set_report_rate_list(profile, LAMZU_POLLING_RATES,
		ARRAY_LENGTH(LAMZU_POLLING_RATES));
	profile->hz = report_rate;

	ratbag_profile_set_debounce(profile, settings->debounce_ms);
	ratbag_profile_set_angle_snapping(profile, settings->angle_snapping);

	const struct dpi_range *dpirange = ratbag_device_data_lamzu_get_dpi_range(device->data);
	if (!dpirange)
		return;

	ratbag_profile_for_each_resolution(profile, resolution) {
		if (resolution->index < settings->dpi_preset_count) {
			dpi_x = settings->dpi_presets[resolution->index].dpi_x * 50 + 50;
			dpi_y = settings->dpi_presets[resolution->index].dpi_y * 50 + 50;
		} else {
			dpi_x = 0;
			dpi_y = 0;
		}

		ratbag_resolution_set_resolution(resolution, dpi_x, dpi_y);
		ratbag_resolution_set_cap(resolution,
					  RATBAG_RESOLUTION_CAP_SEPARATE_XY_RESOLUTION);
		resolution->is_active = (resolution->index == settings->current_dpi_preset);

		ratbag_resolution_set_dpi_list_from_range(
			resolution, dpirange->min, dpirange->max);
	}

	ratbag_profile_for_each_button(profile, button)
		lamzu_read_button(button);
	
	return;
}

/**
 * Write dirty parts of profile to the mouse. Has a side affect of changing
 * the active profile on the mouse since only the active profile can be written
 * to.
*/
static int
lamzu_write_profile(struct ratbag_profile *profile)
{
	struct ratbag_device *device = profile->device;
	unsigned int index = profile->index;
	struct lamzu_data *drv_data;
	struct lamzu_settings *settings;
	struct ratbag_resolution *resolution;
	struct ratbag_button *button;
	int rc;
	uint8_t *buf;
	unsigned int *rate;
	int i;

	rc = lamzu_set_active_profile(device, index);
	if (rc < 0)
		return rc;

	drv_data = ratbag_get_drv_data(device);
	settings = &drv_data->settings[index];

	if (profile->rate_dirty) {
		i = 0;
		/* Report rates are stored in descending order indexed from 1 */
		ARRAY_FOR_EACH(LAMZU_POLLING_RATES, rate) {
			if (*rate == profile->hz) {
				settings->report_rate = 4 - i;
				settings->report_rate_crc = lamzu_compute_crc(&settings->report_rate, 1);
				lamzu_write_settings_raw(device, &settings->report_rate,
					offsetof(struct lamzu_settings, report_rate), 2);
				break;
			}
			i++;
		}
	}

	if (profile->debounce_dirty) {
		settings->debounce_ms = profile->debounce;
		settings->debounce_ms_crc = lamzu_compute_crc(&settings->debounce_ms, 1);
		lamzu_write_settings_raw(device, &settings->debounce_ms,
			offsetof(struct lamzu_settings, debounce_ms), 2);
	}

	if (profile->angle_snapping_dirty) {
		settings->angle_snapping = profile->angle_snapping;
		settings->angle_snapping_crc = lamzu_compute_crc(&settings->angle_snapping, 1);
		lamzu_write_settings_raw(device, &settings->angle_snapping,
			offsetof(struct lamzu_settings, angle_snapping), 2);
	}

	ratbag_profile_for_each_resolution(profile, resolution) {
		if (!resolution->dirty || resolution->index >= settings->dpi_preset_count)
			continue;

		settings->dpi_presets[resolution->index].dpi_x = resolution->dpi_x / 50 - 1;
		settings->dpi_presets[resolution->index].dpi_y = resolution->dpi_y / 50 - 1;
		settings->dpi_presets[resolution->index].zero = 0;
		settings->dpi_presets[resolution->index].crc = lamzu_compute_crc(
			(uint8_t*)&settings->dpi_presets[resolution->index], 2);

		lamzu_write_settings_raw(device, (uint8_t*)&settings->dpi_presets[resolution->index],
			offsetof(struct lamzu_settings, dpi_presets) +
			(resolution->index * sizeof(struct lamzu_dpi)), sizeof(struct lamzu_dpi));
	}

	ratbag_profile_for_each_button(profile, button)
		if (button->dirty)
			lamzu_write_button(button);
}

static int
lamzu_active_profile(struct ratbag_device *device)
{
	int rc;
	union lamzu_message response;
	union lamzu_message request = {
		.data.cmd = LAMZU_CMD_GET_ACTIVE_PROFILE,
	};

	rc = lamzu_request(device, &request, &response);
	if (rc < 0)
		return rc;

	return response.data.params[0];
}

static int
lamzu_probe(struct ratbag_device *device)
{
	int rc;
	unsigned int profile_count, dpi_count, button_count;
	struct ratbag_profile *profile;
	struct lamzu_data *drv_data;
	int active_idx;

	rc = ratbag_open_hidraw(device);
	if (rc)
		return rc;

	if (!ratbag_hidraw_has_report(device, LAMZU_REPORT_ID)) {
		ratbag_close_hidraw(device);
		return -ENODEV;
	}

	drv_data = zalloc(sizeof(*drv_data));
	ratbag_set_drv_data(device, drv_data);

	/* Get device properties from configuration file */
	profile_count = ratbag_device_data_lamzu_get_profile_count(device->data);
	dpi_count = ratbag_device_data_lamzu_get_dpi_count(device->data);
	button_count = ratbag_device_data_lamzu_get_button_count(device->data);

	ratbag_device_init_profiles(
		device,
		max(profile_count, 1),
		max(dpi_count, 2),
		max(button_count, 6),
		LAMZU_LED);

	active_idx = lamzu_active_profile(device);
	if (active_idx < 0) {
		log_error(device->ratbag,
			  "Can't talk to the mouse: '%s' (%d)\n",
			  strerror(-active_idx),
			  active_idx);
		rc = -ENODEV;
		goto err;
	}

	ratbag_device_for_each_profile(device, profile)
	{
		lamzu_read_profile(profile);
		if (profile->index == (unsigned int)active_idx) {
			profile->is_active = true;
		}
	}

	/* Change the active profile back to the original one */
	lamzu_set_active_profile(device, (unsigned int)active_idx);

	log_raw(device->ratbag,
		"'%s' is in profile %d\n",
		ratbag_device_get_name(device),
		profile->index);

	return 0;

err:
	free(drv_data);
	ratbag_set_drv_data(device, NULL);
	return rc;
}

static void
lamzu_remove(struct ratbag_device *device)
{
	ratbag_close_hidraw(device);
	free(ratbag_get_drv_data(device));
}

static int
lamzu_commit(struct ratbag_device *device)
{
	int rc = 0;
	struct ratbag_profile *profile = NULL;
	unsigned int active_profile_index = 0;

	ratbag_device_for_each_profile(device, profile) {
		if (profile->is_active)
			active_profile_index = profile->index;
		
		if (!profile->dirty)
			continue;

		rc = lamzu_write_profile(profile);
		if (rc < 0)
			return rc;
	}

	rc = lamzu_set_active_profile(device, active_profile_index);
	return rc;
}


struct ratbag_driver lamzu_driver = {
	.name = "Lamzu",
	.id = "lamzu",
	.probe = lamzu_probe,
	.remove = lamzu_remove,
	.commit = lamzu_commit,
	.set_active_profile = lamzu_set_active_profile,
};
