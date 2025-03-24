/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define ULOG_TAG aenc_core
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "aenc_core_priv.h"
#include <futils/timetools.h>


enum aenc_encoder_implem aenc_encoder_implem_from_str(const char *str)
{
	if (strcasecmp(str, "FDK_AAC") == 0)
		return AENC_ENCODER_IMPLEM_FDK_AAC;
	else if (strcasecmp(str, "FAKEAAC") == 0)
		return AENC_ENCODER_IMPLEM_FAKEAAC;
	else
		return AENC_ENCODER_IMPLEM_AUTO;
}


const char *aenc_encoder_implem_to_str(enum aenc_encoder_implem implem)
{
	switch (implem) {
	case AENC_ENCODER_IMPLEM_FDK_AAC:
		return "FDK_AAC";
	case AENC_ENCODER_IMPLEM_FAKEAAC:
		return "FAKEAAC";
	case AENC_ENCODER_IMPLEM_AUTO:
	default:
		return "UNKNOWN";
	}
}


enum aenc_rate_control aenc_rate_control_from_str(const char *str)
{
	if (strcasecmp(str, "CBR") == 0)
		return AENC_RATE_CONTROL_CBR;
	else if (strcasecmp(str, "VBR") == 0)
		return AENC_RATE_CONTROL_VBR;
	else
		return AENC_RATE_CONTROL_CBR;
}


const char *aenc_rate_control_to_str(enum aenc_rate_control rc)
{
	switch (rc) {
	case AENC_RATE_CONTROL_CBR:
		return "CBR";
	case AENC_RATE_CONTROL_VBR:
		return "VBR";
	default:
		return "UNKNOWN";
	}
}


bool aenc_default_input_filter(struct mbuf_audio_frame *frame, void *userdata)
{
	int ret;
	bool accept;
	struct aenc_encoder *encoder = userdata;
	const struct adef_format *supported_formats;
	struct adef_frame frame_info;

	if (!frame || !encoder)
		return false;

	ret = mbuf_audio_frame_get_frame_info(frame, &frame_info);
	if (ret != 0)
		return false;

	ret = encoder->ops->get_supported_input_formats(&supported_formats);
	if (ret < 0)
		return false;
	accept = aenc_default_input_filter_internal(
		encoder, frame, &frame_info, supported_formats, ret);
	if (accept)
		aenc_default_input_filter_internal_confirm_frame(
			encoder, frame, &frame_info);
	return accept;
}


bool aenc_default_input_filter_internal(
	struct aenc_encoder *encoder,
	struct mbuf_audio_frame *frame,
	struct adef_frame *frame_info,
	const struct adef_format *supported_formats,
	unsigned int nb_supported_formats)
{
	if (!adef_format_intersect(&frame_info->format,
				   supported_formats,
				   nb_supported_formats)) {
		ULOG_ERRNO(
			"unsupported format:"
			" " ADEF_FORMAT_TO_STR_FMT,
			EPROTO,
			ADEF_FORMAT_TO_STR_ARG(&frame_info->format));
		return false;
	}

	if (frame_info->info.timestamp <= encoder->last_timestamp &&
	    encoder->last_timestamp != UINT64_MAX) {
		ULOG_ERRNO("non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   EPROTO,
			   frame_info->info.timestamp,
			   encoder->last_timestamp);
		return false;
	}

	return true;
}


void aenc_default_input_filter_internal_confirm_frame(
	struct aenc_encoder *encoder,
	struct mbuf_audio_frame *frame,
	struct adef_frame *frame_info)
{
	int err;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};

	/* Save frame timestamp to last_timestamp */
	encoder->last_timestamp = frame_info->info.timestamp;
	encoder->counters.in++;

	/* Set the input time ancillary data to the frame */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_audio_frame_add_ancillary_buffer(
		frame, AENC_ANCILLARY_KEY_INPUT_TIME, &ts_us, sizeof(ts_us));
	if (err < 0)
		ULOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -err);
}


struct aenc_config_impl *
aenc_config_get_specific(const struct aenc_config *config,
			 enum aenc_encoder_implem implem)
{
	/* Check if specific config is present */
	if (!config->implem_cfg)
		return NULL;

	/* Check if implementation is the right one */
	if (config->implem != implem) {
		ULOGI("specific config found, but implementation is %s "
		      "instead of %s. ignoring specific config",
		      aenc_encoder_implem_to_str(config->implem),
		      aenc_encoder_implem_to_str(implem));
		return NULL;
	}

	/* Check if specific config implementation matches the base one */
	if (config->implem_cfg->implem != config->implem) {
		ULOGW("specific config implem (%s) does not match"
		      " base config implem (%s). ignoring specific config",
		      aenc_encoder_implem_to_str(config->implem_cfg->implem),
		      aenc_encoder_implem_to_str(config->implem));
		return NULL;
	}

	/* All tests passed, return specific config */
	return config->implem_cfg;
}
