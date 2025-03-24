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

#ifndef _AENC_CORE_H_
#define _AENC_CORE_H_

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <audio-defs/adefs.h>
#include <libpomp.h>
#include <media-buffers/mbuf_audio_frame.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef AENC_API_EXPORTS
#	ifdef _WIN32
#		define AENC_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define AENC_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !AENC_API_EXPORTS */
#	define AENC_API
#endif /* !AENC_API_EXPORTS */


/**
 * mbuf ancillary data key for the input timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define AENC_ANCILLARY_KEY_INPUT_TIME "aenc.input_time"

/**
 * mbuf ancillary data key for the dequeue timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define AENC_ANCILLARY_KEY_DEQUEUE_TIME "aenc.dequeue_time"

/**
 * mbuf ancillary data key for the output timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define AENC_ANCILLARY_KEY_OUTPUT_TIME "aenc.output_time"

/**
 * mbuf ancillary data key for the input status.
 * This key is only set by certain implementations.
 *
 * Content is a struct aenc_input_status
 */
#define AENC_ANCILLARY_KEY_INPUT_QUEUE_STATUS "aenc.input_status"


/* Forward declarations */
struct aenc_encoder;


struct aenc_input_status {
	/* Number of frames waiting in the input queue */
	uint32_t queue_frames;

	/* Number of frames waiting in the encoder input */
	uint32_t encoder_frames;
};


/* Supported encoder implementations */
enum aenc_encoder_implem {
	/* Automatically select encoder */
	AENC_ENCODER_IMPLEM_AUTO = 0,

	/* Fraunhofer FDK AAC encoder */
	AENC_ENCODER_IMPLEM_FDK_AAC,

	/* Fake AAC encoder */
	AENC_ENCODER_IMPLEM_FAKEAAC,

	AENC_ENCODER_IMPLEM_MAX,
};


/* Rate-control algorithm */
enum aenc_rate_control {
	/* Constant bitrate */
	AENC_RATE_CONTROL_CBR = 0,

	/* Variable bitrate */
	AENC_RATE_CONTROL_VBR,
};


/* Encoder initial configuration, implementation specific extension
 * Each implementation might provide implementation specific configuration with
 * a structure compatible with this base structure (i.e. which starts with the
 * same implem field). */
struct aenc_config_impl {
	/* Encoder implementation for this extension */
	enum aenc_encoder_implem implem;
};


/* Encoder intital configuration */
struct aenc_config {
	/* Encoder instance name (optional, can be NULL, copied internally) */
	const char *name;

	/* Device name (mandatory for some implementations,
	 * copied internally) */
	const char *device;

	/* Encoder implementation (AUTO means no preference,
	 * use the default implementation for the platform) */
	enum aenc_encoder_implem implem;

	/* Preferred encoding thread count (0 means no preference,
	 * use the default value; 1 means no multi-threading;
	 * only relevant for CPU encoding implementations) */
	unsigned int preferred_thread_count;

	/* Limit the number of frames that can be in the encoder pipeline
	 * at any time. The frames will be waiting in the input queue
	 * if the limit is reached. (0 means no limit). */
	unsigned int preferred_max_frames_in_encoder;

	/* Audio frame length in samples:
	 * - 1024: Default configuration.
	 * - 512: Default length in LD/ELD configuration.
	 * - 480: Length in LD/ELD configuration.
	 * - 256: Length for ELD reduced delay mode (x2).
	 * - 240: Length for ELD reduced delay mode (x2).
	 * - 128: Length for ELD reduced delay mode (x4).
	 * - 120: Length for ELD reduced delay mode (x4). */
	unsigned int preferred_frame_length;

	/* Encoding type (mandatory) */
	enum adef_encoding encoding;

	/* Input configuration */
	struct {
		/* Input buffer pool preferred minimum buffer count, used
		 * only if the implementation uses it's own input buffer pool
		 * (0 means no preference, use the default value) */
		unsigned int preferred_min_buf_count;

		/* Input buffers data format (mandatory) */
		struct adef_format format;
	} input;

	/* Output configuration */
	struct {
		/* Output buffer pool preferred minimum buffer count
		 * (0 means no preference, use the default value) */
		unsigned int preferred_min_buf_count;

		/* Preferred output buffers data format
		 * (ADEF_AAC_DATA_FORMAT_UNKNOWN means no preference,
		 * use the default format of the implementation) */
		enum adef_aac_data_format preferred_format;
	} output;

	/* Encoding-specific configuration */
	union {
		/* ISO/IEC 14496-3 MPEG-4 Audio - AAC profile (Low Complexity)
		 * specific configuration */
		struct {
			/* Rate control algorithm (defaults to CBR) */
			enum aenc_rate_control rate_control;

			/* Maximum bitrate in bits per second for VBR and CBR
			 * rate-control modes (mandatory for these modes; also
			 * the initial target bitrate if target_bitrate is 0) */
			unsigned int max_bitrate;

			/* Initial target bitrate in bits per second for
			 * VBR and CBR rate-control modes (optional,
			 * set to max_bitrate if 0) */
			unsigned int target_bitrate;
		} aac_lc;
	};

	/* Implementation specific extensions (optional, can be NULL)
	 * If not null, implem_cfg must match the following requirements:
	 *  - this->implem_cfg->implem == this->implem
	 *  - this->implem != AENC_ENCODER_IMPLEM_AUTO
	 *  - The real type of implem_cfg must be the implementation specific
	 *    structure, not struct aenc_config_impl */
	struct aenc_config_impl *implem_cfg;
};


/* Encoder callback functions */
struct aenc_cbs {
	/* Frame output callback function (mandatory)
	 * The library retains ownership of the output frame and the
	 * application must reference it if needed after returning from the
	 * callback function. The status is 0 in case of success, a negative
	 * errno otherwise. In case of error no frame is output and frame
	 * is NULL.
	 * @param enc: encoder instance handle
	 * @param status: frame output status
	 * @param frame: output frame
	 * @param userdata: user data pointer */
	void (*frame_output)(struct aenc_encoder *enc,
			     int status,
			     struct mbuf_audio_frame *frame,
			     void *userdata);

	/* Flush callback function, called when flushing is complete (optional)
	 * @param enc: encoder instance handle
	 * @param userdata: user data pointer */
	void (*flush)(struct aenc_encoder *enc, void *userdata);

	/* Stop callback function, called when stopping is complete (optional)
	 * @param enc: encoder instance handle
	 * @param userdata: user data pointer */
	void (*stop)(struct aenc_encoder *enc, void *userdata);

	/* Pre-release callback function. (optional)
	 * If defined, this function will be called on each output frame when
	 * its ref-count reaches zero.
	 */
	mbuf_audio_frame_pre_release_t pre_release;
};


/**
 * Get an enum aenc_encoder_implem value from a string.
 * Valid strings are only the suffix of the implementation name (eg. 'FDK_AAC').
 * The case is ignored.
 * @param str: implementation name to convert
 * @return the enum aenc_encoder_implem value or AENC_ENCODER_IMPLEM_AUTO
 *         if unknown
 */
AENC_API enum aenc_encoder_implem aenc_encoder_implem_from_str(const char *str);


/**
 * Get a string from an enum aenc_encoder_implem value.
 * @param implem: implementation value to convert
 * @return a string description of the implementation
 */
AENC_API const char *
aenc_encoder_implem_to_str(enum aenc_encoder_implem implem);


/**
 * Get an enum aenc_rate_control value from a string.
 * Valid strings are only the suffix of the rate control value (eg. 'CBR').
 * The case is ignored.
 * @param str: rate control value to convert
 * @return the enum aenc_rate_control value or AENC_RATE_CONTROL_CBR
 *         if unknown
 */
AENC_API enum aenc_rate_control aenc_rate_control_from_str(const char *str);


/**
 * Get a string from an enum aenc_rate_control value.
 * @param rc: rate control value to convert
 * @return a string description of the rate control
 */
AENC_API const char *aenc_rate_control_to_str(enum aenc_rate_control rc);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_AENC_CORE_H_ */
