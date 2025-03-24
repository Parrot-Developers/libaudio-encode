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

#ifndef _AENC_INTERNAL_H_
#define _AENC_INTERNAL_H_

#include <inttypes.h>

#include <audio-encode/aenc_core.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Specific logging functions : log the instance ID before the log message */
#define AENC_LOG_INT(_pri, _fmt, ...)                                          \
	do {                                                                   \
		char *prefix = (self != NULL && self->base != NULL)            \
				       ? self->base->enc_name                  \
				       : "";                                   \
		ULOG_PRI(_pri,                                                 \
			 "%s%s" _fmt,                                          \
			 prefix != NULL ? prefix : "",                         \
			 prefix != NULL ? ": " : "",                           \
			 ##__VA_ARGS__);                                       \
	} while (0)
#define AENC_LOGD(_fmt, ...) AENC_LOG_INT(ULOG_DEBUG, _fmt, ##__VA_ARGS__)
#define AENC_LOGI(_fmt, ...) AENC_LOG_INT(ULOG_INFO, _fmt, ##__VA_ARGS__)
#define AENC_LOGW(_fmt, ...) AENC_LOG_INT(ULOG_WARN, _fmt, ##__VA_ARGS__)
#define AENC_LOGE(_fmt, ...) AENC_LOG_INT(ULOG_ERR, _fmt, ##__VA_ARGS__)
#define AENC_LOG_ERRNO(_fmt, _err, ...)                                        \
	do {                                                                   \
		char *prefix = (self != NULL && self->base != NULL)            \
				       ? self->base->enc_name                  \
				       : "";                                   \
		ULOGE_ERRNO((_err),                                            \
			    "%s%s" _fmt,                                       \
			    prefix != NULL ? prefix : "",                      \
			    prefix != NULL ? ": " : "",                        \
			    ##__VA_ARGS__);                                    \
	} while (0)
#define AENC_LOGW_ERRNO(_fmt, _err, ...)                                       \
	do {                                                                   \
		char *prefix = (self != NULL && self->base != NULL)            \
				       ? self->base->enc_name                  \
				       : "";                                   \
		ULOGW_ERRNO((_err),                                            \
			    "%s%s" _fmt,                                       \
			    prefix != NULL ? prefix : "",                      \
			    prefix != NULL ? ": " : "",                        \
			    ##__VA_ARGS__);                                    \
	} while (0)
#define AENC_LOG_ERRNO_RETURN_IF(_cond, _err)                                  \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			AENC_LOG_ERRNO("", (_err));                            \
			return;                                                \
		}                                                              \
	} while (0)
#define AENC_LOG_ERRNO_RETURN_ERR_IF(_cond, _err)                              \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			int __pdraw_errno__err = (_err);                       \
			AENC_LOG_ERRNO("", (__pdraw_errno__err));              \
			return -(__pdraw_errno__err);                          \
		}                                                              \
	} while (0)
#define AENC_LOG_ERRNO_RETURN_VAL_IF(_cond, _err, _val)                        \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			AENC_LOG_ERRNO("", (_err));                            \
			/* codecheck_ignore[RETURN_PARENTHESES] */             \
			return (_val);                                         \
		}                                                              \
	} while (0)


struct aenc_ops {
	/**
	 * Get the supported encodings for the implementation.
	 * Each implementation supports at least one encoding, and
	 * optionally more. The returned encodings array
	 * is a static array whose size is the return value of this function.
	 * If this function returns an error (negative errno value), then the
	 * value of *encodings is undefined.
	 * @param formats: pointer to the supported encodings list (output)
	 * @return the size of the encodings array, or a negative errno on
	 * error.
	 */
	int (*get_supported_encodings)(const enum adef_encoding **encodings);

	/**
	 * Get the supported input buffer data formats for the implementation.
	 * Each implementation supports at least one input format, and
	 * optionally more. All input buffers need to be in one of the supported
	 * formats, otherwise they will be discarded. The returned formats array
	 * is a static array whose size is the return value of this function.
	 * If this function returns an error (negative errno value), then the
	 * value of *formats is undefined.
	 * @param formats: pointer to the supported formats list (output)
	 * @return the size of the formats array, or a negative errno on error.
	 */
	int (*get_supported_input_formats)(const struct adef_format **formats);

	/**
	 * Create an encoder implementation instance.
	 * When no longer needed, the instance must be freed using the
	 * destroy() function.
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*create)(struct aenc_encoder *base);

	/**
	 * Deep-copy an implementation specific extension (optional).
	 * When no longer needed, the implementation specific extension returned
	 * by the function must be freed using the free_implem_cfg() function.
	 * @param impl_cfg: the implementation specific extension to copy
	 * @param ret_obj: pointer to an implementation specific extension
	 * (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*copy_implem_cfg)(const struct aenc_config_impl *impl_cfg,
			       struct aenc_config_impl **ret_obj);

	/**
	 * Free an implementation specific extension (optional).
	 * @param impl_cfg: the implementation specific extension to free
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*free_implem_cfg)(struct aenc_config_impl *impl_cfg);

	/**
	 * Flush the encoder implemetation.
	 * This function flushes all queues and optionally discards all buffers
	 * retained by the encoder. If the buffers are not discarded the
	 * frame output callback is called for each frame when the encoding
	 * is complete. The function is asynchronous and returns immediately.
	 * When flushing is complete the flush callback function is called if
	 * defined. After flushing the encoder new input buffers can still
	 * be queued.
	 * @param base: base instance
	 * @param discard: if null, all pending buffers are output, otherwise
	 *                 they are discarded
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*flush)(struct aenc_encoder *base, int discard);

	/**
	 * Stop the encoder implementation.
	 * This function stops any running threads. The function is asynchronous
	 * and returns immediately. When stopping is complete the stop callback
	 * function is called if defined. After stopping the encoder no new
	 * input buffers can be queued and the encoder instance must be freed
	 * using the destroy() function.
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*stop)(struct aenc_encoder *base);

	/**
	 * Free an encoder implementation instance.
	 * This function frees all resources associated with an encoder
	 * implementation instance.
	 * @note this function blocks until all internal threads (if any)
	 * can be joined
	 * @param base: base instance
	 * @return 0 on success, negative errno value in case of error
	 */
	int (*destroy)(struct aenc_encoder *base);

	/**
	 * Get the input buffer pool.
	 * The input buffer pool is defined only for implementations that
	 * require using input buffers from the encoder's own pool. This
	 * function must be called prior to encoding and if the returned
	 * value is not NULL the input buffer pool should be used to get input
	 * buffers. If the input buffers provided are not originating from the
	 * pool, they will be copied resulting in a loss of performance.
	 * @param base: base instance
	 * @return a pointer on the input buffer pool on success, NULL in case
	 * of error of if no pool is used
	 */
	struct mbuf_pool *(*get_input_buffer_pool)(struct aenc_encoder *base);

	/**
	 * Get the input buffer queue.
	 * This function must be called prior to encoding and the input
	 * buffer queue must be used to push input buffers for encoding.
	 * @param base: base instance
	 * @return a pointer on the input buffer queue on success, NULL in case
	 * of error
	 */
	struct mbuf_audio_frame_queue *(*get_input_buffer_queue)(
		struct aenc_encoder *base);
};

struct aenc_encoder {
	/* Reserved */
	struct aenc_encoder *base;
	void *derived;
	const struct aenc_ops *ops;
	struct pomp_loop *loop;
	struct aenc_cbs cbs;
	void *userdata;
	struct aenc_config config;

	int enc_id;
	char *enc_name;

	union {
		struct {
			uint8_t *asc;
			size_t asc_size;
		} aac;
	};
	uint64_t last_timestamp;

	struct {
		/* Frames that have passed the input filter */
		unsigned int in;
		/* Frames that have been pushed to the encoder */
		unsigned int pushed;
		/* Frames that have been pulled from the encoder */
		unsigned int pulled;
		/* Frames that have been output (frame_output) */
		unsigned int out;
	} counters;
};

/**
 * Default filter for the input frame queue.
 * This function is intended to be used as a standalone input filter.
 * It will call aenc_default_input_filter_internal(), and then
 * aenc_default_input_filter_internal_confirm_frame() if the former returned
 * true.
 *
 * @param frame: The frame to filter.
 * @param userdata: The aenc_encoder structure.
 *
 * @return true if the frame passes the checks, false otherwise
 */
AENC_API bool aenc_default_input_filter(struct mbuf_audio_frame *frame,
					void *userdata);

/**
 * Default filter for the input frame queue.
 * This filter does the following checks:
 * - frame is in a supported format
 * - frame info matches input config
 * - frame timestamp is strictly monotonic
 * This version is intended to be used by custom filters, to avoid calls to
 * mbuf_audio_frame_get_frame_info() or get_supported_input_formats().
 *
 * @warning This function does NOT check input validity. Arguments must not be
 * NULL, except for supported_formats if nb_supported_formats is zero.
 *
 * @param encoder: The base audio encoder.
 * @param frame: The frame to filter.
 * @param frame_info: The associated adef_frame.
 * @param supported_formats: The formats supported by the implementation.
 * @param nb_supported_formats: The size of the supported_formats array.
 *
 * @return true if the frame passes the checks, false otherwise
 */
AENC_API bool
aenc_default_input_filter_internal(struct aenc_encoder *encoder,
				   struct mbuf_audio_frame *frame,
				   struct adef_frame *frame_info,
				   const struct adef_format *supported_formats,
				   unsigned int nb_supported_formats);

/**
 * Filter update function.
 * This function should be called at the end of a custom filter. It registers
 * that the frame was accepted. This function saves the frame timestamp for
 * monotonic checks, and sets the AENC_ANCILLARY_KEY_INPUT_TIME ancillary data
 * on the frame.
 *
 * @param encoder: The base audio encoder.
 * @param frame: The accepted frame.
 * @param frame_info: The associated adef_frame.
 */
AENC_API void
aenc_default_input_filter_internal_confirm_frame(struct aenc_encoder *encoder,
						 struct mbuf_audio_frame *frame,
						 struct adef_frame *frame_info);


/**
 * Specific config getter with proper implem checks.
 * @param config: base configuration
 * @param implem: requested implementation
 * @return the specific config if it matches the requested implem,
 *         NULL otherwise
 */
AENC_API struct aenc_config_impl *
aenc_config_get_specific(const struct aenc_config *config,
			 enum aenc_encoder_implem implem);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_AENC_INTERNAL_H_ */
