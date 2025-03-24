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

#ifndef _AENC_H_
#define _AENC_H_

#include <audio-encode/aenc_core.h>

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
 * Get the supported implementations.
 * The returned implementations array is a static array whose size is the return
 * value of this function. If this function returns an error (negative errno),
 * then the value of *implems is undefined.
 * @param implems: pointer to an array of implems (output)
 * @return the size of the implems array, or a negative errno on error.
 */
AENC_API int
aenc_get_supported_implems(const enum aenc_encoder_implem **implems);


/**
 * Get the supported encodings for the given encoder implementation.
 * Each implementation supports at least one encoding, and optionally more.
 * The returned encodings array is a static array whose size is the return value
 * of this function. If this function returns an error (negative errno value),
 * then the value of *encodings is undefined.
 * @param implem: encoder implementation
 * @return bit field of the supported encodings
 */
AENC_API int aenc_get_supported_encodings(enum aenc_encoder_implem implem,
					  const enum adef_encoding **encodings);


/**
 * Get the supported input buffer data formats for the given
 * encoder implementation.
 * Each implementation supports at least one input format,
 * and optionally more. All input buffers need to be in one of
 * the supported formats, otherwise they will be discarded.
 * The returned formats array is a static array whose size is the return value
 * of this function. If this function returns an error (negative errno value),
 * then the value of *formats is undefined.
 * @param implem: decoder implementation
 * @param formats: pointer to the supported formats list (output)
 * @return the size of the formats array, or a negative errno on error.
 */
AENC_API int
aenc_get_supported_input_formats(enum aenc_encoder_implem implem,
				 const struct adef_format **formats);


/**
 * Get the implementation that will be chosen in case AENC_ENCODER_IMPLEM_AUTO
 * is used
 * @return the encoder implementation, or AENC_ENCODER_IMPLEM_AUTO in
 * case of error
 */
AENC_API enum aenc_encoder_implem aenc_get_auto_implem(void);


/**
 * Get the implementation that supports the required encodings.
 * @param encoding: encoding to support
 * @return an encoder implementation, or AENC_ENCODER_IMPLEM_AUTO in
 * case of error
 */
AENC_API enum aenc_encoder_implem
aenc_get_auto_implem_by_encoding(enum adef_encoding encoding);


/**
 * Get the implementation that supports the required encoding and input format.
 * @param encoding: encoding to support
 * @param format: pointer to an input format
 * @return an encoder implementation, or AENC_ENCODER_IMPLEM_AUTO in
 * case of error
 */
AENC_API enum aenc_encoder_implem
aenc_get_auto_implem_by_encoding_and_format(enum adef_encoding encoding,
					    const struct adef_format *format);


/**
 * Deep-copy the encoder configuration structure.
 * When no longer needed, the instance must be freed using the
 * aenc_config_free() function.
 * @param config: the encoder configuration to copy
 * @param ret_obj: pointer to the new aenc_config (output)
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_config_copy(const struct aenc_config *config,
			      struct aenc_config **ret_obj);


/**
 * Free the encoder configuration structure.
 * @param config: the encoder configuration to free
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_config_free(struct aenc_config *config);


/**
 * Create an encoder instance.
 * The configuration and callbacks structures must be filled.
 * The instance handle is returned through the aenc parameter.
 * When no longer needed, the instance must be freed using the
 * aenc_destroy() function.
 * @param loop: event loop to use
 * @param config: encoder configuration
 * @param cbs: encoder callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: encoder instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_new(struct pomp_loop *loop,
		      const struct aenc_config *config,
		      const struct aenc_cbs *cbs,
		      void *userdata,
		      struct aenc_encoder **ret_obj);


/**
 * Flush the encoder.
 * This function flushes all queues and optionally discards all buffers
 * retained by the encoder. If the buffers are not discarded the frame
 * output callback is called for each frame when the encoding is complete.
 * The function is asynchronous and returns immediately. When flushing is
 * complete the flush callback function is called if defined. After flushing
 * the encoder new input buffers can still be queued.
 * @param self: encoder instance handle
 * @param discard: if null, all pending buffers are output, otherwise they
 *        are discarded
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_flush(struct aenc_encoder *self, int discard);


/**
 * Stop the encoder.
 * This function stops any running threads. The function is asynchronous
 * and returns immediately. When stopping is complete the stop callback
 * function is called if defined. After stopping the encoder no new input
 * buffers can be queued and the encoder instance must be freed using the
 * aenc_destroy() function.
 * @param self: encoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_stop(struct aenc_encoder *self);


/**
 * Free an encoder instance.
 * This function frees all resources associated with an encoder instance.
 * @note this function blocks until all internal threads (if any) can be
 * joined; therefore the application should call aenc_stop() and wait for
 * the stop callback function to be called before calling aenc_destroy().
 * @param self: encoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int aenc_destroy(struct aenc_encoder *self);


/**
 * Get the input buffer pool.
 * The input buffer pool is defined only for implementations that require
 * using input buffers from the encoder's own pool. This function must
 * be called prior to encoding and if the returned value is not NULL the
 * input buffer pool must be used to get input buffers. If the input
 * buffers provided are not originating from the pool, they will be copied
 * resulting in a loss of performance.
 * @param self: encoder instance handle
 * @return a pointer on the input buffer pool on success, NULL in case of
 * error of if no pool is used
 */
AENC_API struct mbuf_pool *
aenc_get_input_buffer_pool(struct aenc_encoder *self);


/**
 * Get the input buffer queue.
 * This function must be called prior to encoding and the input
 * buffer queue must be used to push input buffers for encoding.
 * @param self: encoder instance handle
 * @return a pointer on the input buffer queue on success, NULL in case of error
 */
AENC_API struct mbuf_audio_frame_queue *
aenc_get_input_buffer_queue(struct aenc_encoder *self);


/**
 * Get the AAC encoding Audio Specific Config (ASC).
 * The caller must provide buffers with sufficient space for the ASC. When
 * buffers are provided the size parameters must point to the available space in
 * the buffers. If there is enough space the ASC data will be copied into the
 * provided buffer and the size will be replaced by the actual size of the data.
 * The ownership of the ASC buffer stays with the caller. The required size for
 * the buffer can be retrieved by calling the function with NULL ASC data
 * pointers and non-NULL size pointer.
 * @param self: encoder instance handle
 * @param asc: pointer to the ASC data (input/output, optional, can be NULL)
 * @param asc_size: pointer to the ASC size
 *                  (input/output, optional, can be NULL)
 * @return 0 on success, negative errno value in case of error
 */
AENC_API int
aenc_get_aac_asc(struct aenc_encoder *self, uint8_t *asc, size_t *asc_size);


/**
 * Get the encoder implementation used.
 * @param self: encoder instance handle
 * @return the encoder implementation used, or AENC_ENCODER_IMPLEM_AUTO
 * in case of error
 */
AENC_API enum aenc_encoder_implem
aenc_get_used_implem(struct aenc_encoder *self);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_AENC_H_ */
