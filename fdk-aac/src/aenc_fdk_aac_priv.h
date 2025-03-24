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

#ifndef _AENC_FDK_AAC_PRIV_H_
#define _AENC_FDK_AAC_PRIV_H_

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

#include <aac/aac.h>
#include <audio-encode/aenc_core.h>
#include <audio-encode/aenc_fdk_aac.h>
#include <audio-encode/aenc_internal.h>
#include <fdk-aac/aacenc_lib.h>
#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>

#define AENC_MSG_FLUSH 'f'
#define AENC_MSG_STOP 's'


static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}

struct aenc_fdk_aac {
	struct aenc_encoder *base;
	struct mbuf_audio_frame_queue *in_queue;
	struct mbuf_audio_frame_queue *enc_out_queue;
	struct pomp_evt *enc_out_queue_evt;
	struct adef_format output_format;

	pthread_t thread;
	bool thread_launched;
	atomic_bool should_stop;
	atomic_bool flushing;
	atomic_bool flush_discard;
	struct mbox *mbox;

	HANDLE_AACENCODER handle;
	CHANNEL_MODE mode;
	AACENC_InfoStruct info;
	unsigned int input_size;
	int16_t *convert_buf;
	int8_t *output_buf;
};


#endif /* !_AENC_FDK_AAC_PRIV_H_ */
