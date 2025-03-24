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

#define ULOG_TAG aenc_fdk_aac
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "aenc_fdk_aac_priv.h"


#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 24
static struct adef_format supported_formats[NB_SUPPORTED_FORMATS];
static enum adef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	/* Note: The FDK library is based on fixed-point math and only supports
	 * 16-bit integer PCM input. */
	int i = 0;
	supported_formats[i++] = adef_pcm_16b_8000hz_mono;
	supported_formats[i++] = adef_pcm_16b_8000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_11025hz_mono;
	supported_formats[i++] = adef_pcm_16b_11025hz_stereo;
	supported_formats[i++] = adef_pcm_16b_12000hz_mono;
	supported_formats[i++] = adef_pcm_16b_12000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_16000hz_mono;
	supported_formats[i++] = adef_pcm_16b_16000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_22050hz_mono;
	supported_formats[i++] = adef_pcm_16b_22050hz_stereo;
	supported_formats[i++] = adef_pcm_16b_24000hz_mono;
	supported_formats[i++] = adef_pcm_16b_24000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_32000hz_mono;
	supported_formats[i++] = adef_pcm_16b_32000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_44100hz_mono;
	supported_formats[i++] = adef_pcm_16b_44100hz_stereo;
	supported_formats[i++] = adef_pcm_16b_48000hz_mono;
	supported_formats[i++] = adef_pcm_16b_48000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_64000hz_mono;
	supported_formats[i++] = adef_pcm_16b_64000hz_stereo;
	supported_formats[i++] = adef_pcm_16b_88200hz_mono;
	supported_formats[i++] = adef_pcm_16b_88200hz_stereo;
	supported_formats[i++] = adef_pcm_16b_96000hz_mono;
	supported_formats[i++] = adef_pcm_16b_96000hz_stereo;

	supported_encodings[0] = ADEF_ENCODING_AAC_LC;
}


static const char *aac_encoder_error_to_str(AACENC_ERROR err)
{
	switch (err) {
	case AACENC_OK:
		return "OK";
	case AACENC_INVALID_HANDLE:
		return "INVALID_HANDLE";
	case AACENC_MEMORY_ERROR:
		return "MEMORY_ERROR";
	case AACENC_UNSUPPORTED_PARAMETER:
		return "UNSUPPORTED_PARAMETER";
	case AACENC_INVALID_CONFIG:
		return "INVALID_CONFIG";
	case AACENC_INIT_ERROR:
		return "INIT_ERROR";
	case AACENC_INIT_AAC_ERROR:
		return "INIT_AAC_ERROR";
	case AACENC_INIT_SBR_ERROR:
		return "INIT_SBR_ERROR ";
	case AACENC_INIT_TP_ERROR:
		return "INIT_TP_ERROR";
	case AACENC_INIT_META_ERROR:
		return "INIT_META_ERROR";
	case AACENC_INIT_MPS_ERROR:
		return "INIT_MPS_ERROR";
	case AACENC_ENCODE_ERROR:
		return "ENCODE_ERROR";
	case AACENC_ENCODE_EOF:
		return "ENCODE_EOF";
	default:
		return "UNKNOWN";
	}
	return NULL;
}


static void call_flush_done(void *userdata)
{
	struct aenc_fdk_aac *self = userdata;

	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void call_stop_done(void *userdata)
{
	struct aenc_fdk_aac *self = userdata;

	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct aenc_fdk_aac *self = userdata;
	int ret, err;
	char message;

	do {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				AENC_LOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message) {
		case AENC_MSG_FLUSH:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_flush_done, self, self);
			if (err < 0)
				AENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		case AENC_MSG_STOP:
			err = pomp_loop_idle_add_with_cookie(
				self->base->loop, call_stop_done, self, self);
			if (err < 0)
				AENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			break;
		default:
			AENC_LOGE("unknown message: %c", message);
			break;
		}
	} while (ret == 0);
}


static void enc_out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct aenc_fdk_aac *self = userdata;
	struct mbuf_audio_frame *out_frame = NULL;
	int err;

	do {
		err = mbuf_audio_frame_queue_pop(self->enc_out_queue,
						 &out_frame);
		if (err == -EAGAIN) {
			return;
		} else if (err < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_pop:enc_out",
				       -err);
			return;
		}
		struct adef_frame out_info = {};
		err = mbuf_audio_frame_get_frame_info(out_frame, &out_info);
		if (err < 0)
			AENC_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -err);

		self->base->counters.pulled++;

		if (!atomic_load(&self->flush_discard)) {
			self->base->cbs.frame_output(
				self->base, 0, out_frame, self->base->userdata);
			self->base->counters.out++;
		} else {
			AENC_LOGD("discarding frame %d", out_info.info.index);
		}

		mbuf_audio_frame_unref(out_frame);
	} while (err == 0);
}


static int encode_frame(struct aenc_fdk_aac *self,
			struct mbuf_audio_frame *in_frame)
{
	int ret = 0, frame_size;
	const void *plane_data;
	const uint8_t *in_data;
	size_t len;
	struct mbuf_audio_frame *out_frame = NULL;
	struct adef_frame out_info;
	struct adef_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	AACENC_BufDesc in_buf = {0}, out_buf = {0};
	AACENC_InArgs in_args = {0};
	AACENC_OutArgs out_args = {0};
	int in_identifier = IN_AUDIO_DATA;
	int in_size, in_elem_size;
	int out_identifier = OUT_BITSTREAM_DATA;
	int out_size, out_elem_size;
	void *in_ptr, *out_ptr;
	size_t i;
	AACENC_ERROR err;
	struct mbuf_mem *mem;
	void *aac_data;
	uint8_t *data;
	size_t mem_size;

	struct mbuf_audio_frame_cbs frame_cbs = {
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	if (in_frame == NULL)
		return 0;

	ret = mbuf_audio_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (!adef_format_intersect(
		    &in_info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		AENC_LOG_ERRNO(
			"unsupported format:"
			" " ADEF_FORMAT_TO_STR_FMT,
			-ret,
			ADEF_FORMAT_TO_STR_ARG(&in_info.format));
		return ret;
	}

	mbuf_audio_frame_get_buffer(in_frame, &plane_data, &len);
	if (ret == 0) {
		in_data = plane_data;
		mbuf_audio_frame_release_buffer(in_frame, plane_data);
	} else {
		/* TODO: don't forget to drop the frame
		 * otherwise it remains in the queue. */
		AENC_LOG_ERRNO("mbuf_audio_frame_get_buffer", -ret);
		return ret;
	}
	if (len == 0) {
		ret = -EINVAL;
		AENC_LOGE("empty frame");
		return ret;
	}

	ret = mbuf_audio_frame_add_ancillary_buffer(
		in_frame,
		AENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		AENC_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);

	self->base->counters.pushed++;

	if (len > self->input_size) {
		AENC_LOGE("buffer length > input_size");
		return -EINVAL;
	}

	for (i = 0; i < len / 2; i++) {
		const uint8_t *in = &in_data[2 * i];
		self->convert_buf[i] = in[0] | (in[1] << 8);
	}

	in_ptr = self->convert_buf;
	in_size = len;
	in_elem_size = 2;

	in_args.numInSamples = (int)len / 2;
	in_buf.numBufs = 1;
	in_buf.bufs = &in_ptr;
	in_buf.bufferIdentifiers = &in_identifier;
	in_buf.bufSizes = &in_size;
	in_buf.bufElSizes = &in_elem_size;

	out_ptr = self->output_buf;
	out_size = self->input_size;
	out_elem_size = 1;
	out_buf.numBufs = 1;
	out_buf.bufs = &out_ptr;
	out_buf.bufferIdentifiers = &out_identifier;
	out_buf.bufSizes = &out_size;
	out_buf.bufElSizes = &out_elem_size;

	err = aacEncEncode(
		self->handle, &in_buf, &out_buf, &in_args, &out_args);
	if (err != AACENC_OK) {
		ret = err;
		AENC_LOG_ERRNO("aacEncEncode", -ret);
		return ret;
	}

	frame_size = out_args.numOutBytes;
	out_info.info = in_info.info;
	out_info.format = self->output_format;

	ret = mbuf_audio_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_new", -ret);
		goto out;
	}
	ret = mbuf_audio_frame_set_callbacks(out_frame, &frame_cbs);
	if (ret < 0)
		AENC_LOG_ERRNO("mbuf_audio_frame_set_callbacks", -ret);

	/* Ancillary data */
	ret = mbuf_audio_frame_foreach_ancillary_data(
		in_frame, mbuf_audio_frame_ancillary_data_copier, out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_foreach_ancillary_data", -ret);
		goto out;
	}

	mem_size = frame_size;

	ret = mbuf_mem_generic_new(mem_size, &mem);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_mem_generic_new", -ret);
		return ret;
	}
	ret = mbuf_mem_get_data(mem, &aac_data, &mem_size);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	data = aac_data;

	/* Fill mbuf mem */
	memcpy(data, out_ptr, mem_size);

	ret = mbuf_audio_frame_set_buffer(out_frame, mem, 0, mem_size);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_set_buffer", -ret);
		goto out;
	}

	ret = mbuf_mem_unref(mem);
	if (ret != 0)
		AENC_LOG_ERRNO("mbuf_mem_unref", -ret);

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_audio_frame_add_ancillary_buffer(
		out_frame,
		AENC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		AENC_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);

	/* Output the frame */
	ret = mbuf_audio_frame_finalize(out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_finalize", -ret);
		goto out;
	}
	ret = mbuf_audio_frame_queue_push(self->enc_out_queue, out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_push", -ret);
		goto out;
	}

out:
	if (out_frame)
		mbuf_audio_frame_unref(out_frame);

	return ret;
}


static int complete_flush(struct aenc_fdk_aac *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		/* Flush the input queue */
		ret = mbuf_audio_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_flush:input",
				       -ret);
			return ret;
		}
		/* Flush the encoder output queue */
		ret = mbuf_audio_frame_queue_flush(self->enc_out_queue);
		if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_flush:enc_out",
				       -ret);
			return ret;
		}
	}

	atomic_store(&self->flushing, false);
	atomic_store(&self->flush_discard, false);

	/* Call the flush callback on the loop */
	char message = AENC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		AENC_LOG_ERRNO("mbox_push", -ret);

	return ret;
}


static void check_input_queue(struct aenc_fdk_aac *self)
{
	int ret, err = 0;
	struct mbuf_audio_frame *in_frame;

	ret = mbuf_audio_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame */
		/* Encode the frame */
		ret = encode_frame(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				AENC_LOG_ERRNO("encode_frame", -ret);
			err = -ENOSPC;
		}
		if (in_frame) {
			mbuf_audio_frame_unref(in_frame);
			/* Pop the frame for real */
			ret = mbuf_audio_frame_queue_pop(self->in_queue,
							 &in_frame);
			if (ret < 0) {
				AENC_LOG_ERRNO("mbuf_audio_frame_queue_pop",
					       -ret);
				break;
			}
			mbuf_audio_frame_unref(in_frame);
		}
		if (err)
			break;
		/* Peek the next frame */
		ret = mbuf_audio_frame_queue_peek(self->in_queue, &in_frame);
		if (ret < 0 && ret != -EAGAIN && ret != -ENOSPC)
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_peek", -ret);
		if (atomic_load(&self->flushing) && ret == -EAGAIN) {
			in_frame = NULL;
			if (atomic_load(&self->flushing) &&
			    !atomic_load(&self->flush_discard) &&
			    (self->base->counters.pushed ==
			     self->base->counters.pulled)) {
				ret = complete_flush(self);
				if (ret < 0)
					AENC_LOG_ERRNO("complete_flush", -ret);
				continue;
			}
		}
	}

	if (atomic_load(&self->flushing) && ret == -EAGAIN) {
		if (atomic_load(&self->flushing) &&
		    !atomic_load(&self->flush_discard) &&
		    (self->base->counters.pushed ==
		     self->base->counters.pulled)) {
			ret = complete_flush(self);
			if (ret < 0)
				AENC_LOG_ERRNO("complete_flush", -ret);
		}
	}
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct aenc_fdk_aac *self = userdata;
	check_input_queue(self);
}


static void *encoder_thread(void *ptr)
{
	int ret, timeout;
	struct aenc_fdk_aac *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	char message;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	ret = pthread_setname_np("aenc_fdkaac");
	if (ret != 0)
		AENC_LOG_ERRNO("pthread_setname_np", ret);
#	endif
#else
	ret = pthread_setname_np(pthread_self(), "aenc_fdkaac");
	if (ret != 0)
		AENC_LOG_ERRNO("pthread_setname_np", ret);
#endif

	loop = pomp_loop_new();
	if (!loop) {
		AENC_LOG_ERRNO("pomp_loop_new", ENOMEM);
		goto exit;
	}
	ret = mbuf_audio_frame_queue_get_event(self->in_queue, &in_queue_evt);
	if (ret != 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -ret);
		goto exit;
	}
	ret = pomp_evt_attach_to_loop(in_queue_evt, loop, input_event_cb, self);
	if (ret != 0) {
		AENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto exit;
	}

	while ((!atomic_load(&self->should_stop)) ||
	       (atomic_load(&self->flushing))) {
		/* Start flush, discarding all frames */
		if ((atomic_load(&self->flushing)) &&
		    (atomic_load(&self->flush_discard))) {
			ret = complete_flush(self);
			if (ret < 0)
				AENC_LOG_ERRNO("complete_flush", -ret);
			continue;
		}

		/* Wait for an input buffer (without dequeueing it) */
		timeout = ((atomic_load(&self->flushing)) &&
			   (!atomic_load(&self->flush_discard)))
				  ? 0
				  : 5;
		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			AENC_LOG_ERRNO("pomp_loop_wait_and_process", -ret);
			if (!self->should_stop) {
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		} else if (ret == -ETIMEDOUT) {
			check_input_queue(self);
		}
	}

	/* Call the stop callback on the loop */
	message = AENC_MSG_STOP;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		AENC_LOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
		ret = pomp_evt_detach_from_loop(in_queue_evt, loop);
		if (ret != 0)
			AENC_LOG_ERRNO("pomp_evt_detach_from_loop", -ret);
	}
	if (loop != NULL) {
		ret = pomp_loop_destroy(loop);
		if (ret != 0)
			AENC_LOG_ERRNO("pomp_loop_destroy", -ret);
	}

	return NULL;
}


static int get_supported_encodings(const enum adef_encoding **encodings)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*encodings = supported_encodings;
	return NB_SUPPORTED_ENCODINGS;
}


static int get_supported_input_formats(const struct adef_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int flush(struct aenc_encoder *base, int discard_)
{
	int ret;
	struct aenc_fdk_aac *self = NULL;
	bool discard = discard_;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	if (discard) {
		/* Flush the input queue */
		ret = mbuf_audio_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_flush:input",
				       -ret);
			return ret;
		}
	}

	return 0;
}


static int stop(struct aenc_encoder *base)
{
	struct aenc_fdk_aac *self = NULL;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the encoding thread */
	atomic_store(&self->should_stop, true);

	return 0;
}


static int destroy(struct aenc_encoder *base)
{
	int err;
	struct aenc_fdk_aac *self = NULL;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the encoding thread */
	err = stop(base);
	if (err < 0)
		AENC_LOG_ERRNO("stop", -err);
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			AENC_LOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->in_queue != NULL) {
		err = mbuf_audio_frame_queue_destroy(self->in_queue);
		if (err < 0)
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_destroy:input",
				       -err);
	}
	if (self->enc_out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->enc_out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->enc_out_queue_evt,
						base->loop);
		if (err < 0)
			AENC_LOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->enc_out_queue != NULL) {
		err = mbuf_audio_frame_queue_destroy(self->enc_out_queue);
		if (err < 0)
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_destroy:enc_out",
				       -err);
	}
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			AENC_LOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->convert_buf != NULL)
		free(self->convert_buf);
	if (self->output_buf != NULL)
		free(self->output_buf);
	if (self->handle != NULL)
		aacEncClose(&self->handle);

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		AENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	free(self);
	base->derived = NULL;

	return 0;
}


static bool input_filter(struct mbuf_audio_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct adef_frame info;
	struct aenc_fdk_aac *self = userdata;

	AENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flushing) || atomic_load(&self->should_stop))
		return false;

	ret = mbuf_audio_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* Pass default filters first */
	if (!aenc_default_input_filter_internal(self->base,
						frame,
						&info,
						supported_formats,
						NB_SUPPORTED_FORMATS))
		return false;

	/* Input frame must be packed */
	ret = mbuf_audio_frame_get_buffer(frame, &tmp, &tmplen);
	if (ret != 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_get_buffer", -ret);
		return false;
	}
	mbuf_audio_frame_release_buffer(frame, tmp);

	aenc_default_input_filter_internal_confirm_frame(
		self->base, frame, &info);

	return true;
}


static int copy_implem_cfg(const struct aenc_config_impl *impl_cfg,
			   struct aenc_config_impl **ret_obj)
{
	struct aenc_config_fdk_aac *specific =
		(struct aenc_config_fdk_aac *)impl_cfg;
	struct aenc_config_fdk_aac *copy = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	copy = calloc(1, sizeof(*copy));
	ULOG_ERRNO_RETURN_ERR_IF(copy == NULL, ENOMEM);
	*copy = *specific;

	*ret_obj = (struct aenc_config_impl *)copy;

	return 0;
}


static int free_implem_cfg(struct aenc_config_impl *impl_cfg)
{
	struct aenc_config_fdk_aac *specific =
		(struct aenc_config_fdk_aac *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	free((void *)specific);

	return 0;
}


static int create(struct aenc_encoder *base)
{
	int ret = 0;
	struct aenc_fdk_aac *self = NULL;
	struct aenc_config_fdk_aac *specific;
	struct mbuf_audio_frame_queue_args queue_args = {
		.filter = input_filter,
	};
	int aot;
	int mode;
	int afterburner = 1;
	TRANSPORT_TYPE tt;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != ADEF_ENCODING_AAC_LC) {
		ret = -EINVAL;
		AENC_LOG_ERRNO("invalid encoding: %s",
			       -ret,
			       adef_encoding_to_str(base->config.encoding));
		return ret;
	}
	if (base->cbs.frame_output == NULL) {
		ret = -EINVAL;
		AENC_LOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}

	specific = (struct aenc_config_fdk_aac *)aenc_config_get_specific(
		&base->config, AENC_ENCODER_IMPLEM_FDK_AAC);

	if (specific != NULL)
		afterburner = specific->afterburner;

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;
	queue_args.filter_userdata = self;

	if (base->config.output.preferred_format ==
	    ADEF_AAC_DATA_FORMAT_UNKNOWN)
		base->config.output.preferred_format =
			ADEF_AAC_DATA_FORMAT_ADTS;

	switch (base->config.output.preferred_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
	case ADEF_AAC_DATA_FORMAT_ADIF:
	case ADEF_AAC_DATA_FORMAT_ADTS:
		break;
	default:
		AENC_LOGE("unsupported output format %s",
			  adef_aac_data_format_to_str(
				  base->config.output.preferred_format));
		ret = -EINVAL;
		goto error;
	}

	/* Configure encoding attributes */
	switch (base->config.encoding) {
	case ADEF_ENCODING_AAC_LC:
		aot = AAC_AOT_AAC_LC;
	default:
		break;
	}

	/* Initialize the mailbox for inter-thread messages  */
	self->mbox = mbox_new(1);
	if (self->mbox == NULL) {
		ret = -ENOMEM;
		AENC_LOG_ERRNO("mbox_new", -ret);
		goto error;
	}
	ret = pomp_loop_add(base->loop,
			    mbox_get_read_fd(self->mbox),
			    POMP_FD_EVENT_IN,
			    &mbox_cb,
			    self);
	if (ret < 0) {
		AENC_LOG_ERRNO("pomp_loop_add", -ret);
		goto error;
	}

	switch (base->config.input.format.channel_count) {
	case 1:
		mode = MODE_1;
		break;
	case 2:
		mode = MODE_2;
		break;
	case 3:
		mode = MODE_1_2;
		break;
	case 4:
		mode = MODE_1_2_1;
		break;
	case 5:
		mode = MODE_1_2_2;
		break;
	case 6:
		mode = MODE_1_2_2_1;
		break;
	default:
		ret = -EINVAL;
		AENC_LOG_ERRNO("Unsupported WAV channels: %d",
			       -ret,
			       base->config.input.format.channel_count);
		goto error;
	}

	AENC_LOGI("FDK_AAC implementation");

	ret = aacEncOpen(
		&self->handle, 0, base->config.input.format.channel_count);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncOpen: %s", aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncoder_SetParam(self->handle,
				  AACENC_SAMPLERATE,
				  base->config.input.format.sample_rate);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_SAMPLERATE: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncoder_SetParam(self->handle, AACENC_AOT, aot);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_AOT: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncoder_SetParam(self->handle, AACENC_CHANNELMODE, mode);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_CHANNELMODE: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	/* TODO:
	 * 0: MPEG channel ordering
	 * 1: WAV channel ordering. Add to config? */
	ret = aacEncoder_SetParam(self->handle, AACENC_CHANNELORDER, 1);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_CHANNELORDER: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	switch (base->config.output.preferred_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
		tt = TT_MP4_RAW;
		break;
	case ADEF_AAC_DATA_FORMAT_ADIF:
		tt = TT_MP4_ADIF;
		break;
	case ADEF_AAC_DATA_FORMAT_ADTS:
		tt = TT_MP4_ADTS;
		break;
	default:
		ret = -ENOSYS;
		AENC_LOG_ERRNO("unsupported data format", -ret);
		goto error;
	}

	ret = aacEncoder_SetParam(self->handle, AACENC_TRANSMUX, tt);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_TRANSMUX: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	if (base->config.preferred_frame_length != 0) {
		ret = aacEncoder_SetParam(self->handle,
					  AACENC_GRANULE_LENGTH,
					  base->config.preferred_frame_length);
		if (ret != AACENC_OK) {
			AENC_LOGE(
				"aacEncoder_SetParam:AACENC_GRANULE_LENGTH: %s",
				aac_encoder_error_to_str(ret));
			ret = -EPROTO;
			goto error;
		}
	}

	switch (base->config.aac_lc.rate_control) {
	case AENC_RATE_CONTROL_CBR:
		mode = 0; /* Constant bitrate, use bitrate according to
			     ::AACENC_BITRATE */
		ret = aacEncoder_SetParam(self->handle,
					  AACENC_BITRATE,
					  base->config.aac_lc.target_bitrate);
		if (ret != AACENC_OK) {
			AENC_LOGE("aacEncoder_SetParam:AACENC_BITRATE: %s",
				  aac_encoder_error_to_str(ret));
			ret = -EPROTO;
			goto error;
		}
		break;
	case AENC_RATE_CONTROL_VBR:
		/* - 1: Variable bitrate mode, \ref vbrmode
		 * "very low bitrate".
		 *   - 2: Variable bitrate mode, \ref vbrmode
		 * "low bitrate".
		 *   - 3: Variable bitrate mode, \ref vbrmode
		 * "medium bitrate".
		 *   - 4: Variable bitrate mode, \ref vbrmode
		 * "high bitrate".
		 *   - 5: Variable bitrate mode, \ref vbrmode
		 * "very high bitrate". */
		mode = 1;
		break;
	default:
		ret = -EINVAL;
		AENC_LOGE("unsupported rate control: %s",
			  aenc_rate_control_to_str(
				  base->config.aac_lc.rate_control));
		goto error;
	}

	ret = aacEncoder_SetParam(self->handle, AACENC_BITRATEMODE, mode);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_BITRATEMODE: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncoder_SetParam(
		self->handle, AACENC_AFTERBURNER, afterburner);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncoder_SetParam:AACENC_AFTERBURNER: %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncEncode(self->handle, NULL, NULL, NULL, NULL);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncEncode:unable to init encoder %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	ret = aacEncInfo(self->handle, &self->info);
	if (ret != AACENC_OK) {
		AENC_LOGE("aacEncInfo:unable to get encoder info %s",
			  aac_encoder_error_to_str(ret));
		ret = -EPROTO;
		goto error;
	}

	/* Note: bit_depth shall be 16 (as implem-required). */
	self->input_size = base->config.input.format.channel_count *
			   base->config.input.format.bit_depth *
			   self->info.frameLength;

	if (base->config.output.preferred_format == ADEF_AAC_DATA_FORMAT_RAW) {
		/* Set Audio Specific Config */
		self->base->aac.asc = (uint8_t *)calloc(1, self->info.confSize);
		if (self->base->aac.asc == NULL) {
			ret = -ENOMEM;
			AENC_LOG_ERRNO("calloc", -ret);
			goto error;
		}

		memcpy(self->base->aac.asc,
		       self->info.confBuf,
		       self->info.confSize);
		self->base->aac.asc_size = self->info.confSize;
	}

	self->convert_buf = (int16_t *)calloc(1, self->input_size);
	if (self->convert_buf == NULL) {
		ret = -ENOMEM;
		AENC_LOG_ERRNO("calloc", -ret);
		goto error;
	}

	self->output_buf = (int8_t *)calloc(1, self->input_size);
	if (self->output_buf == NULL) {
		ret = -ENOMEM;
		AENC_LOG_ERRNO("calloc", -ret);
		goto error;
	}

	self->output_format = base->config.input.format;
	self->output_format.encoding = base->config.encoding;
	self->output_format.aac.data_format =
		base->config.output.preferred_format;

	/* Create the input buffers queue */
	ret = mbuf_audio_frame_queue_new_with_args(&queue_args,
						   &self->in_queue);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_new_with_args:input",
			       -ret);
		goto error;
	}

	/* Create the encoder output buffers queue */
	ret = mbuf_audio_frame_queue_new(&self->enc_out_queue);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_new:enc_out", -ret);
		goto error;
	}
	ret = mbuf_audio_frame_queue_get_event(self->enc_out_queue,
					       &self->enc_out_queue_evt);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(self->enc_out_queue_evt,
				      base->loop,
				      &enc_out_queue_evt_cb,
				      self);
	if (ret < 0) {
		AENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the encoding thread */
	ret = pthread_create(&self->thread, NULL, encoder_thread, (void *)self);
	if (ret != 0) {
		ret = -ret;
		AENC_LOG_ERRNO("pthread_create", -ret);
		goto error;
	}
	self->thread_launched = true;

	return 0;

error:
	/* Cleanup on error */
	destroy(base);
	base->derived = NULL;
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct aenc_encoder *base)
{
	struct aenc_fdk_aac *self = NULL;

	AENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_audio_frame_queue *
get_input_buffer_queue(struct aenc_encoder *base)
{
	struct aenc_fdk_aac *self = NULL;

	AENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	return self->in_queue;
}


const struct aenc_ops aenc_fdk_aac_ops = {
	.get_supported_encodings = get_supported_encodings,
	.get_supported_input_formats = get_supported_input_formats,
	.copy_implem_cfg = copy_implem_cfg,
	.free_implem_cfg = free_implem_cfg,
	.create = create,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
};
