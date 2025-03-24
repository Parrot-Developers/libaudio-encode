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

#define ULOG_TAG aenc_fakeaac
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "aenc_fakeaac_priv.h"


#define NB_SUPPORTED_ENCODINGS 1
#define NB_SUPPORTED_FORMATS 24
static struct adef_format supported_formats[NB_SUPPORTED_FORMATS];
static enum adef_encoding supported_encodings[NB_SUPPORTED_ENCODINGS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
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


static void call_flush_done(void *userdata)
{
	struct aenc_fakeaac *self = userdata;

	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void call_stop_done(void *userdata)
{
	struct aenc_fakeaac *self = userdata;

	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static int gen_aac_silent_frame(struct aenc_fakeaac *self)
{
	int ret, err;
	uint8_t *sample = NULL;
	size_t sample_size;
	size_t min_sample_size;
	struct aac_bitstream bs;
	memset(&bs, 0, sizeof(bs));

	if (self->fake_sample_mem != NULL)
		return 0;

	/* Get min sample size */
	ret = aac_write_silent_frame(
		NULL,
		self->aac_ctx,
		self->base->config.input.format.channel_count,
		0);
	if (ret < 0) {
		AENC_LOG_ERRNO("aac_write_silent_frame", -ret);
		goto out;
	}
	min_sample_size = ret;

	if (!self->disable_filler) {
		sample_size = self->base->config.aac_lc.target_bitrate * 1024 /
			      self->base->config.input.format.sample_rate / 8;

		/* Ensure that target bitrate is enough */
		if (sample_size < min_sample_size) {
			AENC_LOGW(
				"sample_size (%zu) below the minimum size (%zu)",
				sample_size,
				min_sample_size);
			sample_size = min_sample_size;
		}
	} else {
		sample_size = min_sample_size;
	}

	ret = mbuf_mem_generic_new(sample_size, &self->fake_sample_mem);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}
	ret = mbuf_mem_get_data(
		self->fake_sample_mem, (void **)&sample, &sample_size);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}

	aac_bs_init(&bs, sample, sample_size + 1);

	ret = aac_write_silent_frame(
		&bs,
		self->aac_ctx,
		self->base->config.input.format.channel_count,
		sample_size);
	if (ret < 0) {
		AENC_LOG_ERRNO("aac_write_silent_frame", -ret);
		goto out;
	}

	if (bs.off != sample_size) {
		ret = -EPROTO;
		AENC_LOGE(
			"invalid generated sample length "
			"(expected: %zd, got: %zd)",
			sample_size,
			bs.off);
		goto out;
	}

	ret = 0;

out:
	if (ret < 0) {
		if (self->fake_sample_mem != NULL) {
			err = mbuf_mem_unref(self->fake_sample_mem);
			if (err != 0)
				AENC_LOG_ERRNO("mbuf_mem_unref", -err);
			self->fake_sample_mem = NULL;
		}
	}

	aac_bs_clear(&bs);

	return ret;
}


static int fill_frame(struct aenc_fakeaac *self,
		      struct mbuf_audio_frame *out_frame)
{
	int ret;
	void *data = NULL;
	size_t len = 0;

	ret = gen_aac_silent_frame(self);
	if (ret < 0) {
		AENC_LOG_ERRNO("gen_aac_silent_frame", -ret);
		return ret;
	}

	ret = mbuf_mem_get_data(self->fake_sample_mem, &data, &len);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_mem_get_data", -ret);
		return ret;
	}

	ret = mbuf_audio_frame_set_buffer(
		out_frame, self->fake_sample_mem, 0, len);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_set_buffer", -ret);
		return ret;
	}

	return 0;
}


static int encode_frame(struct aenc_fakeaac *self,
			struct mbuf_audio_frame *in_frame)
{
	int ret;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};
	struct adef_frame in_info;
	struct adef_frame out_info;
	struct mbuf_audio_frame *out_frame;

	struct mbuf_audio_frame_cbs frame_cbs = {
		.pre_release = self->base->cbs.pre_release,
		.pre_release_userdata = self->base->userdata,
	};

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_audio_frame_add_ancillary_buffer(
		in_frame,
		AENC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		AENC_LOG_ERRNO("mbuf_audio_frame_add_ancillary_buffer", -ret);

	ret = mbuf_audio_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -ret);
		return ret;
	}

	self->base->counters.pushed++;

	out_info.format = self->output_format;
	out_info.info = in_info.info;

	/* Frame creation */
	ret = mbuf_audio_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_new", -ret);
		goto out;
	}
	ret = mbuf_audio_frame_set_callbacks(out_frame, &frame_cbs);
	if (ret < 0)
		AENC_LOG_ERRNO("mbuf_audio_frame_set_callbacks", -ret);

	ret = mbuf_audio_frame_foreach_ancillary_data(
		in_frame, mbuf_audio_frame_ancillary_data_copier, out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_foreach_ancillary_data", -ret);
		goto out;
	}

	/* Add generated AAC samples */
	ret = fill_frame(self, out_frame);
	if (ret < 0) {
		AENC_LOG_ERRNO("fill_frame", -ret);
		goto out;
	}

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

	self->base->counters.pulled++;

	self->base->cbs.frame_output(
		self->base, 0, out_frame, self->base->userdata);
	self->base->counters.out++;

	ret = 0;

out:
	if (ret < 0)
		self->base->cbs.frame_output(
			self->base, ret, NULL, self->base->userdata);

	if (out_frame)
		mbuf_audio_frame_unref(out_frame);

	return ret;
}


static void queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	int ret, input_flushed = 0;
	struct aenc_fakeaac *self = userdata;
	struct mbuf_audio_frame *in_frame;

	AENC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	do {
		/* Make sure there is an input buffer available */
		ret = mbuf_audio_frame_queue_peek(self->in_queue, &in_frame);
		if (ret == -EAGAIN) {
			input_flushed = 1;
			break;
		} else if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_peek:input",
				       -ret);
			return;
		}
		mbuf_audio_frame_unref(in_frame);

		/* Dequeue the input buffer */
		ret = mbuf_audio_frame_queue_pop(self->in_queue, &in_frame);
		if (ret == -EAGAIN) {
			goto unref;
		} else if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_pop:input",
				       -ret);
			goto unref;
		}

		/* 'Fake-encode' the input frame */
		ret = encode_frame(self, in_frame);
		if (ret < 0)
			goto unref;

		/* clang-format off */
unref:
		/* clang-format on */
		if (in_frame)
			mbuf_audio_frame_unref(in_frame);
	} while (ret == 0);

	if (atomic_load(&self->flushing) && input_flushed) {
		atomic_store(&self->flushing, false);
		call_flush_done(self);
	}
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
	struct aenc_fakeaac *self = NULL;
	struct mbuf_audio_frame *in_frame;
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
	} else {
		/* Wait for all frames to be processed */
		ret = mbuf_audio_frame_queue_peek(self->in_queue, &in_frame);
		if (ret == -EAGAIN) {
			ret = pomp_loop_idle_add_with_cookie(
				base->loop, call_flush_done, self, self);
			if (ret < 0) {
				AENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -ret);
				return ret;
			}
			return 0;
		} else if (ret < 0) {
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_peek:input",
				       -ret);
			return ret;
		}
		mbuf_audio_frame_unref(in_frame);
		return 0;
	}

	/* Nothing more to do, just schedule calling the flush callback */
	ret = pomp_loop_idle_add_with_cookie(
		base->loop, call_flush_done, self, self);
	if (ret < 0) {
		AENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		return ret;
	}

	return 0;
}


static int stop(struct aenc_encoder *base)
{
	int res;
	struct aenc_fakeaac *self = NULL;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Nothing to do here, just schedule calling the stop callback */
	res = pomp_loop_idle_add_with_cookie(
		base->loop, call_stop_done, self, self);
	if (res < 0) {
		AENC_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
		return res;
	}

	return 0;
}


static int destroy(struct aenc_encoder *base)
{
	int err;
	struct aenc_fakeaac *self = NULL;
	struct pomp_evt *in_queue_evt;

	AENC_LOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	if (self->fake_sample_mem)
		mbuf_mem_unref(self->fake_sample_mem);

	/* Free the resources */
	if (self->in_queue != NULL) {
		err = mbuf_audio_frame_queue_get_event(self->in_queue,
						       &in_queue_evt);
		if (err < 0)
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_get_event",
				       -err);
		if (pomp_evt_is_attached(in_queue_evt, base->loop)) {
			err = pomp_evt_detach_from_loop(in_queue_evt,
							base->loop);
			if (err < 0)
				AENC_LOG_ERRNO("pomp_evt_detach_from_loop",
					       -err);
		}
		err = mbuf_audio_frame_queue_destroy(self->in_queue);
		if (err < 0)
			AENC_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
	}

	err = pomp_loop_idle_remove_by_cookie(base->loop, self);
	if (err < 0)
		AENC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	err = aac_ctx_destroy(self->aac_ctx);
	if (err < 0)
		AENC_LOG_ERRNO("aac_ctx_destroy", -err);

	free(self);
	base->derived = NULL;

	return 0;
}


static bool input_filter(struct mbuf_audio_frame *frame, void *userdata)
{
	int ret;
	struct adef_frame info;
	struct aenc_fakeaac *self = userdata;

	AENC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flushing))
		return false;

	ret = mbuf_audio_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* We do not need to use the default filter, but we still need to update
	 * the frame to set the input time */
	aenc_default_input_filter_internal_confirm_frame(
		self->base, frame, &info);

	return true;
}


static int copy_implem_cfg(const struct aenc_config_impl *impl_cfg,
			   struct aenc_config_impl **ret_obj)
{
	struct aenc_config_fakeaac *specific =
		(struct aenc_config_fakeaac *)impl_cfg;
	struct aenc_config_fakeaac *copy = NULL;
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
	struct aenc_config_fakeaac *specific =
		(struct aenc_config_fakeaac *)impl_cfg;
	ULOG_ERRNO_RETURN_ERR_IF(specific == NULL, EINVAL);

	free((void *)specific);

	return 0;
}


static int create(struct aenc_encoder *base)
{
	int ret = 0;
	struct aenc_fakeaac *self = NULL;
	struct aenc_config_fakeaac *specific;
	struct pomp_evt *in_queue_evt;
	struct mbuf_audio_frame_queue_args queue_args = {
		.filter = input_filter,
	};

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
	if (base->config.output.preferred_format ==
	    ADEF_AAC_DATA_FORMAT_UNKNOWN)
		base->config.output.preferred_format =
			ADEF_AAC_DATA_FORMAT_ADTS;
	switch (base->config.output.preferred_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
	case ADEF_AAC_DATA_FORMAT_ADTS:
		break;
	default:
		AENC_LOGE("unsupported output format %s",
			  adef_aac_data_format_to_str(
				  base->config.output.preferred_format));
		ret = -EINVAL;
		goto error;
	}
	if (base->cbs.frame_output == NULL) {
		ret = -EINVAL;
		AENC_LOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;
	queue_args.filter_userdata = self;

	AENC_LOGI("fake AAC implementation");

	specific = (struct aenc_config_fakeaac *)aenc_config_get_specific(
		&base->config, AENC_ENCODER_IMPLEM_FAKEAAC);

	if (specific != NULL)
		self->disable_filler = specific->disable_filler;

	ret = aac_ctx_new(&self->aac_ctx);
	if (ret < 0) {
		AENC_LOG_ERRNO("aac_ctx_new", -ret);
		goto error;
	}

	self->output_format = base->config.input.format;
	self->output_format.encoding = base->config.encoding;
	self->output_format.aac.data_format =
		base->config.output.preferred_format;

	if (base->config.output.preferred_format == ADEF_AAC_DATA_FORMAT_RAW) {
		/* Set Audio Specific Config */
		struct aac_asc asc = {0};
		ret = aac_asc_from_adef_format(&self->output_format, &asc);
		if (ret < 0) {
			AENC_LOG_ERRNO("aac_asc_from_adef_format", -ret);
			goto error;
		}
		ret = aac_ctx_set_asc(self->aac_ctx, &asc);
		if (ret < 0) {
			AENC_LOG_ERRNO("aac_ctx_set_asc", -ret);
			goto error;
		}
		ret = aac_write_asc(
			&asc, &self->base->aac.asc, &self->base->aac.asc_size);
		if (ret < 0) {
			AENC_LOG_ERRNO("aac_write_asc", -ret);
			goto error;
		}
	} else if (base->config.output.preferred_format ==
		   ADEF_AAC_DATA_FORMAT_ADTS) {
		struct aac_adts adts = {0};
		ret = aac_adts_from_adef_format(&self->output_format, &adts);
		if (ret < 0) {
			AENC_LOG_ERRNO("aac_adts_from_adef_format", -ret);
			goto error;
		}
		ret = aac_ctx_set_adts(self->aac_ctx, &adts);
		if (ret < 0) {
			AENC_LOG_ERRNO("aac_ctx_set_adts", -ret);
			goto error;
		}
	}

	/* Create the input buffers queue */
	ret = mbuf_audio_frame_queue_new_with_args(&queue_args,
						   &self->in_queue);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_new_with_args:input",
			       -ret);
		goto error;
	}
	ret = mbuf_audio_frame_queue_get_event(self->in_queue, &in_queue_evt);
	if (ret < 0) {
		AENC_LOG_ERRNO("mbuf_audio_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		in_queue_evt, base->loop, &queue_evt_cb, self);
	if (ret < 0) {
		AENC_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	return 0;

error:
	/* Cleanup on error */
	destroy(base);
	base->derived = NULL;
	return ret;
}


static struct mbuf_pool *get_input_buffer_pool(struct aenc_encoder *base)
{
	struct aenc_fakeaac *self = NULL;

	AENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_audio_frame_queue *
get_input_buffer_queue(struct aenc_encoder *base)
{
	struct aenc_fakeaac *self = NULL;

	AENC_LOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);
	self = base->derived;

	return self->in_queue;
}


const struct aenc_ops aenc_fakeaac_ops = {
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
