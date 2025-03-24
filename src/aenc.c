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

#define ULOG_TAG aenc
#include <ulog.h>
ULOG_DECLARE_TAG(aenc);

#include "aenc_priv.h"


/* Put preferred implementation first for autoselection */
static const enum aenc_encoder_implem supported_implems[] = {
#ifdef BUILD_LIBAUDIO_ENCODE_FDK_AAC
	AENC_ENCODER_IMPLEM_FDK_AAC,
#endif
#ifdef BUILD_LIBAUDIO_ENCODE_FAKEAAC
	AENC_ENCODER_IMPLEM_FAKEAAC,
#endif
};


/* Forward declaration */
static int aenc_config_free_allocated(struct aenc_config *config);


static const int supported_implems_count =
	sizeof(supported_implems) / sizeof(supported_implems[0]);


static atomic_int s_instance_counter;


static pthread_once_t instance_counter_is_init = PTHREAD_ONCE_INIT;
static void initialize_instance_counter(void)
{
	atomic_init(&s_instance_counter, 0);
}


static const struct aenc_ops *implem_ops(enum aenc_encoder_implem implem)
{
	switch (implem) {
#ifdef BUILD_LIBAUDIO_ENCODE_FDK_AAC
	case AENC_ENCODER_IMPLEM_FDK_AAC:
		return &aenc_fdk_aac_ops;
#endif
#ifdef BUILD_LIBAUDIO_ENCODE_FAKEAAC
	case AENC_ENCODER_IMPLEM_FAKEAAC:
		return &aenc_fakeaac_ops;
#endif
	default:
		return NULL;
	}
}


static int aenc_get_implem(enum aenc_encoder_implem *implem)
{
	ULOG_ERRNO_RETURN_ERR_IF(implem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!supported_implems_count, ENOSYS);

	if (*implem == AENC_ENCODER_IMPLEM_AUTO) {
		*implem = supported_implems[0];
		return 0;
	}

	for (int i = 0; i < supported_implems_count; i++)
		if (*implem == supported_implems[i])
			return 0;

	/* No suitable implementation found */
	return -ENOSYS;
}


int aenc_get_supported_implems(const enum aenc_encoder_implem **implems)
{
	ULOG_ERRNO_RETURN_ERR_IF(!implems, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!supported_implems_count, ENOSYS);

	*implems = supported_implems;

	return supported_implems_count;
}


int aenc_get_supported_encodings(enum aenc_encoder_implem implem,
				 const enum adef_encoding **encodings)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(!encodings, EINVAL);

	ret = aenc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	return implem_ops(implem)->get_supported_encodings(encodings);
}


int aenc_get_supported_input_formats(enum aenc_encoder_implem implem,
				     const struct adef_format **formats)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(!formats, EINVAL);

	ret = aenc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	return implem_ops(implem)->get_supported_input_formats(formats);
}


enum aenc_encoder_implem aenc_get_auto_implem(void)
{
	int ret;
	enum aenc_encoder_implem implem = AENC_ENCODER_IMPLEM_AUTO;

	ret = aenc_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, AENC_ENCODER_IMPLEM_AUTO);

	return implem;
}


enum aenc_encoder_implem
aenc_get_auto_implem_by_encoding(enum adef_encoding encoding)
{
	int res;
	const enum adef_encoding *encodings;

	ULOG_ERRNO_RETURN_VAL_IF(encoding == ADEF_ENCODING_UNKNOWN,
				 EINVAL,
				 AENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(
		!supported_implems_count, ENOSYS, AENC_ENCODER_IMPLEM_AUTO);

	for (int i = 0; i < supported_implems_count; i++) {
		enum aenc_encoder_implem implem = supported_implems[i];

		res = implem_ops(implem)->get_supported_encodings(&encodings);
		if (res < 0)
			continue;

		for (int j = 0; j < res; j++) {
			if (encodings[j] == encoding)
				return implem;
		}
	}

	return AENC_ENCODER_IMPLEM_AUTO;
}


enum aenc_encoder_implem
aenc_get_auto_implem_by_encoding_and_format(enum adef_encoding encoding,
					    const struct adef_format *format)
{
	int res;
	const enum adef_encoding *encodings;
	const struct adef_format *supported_formats;

	ULOG_ERRNO_RETURN_VAL_IF(encoding == ADEF_ENCODING_UNKNOWN,
				 EINVAL,
				 AENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(!format, EINVAL, AENC_ENCODER_IMPLEM_AUTO);
	ULOG_ERRNO_RETURN_VAL_IF(
		!supported_implems_count, ENOSYS, AENC_ENCODER_IMPLEM_AUTO);

	for (int i = 0; i < supported_implems_count; i++) {
		enum aenc_encoder_implem implem = supported_implems[i];

		res = implem_ops(implem)->get_supported_encodings(&encodings);
		if (res < 0)
			continue;

		bool encoding_supported = false;
		for (int j = 0; j < res && !encoding_supported; j++) {
			if (encodings[j] == encoding)
				encoding_supported = true;
		}

		if (encoding_supported) {
			res = implem_ops(implem)->get_supported_input_formats(
				&supported_formats);
			if (res < 0)
				continue;
			if (adef_format_intersect(
				    format, supported_formats, res))
				return implem;
		}
	}

	return AENC_ENCODER_IMPLEM_AUTO;
}


static int aenc_config_copy_allocated(const struct aenc_config *config,
				      struct aenc_config *copy)
{
	int ret;
	enum aenc_encoder_implem implem;
	const struct aenc_ops *ops = NULL;
	struct aenc_config_impl *impl_cfg = NULL;

	implem = config->implem;

	ret = aenc_get_implem(&implem);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	ops = implem_ops(implem);
	ULOG_ERRNO_RETURN_ERR_IF(ops == NULL, ENOSYS);

	if (config->implem_cfg != NULL) {
		ULOG_ERRNO_RETURN_ERR_IF((ops->copy_implem_cfg == NULL),
					 ENOSYS);
		ULOG_ERRNO_RETURN_ERR_IF((ops->free_implem_cfg == NULL),
					 ENOSYS);
	}

	/* Deep copy config */
	*copy = *config;
	copy->name = xstrdup(config->name);
	copy->device = xstrdup(config->device);
	copy->implem_cfg = NULL;

	/* Handle implem specific */
	if (config->implem_cfg != NULL) {
		impl_cfg = aenc_config_get_specific(config, implem);
		if (impl_cfg == NULL) {
			ret = -EPROTO;
			goto error;
		}
		/* Copy implem specific */
		ret = ops->copy_implem_cfg(impl_cfg, &copy->implem_cfg);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	(void)aenc_config_free_allocated(copy);
	return ret;
}


int aenc_config_copy(const struct aenc_config *config,
		     struct aenc_config **ret_obj)
{
	int ret;
	struct aenc_config *copy = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	copy = calloc(1, sizeof(*copy));
	if (copy == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		return ret;
	}

	ret = aenc_config_copy_allocated(config, copy);
	if (ret < 0)
		goto error;

	*ret_obj = copy;

	return 0;

error:
	if (copy != NULL)
		(void)aenc_config_free(copy);
	return ret;
}


static int aenc_config_free_allocated(struct aenc_config *config)
{
	int ret, err;
	enum aenc_encoder_implem implem;
	const struct aenc_ops *ops = NULL;

	implem = config->implem;

	ret = aenc_get_implem(&implem);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	ops = implem_ops(implem);
	ULOG_ERRNO_RETURN_ERR_IF(ops == NULL, ENOSYS);

	/* Handle implem specific */
	if (config->implem_cfg != NULL) {
		ULOG_ERRNO_RETURN_ERR_IF((ops->free_implem_cfg == NULL),
					 ENOSYS);
		/* Free implem specific */
		err = ops->free_implem_cfg(config->implem_cfg);
		if (err < 0)
			ULOG_ERRNO("free_implem_cfg", -err);
	}

	free((void *)config->name);
	config->name = NULL;
	free((void *)config->device);
	config->device = NULL;

	return 0;
}


int aenc_config_free(struct aenc_config *config)
{
	int ret;

	if (config == NULL)
		return 0;

	ret = aenc_config_free_allocated(config);

	free(config);

	return ret;
}


int aenc_new(struct pomp_loop *loop,
	     const struct aenc_config *config,
	     const struct aenc_cbs *cbs,
	     void *userdata,
	     struct aenc_encoder **ret_obj)
{
	int res;
	struct aenc_encoder *self = NULL;
	const struct adef_format *supported_formats;
	int nb_supported_formats;

	(void)pthread_once(&instance_counter_is_init,
			   initialize_instance_counter);

	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	self = calloc(1, sizeof(*self));
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, ENOMEM);

	self->base = self; /* For logging */
	self->loop = loop;
	self->cbs = *cbs;
	self->userdata = userdata;
	self->last_timestamp = UINT64_MAX;
	self->enc_id = (atomic_fetch_add(&s_instance_counter, 1) + 1);

	res = aenc_config_copy_allocated(config, &self->config);
	if (res < 0) {
		ULOG_ERRNO("aenc_config_copy_allocated", -res);
		goto error;
	}

	if (self->config.name != NULL)
		res = asprintf(&self->enc_name, "%s", self->config.name);
	else
		res = asprintf(&self->enc_name, "%02d", self->enc_id);
	if (res < 0) {
		res = -ENOMEM;
		ULOG_ERRNO("asprintf", -res);
		goto error;
	}

	res = aenc_get_implem(&self->config.implem);
	if (res < 0)
		goto error;
	self->ops = implem_ops(self->config.implem);

	nb_supported_formats =
		self->ops->get_supported_input_formats(&supported_formats);
	if (nb_supported_formats < 0) {
		res = nb_supported_formats;
		goto error;
	}
	if (!adef_format_intersect(&config->input.format,
				   supported_formats,
				   nb_supported_formats)) {
		res = -EINVAL;
		ULOG_ERRNO("unsupported input format: " ADEF_FORMAT_TO_STR_FMT,
			   -res,
			   ADEF_FORMAT_TO_STR_ARG(&config->input.format));
		goto error;
	}

	/* Enforce configuration and provide default values */

	switch (self->config.encoding) {
	case ADEF_ENCODING_AAC_LC:
		if (((self->config.aac_lc.rate_control ==
		      AENC_RATE_CONTROL_CBR) ||
		     (self->config.aac_lc.rate_control ==
		      AENC_RATE_CONTROL_VBR)) &&
		    (self->config.aac_lc.max_bitrate == 0)) {
			ULOGE("invalid bitrate");
			res = -EINVAL;
			goto error;
		}
		if (self->config.aac_lc.target_bitrate == 0)
			self->config.aac_lc.target_bitrate =
				self->config.aac_lc.max_bitrate;
		break;
	default:
		res = -EINVAL;
		goto error;
	}


	res = self->ops->create(self);
	if (res < 0)
		goto error;

	*ret_obj = self;
	return 0;

error:
	aenc_destroy(self);
	*ret_obj = NULL;
	return res;
}


int aenc_flush(struct aenc_encoder *self, int discard)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->flush(self, discard);
}


int aenc_stop(struct aenc_encoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->stop(self);
}


int aenc_destroy(struct aenc_encoder *self)
{
	int res = 0, err;

	if (self == NULL)
		return 0;

	if (self->ops != NULL)
		res = self->ops->destroy(self);

	AENC_LOGI("aenc instance stats: [%u [%u %u] %u]",
		  self->counters.in,
		  self->counters.pushed,
		  self->counters.pulled,
		  self->counters.out);

	if (res == 0) {
		switch (self->config.encoding) {
		case ADEF_ENCODING_AAC_LC:
			free(self->aac.asc);
			break;
		default:
			break;
		}

		err = aenc_config_free_allocated(&self->config);
		if (err < 0)
			AENC_LOG_ERRNO("aenc_config_free_allocated", -err);

		free(self->enc_name);
		free(self);
	}

	return res;
}


struct mbuf_pool *aenc_get_input_buffer_pool(struct aenc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_pool(self);
}


struct mbuf_audio_frame_queue *
aenc_get_input_buffer_queue(struct aenc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_queue(self);
}


int aenc_get_aac_asc(struct aenc_encoder *self, uint8_t *asc, size_t *asc_size)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((asc != NULL) && (asc_size == NULL), EINVAL);

	if (self->aac.asc == NULL)
		return -EAGAIN;

	if (asc != NULL) {
		if (*asc_size < self->aac.asc_size)
			return -ENOBUFS;

		memcpy(asc, self->aac.asc, self->aac.asc_size);
		*asc_size = self->aac.asc_size;
	} else if (asc_size != NULL) {
		*asc_size = self->aac.asc_size;
	}

	return 0;
}


enum aenc_encoder_implem aenc_get_used_implem(struct aenc_encoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, AENC_ENCODER_IMPLEM_AUTO);

	return self->config.implem;
}
