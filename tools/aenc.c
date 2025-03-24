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

#ifndef ANDROID
#	ifndef _FILE_OFFSET_BITS
#		define _FILE_OFFSET_BITS 64
#	endif /* _FILE_OFFSET_BITS */
#endif /* ANDROID */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <audio-encode/aenc.h>
#include <audio-raw/araw.h>
#include <futils/futils.h>
#include <libpomp.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

#define ULOG_TAG aenc_prog
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#define DEFAULT_IN_BUF_COUNT 30


struct aenc_prog {
	struct pomp_loop *loop;
	struct aenc_encoder *encoder;
	int finishing;
	int flushed;
	int stopped;
	struct aenc_config config;

	struct pomp_timer *encoder_timer;

	struct {
		struct araw_reader *reader;
		unsigned int count;
		unsigned int max_count;
		int finished;
		struct adef_format format;
		uint64_t fake_ts;
		uint32_t fake_ts_increment;
		struct adef_frame frame_info;
		struct mbuf_pool *pool;
		struct pomp_evt *pool_evt;
		int pool_allocated;
		int waiting;
		struct mbuf_audio_frame_queue *queue;
	} input;
	struct {
		unsigned int count;
		int finished;
		FILE *file;
		enum adef_encoding encoding;
	} output;
};


static int s_stopping;
struct pomp_loop *s_loop;
struct aenc_prog *s_prog;


static void finish_idle(void *userdata)
{
	int res;
	struct aenc_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (self->finishing)
		return;

	if ((s_stopping) || (self->input.finished)) {
		self->finishing = 1;

		/* Flush the encoder */
		res = aenc_flush(self->encoder, (s_stopping) ? 1 : 0);
		if (res < 0)
			ULOG_ERRNO("aenc_flush", -res);
	}

	if (self->encoder_timer)
		pomp_timer_clear(self->encoder_timer);
}


static int encode_frame(struct aenc_prog *self)
{
	int res = 0, err;
	void *mem_data;
	uint8_t *in_data;
	size_t in_capacity;
	struct mbuf_audio_frame *in_frame = NULL;
	struct timespec cur_ts = {0, 0};
	struct araw_frame aframe = {0};
	struct mbuf_mem *in_mem;
	struct adef_frame frame_info;

	res = mbuf_pool_get(self->input.pool, &in_mem);
	if (res == -EAGAIN) {
		self->input.waiting = 1;
		res = pomp_evt_signal(self->input.pool_evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_signal", -res);
		return 0;
	} else if (res < 0) {
		ULOG_ERRNO("mbuf_pool_get:input", -res);
		return res;
	}

	res = mbuf_mem_get_data(in_mem, &mem_data, &in_capacity);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		goto out;
	}
	in_data = mem_data;

	switch (self->config.implem) {
	case AENC_ENCODER_IMPLEM_FAKEAAC:
		frame_info.format = self->input.format;
		frame_info.info.timestamp = self->input.fake_ts;
		frame_info.info.timescale = 1000000;
		self->input.fake_ts += self->input.fake_ts_increment;
		frame_info.info.index = self->input.count;
		break;
	default:
		res = araw_reader_frame_read(
			self->input.reader, in_data, in_capacity, &aframe);
		if (res == -ENOENT) {
			res = 0;
			self->input.finished = 1;
			goto out;
		} else if (res < 0) {
			ULOG_ERRNO("araw_reader_frame_read", -res);
			goto out;
		}

		frame_info = aframe.frame;
		break;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &self->input.frame_info.info.timestamp);

	frame_info.info.capture_timestamp =
		frame_info.info.timescale
			? (uint64_t)frame_info.info.timestamp * 1000000 /
				  frame_info.info.timescale
			: 0;

	res = mbuf_audio_frame_new(&frame_info, &in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_new:input", -res);
		return res;
	}

	res = mbuf_audio_frame_set_buffer(in_frame, in_mem, 0, in_capacity);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_set_buffer", -res);
		goto out;
	}

	res = mbuf_audio_frame_finalize(in_frame);
	if (res < 0)
		ULOG_ERRNO("mbuf_audio_frame_finalize", -res);

	res = mbuf_audio_frame_queue_push(self->input.queue, in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_queue_push", -res);
		goto out;
	}

	self->input.frame_info.info.index++;
	self->input.count++;
	self->output.finished = 0;
	if ((self->input.max_count > 0) &&
	    (self->input.count >= self->input.max_count))
		self->input.finished = 1;

out:
	if (in_frame) {
		err = mbuf_audio_frame_unref(in_frame);
		if (err < 0)
			ULOG_ERRNO("mbuf_audio_frame_unref:input", -err);
	}
	err = mbuf_mem_unref(in_mem);
	if (err < 0)
		ULOG_ERRNO("mbuf_mem_unref:input", -err);

	if (!self->input.finished)
		return res;

	ULOGI("encoding is finished (input, count=%d)", self->input.count);
	if (self->encoder_timer)
		pomp_timer_clear(self->encoder_timer);
	err = pomp_loop_idle_add_with_cookie(
		self->loop, &finish_idle, self, self);
	if (err < 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	return res;
}


static void encoder_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct aenc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	res = encode_frame(self);
	if (res < 0)
		ULOG_ERRNO("encode_frame", -res);
}


static void encode_frame_idle(void *userdata)
{
	struct aenc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (self->finishing)
		return;

	res = encode_frame(self);
	if (res < 0) {
		ULOG_ERRNO("encode_frame", -res);
		return;
	}

	if ((!self->input.finished) && (!self->input.waiting) &&
	    (!self->finishing)) {
		res = pomp_loop_idle_add_with_cookie(
			self->loop, &encode_frame_idle, self, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
	}
}


static void pool_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct aenc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (!self->input.waiting)
		return;

	self->input.waiting = 0;

	if ((!self->input.finished) && (!self->finishing)) {
		res = pomp_loop_idle_add_with_cookie(
			self->loop, &encode_frame_idle, self, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
	}
}


static uint64_t get_timestamp(struct mbuf_audio_frame *frame, const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_audio_frame_get_ancillary_data(frame, key, &data);
	if (res < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(ts))
		goto out;
	memcpy(&ts, raw_data, sizeof(ts));

out:
	mbuf_ancillary_data_unref(data);
	return ts;
}


static int frame_output(struct aenc_prog *self,
			struct mbuf_audio_frame *out_frame)
{
	int res = 0;
	struct adef_frame out_info;
	const void *nalu_data;
	size_t nalu;
	const uint8_t *data;
	uint64_t input_time, dequeue_time, output_time;

	if (self->input.waiting) {
		res = pomp_evt_signal(self->input.pool_evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_signal", -res);
	}

	input_time = get_timestamp(out_frame, AENC_ANCILLARY_KEY_INPUT_TIME);
	dequeue_time =
		get_timestamp(out_frame, AENC_ANCILLARY_KEY_DEQUEUE_TIME);
	output_time = get_timestamp(out_frame, AENC_ANCILLARY_KEY_OUTPUT_TIME);

	res = mbuf_audio_frame_get_frame_info(out_frame, &out_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_get_frame_info", -res);
		return res;
	}

	ULOGI("encoded frame #%d "
	      "(dequeue: %.2fms, encode: %.2fms, overall: %.2fms)",
	      out_info.info.index,
	      (float)(dequeue_time - input_time) / 1000.,
	      (float)(output_time - dequeue_time) / 1000.,
	      (float)(output_time - input_time) / 1000.);

	if (self->output.file == NULL)
		return 0;

	/* Write to file */
	res = mbuf_audio_frame_get_buffer(out_frame, &nalu_data, &nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_get_buffer", -res);
		return res;
	}
	data = nalu_data;

	res = fwrite(data, nalu, 1, self->output.file);
	if (res != 1)
		ULOG_ERRNO("fwrite", errno);

	res = mbuf_audio_frame_release_buffer(out_frame, nalu_data);
	if (res < 0) {
		ULOG_ERRNO("mbuf_audio_frame_release_buffer", -res);
		return res;
	}

	return 0;
}


static void frame_output_cb(struct aenc_encoder *enc,
			    int status,
			    struct mbuf_audio_frame *out_frame,
			    void *userdata)
{
	struct aenc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(out_frame == NULL, EINVAL);

	if (status != 0) {
		ULOGE("encoder error, resync required");
		return;
	}

	res = frame_output(self, out_frame);
	if (res < 0)
		ULOG_ERRNO("frame_output", -res);

	self->output.count++;

	if ((self->input.finished) &&
	    (self->output.count >= self->input.count)) {
		ULOGI("encoding is finished (output, count=%d)",
		      self->output.count);
		self->output.finished = 1;
	}
}


static void flush_cb(struct aenc_encoder *enc, void *userdata)
{
	struct aenc_prog *self = userdata;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("encoder is flushed");
	self->flushed = 1;

	/* Stop the encoder */
	res = aenc_stop(self->encoder);
	if (res < 0)
		ULOG_ERRNO("aenc_stop", -res);
}


static void stop_cb(struct aenc_encoder *enc, void *userdata)
{
	struct aenc_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("encoder is stopped");
	self->stopped = 1;

	pomp_loop_wakeup(self->loop);
}


static const struct aenc_cbs aenc_cbs = {
	.frame_output = frame_output_cb,
	.flush = flush_cb,
	.stop = stop_cb,
};


static void sighandler(int signum)
{
	printf("Stopping...\n");
	s_stopping = 1;
	if (s_loop != NULL) {
		int res;
		ULOGI("encoding interrupted");
		res = pomp_loop_idle_add_with_cookie(
			s_loop, &finish_idle, s_prog, s_prog);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
		res = pomp_loop_wakeup(s_loop);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_wakeup", -res);
	}
	signal(SIGINT, SIG_DFL);
}


enum args_id {
	ARGS_ID_IMPLEM = 256,
	ARGS_ID_MIN_BUF_COUNT,
	ARGS_ID_PREFERRED_FORMAT,
};


static const char short_options[] = "hi:f:n:l:o:e:Lr:b:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"implem", required_argument, NULL, ARGS_ID_IMPLEM},
	{"infile", required_argument, NULL, 'i'},
	{"format", required_argument, NULL, 'f'},
	{"count", required_argument, NULL, 'n'},
	{"loop", required_argument, NULL, 'l'},
	{"fake-live", no_argument, NULL, 'L'},
	{"outfile", required_argument, NULL, 'o'},
	{"encoding", required_argument, NULL, 'e'},
	{"rc", required_argument, NULL, 'r'},
	{"bitrate", required_argument, NULL, 'b'},
	{"preferred-min-buf-count",
	 required_argument,
	 NULL,
	 ARGS_ID_MIN_BUF_COUNT},
	{"preferred-format", required_argument, NULL, ARGS_ID_PREFERRED_FORMAT},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - Audio encoding program\n"
	       "Copyright (c) 2023 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	/* clang-format off */
	printf("Usage: %s [options]\n"
	       "Options:\n"
	       "  -h | --help                          "
		       "Print this message\n"
	       "       --implem <implem_name>          "
		       "Force the implementation to use\n"
	       "  -i | --infile <file_name>            "
		       "WAV input file (*.wav)\n"
	       "  -f | --format <format>               "
		       "Input file data format (e.g. \"PCM_16_44KHZ_STEREO\")\n"
	       "  -n | --count <n>                     "
		       "Encode at most n frames\n"
	       "       --preferred-min-buf-count <n>   "
		       "Prefered minimum input buffer count\n"
	       "  -L | --fake-live                     "
		       "Fake live pipeline: schedule the "
		       "encoder at the input sample rate\n"
	       "  -o | --outfile <file_name>           "
		       "AAC_LC output file (.aac)\n"
	       "  -e | --encoding <enc>                "
		       "Output encoding (e.g. \"AAC_LC\")\n"
	       "  -r | --rc <val>                      "
		       "Rate control algorithm (e.g. \"CBR\" or \"VBR\";"
		       "default is \"CBR\")\n"
	       "  -b | --bitrate <br>                  "
		       "Bitrate (bit/s) for CBR and VBR rate-control; "
		       "default is 320000\n"
	       "       --preferred-format <format>     "
		       "Preferred output format (e.g. \"RAW\", \"ADTS\")\n"
	       "       --preferred-min-buf-count <n>   "
		       "Preferred minimum input buffer count\n"
	       "\n",
	       prog_name);
	/* clang-format on */
}


int main(int argc, char **argv)
{
	int res = 0, status = EXIT_SUCCESS;
	int idx, c;
	struct aenc_prog *self;
	char *input = NULL, *output = NULL;
	struct araw_reader_config reader_config;
	struct timespec cur_ts = {0, 0};
	ssize_t res1;
	size_t in_capacity;
	uint64_t start_time = 0, end_time = 0;
	int use_timer = 0;
	int auto_implem_by_encoding = 0;
	unsigned int preferred_min_buf_count = 0;
	enum aenc_rate_control rate_control = AENC_RATE_CONTROL_CBR;
	unsigned int max_bitrate = 320000;
	unsigned int target_bitrate = 0; /* TODO */
	unsigned int preferred_frame_length = 1024; /* TODO */
	enum adef_aac_data_format preferred_output_format =
		ADEF_AAC_DATA_FORMAT_UNKNOWN;

	s_stopping = 0;
	s_loop = NULL;
	s_prog = NULL;

	welcome(argv[0]);

	signal(SIGINT, sighandler);
	signal(SIGTERM, &sighandler);

	/* Context allocation */
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_prog = self;

	self->loop = pomp_loop_new();
	if (!self->loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_loop = self->loop;

	memset(&self->config, 0, sizeof(self->config));
	self->config.implem = AENC_ENCODER_IMPLEM_AUTO;
	self->config.encoding = ADEF_ENCODING_AAC_LC;

	/* Default values */
	self->config.preferred_frame_length = preferred_frame_length;

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			goto out;

		case ARGS_ID_IMPLEM:
			self->config.implem =
				aenc_encoder_implem_from_str(optarg);
			break;

		case 'i':
			input = optarg;
			break;

		case 'f':
			res = adef_format_from_str(optarg, &self->input.format);
			if (res != 0) {
				ULOG_ERRNO("adef_format_from_str", -res);
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			break;

		case 'n':
			sscanf(optarg, "%d", &self->input.max_count);
			break;

		case 'L':
			use_timer = 1;
			break;

		case 'o':
			output = optarg;
			break;

		case 'e':
			self->config.encoding = adef_encoding_from_str(optarg);
			if (self->config.encoding == ADEF_ENCODING_UNKNOWN) {
				ULOGE("unknown encoding: '%s'", optarg);
				usage(argv[0]);
				status = EXIT_FAILURE;
				goto out;
			}
			auto_implem_by_encoding = 1;
			break;

		case 'r':
			rate_control = aenc_rate_control_from_str(optarg);
			break;

		case 'b':
			sscanf(optarg, "%d", &max_bitrate);
			break;

		case ARGS_ID_MIN_BUF_COUNT:
			sscanf(optarg, "%u", &preferred_min_buf_count);
			break;

		case ARGS_ID_PREFERRED_FORMAT:
			preferred_output_format =
				adef_aac_data_format_from_str(optarg);
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	if (auto_implem_by_encoding &&
	    self->config.implem == AENC_ENCODER_IMPLEM_AUTO)
		self->config.implem =
			aenc_get_auto_implem_by_encoding(self->config.encoding);

	/* Input file */
	switch (self->config.implem) {
	case AENC_ENCODER_IMPLEM_FAKEAAC:
		printf("Input: none (fake)\n");
		if (!adef_is_format_valid(&self->input.format))
			self->input.format = adef_pcm_16b_48000hz_stereo;
		printf("Format: " ADEF_FORMAT_TO_STR_FMT "\n",
		       ADEF_FORMAT_TO_STR_ARG(&self->input.format));
		self->input.fake_ts = 0;
		self->input.fake_ts_increment =
			1000000ULL * self->config.preferred_frame_length /
			self->input.format.sample_rate;
		printf("\n");
		break;
	default:
		if (!input) {
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
		memset(&reader_config, 0, sizeof(reader_config));
		reader_config.format = self->input.format;
		reader_config.frame_length =
			self->config.preferred_frame_length;

		res = araw_reader_new(
			input, &reader_config, &self->input.reader);
		if (res < 0) {
			ULOG_ERRNO("araw_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}

		res = araw_reader_get_config(self->input.reader,
					     &reader_config);
		if (res < 0) {
			ULOG_ERRNO("araw_reader_get_config", -res);
			status = EXIT_FAILURE;
			goto out;
		}

		self->input.format = reader_config.format;

		printf("Input: file '%s'\n", input);
		printf("Format: " ADEF_FORMAT_TO_STR_FMT "\n",
		       ADEF_FORMAT_TO_STR_ARG(&self->input.format));
		printf("\n");
		break;
	}

	/* Output file */
	if (output) {
		self->output.file = fopen(output, "wb");
		if (!self->output.file) {
			ULOGE("failed to open file '%s'", output);
			status = EXIT_FAILURE;
			goto out;
		}

		printf("Output: file '%s'\n", output);
	}
	self->output.encoding = self->config.encoding;
	self->config.output.preferred_format = preferred_output_format;

	/* Initialize the encoder */
	self->config.input.format = self->input.format;

	switch (self->config.encoding) {
	case ADEF_ENCODING_PCM:
		break;
	case ADEF_ENCODING_AAC_LC:
		self->config.aac_lc.rate_control = rate_control;
		self->config.aac_lc.max_bitrate = max_bitrate;
		self->config.aac_lc.target_bitrate = target_bitrate;
		break;
	default:
		break;
	}

	res = aenc_new(
		self->loop, &self->config, &aenc_cbs, self, &self->encoder);
	if (res < 0) {
		ULOG_ERRNO("aenc_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Input buffer pool */
	self->input.pool = aenc_get_input_buffer_pool(self->encoder);
	if (self->input.pool == NULL) {
		switch (self->config.implem) {
		case AENC_ENCODER_IMPLEM_FAKEAAC:
			in_capacity = 0;
			break;
		default:
			res1 = araw_reader_get_min_buf_size(self->input.reader);
			if (res1 < 0) {
				ULOG_ERRNO("araw_reader_get_min_buf_size",
					   (int)-res1);
				status = EXIT_FAILURE;
				goto out;
			}
			in_capacity = res1;
			break;
		}

		/* A maximum size is given to the pool to ensure that it won't
		 * contain the whole WAV file. */
		res = mbuf_pool_new(mbuf_mem_generic_impl,
				    in_capacity,
				    DEFAULT_IN_BUF_COUNT,
				    MBUF_POOL_SMART_GROW,
				    256,
				    "aenc_default_pool",
				    &self->input.pool);
		if (res < 0) {
			ULOG_ERRNO("mbuf_pool_new:input", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		self->input.pool_allocated = 1;
	}

	/* Input buffer pool fd event */
	self->input.pool_evt = pomp_evt_new();
	if (self->input.pool_evt == NULL) {
		ULOG_ERRNO("pomp_evt_new", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	res = pomp_evt_attach_to_loop(
		self->input.pool_evt, self->loop, &pool_event_cb, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Input buffer queue */
	self->input.queue = aenc_get_input_buffer_queue(self->encoder);
	if (self->input.queue == NULL) {
		ULOG_ERRNO("aenc_get_input_buffer_queue", EPROTO);
		status = EXIT_FAILURE;
		goto out;
	}

	self->input.frame_info.format = self->input.format;

	/* Start */
	if (use_timer) {
		self->encoder_timer =
			pomp_timer_new(self->loop, encoder_timer_cb, self);
		if (!self->encoder_timer) {
			ULOGE("pomp_timer_new failed");
			status = EXIT_FAILURE;
			goto out;
		}
		uint32_t frame_time_ms = 1000 *
					 self->config.preferred_frame_length /
					 self->input.format.sample_rate;
		if (frame_time_ms == 0)
			frame_time_ms = 1;
		res = pomp_timer_set_periodic(
			self->encoder_timer, 1, frame_time_ms);
		if (res != 0) {
			ULOG_ERRNO("pomp_timer_set_periodic", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	} else {
		res = pomp_loop_idle_add_with_cookie(
			self->loop, encode_frame_idle, self, self);
		if (res < 0) {
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -res);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &start_time);

	while (!self->stopped)
		pomp_loop_wait_and_process(self->loop, -1);

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &end_time);
	printf("\nOverall time: %.2fs / %.2ffps\n",
	       (float)(end_time - start_time) / 1000000.,
	       self->output.count * 1000000. / (float)(end_time - start_time));

out:
	/* Cleanup */
	if (self) {
		if (self->encoder_timer) {
			pomp_timer_clear(self->encoder_timer);
			res = pomp_timer_destroy(self->encoder_timer);
			if (res < 0)
				ULOG_ERRNO("pomp_timer_destroy", -res);
		}

		if (self->input.reader) {
			res = araw_reader_destroy(self->input.reader);
			if (res < 0)
				ULOG_ERRNO("araw_reader_destroy", -res);
		}

		if (self->input.pool_evt != NULL) {
			if (pomp_evt_is_attached(self->input.pool_evt,
						 self->loop)) {
				res = pomp_evt_detach_from_loop(
					self->input.pool_evt, self->loop);
				if (res < 0)
					ULOG_ERRNO("pomp_evt_detach_from_loop",
						   -res);
			}

			res = pomp_evt_destroy(self->input.pool_evt);
			if (res < 0)
				ULOG_ERRNO("pomp_evt_destroy", -res);
		}

		if (self->input.pool_allocated) {
			res = mbuf_pool_destroy(self->input.pool);
			if (res < 0)
				ULOG_ERRNO("mbuf_pool_destroy:input", -res);
		}

		if (self->encoder) {
			res = aenc_destroy(self->encoder);
			if (res < 0)
				ULOG_ERRNO("aenc_destroy", -res);
		}

		if (self->loop) {
			res = pomp_loop_idle_remove_by_cookie(self->loop, self);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_idle_remove_by_cookie",
					   -res);
			res = pomp_loop_destroy(self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_destroy", -res);
		}

		if (self->output.file)
			fclose(self->output.file);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}
