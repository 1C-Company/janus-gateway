#include "speech_mix.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

/************************************************************************/
/*		config															*/
/************************************************************************/
#define N_MIX_THREADS                           0
#define SPEECH_MIX_SAMPLE_RATE                  48000
#define SPEECH_MAX_ACTIVE_SET                   4
#define SPEECH_MIX_MAX_SET                      8
#define BIG_DELAY_FOR_SPEED_ACCELERATION_MS     500
#define SOFT_ACCELERATION_TIMEOUT_MS            (10 * 1000)
#define SMALL_ACCEPTABLE_DELAY_MS               100
#define COOLDOWN_THR                            0
#define HANGOVER_THR                            20
#define ACTIVITY_THR                            40
#define ACTIVITY_FORGET_FACTOR                  64


#define MAX( x, y )             ( (x)>(y)?(x):(y) )
#define MIN( x, y )             ( (x)<(y)?(x):(y) )
#define ABS( x )                ( (x)>=0?(x):-(x) )

#if N_MIX_THREADS > 1
/************************************************************************/
/*	Threads support														*/
/************************************************************************/
#if	defined _WIN32
#include <process.h>
#include <windows.h>
#define pthread_t HANDLE
#define pthread_create(thhandle,attr,thfunc,tharg) (int)((*(thhandle)=(HANDLE)_beginthreadex(NULL,0,(unsigned int (__stdcall *)(void *))thfunc,tharg,0,NULL))==NULL)
#define pthread_join(thread, result) ((WaitForSingleObject((thread),INFINITE)!=WAIT_OBJECT_0) || !CloseHandle(thread))
#else
#include <pthread.h>
#endif

static void mix_update_activity(speech_mixer_t* mix, unsigned now_ms, int nframe, int step);

typedef struct
{
	speech_mixer_t* m;
	unsigned now_ms;
	int nframe;
	int step;
} thread_param_t;

static void * thread_func_pthread(void * arg)
{
	thread_param_t* par = (thread_param_t*)arg;// h->job;
	speech_mixer_t* mix = par->m;
	int nframe = par->nframe;
	int step = par->step;
	int now_ms = par->now_ms;
	mix_update_activity(mix, now_ms, nframe, step);
	return NULL;
}

static void mix_thread_pool_run(thread_param_t params[], int njobs)
{
	pthread_t* hthreads = calloc(sizeof(pthread_t), njobs);
	int i;
	for (i = 0; i < njobs; i++)
	{
		pthread_create(&hthreads[i], NULL, thread_func_pthread, params+i);
	}
	for (i = 0; i < njobs; i++)
	{
		void* status;
		pthread_join(hthreads[i], &status);
		(void)status;
	}
}

#endif	// #if N_MIX_THREADS > 1

/************************************************************************/
/*		Debug events count												*/
/************************************************************************/
enum
{
	RESET = 0,
	SSRC_CHANGE,
	DISCARD_TOO_SHORT,
	DISCARD_BAD_SSRC,
	DISCARD_DUP,
	DISCARD_TOO_LATE,
	DISCARD_OVERFLOW,
	DISCARD_BY_HARRYUP,
	PACKET_FEC_DECODE,
	PACKET_FEC_DECODE_FAIL,
	PACKET_NORMAL_DECODE,
	PACKET_NORMAL_DECODE_FAIL,
	PACKET_PLC_DECODE,
	PACKET_PLC_DECODE_FAIL,
	PACKET_DTX_DECODE,
	PACKET_DTX_DECODE_FAIL,
	PACKETS_ACCEPTED,
	PACKETS_SN_SPAN,
};

static const char* const msg[] = {
	"reset",
	"SSRC change",
	"discarded too short packets",
	"discarded wrong SSRC packets",
	"discarded duplicated packets",
	"discarded too late packets",
	"packets discarded due to fifo overflow",
	"packets discard by harryup",
	"packets decoded by FEC",
	"packets decode by FEC error",
	"packets decoded ok",
	"packets decoded error",
	"packets decoded by PLC",
	"packets decoded by PLC error",
	"packets decoded by DTX",
	"packets decoded by DTX error",
	"packets accepted",
	"packets SN span"
};


static void dbg_count(rtp_opus_dec_t* h, unsigned id)
{
	assert(id < sizeof(msg) / sizeof(msg[0]));
	h->dbg_count[id]++;
}

static void dbg_count_add(rtp_opus_dec_t* h, unsigned id, int inc)
{
	assert(id < sizeof(msg) / sizeof(msg[0]));
	h->dbg_count[id] += inc;
}


/************************************************************************/
/*		RTP parse														*/
/************************************************************************/
static unsigned read32(const unsigned char* p)
{
	return ((p[0] * 256UL + p[1]) * 256 + p[2]) * 256 + p[3];
}
static unsigned read16(const unsigned char* p)
{
	return p[0] * 256UL + p[1];
}
#define RTP_SN(p)	 read16((unsigned char*)(p)+2)
#define RTP_TS(p)	 read32((unsigned char*)(p)+4)
#define RTP_SSRC(p) read32((unsigned char*)(p)+8)

static int rtp_header_len(const unsigned char* rtp, int rtp_bytes)
{
	int len = 12 + (rtp[0] & 15) * 4;					// + CSRC_count x 4 bytes
	if (rtp[0] & 16)
	{
		len += read16(rtp + len + 2) * 4 + 4;		 // + header extension
	}
	return len;
}

static int rtp_payload_len(const unsigned char* rtp, int rtp_bytes)
{
	if (rtp[0] & 32)
	{
		rtp_bytes -= rtp[rtp_bytes - 1] - 1;				// - padding
	}
	return rtp_bytes - rtp_header_len(rtp, rtp_bytes);
}

/************************************************************************/
/*		init/close														*/
/************************************************************************/
static void opus_dec_init(rtp_opus_dec_t* h, const char* debug_tag)
{
	int error;
	memset(h, 0, sizeof(*h));
	h->decoder = opus_decoder_create(SPEECH_MIX_SAMPLE_RATE, 1, &error);
	h->opus_nominal_frame_len = 960;
	strncpy(h->debug_tag, debug_tag, sizeof(h->debug_tag)-1);
}

static void opus_dec_close(rtp_opus_dec_t* h)
{
	if (h->decoder)
	{
		opus_decoder_destroy(h->decoder);
	}
}

/************************************************************************/
/*		RTP validation													*/
/************************************************************************/
static int empty(const rtp_opus_dec_t* h)
{
	return h->fifo_wr == h->fifo_rd;
}

static int accept_ssrc(rtp_opus_dec_t* h, unsigned ssrc)
{
	if (!h->new_ssrc && !h->cur_ssrc)
	{
		h->new_ssrc = h->cur_ssrc = ssrc;
		dbg_count(h, RESET);
	}

	if (h->cur_ssrc != ssrc)
	{
		if (!h->new_ssrc_cnt || h->new_ssrc == ssrc)
		{
			h->new_ssrc = ssrc;
			if (++h->new_ssrc_cnt > 3 && empty(h))
			{
				// switch to new SSRC
				// SSRC changed if have 3 packets with new SSRC, and no packets with old SSRC
				// ~~~~~~~~~~~~~~~~~~
				h->cur_ssrc = ssrc;
				h->new_ssrc_cnt = 0;
				dbg_count(h, SSRC_CHANGE);
			}
		}
		else
		{
			h->new_ssrc_cnt = 1;
			h->new_ssrc = ssrc;
		}
	}
	else
	{
		h->new_ssrc_cnt = 0;
	}

	return (h->cur_ssrc == ssrc);
}

static int accept_sn(rtp_opus_dec_t* h, int sn)
{
	unsigned i;
	int delay_output;

	if (h->new_ssrc == h->cur_ssrc)
	{
		assert(empty(h));
		h->expected_sn = sn;
		h->new_ssrc ^= ~0;
		return 1;
	}

	for (i = h->fifo_rd; i != h->fifo_wr; i++)
	{
		if (sn == (int)RTP_SN(h->rtp_fifo[i % SPEECH_MIX_N_RTP_FIFO].rtp))
		{
			dbg_count(h, DISCARD_DUP);
			return 0;
		}
	}

	delay_output = (short)(sn - h->expected_sn);
	if (delay_output >= 0)
	{
		return 1;
	}

	dbg_count(h, DISCARD_TOO_LATE);
	return 0;
}

/************************************************************************/
/*		FIFO buffer														*/
/************************************************************************/
static int rtp_opus_fifo_put_packet(rtp_opus_dec_t* h, const unsigned char* rtp, int rtp_bytes, unsigned now_ms)
{
	rtp_packet_t* p = h->rtp_fifo + h->fifo_wr % SPEECH_MIX_N_RTP_FIFO;

	h->count_packets_arrived++;
	if (rtp_bytes < 12)
	{
		dbg_count(h, DISCARD_TOO_SHORT);
		return 0;
	}
	if (!accept_ssrc(h, RTP_SSRC(rtp)))
	{
		dbg_count(h, DISCARD_BAD_SSRC);
		return 0;
	}
	if (!accept_sn(h, RTP_SN(rtp)) && now_ms - h->last_accept_ms < 2000u)
	{
		return 0;
	}
	assert((h->fifo_wr - h->fifo_rd) <= SPEECH_MIX_N_RTP_FIFO);
	if (h->fifo_wr - h->fifo_rd >= SPEECH_MIX_N_RTP_FIFO)
	{
		dbg_count(h, DISCARD_OVERFLOW);
		return 0;
	}
	assert((h->fifo_wr - h->fifo_rd) < SPEECH_MIX_N_RTP_FIFO);
	h->last_accept_ms = now_ms;
	p->arrival_ms = now_ms;
	memcpy(p->rtp, rtp, rtp_bytes);
	p->rtp_bytes = rtp_bytes;
	//MemoryBarrier();
	h->fifo_wr++;

	if (h->dbg_count[PACKETS_ACCEPTED])
	{
		dbg_count_add(h, PACKETS_SN_SPAN, (short)(RTP_SN(rtp) - h->sn_prev));
	}
	else
	{
		dbg_count(h, PACKETS_SN_SPAN);
	}
	dbg_count(h, PACKETS_ACCEPTED);
	h->sn_prev = RTP_SN(rtp);


	return 1;
}

static rtp_packet_t* peek_next_packet(rtp_opus_dec_t* h, unsigned now_ms, int* current_delay)
{
	rtp_packet_t* p0, * p = NULL;
	if (!empty(h))
	{
		unsigned i, gap_sn_min, gap_sn, ts_range = 0;
		gap_sn_min = 0x10000;
		for (i = h->fifo_rd; i != h->fifo_wr; i++)
		{
			gap_sn = (short)(RTP_SN(h->rtp_fifo[i % SPEECH_MIX_N_RTP_FIFO].rtp) - h->expected_sn);
			if ((unsigned short)gap_sn < gap_sn_min)
			{
				gap_sn_min = gap_sn;
				p = h->rtp_fifo + i % SPEECH_MIX_N_RTP_FIFO;
			}
		}

		p0 = h->rtp_fifo + h->fifo_rd % SPEECH_MIX_N_RTP_FIFO;
		*current_delay = now_ms - p0->arrival_ms + (RTP_TS(p0->rtp) - RTP_TS(p->rtp)) / 48;

		for (i = h->fifo_rd; i != h->fifo_wr; i++)
		{
			int ts = RTP_TS(h->rtp_fifo[i % SPEECH_MIX_N_RTP_FIFO].rtp);
			if ((int)(ts - RTP_TS(p->rtp)) > (int)ts_range)
			{
				ts_range = ts - RTP_TS(p->rtp);
			}
		}
		*current_delay = MAX((unsigned)*current_delay, ts_range / 48);
	}
	else
	{
		*current_delay = 0;
	}
	return p;
}

static void release_old_packets(rtp_opus_dec_t* h)
{
	while (!empty(h))
	{
		int delay = (short)(RTP_SN(h->rtp_fifo[h->fifo_rd % SPEECH_MIX_N_RTP_FIFO].rtp) - h->expected_sn);
		if (!delay)
		{
			break;
		}
		if (delay >= 0)
		{
			break;
		}
		h->fifo_rd++;
	}
	assert((h->fifo_wr - h->fifo_rd) <= SPEECH_MIX_N_RTP_FIFO);
}

static int rtp_opus_fifo_get_next(rtp_opus_dec_t* h, unsigned now_ms, short* pcm)
{
	int current_delay;
	int len = 0;
	int thr = SMALL_ACCEPTABLE_DELAY_MS;
	rtp_packet_t* p = peek_next_packet(h, now_ms, &current_delay);

	if (p && (!h->delay_running_min_time || current_delay < h->delay_running_min))
	{
		h->delay_running_min_time = now_ms;
		h->delay_running_min = current_delay;
	}
	if (p && (now_ms - h->delay_running_min_time > SOFT_ACCELERATION_TIMEOUT_MS))
	{
		thr = MAX((unsigned)h->delay_running_min, SMALL_ACCEPTABLE_DELAY_MS);
	}

	if (p && current_delay > BIG_DELAY_FOR_SPEED_ACCELERATION_MS && --h->hurryup_decimator < 0)
	{
		// Very big delay
		h->hurryup_decimator = 4;
		h->expected_ts = RTP_TS(p->rtp);
		h->expected_sn = RTP_SN(p->rtp) + 1;
		release_old_packets(h);
		p = peek_next_packet(h, now_ms, &current_delay);
		dbg_count(h, DISCARD_BY_HARRYUP);
	}

	if (p && thr && current_delay > thr)
	{
		// reduce delay by DTX keep-alive drop
		int payload_bytes = rtp_payload_len(p->rtp, p->rtp_bytes);
		if (payload_bytes <= 2)
		{
			h->expected_ts = RTP_TS(p->rtp);
			h->expected_sn = RTP_SN(p->rtp) + 1;
			release_old_packets(h);
			p = peek_next_packet(h, now_ms, &current_delay);
			dbg_count(h, DISCARD_BY_HARRYUP);
		}
	}

	if (p)
	{
		unsigned sn = RTP_SN(p->rtp);
		unsigned ts = RTP_TS(p->rtp);
		int payload_bytes = rtp_payload_len(p->rtp, p->rtp_bytes);
		int header_len = rtp_header_len(p->rtp, p->rtp_bytes);
		const unsigned char* payload = p->rtp + header_len;
		int dts = ts - h->expected_ts;
		int dsn = sn - h->expected_sn;
		int decode_flag;

		if (dts > 0 && current_delay > thr)
		{
			// reduce delay by decreasing TS interval
			int d = MIN(dts, (current_delay - thr) * 48);
			h->expected_ts += d;
			dts -= d;
			current_delay -= d / 48;
		}

		if (dsn == 1 && dts == h->opus_nominal_frame_len && current_delay <= thr)
		{
			// single gap: use FEC, keep packet
			len = opus_decode(h->decoder, payload, payload_bytes, pcm, dts, 1);	 // decode FEC
			h->expected_sn = RTP_SN(p->rtp);
			if (len > 0)
			{
				dbg_count(h, PACKET_FEC_DECODE);
				h->expected_ts = RTP_TS(p->rtp) - len;
			}
			else
			{
				dbg_count(h, PACKET_FEC_DECODE_FAIL);
			}
		}

		decode_flag = 0;
		if (len <= 0)
		{
			decode_flag = (dts <= 0 && current_delay >= 0 && !dsn)	// next packet on schedule
				|| current_delay > thr								// big delay
				|| (!h->opus_dtx && !dsn);							// next non-DTX packet
		}

		if (decode_flag)
		{
			h->opus_dtx = (payload_bytes <= 3);
			len = opus_decode(h->decoder, payload, payload_bytes, pcm, SPEECH_MIX_MAX_PCM, 0);
			h->expected_sn = RTP_SN(p->rtp) + 1;
			release_old_packets(h);
			if (len > 0)
			{
				h->expected_ts = RTP_TS(p->rtp);
				h->opus_nominal_frame_len = len;
				dbg_count(h, PACKET_NORMAL_DECODE);
			}
			else
			{
				dbg_count(h, PACKET_NORMAL_DECODE_FAIL);
			}
		}
	}
	if (len <= 0)
	{
		if (h->opus_nominal_frame_len > 0)
		{
			len = opus_decode(h->decoder, NULL, 0, pcm, h->opus_nominal_frame_len, 0);
		}
		if (len <= 0)
		{
			len = 960;
			memset(pcm, 0, len * sizeof(short));
		}
		if (h->opus_dtx)
		{
			if (len > 0)
			{
				dbg_count(h, PACKET_DTX_DECODE);
			}
			else
			{
				dbg_count(h, PACKET_DTX_DECODE_FAIL);
			}
		}
		else
		{
			if (len > 0)
			{
				dbg_count(h, PACKET_PLC_DECODE);
			}
			else
			{
				dbg_count(h, PACKET_PLC_DECODE_FAIL);
			}
		}
	}
	if (len > 0)
	{
		if (p)
		{
			h->stat_delay_max = MAX(current_delay, h->stat_delay_max);
			h->stat_delay_sum += current_delay;
			h->stat_delay_last = current_delay;
			h->stat_delay_count++;
		}

		h->expected_ts += len;
	}
	return MAX(0, len);
}


/************************************************************************/
/*		Opus encoder													*/
/************************************************************************/
void opus_enc_init(opus_enc_t * enc, const char* debug_tag)
{
	int error;
	memset(enc, 0, sizeof(opus_enc_t));
	enc->encoder = opus_encoder_create(SPEECH_MIX_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &error);
	enc->agc.gain = 1.0f;
}

void opus_enc_close(opus_enc_t* enc)
{
	opus_encoder_destroy(enc->encoder);
}

static void opus_enc_encode(opus_enc_t* enc, const short* pcm, int npcm)
{
	enc->bitstream_bytes = opus_encode(enc->encoder, pcm, npcm, enc->bitstream, SPEECH_MIX_MAX_CODED_FRAME_BYTES);
}

static void opus_enc_take_bitstream(opus_enc_t* enc, opus_enc_t* src)
{
	enc->bitstream_bytes = src->bitstream_bytes;
	memcpy(enc->bitstream, src->bitstream, src->bitstream_bytes);
}

/************************************************************************/
/*		Mixer															*/
/************************************************************************/
static short clip16(int s)
{
	s = (s > 0x7fff) ? 0x7fff : (s < -0x8000) ? -0x8000 : s;
	return (short)s;
}

static void v_add_32_16(int * sum, const short * pcm, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		sum[i] += pcm[i];
	}
}

static void v_sub3_32_16(const int * a, const short * b, int n, short * dst, float gain0, float gain1)
{
	int i;
	if (gain0 == 1.0f && gain1 == 1.0f)
		{
		for (i = 0; i < n; i++)
		{
			dst[i] = clip16(a[i] - b[i]);
		}
	}
	else
	{
		float dgain = (gain1 - gain0)/n;
		for (i = 0; i < n; i++)
		{
				dst[i] = clip16(a[i] - (int)(b[i] * (gain0 + dgain * i) + 0.5f));
		}
	}
}

static void v_clip_16(const int * sum, int n, short * dst)
{
	int i;
	for (i = 0; i < n; i++)
	{
		dst[i] = clip16(sum[i]);
	}
}

static int v_sum_abs_16(const short* s, int n)
{
	int i;
	int sum = 0;
	for (i = 0; i < n; i++)
	{
		sum += ABS(s[i]);
	}
	return sum;
}

int speech_mixer_init(speech_mixer_t * participant, const char* debug_tag)
{
	memset(participant, 0, sizeof(*participant));
	opus_dec_init(&participant->opus_dec, debug_tag);
	opus_enc_init(&participant->opus_enc, debug_tag);
	return 1;
}

void speech_mixer_close(speech_mixer_t * participant)
{
	opus_dec_close(&participant->opus_dec);
	opus_enc_close(&participant->opus_enc);
}

static void speech_mix_src_put_samples(speech_mixer_t * h, void * pcm, int n)
{
	int ncopy = MIN(n, SPEECH_MIX_MAX_PCM - h->nsrc);
	assert(h->nsrc + n <= SPEECH_MIX_MAX_PCM);
	memcpy(h->src_pcm + h->nsrc, pcm, ncopy * sizeof(short));
	h->nsrc += ncopy;
}

static void speech_mix_src_drop_samples(speech_mixer_t * h, int n)
{
	n = MIN(n, h->nsrc);
	h->nsrc -= n;
	if (h->nsrc > 0)
	{
		memmove(h->src_pcm, h->src_pcm + n, h->nsrc * sizeof(short));
	}
}

static float cost(const speech_mixer_t * h)
{
	return h->mean_activity/ACTIVITY_FORGET_FACTOR + h->instant_activity*0.25f;
}

static int activity_gt(speech_mixer_t* h1, speech_mixer_t* h2)
{
	float a1 = cost(h1);
	float a2 = cost(h2);
	return a1 > a2;
}

static int cooldown_gt(speech_mixer_t* h1, speech_mixer_t* h2)
{
	return (h1->activity_countdown > h2->activity_countdown);
}

static void mix_update_activity(speech_mixer_t* mix, unsigned now_ms, int nframe, int step)
{
	speech_mixer_t* m;
	int i;
	for (m = mix; m; )
	{
		while (m->nsrc < nframe)
		{
			short pcm[48 * 120];
			int npcm = rtp_opus_fifo_get_next(&m->opus_dec, now_ms, pcm);
			speech_mix_src_put_samples(m, pcm, npcm);
		}

		float activity = (float)v_sum_abs_16(m->src_pcm, nframe) / nframe / 0x8000;
		m->instant_activity = activity;
		m->mean_activity += (activity - m->mean_activity) * 0.01f;
		for (i = 0; m && i < step; i++)
		{
			m = m->next;
		}
	}
}

static void pcm16_fadein(short* pcm, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		pcm[i] = (short)(pcm[i] * i / n);
	}
}

static void pcm16_fadeout(short* pcm, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		pcm[i] = (short)(pcm[i] * (n - i) / n);
	}
}

static int v_max_abs(const int* pcm32, int n)
{
	int i;
	int max_val = 0;
	for (i = 0; i < n; i++)
	{
		int v = pcm32[i];
		v = ABS(v);
		max_val = MAX(max_val, v);
	}
	return max_val;

}

static void v_apply_gain(int* pcm32, int n, float gain0, float gain1)
{
	int i;
	float dgain = (gain1 - gain0)/n;
	if (gain0 == 1.0f && gain1 == 1.0f)
	{
		return;
	}
	for (i = 0; i < n; i++)
	{
		pcm32[i] = (int)((float)pcm32[i] * (gain0 + dgain * i) + 0.5f);
	}
}

static void agc_run(agc_t* agc, int* pcm32, int n)
{
	float gain, max_val = (float)v_max_abs(pcm32, n);

	agc->envelope *= 0.999f;
	agc->envelope = MAX(agc->envelope, max_val);
	if (agc->envelope * agc->gain > 32000)
	{
		gain = 32000 / agc->envelope;
	}
	else
	{
		gain = agc->gain*0.95f + 0.05f;
		if (gain > 0.9999f)
		{
			gain = 1.0f;
		}
	}
	v_apply_gain(pcm32, n, agc->gain, gain);
	agc->gain = gain;
}

speech_mixer_t * speech_mix_run(speech_mixer_t* mix, opus_enc_t * commom_enc, int nframe, unsigned now_ms)
{
	int i, nmix_set = 0;
	speech_mixer_t* m;
	speech_mixer_t* mix_set[SPEECH_MIX_MAX_SET] = { 0, };
	int sum[SPEECH_MIX_MAX_PCM] = { 0, };
	short sum16[SPEECH_MIX_MAX_PCM] = { 0, };
	int enc_count = 0;

#if N_MIX_THREADS > 1
	{
		int ntheads = N_MIX_THREADS;
		thread_param_t par[N_MIX_THREADS];
		for (i = 0, m = mix; m && i < ntheads; i++, m = m->next)
		{
			par[i].m = m;
			par[i].now_ms = now_ms;
			par[i].nframe = nframe;
			par[i].step = ntheads;
		}
		if (i < ntheads)
		{
			return;
		}
		mix_thread_pool_run(par, ntheads);
	}
#else
	mix_update_activity(mix, now_ms, nframe, 1);
#endif

	for (m = mix; m; m = m->next)
	{
		for (i = nmix_set - 1; i >= 0 && activity_gt(m, mix_set[i]); i--)
		{
			if (i + 1 < SPEECH_MAX_ACTIVE_SET)
				mix_set[i + 1] = mix_set[i];
		}
		if (i + 1 < SPEECH_MAX_ACTIVE_SET)
			mix_set[i + 1] = m;
		nmix_set = MIN(nmix_set + 1, SPEECH_MAX_ACTIVE_SET);
		if (m->activity_countdown > COOLDOWN_THR)
		{
			m->activity_countdown--;
		}
	}


	for (i = 0; i < nmix_set; i++)
	{
		mix_set[i]->activity_countdown = ACTIVITY_THR;
	}

	nmix_set = 0;
	for (m = mix; m; m = m->next)
	{
		if (m->activity_countdown <= COOLDOWN_THR)
		{
			continue;
		}

		for (i = nmix_set - 1; i >= 0 && cooldown_gt(m, mix_set[i]); i--)
		{
			if (i + 1 < SPEECH_MIX_MAX_SET)
			{
				mix_set[i + 1] = mix_set[i];
			}
			else
			{
				assert(mix_set[i]->opus_enc.state != MIX_NOT_MIXED);
				mix_set[i]->opus_enc.state = MIX_NOT_MIXED;
				mix_set[i]->activity_countdown = COOLDOWN_THR;
			}
		}
		if (i + 1 < SPEECH_MIX_MAX_SET)
		{
			mix_set[i + 1] = m;
		}
		else
		{
			m->opus_enc.state = MIX_NOT_MIXED;
			m->activity_countdown = COOLDOWN_THR;
		}
		nmix_set = MIN(nmix_set + 1, SPEECH_MIX_MAX_SET);
	}

	for (i = 0; i < nmix_set; i++)
	{
		assert(mix_set[i]->activity_countdown > 0);
	}

	for (m = mix; m; m = m->next)
	{
		switch(m->opus_enc.state)
		{
			case MIX_NOT_MIXED:
			default:
				if (m->activity_countdown)
				{
					m->opus_enc.state = MIX_FADEIN;
				}
				break;
			case MIX_FADEIN:
			case MIX_ACTIVE:
				m->opus_enc.state = m->activity_countdown > HANGOVER_THR ? MIX_ACTIVE : MIX_FADEOUT;
				break;
			case MIX_FADEOUT:
				m->opus_enc.state = m->activity_countdown > HANGOVER_THR ? MIX_FADEIN : MIX_COOLDOWN;
				break;
			case MIX_COOLDOWN:
				if (m->activity_countdown > HANGOVER_THR)
				{
					m->opus_enc.state = MIX_FADEIN;
				}
				else
				{
					if (m->activity_countdown > COOLDOWN_THR)
						m->activity_countdown--;
					if (m->activity_countdown <= COOLDOWN_THR)
						m->opus_enc.state = MIX_NOT_MIXED;
				}
				break;

		}
		if (m->opus_enc.state == MIX_FADEIN)
		{
			pcm16_fadein(m->src_pcm, nframe);
		}
		if (m->opus_enc.state == MIX_FADEOUT)
		{
			pcm16_fadeout(m->src_pcm, nframe);
		}
	}

	for (m = mix; m; m = m->next)
	{
		if (m->opus_enc.state != MIX_NOT_MIXED && m->opus_enc.state != MIX_COOLDOWN)
		{
			v_add_32_16(sum, m->src_pcm, nframe);
		}
	}

	float prev_agc_gain = commom_enc->agc.gain;
	agc_run(&commom_enc->agc, sum, nframe);
	v_clip_16(sum, nframe, sum16);
	opus_enc_encode(commom_enc, sum16, nframe);
	enc_count = 0;
	for (m = mix; m; m = m->next)
	{
		if (m->opus_enc.state != MIX_NOT_MIXED)
		{
			if (m->opus_enc.state != MIX_COOLDOWN)
			{
				v_sub3_32_16(sum, m->src_pcm, nframe, m->dst_pcm, prev_agc_gain, commom_enc->agc.gain);
			}
			else
			{
				v_clip_16(sum, nframe, m->dst_pcm);
			}
			if (m->opus_enc.state == MIX_FADEIN)
			{
				int nrollback = SPEECH_MIX_ENCODER_HISTORY / nframe;
				opus_encoder_ctl(m->opus_enc.encoder, OPUS_RESET_STATE);
				for (i = SPEECH_MIX_ENCODER_HISTORY - nrollback*nframe; i < SPEECH_MIX_ENCODER_HISTORY; i += nframe)
				{
					opus_enc_encode(&m->opus_enc, commom_enc->pcm_history+i, nframe);
				}
			}
			opus_enc_encode(&m->opus_enc, m->dst_pcm, nframe);
			enc_count++;
		}
		else
		{
			v_clip_16(sum, nframe, m->dst_pcm);
			opus_enc_take_bitstream(&m->opus_enc, commom_enc);
		}

		speech_mix_src_drop_samples(m, nframe);
		m->sample_count += nframe;
	}
	memmove(commom_enc->pcm_history, commom_enc->pcm_history+nframe, sizeof(commom_enc->pcm_history) - nframe*sizeof(short));
	memcpy(commom_enc->pcm_history + SPEECH_MIX_ENCODER_HISTORY - nframe, sum16, nframe*sizeof(short));
	assert(enc_count <= SPEECH_MIX_MAX_SET);
	return nmix_set > 0 ? mix_set[0] : NULL;
}

int speech_mixer_put_rtp_packet(speech_mixer_t* h, const unsigned char* rtp, int rtp_bytes, unsigned now_ms)
{
	return rtp_opus_fifo_put_packet(&h->opus_dec, rtp, rtp_bytes, now_ms);
}

char* speech_mix_events_stat_string(speech_mixer_t * mix)
{
	rtp_opus_dec_t* h = &mix->opus_dec;
	unsigned int i, cb = 0, n = 1024;
	char* s = malloc(n);
	cb = snprintf(s + cb, n - cb, "%10s: packets arrived: %d accepted %d lost %d\n", h->debug_tag, h->count_packets_arrived,
		h->dbg_count[PACKETS_ACCEPTED], h->dbg_count[PACKETS_SN_SPAN] - h->dbg_count[PACKETS_ACCEPTED]);

	cb += snprintf(s + cb, n - cb, "%20s %d\n", "max delay", h->stat_delay_max);
	if (h->stat_delay_count)
	{
		cb += snprintf(s + cb, n - cb, "%20s %d\n", "mean delay", h->stat_delay_sum / h->stat_delay_count);
	}
	cb += snprintf(s + cb, n - cb, "%20s %d\n", "last delay", h->stat_delay_last);


	for (i = 0; i < sizeof(msg) / sizeof(msg[0]); i++)
	{
		if (h->dbg_count[i])
			cb += snprintf(s + cb, n - cb, "%20s %d\n", msg[i], h->dbg_count[i]);
		if (cb > n - 1024)
		{
			char* q = realloc(s, 2 * n);
			if (q)
			{
				s = q;
				n = 2 * n;
			}
		}
	}

	return s;
}
