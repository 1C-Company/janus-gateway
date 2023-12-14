#ifndef speech_mix_H_INCLUDED
#define speech_mix_H_INCLUDED

#include "opus.h"

#define SPEECH_MIX_MAX_PCM                      5760    // 120 ms
#define SPEECH_MIX_N_RTP_FIFO                   128
#define SPEECH_MIX_MAX_RTP_PACKET_BYTES         1500
#define SPEECH_MIX_MAX_CODED_FRAME_BYTES        1000
#define SPEECH_MIX_ENCODER_HISTORY              (480*20)  // 200 ms


#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus

typedef struct
{
    unsigned char rtp[SPEECH_MIX_MAX_RTP_PACKET_BYTES];
    int rtp_bytes;
    unsigned arrival_ms;
} rtp_packet_t;

typedef struct
{
    rtp_packet_t rtp_fifo[SPEECH_MIX_N_RTP_FIFO];
    volatile unsigned fifo_rd;
    volatile unsigned fifo_wr;

    unsigned new_ssrc;
    int new_ssrc_cnt;
    unsigned cur_ssrc;

    unsigned short expected_sn;
    unsigned last_accept_ms;

    OpusDecoder* decoder;
    int opus_nominal_frame_len;
    unsigned expected_ts;
    int opus_dtx;

    int hurryup_decimator;

    int count_packets_arrived;
    int dbg_count[19];
    int stat_delay_max;
    int stat_delay_last;
    int stat_delay_sum;
    int stat_delay_count;
    int delay_running_min;
    int delay_running_min_time;
    char debug_tag[64];
    int sn_prev;
} rtp_opus_dec_t;

typedef struct
{
    float envelope;
    float gain;
} agc_t;

typedef struct
{
    OpusEncoder* encoder;
    enum
    {
        MIX_NOT_MIXED,
        MIX_FADEIN,
        MIX_ACTIVE,
        MIX_FADEOUT,
        MIX_COOLDOWN
    } state;
    unsigned char bitstream[SPEECH_MIX_MAX_CODED_FRAME_BYTES];
    int bitstream_bytes;
    agc_t agc;
    short pcm_history[SPEECH_MIX_ENCODER_HISTORY];
} opus_enc_t;

typedef struct speech_mixer_t
{
    struct speech_mixer_t * next;
    rtp_opus_dec_t opus_dec;
    opus_enc_t opus_enc;
    short src_pcm[SPEECH_MIX_MAX_PCM];
    short dst_pcm[SPEECH_MIX_MAX_PCM];
    int nsrc;
    float instant_activity;
    float mean_activity;
    int activity_countdown;
    int sample_count;
    void * user_data;
} speech_mixer_t;

void  opus_enc_init(opus_enc_t * enc, const char *debug_tag);
void  opus_enc_close(opus_enc_t* enc);
int   speech_mixer_init(speech_mixer_t * participant, const char* debug_tag);
void  speech_mixer_close(speech_mixer_t * participant);
int   speech_mixer_put_rtp_packet(speech_mixer_t* h, const unsigned char* rtp, int rtp_bytes, unsigned now_ms);
speech_mixer_t * speech_mix_run(speech_mixer_t * mix, opus_enc_t * enc, int nframe, unsigned now_ms);
char* speech_mix_events_stat_string(speech_mixer_t * mix);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //speech_mix_H_INCLUDED
