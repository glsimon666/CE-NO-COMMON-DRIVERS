/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2025 Amlogic, Inc. All rights reserved.
 *
 * avsync_v2.h - Unified AV Sync Driver UAPI
 *
 * Replaces the fragmented approach of msync.c + video.c sync +
 * videoqueue.c sync + tsync.c + timestamp.c with a single coherent
 * interface. One session = one complete AV sync context.
 */
#ifndef __UAPI_AML_AVSYNC_V2_H__
#define __UAPI_AML_AVSYNC_V2_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

struct avs_pts_sample {
	uint64_t pts_90k;
	uint64_t mono_ns;
	uint64_t delay_90k;
	uint32_t flags;
#define AVS_FLAG_DISCONTINUITY  (1u << 0)
#define AVS_FLAG_EOS            (1u << 1)
#define AVS_FLAG_SEAMLESS       (1u << 2)
#define AVS_FLAG_IDR            (1u << 3)
#define AVS_FLAG_BFRAME         (1u << 4)
#define AVS_FLAG_FIELD_BOTTOM   (1u << 5)
#define AVS_FLAG_REPEAT         (1u << 6)
};

struct avs_sync {
	uint64_t wall_clock_90k;
	uint64_t last_video_pts;
	uint64_t last_audio_pts;
	int64_t  av_diff_90k;
	int64_t  filtered_diff_90k;
	uint32_t action;
	uint32_t count;
	uint32_t confidence;
	int64_t  timing_adjust_ns;
	uint64_t audio_fifo_level;
	uint64_t sink_latency_est_ns;
	int64_t  acr_deviation_ppm;
	uint64_t vsync_jitter_ns;
	uint32_t reserved[4];
};

enum avs_action {
	AVS_ACT_NONE          = 0,
	AVS_ACT_DROP_VIDEO    = 1,
	AVS_ACT_REPEAT_VIDEO  = 2,
	AVS_ACT_ADJUST_TIMING = 3,
	AVS_ACT_INSERT_REPEAT = 4,
	AVS_ACT_HOLD_FRAME    = 5,
};

struct avs_session_cfg {
	uint32_t flags;
#define AVS_CFG_PASSTHROUGH   (1u << 0)
#define AVS_CFG_LIVE          (1u << 1)
#define AVS_CFG_VFR           (1u << 2)
#define AVS_CFG_INTERLACED    (1u << 3)
#define AVS_CFG_DOLBY_VISION  (1u << 4)

	uint32_t video_frame_rate_num;
	uint32_t video_frame_rate_den;
	uint32_t audio_sample_rate;
	uint32_t audio_frame_duration_90k;
	int64_t  start_buf_thres_90k;
	uint32_t reserved[8];
};

enum avs_event_type {
	AVS_EVENT_VIDEO_START         = 0,
	AVS_EVENT_VIDEO_STOP          = 1,
	AVS_EVENT_VIDEO_PAUSE         = 2,
	AVS_EVENT_VIDEO_RESUME        = 3,
	AVS_EVENT_AUDIO_START         = 4,
	AVS_EVENT_AUDIO_STOP          = 5,
	AVS_EVENT_AUDIO_PAUSE         = 6,
	AVS_EVENT_AUDIO_RESUME        = 7,
	AVS_EVENT_AUDIO_FORMAT_CHANGE = 8,
	AVS_EVENT_DISCONTINUITY       = 9,
	AVS_EVENT_FLUSH               = 10,
	AVS_EVENT_TRICKMODE_START     = 11,
	AVS_EVENT_TRICKMODE_STOP      = 12,
	AVS_EVENT_HDMI_SINK_CHANGE    = 13,
};

struct avs_event {
	uint32_t type;
	uint32_t value;
	uint64_t mono_ns;
};

struct avs_caps {
	uint32_t version_major;
	uint32_t version_minor;
	uint32_t max_sessions;
	uint32_t flags;
#define AVS_CAP_PASSTHROUGH       (1u << 0)
#define AVS_CAP_TIMING_ADJUST     (1u << 1)
#define AVS_CAP_ACR_FEEDBACK      (1u << 2)
#define AVS_CAP_PER_FRAME_ACTION  (1u << 3)
#define AVS_CAP_FRACTIONAL_REPEAT (1u << 4)
	uint32_t timing_adjust_step_ns;
	uint32_t max_timing_adjust_ns;
	uint32_t reserved[8];
};

struct avs_diag {
	uint64_t total_video_frames;
	uint64_t total_audio_frames;
	uint64_t total_drops;
	uint64_t total_repeats;
	uint64_t total_timing_adjusts;
	uint64_t total_discontinuities;
	int64_t  cumulative_av_error_90k;
	uint64_t session_uptime_ns;
	uint32_t underflow_count;
	uint32_t overflow_count;
	uint32_t reserved[8];
};

/* IOCTL definitions */
#define AVSYNC_V2_MAGIC  'A'

#define AVSYNC_V2_IOC_GET_CAPS   _IOR(AVSYNC_V2_MAGIC, 0x00, struct avs_caps)

#define AVSYNC_V2_IOC_CREATE     _IOW(AVSYNC_V2_MAGIC, 0x01, struct avs_session_cfg)
#define AVSYNC_V2_IOC_DESTROY    _IO(AVSYNC_V2_MAGIC, 0x02)

#define AVSYNC_V2_IOC_PUSH_VPTS  _IOW(AVSYNC_V2_MAGIC, 0x10, struct avs_pts_sample)
#define AVSYNC_V2_IOC_PUSH_APTS  _IOW(AVSYNC_V2_MAGIC, 0x11, struct avs_pts_sample)
#define AVSYNC_V2_IOC_GET_SYNC   _IOR(AVSYNC_V2_MAGIC, 0x20, struct avs_sync)

#define AVSYNC_V2_IOC_SET_AUDIO_LATENCY  _IOW(AVSYNC_V2_MAGIC, 0x30, uint64_t)
#define AVSYNC_V2_IOC_SET_PASSTHROUGH    _IOW(AVSYNC_V2_MAGIC, 0x31, uint32_t)
#define AVSYNC_V2_IOC_SET_PLAYBACK_RATE  _IOW(AVSYNC_V2_MAGIC, 0x32, uint32_t)

#define AVSYNC_V2_IOC_SEND_EVENT _IOW(AVSYNC_V2_MAGIC, 0x40, struct avs_event)
#define AVSYNC_V2_IOC_POLL_EVENT _IOR(AVSYNC_V2_MAGIC, 0x41, struct avs_event)

#define AVSYNC_V2_IOC_GET_DIAG   _IOR(AVSYNC_V2_MAGIC, 0x50, struct avs_diag)
#define AVSYNC_V2_IOC_RESET_DIAG _IO(AVSYNC_V2_MAGIC, 0x51)

#endif /* __UAPI_AML_AVSYNC_V2_H__ */
