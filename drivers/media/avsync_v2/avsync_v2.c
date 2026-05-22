// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Amlogic, Inc. All rights reserved.
 *
 * avsync_v2.c - Unified AV Sync Driver v2
 *
 * Single-driver replacement for msync.c + video.c sync + videoqueue.c sync
 * + tsync.c fragments.  One session = one complete A/V sync context.
 *
 * Design principles:
 *   - 64-bit monotonic PTS everywhere (no 32-bit wrapping)
 *   - Monotonic capture timestamps on every PTS sample
 *   - Single state machine, internally choosing master clock
 *   - Passthrough-aware: different thresholds for compressed audio
 *   - HDMI ACR clock feedback for sink-side drift compensation
 *   - Sub-frame timing adjustment hints (not just drop/repeat)
 *   - Confidence reporting: kernel tells userspace whether to trust
 *     the suggestion, avoiding "fighting layers" syndrome
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/amlogic/media/vout/vout_notify.h>
#include <uapi/amlogic/avsync_v2.h>

#define DRIVER_NAME    "avsync_v2"
#define DEV_CLASS_NAME "avsync_v2"
#define MAX_SESSIONS   4

/*********************************************************************
 * Constants
 *********************************************************************/

#define UNIT90K            90000ULL
#define DEFAULT_BUF_THRES  (UNIT90K * 6 / 10)    /* 600 ms */
#define DISC_THRES_MIN     (UNIT90K / 3)          /* ~3.7 ms   */
#define DISC_THRES_MAX     (UNIT90K * 20)         /* 20 s      */
#define MAX_AV_DIFF        (UNIT90K * 5)          /* 5 s       */
#define TRANSIT_INTERVAL_MS 1000                  /* 1 s       */
#define CHECK_INTERVAL_MS   300
#define WAIT_INTERVAL_MS    2000
#define VIDEO_START_TO_S    2                     /* 2 s       */

/* Passthrough thresholds (conservative) */
#define PASSTHROUGH_DROP_THRESHOLD_90K  (UNIT90K * 2 / 20)  /* 100 ms  */
/* PCM thresholds (aggressive) */
#define PCM_DROP_THRESHOLD_90K          (UNIT90K / 20)      /*  50 ms  */

/* Timing adjustment range (sub-frame) */
#define TIMING_ADJ_STEP_NS  1000        /* 1 µs step            */
#define TIMING_ADJ_MAX_NS   3000000     /* 3 ms max per frame   */

/* HDMI ACR deviation warning threshold (ppm) */
#define ACR_DEVIATION_WARN_PPM  500

/* Jitter filter: simple EWMA in kernel */
#define JITTER_EWMA_ALPHA_SHIFT  4       /* alpha = 1/16         */

/*********************************************************************
 * Session state machine
 *********************************************************************/

enum avs_state {
	AVS_STATE_IDLE,
	AVS_STATE_BUFFERING,
	AVS_STATE_STARTING,
	AVS_STATE_RUNNING,
	AVS_STATE_PAUSED,
	AVS_STATE_STOPPING,
};

static const char * const state_names[] = {
	[AVS_STATE_IDLE]      = "IDLE",
	[AVS_STATE_BUFFERING] = "BUFFERING",
	[AVS_STATE_STARTING]  = "STARTING",
	[AVS_STATE_RUNNING]   = "RUNNING",
	[AVS_STATE_PAUSED]    = "PAUSED",
	[AVS_STATE_STOPPING]  = "STOPPING",
};

/*********************************************************************
 * Jitter low-pass filter (kernel-side EWMA)
 *********************************************************************/

struct avs_jitter_filter {
	int64_t  mean;         /* EWMA mean (90k units)          */
	int64_t  variance;     /* EWMA variance (90k²)           */
	int64_t  drift;        /* EWMA drift rate per sample     */
};

static void jitter_init(struct avs_jitter_filter *f)
{
	memset(f, 0, sizeof(*f));
}

static void jitter_sample(struct avs_jitter_filter *f, int64_t value)
{
	int64_t delta = value - f->mean;
	f->mean     += delta >> JITTER_EWMA_ALPHA_SHIFT;
	f->variance += ((delta * delta / (1 << JITTER_EWMA_ALPHA_SHIFT))
			- f->variance) >> JITTER_EWMA_ALPHA_SHIFT;
	f->drift    += (delta - f->drift) >> JITTER_EWMA_ALPHA_SHIFT;
}

static int64_t jitter_predict(const struct avs_jitter_filter *f,
			      int steps_ahead)
{
	return f->mean + f->drift * steps_ahead;
}

static uint32_t jitter_confidence(const struct avs_jitter_filter *f)
{
	int64_t var = f->variance;
	if (var < 0)
		var = -var;
	if (var == 0)
		return 1000;
	/* confidence = 1 / (1 + variance / noise_scale) * 1000 */
	uint64_t ratio = (uint64_t)var * 1000 / (UNIT90K * UNIT90K / 100);
	if (ratio >= 1000)
		return 0;
	return 1000 - (uint32_t)ratio;
}

/*********************************************************************
 * Session structure
 *********************************************************************/

struct avs_session {
	uint32_t          id;
	atomic_t          refcnt;
	struct list_head  node;

	/* ---- char device ---- */
	struct device    *dev;
	struct cdev       cdev;
	dev_t             devt;
	char              name[24];

	/* ---- synchronisation ---- */
	struct mutex      lock;
	enum avs_state    state;
	bool              passthrough;
	bool              live_stream;
	bool              vfr;
	bool              interlaced;
	bool              dolby_vision;
	uint32_t          playback_rate;   /* 1000 = 1.0x          */

	/* ---- wall clock ---- */
	spinlock_t        wall_lock;
	uint64_t          wall_clock_90k;
	uint64_t          wall_remainder;  /* fractional increment   */
	bool              clock_started;

	/* ---- video ---- */
	struct avs_pts_sample  first_v;
	struct avs_pts_sample  last_v;
	uint64_t          vsync_pts_inc;  /* 90k per vsync          */

	/* ---- audio ---- */
	struct avs_pts_sample  first_a;
	struct avs_pts_sample  last_a;
	uint64_t          audio_latency_ns;
	uint32_t          audio_frame_dur_90k;

	/* ---- sync filter ---- */
	struct avs_jitter_filter  v_jitter;
	struct avs_jitter_filter  a_jitter;

	/* ---- computed state ---- */
	int64_t           last_av_diff;
	uint32_t          consecutive_actions;
	uint64_t          sink_latency_est_ns;
	int64_t           acr_deviation_ppm;

	/* ---- event ---- */
	wait_queue_head_t poll_wq;
	uint32_t          event_pending;
	struct avs_event  pending_event;

	/* ---- delayed works ---- */
	struct workqueue_struct *wq;
	struct delayed_work  buffering_work;
	struct delayed_work  starting_work;
	bool              buffering_work_on;
	bool              starting_work_on;

	/* ---- diagnostic ---- */
	struct avs_diag   diag;
};

/*********************************************************************
 * Driver globals
 *********************************************************************/

static struct {
	dev_t             major;
	struct class     *class;
	struct device    *dev;
	struct cdev       cdev;
	spinlock_t        lock;
	struct list_head  sessions;
	uint8_t           id_pool[MAX_SESSIONS];
	bool              ready;
	uint32_t          sync_duration_num;
	uint32_t          sync_duration_den;
	uint64_t          vsync_pts_inc;
} avs_drv;

static int avs_log_level = 2; /* LOG_INFO */

#define avs_dbg(lvl, fmt, ...) \
	do { if ((lvl) <= avs_log_level) pr_info("[avsync_v2] " fmt, ##__VA_ARGS__); } while (0)

/*********************************************************************
 * Forward declarations
 *********************************************************************/

static void avs_session_vsync(struct avs_session *s);
static void avs_recalc_sync(struct avs_session *s);
static int  avs_session_open(struct inode *inode, struct file *file);
static int  avs_session_release(struct inode *inode, struct file *file);
static long avs_session_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg);
static unsigned int avs_session_poll(struct file *file,
				     poll_table *wait);

static const struct file_operations avs_session_fops = {
	.owner          = THIS_MODULE,
	.open           = avs_session_open,
	.release        = avs_session_release,
	.unlocked_ioctl = avs_session_ioctl,
	.poll           = avs_session_poll,
};

/*********************************************************************
 * Vsync ISR: the heartbeat of the whole system
 *********************************************************************/

static void avs_session_vsync(struct avs_session *s)
{
	unsigned long flags;
	uint64_t inc_90k;
	uint64_t rem;

	if (!s->clock_started)
		return;

	/* N = f_vsync * 90000 * playback_rate / 1000   */
	inc_90k = avs_drv.vsync_pts_inc;
	inc_90k = (inc_90k * s->playback_rate) / 1000;

	spin_lock_irqsave(&s->wall_lock, flags);
	s->wall_clock_90k += inc_90k;
	s->wall_remainder += (inc_90k * s->playback_rate) % 1000;
	if (s->wall_remainder >= 1000) {
		s->wall_clock_90k += s->wall_remainder / 1000;
		s->wall_remainder %= 1000;
	}
	spin_unlock_irqrestore(&s->wall_lock, flags);

	if (s->state != AVS_STATE_RUNNING)
		return;

	avs_recalc_sync(s);
}

static int avs_notifier_callback(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct avs_session *s;
	struct list_head *p;
	unsigned long flags;

	if (!avs_drv.ready)
		return 0;

	spin_lock_irqsave(&avs_drv.lock, flags);
	list_for_each(p, &avs_drv.sessions) {
		s = list_entry(p, struct avs_session, node);
		avs_session_vsync(s);
	}
	spin_unlock_irqrestore(&avs_drv.lock, flags);
	return 0;
}

static struct notifier_block avs_vsync_notifier = {
	.notifier_call = avs_notifier_callback,
};

/*********************************************************************
 * Mode change: recalculate timing parameters when video mode changes
 *********************************************************************/

static void avs_recalc_mode(void)
{
	const struct vinfo_s *info = get_current_vinfo();
	uint64_t inc;
	unsigned long flags;
	uint32_t den, num;

	if (!info)
		return;

	inc = 90000ULL * info->sync_duration_den / info->sync_duration_num;

	if (info->sync_duration_num == 2997 ||
	    info->sync_duration_num == 5994) {
		den = 1001;
		num = 3000 * 1000 / den;
		num *= (info->sync_duration_num / 2997);
	} else {
		den = info->sync_duration_den;
		num = info->sync_duration_num;
	}

	spin_lock_irqsave(&avs_drv.lock, flags);
	avs_drv.vsync_pts_inc        = inc;
	avs_drv.sync_duration_den    = den;
	avs_drv.sync_duration_num    = num;
	spin_unlock_irqrestore(&avs_drv.lock, flags);

	avs_dbg(2, "mode change: inc_90k=%llu den=%u num=%u\n",
		inc, den, num);
}

static int avs_mode_notifier_callback(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	if (event == VOUT_EVENT_MODE_CHANGE)
		avs_recalc_mode();
	return 0;
}

static struct notifier_block avs_mode_notifier = {
	.notifier_call = avs_mode_notifier_callback,
};

/*********************************************************************
 * Sync recalculation: core algorithm
 *
 * This is called every vsync while RUNNING.
 * It compares the current wall clock with the expected PTS of the
 * audio stream and issues a verdict.
 *********************************************************************/

static void avs_recalc_sync(struct avs_session *s)
{
	uint64_t wall;
	uint64_t vpts, apts;
	int64_t  diff;
	uint64_t audio_lat_90k;

	wall = s->wall_clock_90k;

	/* Convert audio latency from ns to 90k units */
	audio_lat_90k = s->audio_latency_ns * UNIT90K / 1000000000ULL;

	/*
	 * AV diff = (audio_pts_played - audio_latency) - wall_clock
	 * Positive = audio is ahead (video needs to catch up)
	 * Negative = video is ahead (audio needs to catch up)
	 * For passthrough: can only adjust video, so we bias
	 * towards video manipulation.
	 */
	if (s->last_a.pts_90k) {
		apts = s->last_a.pts_90k;
		if (apts > audio_lat_90k + s->last_a.delay_90k)
			apts -= audio_lat_90k + s->last_a.delay_90k;
		else
			apts = 0;
		diff = (int64_t)(apts - wall);
	} else {
		diff = 0;
	}

	/* Feed jitter filters */
	jitter_sample(&s->a_jitter, diff);
	if (s->last_v.pts_90k) {
		vpts = s->last_v.pts_90k;
		int64_t vdiff = (int64_t)(vpts - s->last_v.delay_90k - wall);
		jitter_sample(&s->v_jitter, vdiff);
	}

	s->last_av_diff = diff;

	/*
	 * Drift compensation: if ACR deviation is non-zero,
	 * the HDMI sink plays audio at a slightly different rate.
	 * Compensate by adding a bias to the sync diff.
	 * 1 ppm = the sink gains/loses 1 ns per ms of playback.
	 * Convert to 90k units: diff_compensation = deviation_ppm *
	 * (elapsed_ms * 90k) / 1e6
	 */
	if (s->acr_deviation_ppm) {
		uint64_t elapsed_ns = s->diag.session_uptime_ns;
		int64_t drift_90k = (int64_t)(elapsed_ns / 1000000ULL) *
			s->acr_deviation_ppm * 90 / 1000000;
		diff -= drift_90k;
	}

	avs_dbg(3, "[%u] diff=%lld wall=%llu apts=%llu vpts=%llu\n",
		s->id, diff, wall, apts, vpts);
}

/*********************************************************************
 * Action decision: what to do about the AV diff
 *
 * Separate thresholds for passthrough / PCM.
 * Confidence gating: only suggest action if the filter is confident.
 *********************************************************************/

static enum avs_action avs_decide_action(struct avs_session *s,
					 int64_t *count_out,
					 int64_t *timing_ns_out)
{
	int64_t diff_pred;
	uint32_t conf;
	int64_t threshold;

	/* Use filtered prediction (1 frame ahead) */
	diff_pred = jitter_predict(&s->a_jitter, 1);
	conf = jitter_confidence(&s->a_jitter);

	if (s->passthrough)
		threshold = PASSTHROUGH_DROP_THRESHOLD_90K;
	else
		threshold = PCM_DROP_THRESHOLD_90K;

	*count_out = 0;
	*timing_ns_out = 0;

	/* Not confident enough? Do nothing. */
	if (conf < 500)
		return AVS_ACT_NONE;

	if (diff_pred > -threshold && diff_pred < threshold)
		return AVS_ACT_NONE; /* within tolerance */

	/* Passthrough: only adjust video, try sub-frame timing first */
	if (s->passthrough) {
		/* Sub-frame timing adjustment (fine-grained) */
		*timing_ns_out = diff_pred * 1000000000LL / (int64_t)UNIT90K;
		if (diff_pred > 0 && *timing_ns_out < TIMING_ADJ_MAX_NS) {
			/* Audio slightly ahead: slow down video
			 * (positive timing_adjust = longer frame) */
			return AVS_ACT_ADJUST_TIMING;
		}
		if (diff_pred < 0 && -*timing_ns_out < TIMING_ADJ_MAX_NS) {
			/* Video slightly ahead */
			return AVS_ACT_ADJUST_TIMING;
		}
	}

	/* Beyond sub-frame range: need frame drop/repeat */
	if (diff_pred > threshold) {
		/* Audio ahead: need to slow down video or drop audio
		 * For passthrough: only video options available */
		int64_t frames = diff_pred / (int64_t)avs_drv.vsync_pts_inc;
		*count_out = frames > 0 ? frames : 1;

		/* Limit consecutive actions to prevent oscillation */
		if (s->consecutive_actions > 3)
			*count_out = 1;

		/* Prefer timing adjust over drop when possible */
		if (s->passthrough && *count_out <= 1) {
			*timing_ns_out = diff_pred *
				1000000000LL / (int64_t)UNIT90K;
			*count_out = 0;
			return AVS_ACT_ADJUST_TIMING;
		}

		return AVS_ACT_DROP_VIDEO;
	}

	if (diff_pred < -threshold) {
		/* Video ahead: repeat frame or insert timing delay */
		int64_t frames = -diff_pred / (int64_t)avs_drv.vsync_pts_inc;
		*count_out = frames > 0 ? frames : 1;

		if (s->consecutive_actions > 3)
			*count_out = 1;

		if (s->passthrough) {
			*timing_ns_out = -diff_pred *
				1000000000LL / (int64_t)UNIT90K;
			*count_out = 0;
			return AVS_ACT_ADJUST_TIMING;
		}

		return AVS_ACT_REPEAT_VIDEO;
	}

	return AVS_ACT_NONE;
}

/*********************************************************************
 * Push a PTS sample (video or audio)
 *********************************************************************/

static void avs_push_sample(struct avs_session *s,
			    struct avs_pts_sample *sample,
			    bool is_video)
{
	bool discontinuity;

	discontinuity = sample->flags & AVS_FLAG_DISCONTINUITY;

	if (is_video) {
		discontinuity = discontinuity ||
			(abs((int64_t)(sample->pts_90k - s->last_v.pts_90k)) >
			 DISC_THRES_MAX && s->last_v.pts_90k != 0);

		if (s->first_v.pts_90k == 0)
			memcpy(&s->first_v, sample, sizeof(*sample));
		memcpy(&s->last_v, sample, sizeof(*sample));

		if (discontinuity) {
			jitter_init(&s->v_jitter);
			s->diag.total_discontinuities++;
		}
		s->diag.total_video_frames++;
	} else {
		discontinuity = discontinuity ||
			(abs((int64_t)(sample->pts_90k - s->last_a.pts_90k)) >
			 DISC_THRES_MAX && s->last_a.pts_90k != 0);

		if (s->first_a.pts_90k == 0)
			memcpy(&s->first_a, sample, sizeof(*sample));
		memcpy(&s->last_a, sample, sizeof(*sample));

		if (discontinuity) {
			jitter_init(&s->a_jitter);
			s->diag.total_discontinuities++;
		}
		s->diag.total_audio_frames++;
	}
}

/*********************************************************************
 * Transition state machine
 *********************************************************************/

static void avs_transition_to(struct avs_session *s, enum avs_state new_state)
{
	enum avs_state old = s->state;

	if (old == new_state)
		return;

	avs_dbg(2, "[%u] state %s -> %s\n",
		s->id, state_names[old], state_names[new_state]);
	s->state = new_state;

	switch (new_state) {
	case AVS_STATE_RUNNING:
		s->diag.session_uptime_ns = 0;
		break;
	case AVS_STATE_IDLE:
		s->clock_started = false;
		s->wall_clock_90k = 0;
		memset(&s->first_v, 0, sizeof(s->first_v));
		memset(&s->last_v, 0, sizeof(s->last_v));
		memset(&s->first_a, 0, sizeof(s->first_a));
		memset(&s->last_a, 0, sizeof(s->last_a));
		jitter_init(&s->v_jitter);
		jitter_init(&s->a_jitter);
		s->last_av_diff = 0;
		s->consecutive_actions = 0;
		break;
	default:
		break;
	}
}

/*********************************************************************
 * Buffering work: wait for enough audio/video data before starting
 *********************************************************************/

static void avs_buffering_work_fn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct avs_session *s =
		container_of(dwork, struct avs_session, buffering_work);
	bool start_now = false;

	mutex_lock(&s->lock);
	if (!s->buffering_work_on)
		goto out;

	s->buffering_work_on = false;

	/* Check if we have both A and V data */
	if (s->last_v.pts_90k && s->last_a.pts_90k) {
		/* Both ready: start immediately */
		start_now = true;
		avs_dbg(2, "[%u] buffering done, both A & V present\n", s->id);
	} else if (s->last_v.pts_90k) {
		/* Video only: give more time for audio */
		avs_dbg(2, "[%u] waiting for audio...\n", s->id);
		queue_delayed_work(s->wq, &s->buffering_work,
				   msecs_to_jiffies(WAIT_INTERVAL_MS));
		s->buffering_work_on = true;
		goto out;
	} else if (s->last_a.pts_90k) {
		/* Audio only: give more time for video */
		avs_dbg(2, "[%u] waiting for video...\n", s->id);
		queue_delayed_work(s->wq, &s->buffering_work,
				   msecs_to_jiffies(WAIT_INTERVAL_MS));
		s->buffering_work_on = true;
		goto out;
	} else {
		/* Neither: timeout, force start */
		start_now = true;
	}

	if (start_now) {
		avs_transition_to(s, AVS_STATE_STARTING);

		/* Anchor wall clock to the later of first V and A PTS */
		uint64_t anchor_pts;
		if (s->last_a.pts_90k && s->last_v.pts_90k)
			anchor_pts = s->last_a.pts_90k < s->last_v.pts_90k ?
				s->last_a.pts_90k : s->last_v.pts_90k;
		else if (s->last_a.pts_90k)
			anchor_pts = s->last_a.pts_90k;
		else
			anchor_pts = s->last_v.pts_90k;

		s->wall_clock_90k = anchor_pts;
		s->clock_started = true;

		/* Short delay then -> RUNNING */
		queue_delayed_work(s->wq, &s->starting_work,
				   msecs_to_jiffies(TRANSIT_INTERVAL_MS));
		s->starting_work_on = true;
	}

out:
	mutex_unlock(&s->lock);
}

static void avs_starting_work_fn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct avs_session *s =
		container_of(dwork, struct avs_session, starting_work);

	mutex_lock(&s->lock);
	if (s->starting_work_on) {
		s->starting_work_on = false;
		avs_transition_to(s, AVS_STATE_RUNNING);
		avs_dbg(1, "[%u] now RUNNING\n", s->id);
	}
	/* Wake up pollers */
	s->event_pending |= (1u << 0);
	wake_up_interruptible(&s->poll_wq);
	mutex_unlock(&s->lock);
}

/*********************************************************************
 * IOCTL handlers
 *********************************************************************/

static long avs_do_get_caps(struct avs_caps __user *arg)
{
	struct avs_caps caps = {
		.version_major  = 2,
		.version_minor  = 0,
		.max_sessions   = MAX_SESSIONS,
		.flags          = AVS_CAP_PASSTHROUGH
				| AVS_CAP_TIMING_ADJUST
				| AVS_CAP_ACR_FEEDBACK
				| AVS_CAP_PER_FRAME_ACTION,
		.timing_adjust_step_ns = TIMING_ADJ_STEP_NS,
		.max_timing_adjust_ns = TIMING_ADJ_MAX_NS,
	};

	if (copy_to_user(arg, &caps, sizeof(caps)))
		return -EFAULT;
	return 0;
}

static struct avs_session *avs_session_alloc(uint32_t id)
{
	struct avs_session *s;
	static const struct avs_diag zero_diag;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return ERR_PTR(-ENOMEM);

	s->id = id;
	atomic_set(&s->refcnt, 1);
	mutex_init(&s->lock);
	spin_lock_init(&s->wall_lock);
	init_waitqueue_head(&s->poll_wq);
	s->playback_rate = 1000;
	jitter_init(&s->v_jitter);
	jitter_init(&s->a_jitter);
	s->state = AVS_STATE_IDLE;
	s->diag = zero_diag;

	s->wq = alloc_ordered_workqueue("avs_wq_%u", WQ_HIGHPRI, id);
	if (!s->wq) {
		kfree(s);
		return ERR_PTR(-ENOMEM);
	}

	snprintf(s->name, sizeof(s->name), "avsync_v2_%u", id);

	INIT_DELAYED_WORK(&s->buffering_work, avs_buffering_work_fn);
	INIT_DELAYED_WORK(&s->starting_work, avs_starting_work_fn);

	return s;
}

static void avs_session_free(struct avs_session *s)
{
	if (!s)
		return;

	if (!atomic_dec_and_test(&s->refcnt))
		return;

	if (s->wq)
		destroy_workqueue(s->wq);

	if (s->dev)
		device_destroy(avs_drv.class, s->devt);

	cdev_del(&s->cdev);
	mutex_destroy(&s->lock);
	kfree(s);
}

static long avs_do_create(struct avs_session_cfg __user *arg)
{
	struct avs_session_cfg cfg;
	struct avs_session *s;
	unsigned long flags;
	uint32_t id;
	int ret;

	if (copy_from_user(&cfg, arg, sizeof(cfg)))
		return -EFAULT;

	/* Allocate session ID */
	spin_lock_irqsave(&avs_drv.lock, flags);
	for (id = 0; id < MAX_SESSIONS; id++) {
		if (!avs_drv.id_pool[id]) {
			avs_drv.id_pool[id] = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&avs_drv.lock, flags);

	if (id >= MAX_SESSIONS)
		return -EBUSY;

	s = avs_session_alloc(id);
	if (IS_ERR(s)) {
		avs_drv.id_pool[id] = 0;
		return PTR_ERR(s);
	}

	s->passthrough   = !!(cfg.flags & AVS_CFG_PASSTHROUGH);
	s->live_stream   = !!(cfg.flags & AVS_CFG_LIVE);
	s->vfr           = !!(cfg.flags & AVS_CFG_VFR);
	s->interlaced    = !!(cfg.flags & AVS_CFG_INTERLACED);
	s->dolby_vision  = !!(cfg.flags & AVS_CFG_DOLBY_VISION);
	s->audio_frame_dur_90k = cfg.audio_frame_duration_90k;

	/* Create char device for this session */
	s->devt = MKDEV(MAJOR(avs_drv.major), id + 1);
	cdev_init(&s->cdev, &avs_session_fops);
	s->cdev.owner = THIS_MODULE;
	ret = cdev_add(&s->cdev, s->devt, 1);
	if (ret) {
		avs_drv.id_pool[id] = 0;
		avs_session_free(s);
		return ret;
	}

	s->dev = device_create(avs_drv.class, NULL, s->devt, s, s->name);
	if (IS_ERR(s->dev)) {
		ret = PTR_ERR(s->dev);
		avs_drv.id_pool[id] = 0;
		avs_session_free(s);
		return ret;
	}

	spin_lock_irqsave(&avs_drv.lock, flags);
	list_add(&s->node, &avs_drv.sessions);
	spin_unlock_irqrestore(&avs_drv.lock, flags);

	avs_dbg(1, "[%u] session created, pass:%d live:%d vfr:%d\n",
		s->id, s->passthrough, s->live_stream, s->vfr);

	/* Return the session ID to userspace via the struct's first field */
	cfg.reserved[0] = id;
	if (copy_to_user(arg, &cfg, sizeof(cfg))) {
		spin_lock_irqsave(&avs_drv.lock, flags);
		list_del(&s->node);
		spin_unlock_irqrestore(&avs_drv.lock, flags);
		avs_drv.id_pool[id] = 0;
		avs_session_free(s);
		return -EFAULT;
	}

	return 0;
}

static long avs_do_destroy(struct avs_session *s)
{
	unsigned long flags;

	avs_dbg(1, "[%u] destroying session\n", s->id);

	cancel_delayed_work_sync(&s->buffering_work);
	cancel_delayed_work_sync(&s->starting_work);

	avs_transition_to(s, AVS_STATE_IDLE);

	spin_lock_irqsave(&avs_drv.lock, flags);
	list_del(&s->node);
	avs_drv.id_pool[s->id] = 0;
	spin_unlock_irqrestore(&avs_drv.lock, flags);

	avs_session_free(s);
	return 0;
}

static long avs_do_push_pts(struct avs_session *s,
			    struct avs_pts_sample __user *arg,
			    bool is_video)
{
	struct avs_pts_sample sample;

	if (copy_from_user(&sample, arg, sizeof(sample)))
		return -EFAULT;

	mutex_lock(&s->lock);

	avs_push_sample(s, &sample, is_video);

	/*
	 * Auto-start: if session is IDLE and both V and A
	 * have started arriving, transition to BUFFERING.
	 */
	if (s->state == AVS_STATE_IDLE &&
	    ((is_video && s->last_a.pts_90k) ||
	     (!is_video && s->last_v.pts_90k))) {

		avs_transition_to(s, AVS_STATE_BUFFERING);

		queue_delayed_work(s->wq, &s->buffering_work,
				   msecs_to_jiffies(WAIT_INTERVAL_MS));
		s->buffering_work_on = true;
	}

	mutex_unlock(&s->lock);
	return 0;
}

static long avs_do_get_sync(struct avs_session *s,
			    struct avs_sync __user *arg)
{
	struct avs_sync info;
	int64_t timing_ns;
	int64_t count;
	enum avs_action action;

	memset(&info, 0, sizeof(info));

	mutex_lock(&s->lock);

	info.wall_clock_90k   = s->wall_clock_90k;
	info.last_video_pts   = s->last_v.pts_90k;
	info.last_audio_pts   = s->last_a.pts_90k;
	info.av_diff_90k      = s->last_av_diff;
	info.filtered_diff_90k = jitter_predict(&s->a_jitter, 1);
	info.audio_fifo_level = 0; /* TODO: read from audio HW */
	info.sink_latency_est_ns = s->sink_latency_est_ns;
	info.acr_deviation_ppm = s->acr_deviation_ppm;

	if (s->state == AVS_STATE_RUNNING) {
		action = avs_decide_action(s, &count, &timing_ns);
		info.action          = action;
		info.count           = (uint32_t)count;
		info.confidence      = jitter_confidence(&s->a_jitter);
		info.timing_adjust_ns = timing_ns;

		if (action != AVS_ACT_NONE)
			s->consecutive_actions++;
		else
			s->consecutive_actions = 0;

		switch (action) {
		case AVS_ACT_DROP_VIDEO:
			s->diag.total_drops += (uint64_t)count;
			break;
		case AVS_ACT_REPEAT_VIDEO:
			s->diag.total_repeats += (uint64_t)count;
			break;
		case AVS_ACT_ADJUST_TIMING:
			s->diag.total_timing_adjusts++;
			break;
		default:
			break;
		}
	}

	mutex_unlock(&s->lock);

	if (copy_to_user(arg, &info, sizeof(info)))
		return -EFAULT;
	return 0;
}

static long avs_do_set_audio_latency(struct avs_session *s,
				     uint64_t __user *arg)
{
	uint64_t latency_ns;

	if (copy_from_user(&latency_ns, arg, sizeof(latency_ns)))
		return -EFAULT;

	mutex_lock(&s->lock);
	s->audio_latency_ns = latency_ns;
	mutex_unlock(&s->lock);

	avs_dbg(2, "[%u] audio latency set to %llu ns\n",
		s->id, latency_ns);
	return 0;
}

static long avs_do_set_passthrough(struct avs_session *s,
				   uint32_t __user *arg)
{
	uint32_t val;

	if (copy_from_user(&val, arg, sizeof(val)))
		return -EFAULT;

	mutex_lock(&s->lock);
	s->passthrough = !!val;

	/* Reset jitter filters when switching mode */
	jitter_init(&s->a_jitter);
	jitter_init(&s->v_jitter);
	s->consecutive_actions = 0;

	mutex_unlock(&s->lock);

	avs_dbg(1, "[%u] passthrough = %d\n", s->id, s->passthrough);
	return 0;
}

static long avs_do_set_playback_rate(struct avs_session *s,
				     uint32_t __user *arg)
{
	uint32_t rate;

	if (copy_from_user(&rate, arg, sizeof(rate)))
		return -EFAULT;

	mutex_lock(&s->lock);
	s->playback_rate = rate;
	mutex_unlock(&s->lock);

	avs_dbg(1, "[%u] playback rate = %u (%.2fx)\n",
		s->id, rate, rate / 1000.0);
	return 0;
}

static long avs_do_send_event(struct avs_session *s,
			      struct avs_event __user *arg)
{
	struct avs_event ev;

	if (copy_from_user(&ev, arg, sizeof(ev)))
		return -EFAULT;

	mutex_lock(&s->lock);

	switch (ev.type) {
	case AVS_EVENT_VIDEO_STOP:
	case AVS_EVENT_AUDIO_STOP:
		avs_transition_to(s, AVS_STATE_IDLE);
		break;
	case AVS_EVENT_VIDEO_PAUSE:
	case AVS_EVENT_AUDIO_PAUSE:
		if (s->state == AVS_STATE_RUNNING)
			avs_transition_to(s, AVS_STATE_PAUSED);
		break;
	case AVS_EVENT_VIDEO_RESUME:
	case AVS_EVENT_AUDIO_RESUME:
		if (s->state == AVS_STATE_PAUSED) {
			avs_transition_to(s, AVS_STATE_RUNNING);
			s->clock_started = true;
		}
		break;
	case AVS_EVENT_DISCONTINUITY:
		/* Reset jitter filters on both streams */
		jitter_init(&s->v_jitter);
		jitter_init(&s->a_jitter);
		s->diag.total_discontinuities++;
		break;
	case AVS_EVENT_FLUSH:
		avs_transition_to(s, AVS_STATE_IDLE);
		break;
	case AVS_EVENT_AUDIO_FORMAT_CHANGE:
		s->audio_latency_ns += 50000000ULL; /* +50ms mute period */
		avs_dbg(1, "[%u] audio format change, latency adjusted\n",
			s->id);
		break;
	default:
		break;
	}

	mutex_unlock(&s->lock);
	return 0;
}

static long avs_do_get_diag(struct avs_session *s,
			    struct avs_diag __user *arg)
{
	struct avs_diag diag;

	mutex_lock(&s->lock);
	diag = s->diag;
	mutex_unlock(&s->lock);

	if (copy_to_user(arg, &diag, sizeof(diag)))
		return -EFAULT;
	return 0;
}

static long avs_do_reset_diag(struct avs_session *s)
{
	mutex_lock(&s->lock);
	memset(&s->diag, 0, sizeof(s->diag));
	mutex_unlock(&s->lock);
	return 0;
}

/*********************************************************************
 * Session file ops
 *********************************************************************/

static int avs_session_open(struct inode *inode, struct file *file)
{
	struct avs_session *s = container_of(inode->i_cdev,
					     struct avs_session, cdev);

	atomic_inc(&s->refcnt);
	file->private_data = s;

	avs_dbg(3, "[%u] opened (ref=%d)\n", s->id,
		atomic_read(&s->refcnt));
	return 0;
}

static int avs_session_release(struct inode *inode, struct file *file)
{
	struct avs_session *s = file->private_data;

	avs_dbg(3, "[%u] released (ref=%d)\n", s->id,
		atomic_read(&s->refcnt) - 1);
	avs_session_free(s);
	return 0;
}

static long avs_session_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct avs_session *s = file->private_data;

	switch (cmd) {
	case AVSYNC_V2_IOC_GET_CAPS:
		return avs_do_get_caps((struct avs_caps __user *)arg);

	case AVSYNC_V2_IOC_CREATE:
		return avs_do_create((struct avs_session_cfg __user *)arg);

	case AVSYNC_V2_IOC_DESTROY:
		return avs_do_destroy(s);

	case AVSYNC_V2_IOC_PUSH_VPTS:
		return avs_do_push_pts(s,
			(struct avs_pts_sample __user *)arg, true);

	case AVSYNC_V2_IOC_PUSH_APTS:
		return avs_do_push_pts(s,
			(struct avs_pts_sample __user *)arg, false);

	case AVSYNC_V2_IOC_GET_SYNC:
		return avs_do_get_sync(s, (struct avs_sync __user *)arg);

	case AVSYNC_V2_IOC_SET_AUDIO_LATENCY:
		return avs_do_set_audio_latency(s,
			(uint64_t __user *)arg);

	case AVSYNC_V2_IOC_SET_PASSTHROUGH:
		return avs_do_set_passthrough(s,
			(uint32_t __user *)arg);

	case AVSYNC_V2_IOC_SET_PLAYBACK_RATE:
		return avs_do_set_playback_rate(s,
			(uint32_t __user *)arg);

	case AVSYNC_V2_IOC_SEND_EVENT:
		return avs_do_send_event(s,
			(struct avs_event __user *)arg);

	case AVSYNC_V2_IOC_GET_DIAG:
		return avs_do_get_diag(s, (struct avs_diag __user *)arg);

	case AVSYNC_V2_IOC_RESET_DIAG:
		return avs_do_reset_diag(s);

	default:
		return -ENOTTY;
	}
}

static unsigned int avs_session_poll(struct file *file,
				     poll_table *wait)
{
	struct avs_session *s = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &s->poll_wq, wait);
	if (s->event_pending) {
		mask |= POLLPRI | POLLIN;
		s->event_pending = 0;
	}
	return mask;
}

/*********************************************************************
 * Global device file ops (for CAPS query, session creation)
 *********************************************************************/

static long avs_global_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	switch (cmd) {
	case AVSYNC_V2_IOC_GET_CAPS:
		return avs_do_get_caps((struct avs_caps __user *)arg);
	case AVSYNC_V2_IOC_CREATE:
		return avs_do_create((struct avs_session_cfg __user *)arg);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations avs_global_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = avs_global_ioctl,
};

/*********************************************************************
 * Module init / exit
 *********************************************************************/

static int __init avsync_v2_init(void)
{
	int ret;

	INIT_LIST_HEAD(&avs_drv.sessions);
	spin_lock_init(&avs_drv.lock);

	/* Allocate device numbers */
	ret = alloc_chrdev_region(&avs_drv.major, 0,
				  1 + MAX_SESSIONS, DRIVER_NAME);
	if (ret) {
		pr_err("[avsync_v2] alloc_chrdev_region failed: %d\n", ret);
		return ret;
	}

	/* Create device class */
	avs_drv.class = class_create(DEV_CLASS_NAME);
	if (IS_ERR(avs_drv.class)) {
		ret = PTR_ERR(avs_drv.class);
		pr_err("[avsync_v2] class_create failed: %d\n", ret);
		goto err_chrdev;
	}

	/* Create the global control device */
	avs_drv.dev = device_create(avs_drv.class, NULL,
				    MKDEV(MAJOR(avs_drv.major), 0),
				    NULL, DRIVER_NAME);
	if (IS_ERR(avs_drv.dev)) {
		ret = PTR_ERR(avs_drv.dev);
		pr_err("[avsync_v2] device_create failed: %d\n", ret);
		goto err_class;
	}

	cdev_init(&avs_drv.cdev, &avs_global_fops);
	avs_drv.cdev.owner = THIS_MODULE;
	ret = cdev_add(&avs_drv.cdev, MKDEV(MAJOR(avs_drv.major), 0), 1);
	if (ret) {
		pr_err("[avsync_v2] cdev_add failed: %d\n", ret);
		goto err_device;
	}

	/* Register vsync notifier */
	vout_register_client(&avs_vsync_notifier);

	/* Register mode change notifier */
	vout_register_client(&avs_mode_notifier);

	/* Initial mode calculation */
	avs_recalc_mode();

	avs_drv.ready = true;

	pr_info("[avsync_v2] driver loaded, major=%d, max_sessions=%d\n",
		MAJOR(avs_drv.major), MAX_SESSIONS);
	return 0;

err_device:
	device_destroy(avs_drv.class, MKDEV(MAJOR(avs_drv.major), 0));
err_class:
	class_destroy(avs_drv.class);
err_chrdev:
	unregister_chrdev_region(avs_drv.major, 1 + MAX_SESSIONS);
	return ret;
}

static void __exit avsync_v2_exit(void)
{
	struct avs_session *s, *tmp;
	unsigned long flags;

	avs_drv.ready = false;

	vout_unregister_client(&avs_mode_notifier);
	vout_unregister_client(&avs_vsync_notifier);

	/* Tear down all sessions */
	spin_lock_irqsave(&avs_drv.lock, flags);
	list_for_each_entry_safe(s, tmp, &avs_drv.sessions, node) {
		list_del(&s->node);
		spin_unlock_irqrestore(&avs_drv.lock, flags);
		cancel_delayed_work_sync(&s->buffering_work);
		cancel_delayed_work_sync(&s->starting_work);
		avs_transition_to(s, AVS_STATE_IDLE);
		avs_drv.id_pool[s->id] = 0;
		avs_session_free(s);
		spin_lock_irqsave(&avs_drv.lock, flags);
	}
	spin_unlock_irqrestore(&avs_drv.lock, flags);

	cdev_del(&avs_drv.cdev);
	device_destroy(avs_drv.class, MKDEV(MAJOR(avs_drv.major), 0));
	class_destroy(avs_drv.class);
	unregister_chrdev_region(avs_drv.major, 1 + MAX_SESSIONS);

	pr_info("[avsync_v2] driver unloaded\n");
}

module_init(avsync_v2_init);
module_exit(avsync_v2_exit);

MODULE_DESCRIPTION("Amlogic Unified AV Sync Driver v2");
MODULE_AUTHOR("Amlogic, Inc.");
MODULE_LICENSE("Dual BSD/GPL");
