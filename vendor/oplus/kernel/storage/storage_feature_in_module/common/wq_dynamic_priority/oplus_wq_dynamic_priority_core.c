// SPDX-License-Identifier: GPL-2.0
/*
 * OPLUS Workqueue Dynamic Priority & Block Layer Optimization
 *
 * Copyright (C) OPLUS
 */
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(6, 12, 0)

/* ============================================================
 * Section 1: Includes & Macros
 * ============================================================ */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/ioprio.h>
#include <linux/blk-mq.h>
#include <linux/reboot.h>
#include <linux/atomic.h>
#include <uapi/linux/major.h>
#include <trace/hooks/wqlockup.h>
#include <trace/hooks/blk.h>
#include <trace/events/block.h>
#include "elevator.h"
#include "dm-verity.h"
#include "dm-verity-fec.h"
#include "oplus_wq_dynamic_priority.h"
#include <linux/sa_common.h>

#define VIRTUAL_KWORKER_NORMAL_NICE (-1000)
#define WQ_CMP(str)  (strncmp(wq->name, str, sizeof(str) - 1) == 0)

/* ============================================================
 * Section 2: Module Parameters & Statistics
 * ============================================================ */
static struct workqueue_attrs *ux_wq_attrs;
static struct workqueue_struct *oplus_kverityd_wq;
static void *orig_verity_prefetch_io;
static void (*orig_verity_fec_finish_io)(struct dm_verity_io *io);
static struct work_struct verity_kp_register_work;
static atomic_t verity_work_found = ATOMIC_INIT(0);
static void verity_kp_register_workfn(struct work_struct *work);

/* Block dispatch counters */
long blk_sched = 0;
long ublk_sched = 0;
module_param(blk_sched, long, 0660);
module_param(ublk_sched, long, 0660);

/* Verity counters */
static long kverity_cnt = 0;
static long kverity_ux_cnt = 0;
module_param(kverity_cnt, long, 0660);
module_param(kverity_ux_cnt, long, 0660);

/* Feature switches */
static bool kverify_always_ux = true;
bool kblockd_always_ux = true;
module_param(kverify_always_ux, bool, 0660);
module_param(kblockd_always_ux, bool, 0660);

/* Sched latency stats - ALL requests */
static unsigned long lat_all_cnt = 0;
static unsigned long lat_all_sum_us = 0;
static unsigned long lat_all_10ms_cnt = 0;
module_param(lat_all_cnt, long, 0660);
module_param(lat_all_sum_us, long, 0660);
module_param(lat_all_10ms_cnt, long, 0660);

/* Sched latency stats - RT IO only */
static unsigned long lat_rt_cnt = 0;
static unsigned long lat_rt_sum_us = 0;
static unsigned long lat_rt_10ms_cnt = 0;
module_param(lat_rt_cnt, long, 0660);
module_param(lat_rt_sum_us, long, 0660);
module_param(lat_rt_10ms_cnt, long, 0660);

/* ============================================================
 * Section 3: Workqueue Optimization
 * ============================================================ */
struct config_wq_flags {
	char *target_str;
	unsigned int new_flags;
};

static struct config_wq_flags oplus_wq_config[] = {
	{ "loop", WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI },
	{ NULL, 0 }
};

static void android_rvh_alloc_and_link_pwqs_handler(void *unused,
		struct workqueue_struct *wq, int *ret, bool *skip)
{
	if (WQ_CMP("opluskverityd") || WQ_CMP("loop")) {
		*ret = apply_workqueue_attrs_locked(wq, ux_wq_attrs);
		*skip = true;
	}
}

static int handler_alloc_workqueue_pre(struct kprobe *p, struct pt_regs *regs)
{
	const char *fmt = (const char *)regs->regs[0];
	unsigned int flags = (unsigned int)regs->regs[1];
	struct config_wq_flags *item = oplus_wq_config;

	if (fmt) {
		while (item->target_str) {
			if ((strlen(fmt) >= strlen(item->target_str)) &&
			    !strncmp(fmt, item->target_str, strlen(item->target_str)) &&
			    (item->new_flags != flags)) {
				pr_info("alloc_workqueue: matching fmt '%s', flags 0x%x -> 0x%x\n",
					fmt, flags, item->new_flags);
				regs->regs[1] = item->new_flags;
				break;
			}
			item++;
		}
	}
	return 0;
}

static struct kprobe oplus_alloc_workqueue_kp = {
	.symbol_name = "alloc_workqueue",
	.pre_handler = handler_alloc_workqueue_pre,
};

static void android_rvh_create_worker_handler(void *unused,
		struct task_struct *task, struct workqueue_attrs *attrs)
{
	if (attrs->nice == VIRTUAL_KWORKER_NORMAL_NICE) {
		set_user_nice(task, MIN_NICE);
		oplus_set_ux_state_lock(task, SA_TYPE_LIGHT, -1, true);
		//sched_set_fifo_low(task);
		if (task->comm[8] == 'u')
			task->comm[8] = 'X';
	}
}

/* ============================================================
 * Section 4: Block Layer Dispatch Hooks
 * ============================================================ */
static void android_vh_blk_mq_kick_requeue_list_handler(void *unused,
		struct request_queue *q, unsigned long delay, bool *skip)
{
	mod_delayed_work_on(WORK_CPU_UNBOUND, oplus_kverityd_wq,
			    &q->requeue_work, 0);
	*skip = true;
}

struct hctx_sched_entry {
	struct blk_mq_hw_ctx *hctx;
	struct kthread_delayed_work dwork;
};

static void android_vh_blk_mq_delay_run_hw_queue_handler(void *unused,
		int cpu, struct blk_mq_hw_ctx *hctx, unsigned long delay, bool *skip)
{
	struct request_queue *q = hctx->queue;
	struct elevator_queue *e = q->elevator;
	struct hctx_sched_entry *entry;

	blk_sched++;

	if (unlikely(kblockd_always_ux)) {
		*skip = true;
		ublk_sched++;
		mod_delayed_work_on(cpu, oplus_kverityd_wq, &hctx->run_work, delay);
		return;
	}

	if (e && (e->flags & ELEVATOR_F_DISPATCH_SEP)) {
		entry = (struct hctx_sched_entry *)q->android_oem_data1;

		if (!sd_has_work_for_prioclass(hctx, IOPRIO_CLASS_RT))
			return;

		if (cpu == WORK_CPU_UNBOUND)
			cpu = raw_smp_processor_id();

		if (q->nr_hw_queues > 1)
			entry = &entry[cpu];

		ublk_sched++;
		kthread_mod_delayed_work(blk_workers[cpu], &entry->dwork, 0);
	}
}

/*
 * Tracepoint probe for block_rq_complete - works with all schedulers
 * Statistics: total count, sum latency (us), >10ms count
 * Separate stats for RT IO vs ALL IO
 */
static void block_rq_complete_handler(void *unused, struct request *rq,
		blk_status_t error, unsigned int nr_bytes)
{
	u64 sched_lat_ns;
	u64 sched_lat_us;
	u8 ioprio_class;

	if (!rq->io_start_time_ns || !rq->start_time_ns)
		return;

	/* Sanity check: io_start_time must be after start_time */
	if (rq->io_start_time_ns < rq->start_time_ns)
		return;

	sched_lat_ns = rq->io_start_time_ns - rq->start_time_ns;

	sched_lat_us = sched_lat_ns / 1000;
	ioprio_class = IOPRIO_PRIO_CLASS(rq->ioprio);

	/* ALL IO stats */
	lat_all_cnt++;
	lat_all_sum_us += sched_lat_us;
	if (sched_lat_ns > 10 * NSEC_PER_MSEC)
		lat_all_10ms_cnt++;

	/* RT IO stats */
	if (ioprio_class == IOPRIO_CLASS_RT) {
		lat_rt_cnt++;
		lat_rt_sum_us += sched_lat_us;
		if (sched_lat_ns > 10 * NSEC_PER_MSEC)
			lat_rt_10ms_cnt++;
	}
}

/* ============================================================
 * Section 5: Tracepoint Framework
 * ============================================================ */
struct tracepoints_table {
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool registered;
};

static struct tracepoints_table tp_table[] = {
	{ .name = "android_rvh_alloc_and_link_pwqs",
	  .func = android_rvh_alloc_and_link_pwqs_handler },
	{ .name = "android_rvh_create_worker",
	  .func = android_rvh_create_worker_handler },
	{ .name = "android_vh_blk_mq_delay_run_hw_queue",
	  .func = android_vh_blk_mq_delay_run_hw_queue_handler },
	{ .name = "android_vh_blk_mq_kick_requeue_list",
	  .func = android_vh_blk_mq_kick_requeue_list_handler },
	{ .name = "block_rq_complete",
	  .func = block_rq_complete_handler },
};

#define TP_TABLE_SIZE	ARRAY_SIZE(tp_table)

static void lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	int i;

	for (i = 0; i < TP_TABLE_SIZE; i++) {
		if (strcmp(tp_table[i].name, tp->name) == 0)
			tp_table[i].tp = tp;
	}
}

static int register_tracepoints(int start, int end)
{
	int i;

	if (end >= TP_TABLE_SIZE)
		end = TP_TABLE_SIZE - 1;

	for (i = start; i <= end; i++) {
		if (!tp_table[i].tp) {
			pr_err("%s: tracepoint %s not found\n",
			       THIS_MODULE->name, tp_table[i].name);
			return -ENOENT;
		}

		if (!tp_table[i].registered) {
			tracepoint_probe_register(tp_table[i].tp,
						  tp_table[i].func, NULL);
			tp_table[i].registered = true;
		}
	}

	return 0;
}

static void unregister_tracepoints(int start, int end)
{
	int i;

	for (i = start; i <= end && i < TP_TABLE_SIZE; i++) {
		if (tp_table[i].registered) {
			tracepoint_probe_unregister(tp_table[i].tp,
						    tp_table[i].func, NULL);
			tp_table[i].registered = false;
		}
	}
}

static void unregister_all_tracepoints(void)
{
	unregister_tracepoints(0, TP_TABLE_SIZE - 1);
}

/* ============================================================
 * Section 6: DM-Verity Optimization
 * ============================================================ */
static bool oplus_verity_fec_is_enabled(struct dm_verity *v)
{
	return v->fec && v->fec->dev;
}

static inline bool verity_is_system_shutting_down(void)
{
	return system_state == SYSTEM_HALT ||
	       system_state == SYSTEM_POWER_OFF ||
	       system_state == SYSTEM_RESTART;
}

static void restart_io_error(struct work_struct *w)
{
	kernel_restart("dm-verity device has I/O error");
}

static void oplus_verity_finish_io(struct dm_verity_io *io, blk_status_t status)
{
	struct dm_verity *v = io->v;
	struct bio *bio = dm_bio_from_per_bio_data(io, v->ti->per_io_data_size);

	bio->bi_end_io = io->orig_bi_end_io;
	bio->bi_status = status;

	orig_verity_fec_finish_io(io);

	if (unlikely(status != BLK_STS_OK) &&
	    unlikely(!(bio->bi_opf & REQ_RAHEAD)) &&
	    !io->had_mismatch &&
	    !verity_is_system_shutting_down()) {
		if (v->error_mode == DM_VERITY_MODE_PANIC)
			panic("dm-verity device has I/O error");
		if (v->error_mode == DM_VERITY_MODE_RESTART) {
			static DECLARE_WORK(restart_work, restart_io_error);
			queue_work(v->verify_wq, &restart_work);
			return;
		}
	}

	bio_endio(bio);
}

static void (*orig_verity_work)(struct work_struct *work);

static void oplus_verity_end_io(struct bio *bio)
{
	struct dm_verity_io *io = bio->bi_private;

	if (bio->bi_status &&
	    (!oplus_verity_fec_is_enabled(io->v) ||
	     verity_is_system_shutting_down() ||
	     (bio->bi_opf & REQ_RAHEAD))) {
		oplus_verity_finish_io(io, bio->bi_status);
		return;
	}

	INIT_WORK(&io->work, orig_verity_work);
	queue_work(oplus_kverityd_wq, &io->work);
}

static struct kprobe oplus_verity_fec_init_kp = {
	.symbol_name = "verity_fec_init_io",
};

static int verity_fec_init_pre_handler(struct kprobe *p, struct pt_regs *regs)
{
	struct dm_verity_io *io;
	struct bio *bio;
	unsigned short ioprio_class;

	io = (struct dm_verity_io *)regs->regs[0];
	if (!io || !io->v || !io->v->ti)
		return 0;

	bio = dm_bio_from_per_bio_data(io, io->v->ti->per_io_data_size);
	if (!bio)
		return 0;

	ioprio_class = IOPRIO_PRIO_CLASS(bio->bi_ioprio);
	kverity_cnt++;
	if (ioprio_class == IOPRIO_CLASS_RT || kverify_always_ux) {
		bio->bi_end_io = oplus_verity_end_io;
		kverity_ux_cnt++;
	}

	return 0;
}

static struct kprobe oplus_submit_bio_kp = {
	.symbol_name = "submit_bio_noacct",
};

static int submit_bio_pre_handler(struct kprobe *p, struct pt_regs *regs)
{
	struct bio *bio = (struct bio *)regs->regs[0];

	if (!bio || !in_task())
		return 0;

	if (test_task_ux(current))
		bio->bi_ioprio = IOPRIO_PRIO_VALUE(IOPRIO_CLASS_RT, 5);

	return 0;
}

static int find_kverity_work_handler(struct kprobe *p, struct pt_regs *regs)
{
	struct workqueue_struct *wq;
	struct work_struct *work;
	work_func_t func;

	if (atomic_read(&verity_work_found))
		return 0;

	wq = (struct workqueue_struct *)regs->regs[1];
	work = (struct work_struct *)regs->regs[2];

	if (!wq || !work || !WQ_CMP("kverityd"))
		return 0;

	func = work->func;

	if ((void *)func == orig_verity_prefetch_io)
		return 0;

	if (atomic_inc_return(&verity_work_found) == 1) {
		orig_verity_work = func;
		pr_info("Found verity_work = %pS\n", orig_verity_work);
		schedule_work(&verity_kp_register_work);
	}

	return 0;
}

static struct kprobe find_kverity_work_kp = {
	.symbol_name = "queue_work_on",
	.pre_handler = find_kverity_work_handler,
};

static void verity_kp_register_workfn(struct work_struct *work)
{
	int err;

	if (!orig_verity_work) {
		pr_err("%s: can't find orig_verity_work\n", __func__);
		return;
	}

	oplus_verity_fec_init_kp.pre_handler = verity_fec_init_pre_handler;
	err = register_kprobe(&oplus_verity_fec_init_kp);
	if (err < 0) {
		pr_err("register verity_fec_init_io failed: %d\n", err);
		return;
	}

	unregister_kprobe(&find_kverity_work_kp);
}

static void *lookup_name_via_kprobe(const char *name)
{
	struct kprobe kp = { .symbol_name = name };
	void *addr;

	if (register_kprobe(&kp) < 0)
		return NULL;

	addr = (void *)kp.addr;
	unregister_kprobe(&kp);
	return addr;
}

static int kverity_hook_init(void)
{
	int err;

	orig_verity_prefetch_io = lookup_name_via_kprobe("verity_prefetch_io");
	if (!orig_verity_prefetch_io) {
		 pr_err("verity_prefetch_io not found\n");
		 return -ENOENT;
	}

	orig_verity_fec_finish_io = lookup_name_via_kprobe("verity_fec_finish_io");
	if (!orig_verity_fec_finish_io) {
		 pr_err("verity_fec_finish_io not found\n");
		 return -ENOENT;
	}

	INIT_WORK(&verity_kp_register_work, verity_kp_register_workfn);

	err = register_kprobe(&find_kverity_work_kp);
	if (err < 0) {
		pr_err("register find_kverity_work_kp failed: %d\n", err);
		return err;
	}

	return 0;
}

static void kverity_hook_exit(void)
{
	cancel_work_sync(&verity_kp_register_work);

	if (atomic_read(&verity_work_found))
		unregister_kprobe(&oplus_verity_fec_init_kp);
	else
		unregister_kprobe(&find_kverity_work_kp);
}

/* ============================================================
 * Section 7: Module Init/Exit
 * ============================================================ */
static int __init oplus_wq_hook_init(void)
{
	int err;

	/* Step 1: Register kprobes */
	err = register_kprobe(&oplus_alloc_workqueue_kp);
	if (err < 0) {
		pr_err("%s: kprobe alloc_workqueue failed: %d\n", __func__, err);
		return err;
	}

	oplus_submit_bio_kp.pre_handler = submit_bio_pre_handler;
	err = register_kprobe(&oplus_submit_bio_kp);
	if (err < 0) {
		pr_err("%s: kprobe submit_bio failed: %d\n", __func__, err);
		goto err_unregister_alloc_wq_kp;
	}

	/* Step 2: Allocate ux_wq_attrs first (needed by tracepoint 0-1 handlers) */
	ux_wq_attrs = alloc_workqueue_attrs();
	if (!ux_wq_attrs) {
		pr_err("%s: alloc ux_wq_attrs failed\n", __func__);
		err = -ENOMEM;
		goto err_unregister_submit_bio_kp;
	}
	ux_wq_attrs->nice = VIRTUAL_KWORKER_NORMAL_NICE;

	/* Step 3: Lookup and register tracepoints 0-1 (use ux_wq_attrs) */
	for_each_kernel_tracepoint(lookup_tracepoints, NULL);

	err = register_tracepoints(0, 1);
	if (err) {
		pr_err("%s: register tracepoints 0-1 failed\n", __func__);
		goto err_free_attrs;
	}

	/* Step 4: Create oplus_kverityd_wq (needed by tracepoint 2-4 handlers)
	 * Use "opluskverityd" name to trigger android_rvh_alloc_and_link_pwqs_handler
	 * which applies ux_wq_attrs for UX inheritance */
	oplus_kverityd_wq = alloc_workqueue("opluskverityd",
					    WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!oplus_kverityd_wq) {
		pr_err("%s: alloc oplus_kverityd_wq failed\n", __func__);
		err = -ENOMEM;
		goto err_unregister_tp_0_1;
	}

	/* Step 5: Register tracepoints 2-4 (use oplus_kverityd_wq) */
	err = register_tracepoints(2, 4);
	if (err) {
		pr_err("%s: register tracepoints 2-4 failed\n", __func__);
		goto err_destroy_wq;
	}

	/* Step 6: Initialize kverity hook */
	err = kverity_hook_init();
	if (err) {
		pr_err("%s: kverity_hook_init failed: %d\n", __func__, err);
		goto err_unregister_tp_2_4;
	}

	/* Step 7: Initialize IO scheduler */
	simple_deadline_init();

	pr_info("%s: init success\n", __func__);
	return 0;

err_unregister_tp_2_4:
	unregister_tracepoints(2, 4);
err_destroy_wq:
	destroy_workqueue(oplus_kverityd_wq);
err_unregister_tp_0_1:
	unregister_tracepoints(0, 1);
err_free_attrs:
	free_workqueue_attrs(ux_wq_attrs);
err_unregister_submit_bio_kp:
	unregister_kprobe(&oplus_submit_bio_kp);
err_unregister_alloc_wq_kp:
	unregister_kprobe(&oplus_alloc_workqueue_kp);
	pr_err("%s: init failed\n", __func__);
	return err;
}

static void __exit oplus_wq_hook_exit(void)
{
	simple_deadline_exit();
	kverity_hook_exit();
	unregister_all_tracepoints();
	destroy_workqueue(oplus_kverityd_wq);
	free_workqueue_attrs(ux_wq_attrs);
	unregister_kprobe(&oplus_submit_bio_kp);
	unregister_kprobe(&oplus_alloc_workqueue_kp);
}

module_init(oplus_wq_hook_init);
module_exit(oplus_wq_hook_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lijiang");
MODULE_AUTHOR("Gray Jia");
MODULE_DESCRIPTION("OPLUS Block Layer & Workqueue Optimization");

#endif /* LINUX_VERSION_CODE > KERNEL_VERSION(6, 6, 0) */