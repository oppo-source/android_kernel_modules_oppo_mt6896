/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_mtk_debug.c
** Description : oplus display mtk debug
** Version : 1.0
** Date : 2020/12/06
**
** ------------------------------- Revision History: -----------
**  <author>        <date>        <version >        <desc>
**  Xiaolei.Gao    2020/12/06        1.0           Build this moudle
******************************************************************/
#include "oplus_display_mtk_debug.h"
#include "oplus_display_panel.h"
#include "mtk_panel_ext.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_debug.h"
#include <soc/oplus/boot/oplus_project.h>

extern bool g_mobile_log;
extern bool g_detail_log;
extern bool g_irq_log;
extern bool g_fence_log;
extern unsigned int g_trace_log;
extern int mtk_cmdq_msg;

unsigned int oplus_display_log_level = OPLUS_LOG_LEVEL_INFO;
EXPORT_SYMBOL(oplus_display_log_level);
unsigned int oplus_display_log_type = OPLUS_DEBUG_LOG_DISABLED;
EXPORT_SYMBOL(oplus_display_log_type);
unsigned int oplus_display_trace_enable = OPLUS_DISPLAY_DISABLE_TRACE;
EXPORT_SYMBOL(oplus_display_trace_enable);
static bool g_mobile_log_default_state = false;

extern void drm_invoke_fps_chg_callbacks(unsigned int new_fps);
extern void set_logger_enable(int enable);
#ifdef OPLUS_FEATURE_DISPLAY
extern void pq_dump_all(unsigned int dump_flag);
#endif

/* log level config */
int oplus_disp_drv_log_level = OPLUS_DISP_DRV_LOG_LEVEL_INFO; /*After STR5 should set OPLUS_DISP_DRV_LOG_LEVEL_INFO*/
EXPORT_SYMBOL(oplus_disp_drv_log_level);

int oplus_display_set_mtk_loglevel(void *buf)
{
	struct kernel_loglevel *loginfo = buf;
	unsigned int enabled = 0;
	unsigned int loglevel = 0;

	enabled = loginfo->enable;
	loglevel = loginfo->log_level;

	printk("%s,mtk log level is 0x%x,enable=%d", __func__, loglevel, enabled);

	if (enabled == 1) {
		if ((loglevel & MTK_LOG_LEVEL_MOBILE_LOG) == MTK_LOG_LEVEL_MOBILE_LOG) {
			g_mobile_log = true;
			set_logger_enable(1);
			mtk_cmdq_msg = 1;
		}
		if ((loglevel & MTK_LOG_LEVEL_DETAIL_LOG) == MTK_LOG_LEVEL_DETAIL_LOG)
			g_detail_log = true;
		if ((loglevel & MTK_LOG_LEVEL_FENCE_LOG) == MTK_LOG_LEVEL_FENCE_LOG)
			g_fence_log = true;
		if ((loglevel & MTK_LOG_LEVEL_IRQ_LOG) == MTK_LOG_LEVEL_IRQ_LOG)
			g_irq_log = true;
		if ((loglevel & MTK_LOG_LEVEL_TRACE_LOG) == MTK_LOG_LEVEL_TRACE_LOG)
			g_trace_log = true;
		if ((loglevel & MTK_LOG_LEVEL_DUMP_REGS) == MTK_LOG_LEVEL_DUMP_REGS) {
			if (!g_mobile_log) {
				g_mobile_log = true;
				g_mobile_log_default_state = false;
			} else {
				g_mobile_log_default_state = true;
			}
#ifdef OPLUS_FEATURE_DISPLAY
			pq_dump_all(0xFF);
#endif
			if (!g_mobile_log_default_state) {
				g_mobile_log = false;
			}
		}
	} else {
		if ((loglevel & MTK_LOG_LEVEL_MOBILE_LOG) == MTK_LOG_LEVEL_MOBILE_LOG) {
			g_mobile_log = false;
			set_logger_enable(0);
			mtk_cmdq_msg = 0;
		}
		if ((loglevel & MTK_LOG_LEVEL_DETAIL_LOG) == MTK_LOG_LEVEL_DETAIL_LOG)
			g_detail_log = false;
		if ((loglevel & MTK_LOG_LEVEL_FENCE_LOG) == MTK_LOG_LEVEL_FENCE_LOG)
			g_fence_log = false;
		if ((loglevel & MTK_LOG_LEVEL_IRQ_LOG) == MTK_LOG_LEVEL_IRQ_LOG)
			g_irq_log = false;
		if ((loglevel & MTK_LOG_LEVEL_TRACE_LOG) == MTK_LOG_LEVEL_TRACE_LOG)
			g_trace_log = false;
		if ((loglevel & MTK_LOG_LEVEL_DUMP_REGS) == MTK_LOG_LEVEL_DUMP_REGS) {
			if (!g_mobile_log_default_state) {
				g_mobile_log = false;
			}
		}
	}

	return 0;
}

int oplus_display_set_limit_fps(void *buf)
{
	unsigned int limit_fps = 0;
	unsigned int *p_fps = buf;

	limit_fps = (*p_fps);

	drm_invoke_fps_chg_callbacks(limit_fps);
	return 0;
}

void oplus_printf_backlight_log(struct drm_crtc *crtc, unsigned int bl_lvl) {
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_panel_params *params = mtk_crtc->panel_ext->params;
	struct timespec64 now;
	struct tm broken_time;
	static time64_t time_last = 0;
	struct backlight_log *bl_log;
	int i = 0;
	int len = 0;
	char backlight_log_buf[1024];


	if (!comp) {
		DDPPR_ERR("failed to get comp\n");
		return;
	}

	ktime_get_real_ts64(&now);
	time64_to_tm(now.tv_sec, 0, &broken_time);
	if (now.tv_sec - time_last >= 60) {
		pr_info("<%s> dsi_display_set_backlight time:%02d:%02d:%02d.%03ld,bl_lvl:%d\n",
				params->vendor, broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, now.tv_nsec / 1000000, bl_lvl);
		time_last = now.tv_sec;
	}

	if (comp->id == DDP_COMPONENT_DSI1) {
		bl_log = &oplus_bl_log[DISPLAY_SECONDARY];
	} else {
		bl_log = &oplus_bl_log[DISPLAY_PRIMARY];
	}


	bl_log->backlight[bl_log->bl_count] = bl_lvl;
	bl_log->past_times[bl_log->bl_count] = now;
	bl_log->bl_count++;
	if (bl_log->bl_count >= BACKLIGHT_CACHE_MAX) {
		bl_log->bl_count = 0;
		memset(backlight_log_buf, 0, sizeof(backlight_log_buf));
		for (i = 0; i < BACKLIGHT_CACHE_MAX; i++) {
			time64_to_tm(bl_log->past_times[i].tv_sec, 0, &broken_time);
			len += snprintf(backlight_log_buf + len, sizeof(backlight_log_buf) - len,
				"%02d:%02d:%02d.%03ld:%d,", broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, bl_log->past_times[i].tv_nsec / 1000000, bl_log->backlight[i]);
		}
		pr_info("<%s> len:%d dsi_display_set_backlight %s\n", params->vendor, len, backlight_log_buf);
	}
}
EXPORT_SYMBOL(oplus_printf_backlight_log);


int trig_db_enable;
void oplus_display_set_mobile_log(bool en) {
  	if ((get_eng_version() == AGING) || trig_db_enable)
  		g_mobile_log = en;
}

void oplus_display_kill_surfaceflinger(void) {
  	struct task_struct *p;
  	if ((get_eng_version() == AGING) || trig_db_enable) {
  		read_lock(&tasklist_lock);
  		for_each_process(p) {
  			get_task_struct(p);
  			if (strcmp(p->comm, "surfaceflinger") == 0) {
  				send_sig_info(SIGABRT, SEND_SIG_PRIV, p);
  			}
  			put_task_struct(p);
  		}
  		read_unlock(&tasklist_lock);
  	}
}

extern bool logger_enable;
void set_logger_enable(int enable)
{
  	if (enable == 1) {
  		init_log_buffer();
  		logger_enable = 1;
  	} else if (enable == 0) {
  		logger_enable = 0;
  	}
  }
EXPORT_SYMBOL(set_logger_enable);

MODULE_AUTHOR("Xiaolei Gao");
MODULE_DESCRIPTION("OPPO debug device");
MODULE_LICENSE("GPL v2");

