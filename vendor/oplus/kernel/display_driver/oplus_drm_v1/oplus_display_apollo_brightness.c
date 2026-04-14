/***************************************************************
  ** Copyright (C), 2024, OPLUS Mobile Comm Corp., Ltd
  **
  ** File : oplus_display_apollo_brightness.c
  ** Description : oplus display apollo_brightness feature
  ** Version : 1.0
  ** Date : 2024/05/20
  ** Author : Display
  ******************************************************************/
 #include <linux/delay.h>
#include <linux/ktime.h>
 
#include "mtk_panel_ext.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_debug.h"
#include "mtk_drm_trace.h"

#include "oplus_display_apollo_brightness.h" 
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

unsigned int ffl_backlight_backup = 0;
extern unsigned int oplus_bl_demura_dbv_switched;
extern long long mutex_sof_ns;

void oplus_display_apollo_init_para(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc) {
		pr_err("oplus_display_apollo_crtc_init_para init fail\n");
		return;
	}

	mtk_crtc->oplus_apollo_br = kzalloc(sizeof(struct oplus_apollo_brightness),
			GFP_KERNEL);
	pr_info("oplus_apollo_brightness need allocate memory\n");

	if (!mtk_crtc->oplus_apollo_br) {
		pr_err("oplus_apollo_brightness allocate memory fail\n");
		return;
	}

	mtk_crtc->oplus_apollo_br->oplus_power_on = true;
	mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching = false;
	mtk_crtc->oplus_apollo_br->oplus_te_tag_ns = 0;
	mtk_crtc->oplus_apollo_br->oplus_te_diff_ns = 0;
	mtk_crtc->oplus_apollo_br->cur_vsync = 0;
	mtk_crtc->oplus_apollo_br->limit_superior_ns = 0;
	mtk_crtc->oplus_apollo_br->limit_inferior_ns = 0;
	mtk_crtc->oplus_apollo_br->transfer_time_us = 0;
	mtk_crtc->oplus_apollo_br->pending_vsync = 0;
	mtk_crtc->oplus_apollo_br->pending_limit_superior_ns = 0;
	mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns = 0;
	mtk_crtc->oplus_apollo_br->pending_transfer_time_us = 0;
	mtk_crtc->oplus_apollo_br->pending_pu_enable = 0;
	mtk_crtc->oplus_apollo_br->pending_h_roi = 0;
	mtk_crtc->oplus_apollo_br->h_roi = 0;
	mtk_crtc->oplus_apollo_br->last_frame_pu_enable = 0;
	mtk_crtc->oplus_apollo_br->h_max = mtk_crtc_get_height_by_comp(__func__, crtc, NULL, true);
}

static void oplus_bl_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

int mtk_drm_setbacklight_without_lock(struct drm_crtc *crtc, unsigned int level, unsigned int panel_ext_param, unsigned int cfg_flag)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_cmdq_cb_data *cb_data;
	struct cmdq_client *client;
	static unsigned int bl_cnt;
	int delay_us, time_gap_ns;
	bool is_frame_mode;
	bool is_doze_mode = false;
	int index = drm_crtc_index(crtc);
	int ret = 0;

	CRTC_MMP_EVENT_START(index, backlight, (unsigned long)crtc,
			level);

	if (!(mtk_crtc->enabled)) {
		DDPINFO("Sleep State set backlight stop --crtc not ebable\n");
		CRTC_MMP_EVENT_END(index, backlight, 0, 0);

		return 0;
	}

	if (!(comp && mtk_ddp_comp_get_type(comp->id) == MTK_DSI)) {
		DDPINFO("%s no output comp\n", __func__);
		CRTC_MMP_EVENT_END(index, backlight, 0, 1);

		return 0;
	}

	if (!(comp->funcs && comp->funcs->io_cmd)) {
		DDPPR_ERR("%s:%d NULL comp\n", __func__, __LINE__);
		return 0;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("cb data creation failed\n");
		CRTC_MMP_EVENT_END(index, backlight, 0, 2);
		return -EINVAL;
	}

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	/* setbacklight would use VM CMD in  DSI VDO mode only */
	client = (is_frame_mode) ? mtk_crtc->gce_obj.client[CLIENT_CFG] :
					mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];
	cmdq_handle = cmdq_pkt_create(client);

	/* frame done gce event revise, fix by Faker at 2022/10/31 */
	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);
	/* frame done gce event revise, end */

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);

	}

	ffl_backlight_backup = level;

	/* frame done gce event revise, fix by Faker at 2022/10/31 */
	/*
	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, is_frame_mode);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, is_frame_mode);
	*/
	/* frame done gce event revise, end */

	if (mtk_crtc->oplus_apollo_br->oplus_power_on == false) {
		level = 0;
	}
	if (mtk_crtc->oplus_apollo_br->cur_vsync == 0) {
		mtk_crtc->oplus_apollo_br->cur_vsync = mtk_crtc->oplus_apollo_br->pending_vsync;
		mtk_crtc->oplus_apollo_br->limit_superior_ns = mtk_crtc->oplus_apollo_br->pending_limit_superior_ns;
		mtk_crtc->oplus_apollo_br->limit_inferior_ns = mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns;
		mtk_crtc->oplus_apollo_br->transfer_time_us = mtk_crtc->oplus_apollo_br->pending_transfer_time_us;
	}

	if (crtc->state && crtc->state->enable) {
		comp->funcs->io_cmd(comp, NULL, DSI_GET_AOD_STATE, &is_doze_mode);
		DDPINFO("[demura_dbv_switch]check doze mode=%d\n", is_doze_mode);
	}
	if (oplus_display_get_demura_support()) {
		if ((is_doze_mode) || (oplus_ofp_is_supported() && oplus_ofp_backlight_filter(level))) {
			pr_info("aod/fod state, ignore current backlight with demura\n");
		} else {
			oplus_panel_backlight_demura_dbv_switch(crtc, cmdq_handle, level);
			if (oplus_bl_demura_dbv_switched) {
				delay_us = mtk_crtc->oplus_apollo_br->cur_vsync / 2 / 1000;
				mtk_drm_trace_begin("DSI_SET_DEMURA_BL cmdq sleep %d us", delay_us);
				cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R06);
				mtk_drm_trace_end();
				oplus_bl_demura_dbv_switched = 0;
			}
		}
	}

	if (mtk_crtc->oplus_apollo_br->oplus_backlight_need_sync && mtk_crtc->oplus_apollo_br->cur_vsync != 0) {
		/* backlight sync start */
		mtk_drm_trace_begin("cur_vsync(%d) pending_vsync(%d)", mtk_crtc->oplus_apollo_br->cur_vsync, mtk_crtc->oplus_apollo_br->pending_vsync);
		time_gap_ns = ktime_get() > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns ? ktime_get() - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns : 0;
		if (mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching && mtk_crtc->oplus_apollo_br->oplus_timing_switch_tag_ns > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns) {
			usleep_range((mtk_crtc->oplus_apollo_br->cur_vsync + mtk_crtc->oplus_apollo_br->pending_vsync)/1000, (mtk_crtc->oplus_apollo_br->cur_vsync + mtk_crtc->oplus_apollo_br->pending_vsync)/1000 + 100);
			time_gap_ns = ktime_get() > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns ? ktime_get() - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns : 0;
			oplus_update_apollo_para(crtc);
			delay_us = (mtk_crtc->oplus_apollo_br->cur_vsync - time_gap_ns) / 1000 + mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us;
			if (delay_us > 0) {
				mtk_drm_trace_begin("cmdq sleep %d us-2", delay_us);
				cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R06);
				mtk_drm_trace_end();
			}
		} else if (time_gap_ns >= 0 && time_gap_ns <= mtk_crtc->oplus_apollo_br->cur_vsync) {
			if (mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching) {
				usleep_range((mtk_crtc->oplus_apollo_br->cur_vsync + mtk_crtc->oplus_apollo_br->pending_vsync)/1000, (mtk_crtc->oplus_apollo_br->cur_vsync + mtk_crtc->oplus_apollo_br->pending_vsync)/1000 + 100);
				time_gap_ns = ktime_get() > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns ? ktime_get() - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns : 0;
				oplus_update_apollo_para(crtc);
			}
			if (time_gap_ns < mtk_crtc->oplus_apollo_br->limit_superior_ns) {
				if ((mtk_crtc->oplus_apollo_br->oplus_te_tag_ns > mutex_sof_ns
						&& mtk_crtc->oplus_apollo_br->oplus_te_tag_ns - mutex_sof_ns < mtk_crtc->oplus_apollo_br->cur_vsync / 2)
					|| (mutex_sof_ns > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns
						&& mutex_sof_ns - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns < mtk_crtc->oplus_apollo_br->cur_vsync / 2)) {
					delay_us = mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us - mtk_crtc->oplus_apollo_br->transfer_time_us;
				} else {
					delay_us = (mtk_crtc->oplus_apollo_br->limit_superior_ns - time_gap_ns) / 1000;
				}
				if (delay_us > 0) {
					mtk_drm_trace_begin("cmdq sleep %d us-0", delay_us);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R06);
					mtk_drm_trace_end();
				}
			} else if (time_gap_ns > mtk_crtc->oplus_apollo_br->limit_inferior_ns) {
				delay_us = (mtk_crtc->oplus_apollo_br->cur_vsync - time_gap_ns) / 1000 + mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us;
				if (delay_us > 0) {
					mtk_drm_trace_begin("cmdq sleep %d us-1", delay_us);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R06);
					mtk_drm_trace_end();
				}
			}
		} else {
			pr_err("%s backlight sync failed ! time_gap_ns(%d) is large than cur_vsync(%d)\n", __func__, time_gap_ns, mtk_crtc->oplus_apollo_br->cur_vsync);
		}
		mtk_drm_trace_end();
		/* backlight sync end */
	}

	/* set backlight */
	oplus_printf_backlight_log(crtc, level);
#ifdef OPLUS_FEATURE_DISPLAY
			oplus_drm_sethpwm_bl(crtc, cmdq_handle, level, true);
#else
			if (comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
#endif

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		/* frame done gce event revise, fix by Faker at 2022/10/31 */
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		/* frame done gce event revise, end */
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	CRTC_MMP_MARK(index, backlight, bl_cnt, 0);
	bl_cnt++;

	cb_data->crtc = crtc;
	cb_data->cmdq_handle = cmdq_handle;
	// #ifdef OPLUS_BUG_STABILITY
	cb_data->bl = level;
	// #endif OPLUS_BUG_STABILITY

	if (cmdq_pkt_flush_threaded(cmdq_handle, oplus_bl_cmdq_cb, cb_data) < 0) {
		DDPPR_ERR("failed to flush bl_cmdq_cb\n");
		ret = -EINVAL;
	}

	CRTC_MMP_EVENT_END(index, backlight, (unsigned long)crtc,
			level);

	return ret;
}
