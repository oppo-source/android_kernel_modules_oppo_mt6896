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
 
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "oplus_adfr.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif

extern unsigned int hpwm_mode;
unsigned int hpwm_mode_90hz = 0;
unsigned int hpwm_fps_mode; 
bool pwm_power_on = false;
EXPORT_SYMBOL(pwm_power_on);
extern unsigned int oplus_display_brightness;
unsigned int hpwm_90nit_set_temp;
EXPORT_SYMBOL(hpwm_90nit_set_temp);
unsigned int last_backlight = 0;
EXPORT_SYMBOL(last_backlight);

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
EXPORT_SYMBOL(get_mode_by_id);

int oplus_drm_sethpwm_bl(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, unsigned int level, bool is_sync)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_params *params = mtk_crtc->panel_ext->params;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	int hpwm_dbv = 0;


	oplus_display_brightness = level;
	hpwm_90nit_set_temp = 0;

	if (params) {
		if((!strcmp(params->vendor, "22823_Tianma_NT37705"))
			|| (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
			hpwm_dbv = 0x643;
		} else if (!strcmp(params->vendor, "22047_boe_NT37705")) {
			hpwm_dbv = 0x644;
		}
	} else {
		DDPPR_ERR("failed to get params\n");
		return 0;
	}


	DDPINFO("%s: DSI_SET_BL: hpwm_90nit_set_temp:%d,vact_timing_fps:%d,pwm_turbo:%d,pwm_power_on=%d last_backlight=%d\n", __func__,
		hpwm_90nit_set_temp, mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps, hpwm_mode, pwm_power_on,last_backlight);

/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_backlight_filter(level)) {
			goto end;
		}
	}
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */

	if (hpwm_mode && hpwm_dbv) {
		DDPINFO("DSI_SET_BL: hpwm_90nit_set_temp:%d,vact_timing_fps:%d,pwm_turbo:%d,pwm_power_on=%d\n", hpwm_90nit_set_temp,
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps, hpwm_mode, pwm_power_on);
		if ((((level <= hpwm_dbv) && (last_backlight > hpwm_dbv)) || (pwm_power_on == true && level <= hpwm_dbv))) {
			if ((!strcmp(params->vendor, "22047_boe_NT37705")) || (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
				if (mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 60) {
					DDPINFO("%s DSI_SET_BL sleep 2ms.<0x%x\n", __func__, hpwm_dbv);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(2000), CMDQ_GPR_R06);
				}
			} else {
				DDPINFO("%s DSI_SET_BL sleep 0ms.<0x%x\n", __func__, hpwm_dbv);
			}
			if (comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &level);
			hpwm_90nit_set_temp = 1;
			pwm_power_on = false;
			last_backlight =  level;
		} else if ((((level > hpwm_dbv) && (last_backlight <= hpwm_dbv)) || (pwm_power_on == true && level > hpwm_dbv))) {
			if ((!strcmp(params->vendor, "22047_boe_NT37705")) || (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
				if (mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 60) {
					DDPINFO("%s DSI_SET_BL sleep 2ms.>0x%x\n", __func__, hpwm_dbv);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(2000), CMDQ_GPR_R06);
				}
			} else {
				DDPINFO("%s DSI_SET_BL sleep 0ms.>0x%x\n", __func__, hpwm_dbv);
			}

			if (comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &level);
			hpwm_90nit_set_temp = 2;
			pwm_power_on = false;
			last_backlight =  level;
		}

		if (hpwm_90nit_set_temp == 0) {
			if ((!strcmp(params->vendor, "22047_boe_NT37705")) || (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
				if (mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 60) {
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(2000), CMDQ_GPR_R06);
					DDPINFO("%s DSI_SET_BL sleep 2ms.\n", __func__);
				}
			}
			if (comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
			last_backlight =  level;
		}

		if (hpwm_90nit_set_temp && (is_sync == false)) {
			if (mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 60) {
				cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(9300), CMDQ_GPR_R06);
				DDPINFO("%s cmdq sleep 9.3ms \n", __func__);
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
				if (oplus_temp_compensation_is_supported()) {
					oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_FIRST_HALF_FRAME_SETTING);
				}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
			}
			hpwm_90nit_set_temp = 0;
		}
	} else {
		if ((!strcmp(params->vendor, "22047_boe_NT37705")) || (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
			if (mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 60) {
				DDPINFO("%s DSI_SET_BL sleep 2ms.\n", __func__);
				cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(2000), CMDQ_GPR_R06);
			}
		}
		if (comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
		last_backlight =  level;
	}
end:
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		if ((!strcmp(params->vendor, "22047_boe_NT37705")) || (!strcmp(params->vendor, "22047_Tianma_NT37705"))) {
			if (!((hpwm_90nit_set_temp > 0) && (is_sync == true))) {
				oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
			}
		} else {
			oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

	return 0;
}

int mtk_crtc_set_high_pwm_switch(struct drm_crtc *crtc, unsigned int en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct mtk_dsi *dsi = NULL;
	struct mtk_panel_ext *ext = mtk_crtc->panel_ext;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_crtc_state *state =
	    to_mtk_crtc_state(mtk_crtc->base.state);
	struct drm_display_mode *drm_mode = NULL;
	unsigned int src_mode =
	    state->prop_val[CRTC_PROP_DISP_MODE_IDX];
	unsigned int fps = 0;
	int src_vrefresh = 0;

	if (ext == NULL) {
		DDPINFO("%s %d mtk_crtc->panel_ext is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	DDPPR_ERR("%s, en=%d,hpwm_mode=%d\n", __func__,en,hpwm_mode);

	if (hpwm_mode == en) {
        DDPPR_ERR("hpwm_mode no changer\n");
		return 0;
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (!mtk_crtc->enabled)
		goto done;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		DDPPR_ERR("request output fail\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	dsi = container_of(comp, struct mtk_dsi, ddp_comp);
	if (!dsi) {
		DDPPR_ERR("request dsi fail\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	drm_mode = get_mode_by_id(&dsi->conn, src_mode);
	if (!drm_mode) {
		DDPPR_ERR("invalid drm_display_mode\n");
		return -EINVAL;
	}
	src_vrefresh = drm_mode_vrefresh(drm_mode);

	if ((!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705"))
		|| (!strcmp(mtk_crtc->panel_ext->params->vendor, "22047_Tianma_NT37705"))
		|| (!strcmp(mtk_crtc->panel_ext->params->vendor, "22047_boe_NT37705"))) {
		if (src_vrefresh == 90) {
			hpwm_mode_90hz = 255;
			pr_info("%s, src_mode=%d if 2 goto done, src_vrefresh=%d\n", __func__, src_mode, src_vrefresh);
			goto done;
		} else {
			hpwm_mode_90hz = 1;
		}
	}
	ext->params->f_high_pwm_en = en;

	mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(crtc, &cmdq_handle);
		/** wait one TE (no clear te) **/
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (mtk_drm_lcm_is_connect(mtk_crtc))
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);

		/* because only mode = 3 (60fps), we need changed to 120fps */
		pr_info("%s, src_mode=%d, src_vrefresh=%d\n", __func__, src_mode, src_vrefresh);
		if (src_vrefresh == 60) {
			fps = 120;
			if (!en)
				hpwm_fps_mode = !en;
			if (comp && comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
			/** wait one TE **/
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc))
				cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			/** wait one TE **/
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc))
				cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			hpwm_fps_mode = en;
		}

		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM, &en);

		/** wait one TE **/
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (mtk_drm_lcm_is_connect(mtk_crtc))
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);

		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_ELVSS, &en);

		/* because only mode = 3 (60fps), we need recovery 60fps */
		if (src_vrefresh == 60) {
			fps = 60;
			/** wait one TE **/
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc))
				cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (comp && comp->funcs && comp->funcs->io_cmd)
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
		}

	/* because only mode = 0 (120fps), we need set fps close pwm */
	if (src_vrefresh == 120) {
		fps = 120;
		hpwm_fps_mode = en;
		/** wait one TE **/
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (mtk_drm_lcm_is_connect(mtk_crtc))
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
	}

	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
	if ((!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705"))
		|| (!strcmp(mtk_crtc->panel_ext->params->vendor, "22047_Tianma_NT37705"))
		|| (!strcmp(mtk_crtc->panel_ext->params->vendor, "22047_boe_NT37705"))) {
		if (en == 0) {
			hpwm_mode_90hz = 0;
		}
	}

	done:
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

		return 0;
	}