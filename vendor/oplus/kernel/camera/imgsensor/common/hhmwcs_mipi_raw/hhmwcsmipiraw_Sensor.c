// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hhmwcsmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "hhmwcsmipiraw_Sensor.h"

#define SENSOR_NAME  SENSOR_DRVNAME_HHMWCS_MIPI_RAW
#define PFX "hhmwcs_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define HHMWCS_EEPROM_I2C_ADDR	(0xA2)

#define SENSOR_ID	          (0x002b)
#define UNIQUE_ID_SIZE		  (0X10)

#define HHMWCS_STEREO_START_ADDR    (0)
#define HHMWCS_AESYNC_START_ADDR    (0)
#define TEMP_ERR              (-100)


/*  description of the register
    Bit[7:2]: Not used
    Bit[1]: New setting
    Writing 1 to this bit will update all
    registers next frame
    Bit[0]: Restart
    Writing 1 to this bit will restart a new
    session and update all registers

    example: write 0x2 to trigger the new exposure/gain
*/
#define REG_RESTART_ADDR      0xFE

/*  description of the register
    Page 0: System registers
    Page 1: sensor_ctrl
    Page 3: ISP
    Page 6: OTP
*/
#define REG_PAGE_CTRL_ADDR 0xFD
#define PAGE_SYS_REG 0
#define PAGE_SENSOR_CTRL 1
#define PAGE_ISP 3
#define PAGE_OTP 6

#define REG_ANALOG_GAIN_ADDR 0x22
#define REG_EXPOSURE_ADDR_H 0x0E
#define REG_EXPOSURE_ADDR_L 0x0F

#define REG_VBLANK_ADDR_H 0x14
#define REG_VBLANK_ADDR_L 0x15

struct pmic_auxadc_data {
	unsigned int pullup_v;
	unsigned int pullup_r;
};

static u64 stream_off_time = 0;

static struct subdrv_ctx *g_ctx = NULL;
static kal_uint8 sensor_unique_id[UNIQUE_ID_SIZE] = {0};
static struct iio_channel *temperature_channel = NULL;
static u16 get_gain2reg(u32 gain);
/* static int hhmwcs_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len); */
static int hhmwcs_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* static int hhmwcs_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len); */
static int hhmwcs_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);

static int hhmwcs_set_gain_convert(struct subdrv_ctx *ctx, u32 gain);
static int hhmwcs_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter);
static int hhmwcs_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
static int hhmwcs_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size);
static int hhmwcs_set_register(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_get_register(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts);
static int hhmwcs_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hhmwcs_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);
static int hhmwcs_get_unique_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void hhmwcs_set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror);
/* static void read_unique_sensorid(struct subdrv_ctx *ctx); */

static int hhmwcs_common_control(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);

/* static bool g_id_from_dts_flag = false; */

struct mtk_sensor_ctle_param hhmwcs_static_ctle_param = {
	.eq_bw = 0x3,
};

static struct eeprom_map_info hhmwcs_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x001l, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x1260, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
	{ EEPROM_META_DISTORTION_DATA, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	/* {SENSOR_FEATURE_SET_TEST_PATTERN, hhmwcs_set_test_pattern}, */
	{SENSOR_FEATURE_CHECK_SENSOR_ID, hhmwcs_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, hhmwcs_get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, hhmwcs_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, hhmwcs_get_eeprom_calibration},
	/* {SENSOR_FEATURE_GET_SENSOR_OTP_ALL, hhmwcs_get_otp_checksum_data}, */
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, hhmwcs_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, hhmwcs_streaming_resume},
	{SENSOR_FEATURE_SET_GAIN, hhmwcs_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, hhmwcs_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, hhmwcs_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH, hhmwcs_extend_frame_length},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, hhmwcs_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, hhmwcs_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_SET_REGISTER, hhmwcs_set_register},
	{SENSOR_FEATURE_GET_REGISTER, hhmwcs_get_register},
	{SENSOR_FEATURE_SET_FRAMELENGTH, hhmwcs_set_frame_length},
	{SENSOR_FEATURE_GET_SENSOR_UNIQUE_ID, hhmwcs_get_unique_id},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x0065009a,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA2,
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 800,
			.vsize = 600,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 800,
			.vsize = 600,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 800,
			.vsize = 600,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 800,
			.vsize = 600,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	}
};

static struct subdrv_mode_struct mode_struct[] = {
	{/* 800*600 @30FPS  */
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = hhmwcs_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(hhmwcs_preview_capture_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1227,
		.max_framerate = 300,
		.mipi_pixel_rate = 33000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 800,
			.scale_h = 600,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 800,
			.h1_size = 600,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 800,
			.h2_tg_size = 600,
		},
		.ae_binning_ratio = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 79,
		},
	},
	{/* 800*600 @10FPS multispectral */
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = hhmwcs_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(hhmwcs_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 3683,
		.max_framerate = 100,
		.mipi_pixel_rate = 33000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 800,
			.scale_h = 600,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 800,
			.h1_size = 600,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 800,
			.h2_tg_size = 600,
		},
		.ae_binning_ratio = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 80,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MULTISPECTRAL,
	},
	{/* 800*600 @10FPS */
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = hhmwcs_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(hhmwcs_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 3683,
		.max_framerate = 100,
		.mipi_pixel_rate = 33000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 800,
			.scale_h = 600,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 800,
			.h1_size = 600,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 800,
			.h2_tg_size = 600,
		},
		.ae_binning_ratio = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 78,
		},
	},
	{ /* 800*600 @60FPS  */
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = hhmwcs_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(hhmwcs_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 611,
		.max_framerate = 600,
		.mipi_pixel_rate = 33000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 800,
			.scale_h = 600,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 800,
			.h1_size = 600,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 800,
			.h2_tg_size = 600,
		},
		.ae_binning_ratio = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 80,
		},
	},
	{/* 1600*1200 @30FPS  */
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = hhmwcs_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(hhmwcs_slim_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.max_framerate = 300,
		.mipi_pixel_rate = 66000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 1600,
			.scale_h = 1200,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1600,
			.h1_size = 1200,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1600,
			.h2_tg_size = 1200,
		},
		.ae_binning_ratio = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 88,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = HHMWCS_SENSOR_ID,
	.reg_addr_sensor_id = {0x02, 0x03}, /*page0, 0x00, 0x01, 0x02, 0x03*/
	.i2c_addr_table = {0x7a, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_8_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.mirror = IMAGE_HV_MIRROR,

	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	.ana_gain_def = BASEGAIN * 1,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = 15872, /*BASEGAIN * 15.5*/
	.ana_gain_type = 1, /*0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC*/
	.ana_gain_step = 1,
	.ana_gain_table = hhmwcs_ana_gain_table,
	.ana_gain_table_size = sizeof(hhmwcs_ana_gain_table),
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max =  0x6fff,  /* frame length can auto extend*/
	.exposure_step = 1,
	.exposure_margin = 7,
	.temperature_support = TRUE,

	.frame_length_max = 0x7fff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 934000,
	.pdaf_type = PDAF_SUPPORT_NA,
	.g_gain2reg = get_gain2reg,
	.g_temp = get_sensor_temperature,

	.reg_addr_stream = 0xC2,
	.long_exposure_support = FALSE,
	.reg_addr_frame_count = PARAM_UNDEFINED,

	.init_setting_table = hhmwcs_init_setting,
	.init_setting_len = ARRAY_SIZE(hhmwcs_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xD1EFF68B,
	.ctle_param = &hhmwcs_static_ctle_param,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = hhmwcs_common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, {24}, 0},
	{HW_ID_RST, {0}, 0},
	{HW_ID_DOVDD, {1800000, 1800000}, 1000},
	{HW_ID_MCLK_DRIVING_CURRENT, {4}, 1000},
	{HW_ID_AVDD, {2804000, 2804000}, 1000},
	{HW_ID_AVDD1, {2800000, 2800000}, 6000},
	{HW_ID_RST, {1}, 10000},
};

struct subdrv_entry hhmwcs_mipi_raw_entry = {
	.name = "hhmwcs_mipi_raw",
	.id = HHMWCS_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

static struct pmic_auxadc_data adc_data = {
	.pullup_v = 184000,
	.pullup_r = 100000,
};
/* FUNCTION */

static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable)
{
	check_current_scenario_id_bound(ctx);
	if (!(enable ^ ctx->is_streaming)) {
		DRV_LOGE(ctx, "enable %d, ctx->is_streaming %d", enable, ctx->is_streaming);
		return;
	}

	if (enable) {
		if (stream_off_time != 0) {
			int delay_time = 101 - (ktime_get_boottime_ns() - stream_off_time) / 1000000;
			DRV_LOGE(ctx, "check stream off delay_time(%dms)", delay_time);
			if (delay_time > 0) {
				mdelay(delay_time);
			}
		}
		subdrv_i2c_wr_u8_u8(ctx, 0xfb, 0x01);
	}

	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_ISP);
	subdrv_i2c_wr_u8_u8(ctx, ctx->s_ctx.reg_addr_stream, enable);
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);
	if (!enable) {
		stream_off_time = ktime_get_boottime_ns();
	}

	ctx->is_streaming = enable;
	DRV_LOGE(ctx, "X! enable:%u\n", enable);
}

static int hhmwcs_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			hhmwcs_set_shutter_convert(ctx, *(u32 *)para);
		streaming_ctrl(ctx, true);

		return 0;
}

static int hhmwcs_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);

		return 0;
}
static unsigned int read_hhmwcs_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != hhmwcs_eeprom_info[meta_id].meta)
		return -1;

	if (size != hhmwcs_eeprom_info[meta_id].size)
		return -1;

	addr = hhmwcs_eeprom_info[meta_id].start;
	readsize = hhmwcs_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA3,
	.i2c_write_id = 0xA2,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x0010,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x0098,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C7,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int hhmwcs_get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct oplus_eeprom_info_struct* infoPtr;
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);
	infoPtr->afInfo[0] = (kal_uint8)((infoPtr->afInfo[1] << 7) | (infoPtr->afInfo[0] >> 1));
	infoPtr->afInfo[1] = (kal_uint8)(infoPtr->afInfo[1] >> 1);
	infoPtr->afInfo[2] = (kal_uint8)((infoPtr->afInfo[3] << 7) | (infoPtr->afInfo[2] >> 1));
	infoPtr->afInfo[3] = (kal_uint8)(infoPtr->afInfo[3] >> 1);
	infoPtr->afInfo[4] = (kal_uint8)((infoPtr->afInfo[5] << 7) | (infoPtr->afInfo[4] >> 1));
	infoPtr->afInfo[5] = (kal_uint8)(infoPtr->afInfo[5] >> 1);

	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, HHMWCS_EEPROM_I2C_ADDR >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
		kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
	ret = adaptor_i2c_wr_p8(ctx->i2c_client, HHMWCS_EEPROM_I2C_ADDR >> 1,
			addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xE000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, HHMWCS_EEPROM_I2C_ADDR >> 1, reg, 0x03);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, HHMWCS_EEPROM_I2C_ADDR >> 1, reg, 0x02);
	}

	return ret;
}

static kal_uint16 get_64align_addr(kal_uint16 data_base) {
	kal_uint16 multiple = 0;
	kal_uint16 surplus = 0;
	kal_uint16 addr_64align = 0;

	multiple = data_base / 64;
	surplus = data_base % 64;
	if(surplus) {
		addr_64align = (multiple + 1) * 64;
	} else {
		addr_64align = multiple * 64;
	}
	return addr_64align;
}

static kal_int32 eeprom_table_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {
	kal_uint16 idx;
	kal_uint16 idy;
	kal_int32 ret = ERROR_NONE;
	UINT32 i = 0;

	idx = data_length/WRITE_DATA_MAX_LENGTH;
	idy = data_length%WRITE_DATA_MAX_LENGTH;

	LOG_INF("[test] data_base(0x%x) data_length(%d) idx(%d) idy(%d)\n", data_base, data_length, idx, idy);

	for (i = 0; i < idx; i++) {
		ret = table_write_eeprom_30Bytes(ctx, (data_base + WRITE_DATA_MAX_LENGTH * i),
				&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: i=%d\n", i);
			return -1;
		}
		msleep(6);
	}

	msleep(6);
	if(idy) {
		ret = table_write_eeprom_30Bytes(ctx, (data_base + WRITE_DATA_MAX_LENGTH*idx),
				&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
			return -1;
		}
	}
	return 0;
}

static kal_int32 eeprom_64align_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {
	kal_uint16 addr_64align = 0;
	kal_uint16 part1_length = 0;
	kal_uint16 part2_length = 0;
	kal_int32 ret = ERROR_NONE;

	addr_64align = get_64align_addr(data_base);

	part1_length = addr_64align - data_base;
	if(part1_length > data_length) {
		part1_length = data_length;
	}
	part2_length = data_length - part1_length;

	write_eeprom_protect(ctx, 0);
	msleep(6);

	if (part1_length) {
		ret = eeprom_table_write(ctx, data_base, pData, part1_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part1\n");
			msleep(6);
			return -1;
		}
	}

	msleep(6);
	if (part2_length) {
		ret = eeprom_table_write(ctx, addr_64align, pData + part1_length, part2_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part2\n");
			msleep(6);
			return -1;
		}
	}
	msleep(6);
	write_eeprom_protect(ctx, 1);
	msleep(6);

	return 0;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
			ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint8 *pData;

	if(pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
					   pStereodata->uSensorId,
					   pStereodata->uDeviceId,
					   pStereodata->baseAddr,
					   pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == HHMWCS_SENSOR_ID)
			&& (data_length == CALI_DATA_SLAVE_LENGTH)
			&& (data_base == HHMWCS_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

			eeprom_64align_write(ctx, data_base, pData, data_length);

			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
			LOG_INF("write_Module_data Write end\n");

		} else if ((pStereodata->uSensorId == HHMWCS_SENSOR_ID)
			&& (data_length < AESYNC_DATA_LENGTH_TOTAL)
			&& (data_base == HHMWCS_AESYNC_START_ADDR)) {
			LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

			eeprom_64align_write(ctx, data_base, pData, data_length);

			LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+1),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+2),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+3),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+4),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+5),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+6),
					read_cmos_eeprom_8(ctx, HHMWCS_AESYNC_START_ADDR+7));
			LOG_INF("AESync write_Module_data Write end\n");
		} else {
			LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return -1;
		}
	} else {
		LOG_INF("omegas2 write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

static int hhmwcs_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int ret = ERROR_NONE;
	ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
	if (ret != ERROR_NONE) {
		*len = (u32)-1; /*write eeprom failed*/
		LOG_INF("ret=%d\n", ret);
	}
	return 0;
}

static int hhmwcs_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_SLAVE_LENGTH)
		*len = CALI_DATA_SLAVE_LENGTH;
	LOG_INF("feature_data mode:%d  lens:%d", *feature_data_16, *len);
	read_hhmwcs_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
			(BYTE *)feature_return_para_32, *len);
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
					BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, HHMWCS_EEPROM_I2C_ADDR >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int hhmwcs_get_unique_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT8 *feature_data_8 = (UINT8 *) para;
	DRV_LOG(ctx, "get unique id\n");

	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_OTP);

	if (sensor_unique_id[0] == 0) {
		for (int i = 0; i < *len; i++) {
			sensor_unique_id[i] = subdrv_i2c_rd_u8_u8(ctx, i);
			DRV_LOG(ctx, "sensor_unique_id[%d] = 0x%2x\n", i, sensor_unique_id[i]);
		}
	} else {
		DRV_LOG(ctx, "unique id has already read\n");
	}

	memcpy(feature_data_8, (UINT8 *)sensor_unique_id, sizeof(sensor_unique_id));
	*len = sizeof(sensor_unique_id);
	return 0;
}

static int hhmwcs_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SYS_REG);

	return (subdrv_i2c_rd_u8_u8(ctx, 0x02) << 8)  |  subdrv_i2c_rd_u8_u8(ctx, 0x03);
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = KAL_TRUE;

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = return_sensor_id(ctx);
			DRV_LOGE(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == SENSOR_ID) {
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOGE(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOG(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	u64 time_boot_begin = 0;
	stream_off_time = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	/*subdrv_i2c_wr_regs_u8_u8(ctx, hhmwcs_soft_reset, ARRAY_SIZE(hhmwcs_soft_reset));
	mdelay(3);*/
	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en))
		time_boot_begin = ktime_get_boottime_ns();
	DRV_LOGE(ctx, "hhmwcs is openning\n");
	subdrv_i2c_wr_regs_u8_u8(ctx, hhmwcs_init_setting, ARRAY_SIZE(hhmwcs_init_setting));
	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en)) {
		ctx->sensor_pw_on_profile.i2c_init_period = ktime_get_boottime_ns() - time_boot_begin;
		ctx->sensor_pw_on_profile.i2c_init_table_len = ARRAY_SIZE(hhmwcs_init_setting);
	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}

static u16 get_gain2reg(u32 gain)
{
	return  0x10 * gain/BASEGAIN;
}

/* base_vts means min framelength */
static int base_vts[] = {
	610,  /*preview*/
	610,  /*capture*/
	610,  /*normal video*/
	610,  /*his video*/
	1220, /*slim video*/
};

void hhmwcs_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
{
	u32 fll_step = 0;
	u32 vblank = 0;
	u32 diff_frame_length;
	check_current_scenario_id_bound(ctx);

	fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;

	diff_frame_length = base_vts[ctx->current_scenario_id] ?
			base_vts[ctx->current_scenario_id] : base_vts[0];

	switch (ctx->current_scenario_id) {
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		vblank = fll - diff_frame_length + 1;
		break;
	default:
		vblank = (fll - diff_frame_length + 1) * 2 - 1;
	}

	ctx->frame_length = fll;

	if (fll_step)
		fll = round_up(fll, fll_step);

	if(vblank < ctx->s_ctx.frame_length_max - base_vts[ctx->current_scenario_id]) {
		subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);
		subdrv_i2c_wr_u8_u8(ctx, REG_VBLANK_ADDR_H, (vblank >> 8) & 0xFF);
		subdrv_i2c_wr_u8_u8(ctx, REG_VBLANK_ADDR_L,  vblank & 0xFF);
	}

	DRV_LOG(ctx, "ctx->frame_length(%d), vblank(%d)\n", ctx->frame_length, vblank);
}

void hhmwcs_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u set default\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = 0;
	}
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_NONE:
			if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
				ctx->s_ctx.mode[scenario_id].min_exposure_line) {
				*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
				*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
			} else {
				*exposure_step = ctx->s_ctx.exposure_step;
				*min_shutter = ctx->s_ctx.exposure_min;
			}
			break;
		default:
			*exposure_step = ctx->s_ctx.exposure_step;
			*min_shutter = ctx->s_ctx.exposure_min;
			break;
		}
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	g_ctx = ctx;

	return 0;
}

static int hhmwcs_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16 frame_length = (u16) (*para);
	if (frame_length) {
		ctx->frame_length = frame_length;
	}
	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	hhmwcs_write_frame_length(ctx, ctx->frame_length);

	DRV_LOG(ctx, "fll(input/output/min):%u/%u/%u\n",
		frame_length, ctx->frame_length, ctx->min_frame_length);
	return 0;
}

static int hhmwcs_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	u32 fine_integ_line = 0;
	u32 cit_step = 0;

	u8 exposure_margin = ctx->s_ctx.exposure_margin;
	DRV_LOG(ctx, "shutter:%llu, frame_length:%u  exposure_margin:%d\n", shutter, frame_length, exposure_margin);

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, (u64)ctx->s_ctx.exposure_min);
	shutter = min(shutter, (u64)ctx->s_ctx.exposure_max);
	/* check boundary of framelength */

	cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
	if (cit_step)
		shutter = round_up(shutter, cit_step);
	ctx->frame_length =	max(shutter + exposure_margin, (u64)ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;

	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		hhmwcs_write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);
	subdrv_i2c_wr_u8_u8(ctx, REG_EXPOSURE_ADDR_H, (shutter >>  8) & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, REG_EXPOSURE_ADDR_L,  shutter & 0xFF);
	subdrv_i2c_wr_u8_u8(ctx, REG_RESTART_ADDR,  0x2);

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);

	return 0;
}

static int hhmwcs_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return hhmwcs_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], ((u64*)para)[1]);
}

static int hhmwcs_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter)
{
	return hhmwcs_set_shutter_frame_length_convert(ctx, shutter, 0);
}

static int hhmwcs_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return hhmwcs_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], 0);
}

static int hhmwcs_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64*)para;
	u64 *shutters =(u64 *)(*feature_data);
	u16 exp_cnt = (u64) (*(feature_data + 1));
	u64 framelength = (u64) (*(feature_data + 2));

	if(exp_cnt != 1) {
		LOG_INF("exp_cnt(%d) != 1\n", exp_cnt);
	}

	return hhmwcs_set_shutter_frame_length_convert(ctx, shutters[0], framelength);
}

static int hhmwcs_set_gain_convert(struct subdrv_ctx *ctx, u32 gain)
{
	u16 rg_gain;
	u32 ana_gain_min = ctx->ana_gain_min;
	u32 ana_gain_max = ctx->ana_gain_max;

	/* check boundary of gain */
	gain = max(gain, ana_gain_min);
	gain = min(gain, ana_gain_max);

	/* mapping of gain to register value */
	rg_gain = get_gain2reg(gain);

	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;

	/* write gain */
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);
	subdrv_i2c_wr_u8_u8(ctx, REG_ANALOG_GAIN_ADDR, rg_gain);
	subdrv_i2c_wr_u8_u8(ctx, REG_RESTART_ADDR, 0x02); /*fresh*/

	DRV_LOG(ctx, "%s gain(%d) rg_gain[0x%x]\n", __func__, gain, rg_gain);

	return 0;
}

static int hhmwcs_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;

	return hhmwcs_set_gain_convert(ctx, gain);
}

void hhmwcs_set_dummy(struct subdrv_ctx *ctx)
{
}

static int hhmwcs_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (u32)((u64*)para)[0];
	u32 framerate = (u32)((u64*)para)[1];
	u32 frame_length;
	u32 frame_length_step;

	LOG_INF("scenario_id(%d), framerate(%d)", scenario_id, framerate);

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate (%u) is invalid\n", framerate);
		return 0;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength (%u) is invalid\n",
			ctx->s_ctx.mode[scenario_id].linelength);
		return 0;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	frame_length_step = ctx->s_ctx.mode[scenario_id].framelength_step;
	frame_length = frame_length_step ?
		(frame_length - (frame_length % frame_length_step)) : frame_length;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->s_ctx.reg_addr_auto_extend ||
			(ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))) {
		hhmwcs_set_dummy(ctx);
	}

	return 0;
}

static int hhmwcs_extend_frame_length_convert(struct subdrv_ctx *ctx, u32 ns)
{
	return 0;
}

static int hhmwcs_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 ns = (u32)((u64*)para)[0];

	return hhmwcs_extend_frame_length_convert(ctx, ns);
}
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	ctx->sof_cnt = sof_cnt;

	return 0;
}

static int hhmwcs_set_register(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u8 page = ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr & 0xFF00;
	u8 addr = ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr & 0xFF;

	return 0;
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, page);
	subdrv_i2c_wr_u8_u8(ctx, addr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData & 0xFF);

	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);	/*page1*/
	subdrv_i2c_wr_u8_u8(ctx, REG_RESTART_ADDR, 0x02);	/*fresh*/

	pr_err("%s RegAddr: 0x%08x, RegData: 0x%04x \n",
		__func__, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);

	return 0;
}

static int hhmwcs_get_register(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u8 page = ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr & 0xFF00;
	u8 addr = ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr & 0xFF;

	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, page);
	((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData =
		subdrv_i2c_rd_u8_u8(ctx, addr);
	pr_err("%s RegAddr: 0x%08x, RegData: 0x%04x \n",
		__func__, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);

	return 0;
}

static int hhmwcs_common_control(struct subdrv_ctx *ctx,
			enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int ret = ERROR_NONE;
	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u64 time_boot_begin = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	struct adaptor_ctx *_adaptor_ctx = NULL;
	struct v4l2_subdev *sd = NULL;

	if (ctx->i2c_client)
		sd = i2c_get_clientdata(ctx->i2c_client);
	if (sd)
		_adaptor_ctx = to_ctx(sd);
	if (!_adaptor_ctx) {
		DRV_LOGE(ctx, "null _adaptor_ctx\n");
		return -ENODEV;
	}

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
		ret = ERROR_INVALID_SCENARIO_ID;
	}
	update_mode_info(ctx, scenario_id);

	if (ctx->s_ctx.mode[scenario_id].mode_setting_table != NULL) {
		DRV_LOG_MUST(ctx, "E: sid:%u size:%u\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len);
		if (ctx->power_on_profile_en)
			time_boot_begin = ktime_get_boottime_ns();

		/* initail setting */
		/*subdrv_i2c_wr_regs_u8_u8(ctx, hhmwcs_soft_reset, ARRAY_SIZE(hhmwcs_soft_reset));
		mdelay(3);*/
		DRV_LOGE(ctx, "i2c_write_id: %x \n", ctx->i2c_write_id);
		i2c_table_rewrite(ctx, ctx->s_ctx.mode[scenario_id].mode_setting_table,
				ctx->s_ctx.mode[scenario_id].mode_setting_len);

		if (ctx->power_on_profile_en) {
			ctx->sensor_pw_on_profile.i2c_cfg_period =
					ktime_get_boottime_ns() - time_boot_begin;

			ctx->sensor_pw_on_profile.i2c_cfg_table_len =
					ctx->s_ctx.mode[scenario_id].mode_setting_len;
		}
		DRV_LOG(ctx, "X: sid:%u size:%u\n", scenario_id,
			ctx->s_ctx.mode[scenario_id].mode_setting_len);
	} else {
		DRV_LOGE(ctx, "please implement mode setting(sid:%u)!\n", scenario_id);
	}

	if (check_is_no_crop(ctx, scenario_id) && probe_eeprom(ctx)) {
		idx = ctx->eeprom_index;
		support = info[idx].xtalk_support;
		pbuf = info[idx].preload_xtalk_table;
		size = info[idx].xtalk_size;
		addr = info[idx].sensor_reg_addr_xtalk;
		if (support) {
			if (pbuf != NULL && addr > 0 && size > 0) {
				subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
				DRV_LOG(ctx, "set XTALK calibration data done.");
			}
		}
	}
	hhmwcs_set_mirror_flip(ctx, ctx->s_ctx.mirror);

	return ret;
}

static unsigned int calculate_r_ntc(unsigned long long v_in, unsigned int pullup_r, unsigned int pullup_v)
{
	unsigned int r_ntc;
	if (v_in > pullup_v)
		return 0;
	r_ntc = (unsigned int)div_u64(v_in * pullup_r, (pullup_v - v_in));
	return r_ntc;
}

static int ntc_r_to_temp(struct subdrv_ctx *ctx, unsigned int val)
{
	int temp, temp_hi, temp_lo, r_hi, r_lo, temp_frac;
	int i;
	int temp_table_number = ARRAY_SIZE(temperature_lookup_table) / 2;

	for (i = 0; i < temp_table_number; i++) {
		if (val >= temperature_lookup_table[2 * i + 1])
			break;
	}

	if (i == 0) {
		temp = temperature_lookup_table[0];
	} else if (i >= temp_table_number) {
		temp = temperature_lookup_table[2 *
			(temp_table_number - 1)];
	} else {
		r_hi = temperature_lookup_table[2 * i - 1];
		r_lo = temperature_lookup_table[2 * i + 1];

		temp_hi = temperature_lookup_table[2 * i - 2];
		temp_lo = temperature_lookup_table[2 * i];

		temp_frac = (temp_lo - temp_hi) * (val - r_hi);
		temp_frac /= r_lo - r_hi;
		temp = temp_hi + temp_frac;

		DRV_LOG(ctx, "temp %d, temp_frac %d, val %d, r_hi %d, r_lo %d, temp_hi %d, temp_lo %d", temp, temp_frac, val, r_hi, r_lo, temp_hi, temp_lo);
	}
	return temp;
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int temperature_convert = 0;
	int adc_raw;
	int ret;
	unsigned long long v_in;
	unsigned int r_ntc;
	if (NULL == temperature_channel) {
		temperature_channel = iio_channel_get(&ctx->i2c_client->dev, "temperature-channel");
		if (NULL == temperature_channel || IS_ERR(temperature_channel)) {
			DRV_LOGE(ctx, "temperature channel get err!\n");
			return TEMP_ERR;
		}
	}
	ret = iio_read_channel_raw(temperature_channel, &adc_raw);
	if (ret < 0) {
		DRV_LOGE(ctx, "temperature channel read err!\n");
		return TEMP_ERR;
	}
	/* get volt */
	v_in = ((unsigned long long)adc_raw * 184000) >> 15;
	DRV_LOG(ctx, "adc_raw: 0x%x, current volt: %llu.\n", adc_raw, v_in);
	/* ger r */
	r_ntc = calculate_r_ntc(v_in, adc_data.pullup_r, adc_data.pullup_v);

	/* get tempreture */
	temperature_convert = ntc_r_to_temp(ctx, r_ntc);

	DRV_LOG(ctx, "r_ntc %d, temperature_convert %d", r_ntc, temperature_convert);

	return temperature_convert;
}

static void hhmwcs_set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
	subdrv_i2c_wr_u8_u8(ctx, REG_PAGE_CTRL_ADDR, PAGE_SENSOR_CTRL);
	switch (image_mirror) {
	case IMAGE_NORMAL:
		subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x00);
	break;
	case IMAGE_V_MIRROR:
		subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x01);
	break;
	case IMAGE_H_MIRROR:
		subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x02);
	break;
	case IMAGE_HV_MIRROR:
		subdrv_i2c_wr_u8_u8(ctx, 0x12, 0x03);
	break;
	}
}
