// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 cjmainmipiraw_Sensor.c
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
#include "cjmainmipiraw_Sensor.h"

#define CJMAIN_EEPROM_READ_ID	0xA0
#define CJMAIN_EEPROM_WRITE_ID	0xA1
#define CJMAIN_EEPROM_MOD_INFO 0x0000
#define CJMAIN_EEPROM_SENSOR_ID 0x0006
#define CJMAIN_EEPROM_LENS_ID 0x0008
#define CJMAIN_EEPROM_VCM_ID 0x000A
#define CJMAIN_EEPROM_MOD_INFO_FLAG 0x0010
#define CJMAIN_EEPROM_MOD_AF 0x0092
#define CJMAIN_EEPROM_MOD_AF_MACRO 0x0092
#define CJMAIN_EEPROM_MOD_AF_INF 0x0094
#define CJMAIN_EEPROM_MOD_AF_FLAG 0x0098
#define CJMAIN_EEPROM_MOD_QR_CODE 0x00B0
#define CJMAIN_EEPROM_MOD_QR_CODE_FLAG 0x00C7
#define CJMAIN_MAX_OFFSET		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "cjmain_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x8000
#define OTP_QCOM_PDAF_DATA_LENGTH 0xA62
#define OTP_QCOM_PDAF_OFFSET_DATA_LENGTH 0x4C8
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x0600
#define OTP_QCOM_PDAF_OFFSET_DATA_START_ADDR 0x1100
#define GET_SENSOR_ID_RETRY_CNT    5

#define QSC_EN_ADDR 0x3206
/* IMAGE_HV_MIRROR */
#define MIRROR_FLIP 0x3

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static kal_uint8 otp_qcom_pdaf_offset_data[OTP_QCOM_PDAF_OFFSET_DATA_LENGTH] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int cjmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts);
static int cjmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void get_sensor_cali(void *arg);
/*static void calculate_prsh_length_lines(struct subdrv_ctx *ctx,
	struct mtk_hdr_ae *ae_ctrl,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id);*/
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int cjmain_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int cjmain_get_otp_qcom_pdaf_offset_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_get_linetime_in_ns(void *arg,
	u32 scenario_id, u32 *linetime_in_ns, enum GET_LINETIME_ENUM linetime_type,
	enum IMGSENSOR_EXPOSURE exp_idx);
static void cjmain_set_dcg_vs_multi_gain_in_lut(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt);
static void cjmain_set_hdr_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt);
static int cjmain_set_dual_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_set_hdr_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjmain_disable_fast_mode(struct subdrv_ctx *ctx);
/* STRUCT */

struct mtk_sensor_ctle_param cjmain_static_ctle_param = {
	.cdr_delay = 0x9,
};

static struct eeprom_map_info cjmain_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x009A, 0x009B, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0010, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x6100, 0x6799, 0x679A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x6800, 0x6E99, 0x6E9A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0x6F00, 0x0000, 0x0000, CALI_DATA_MASTER_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, cjmain_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, cjmain_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, cjmain_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, cjmain_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, cjmain_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, cjmain_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, cjmain_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_AWB_GAIN, cjmain_set_awb_gain},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, cjmain_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_OFFSET_DATA, cjmain_get_otp_qcom_pdaf_offset_data},
	{SENSOR_FEATURE_SET_DUAL_GAIN, cjmain_set_dual_gain},
	{SENSOR_FEATURE_SET_HDR_TRI_GAIN, cjmain_set_hdr_tri_gain},
	{SENSOR_FEATURE_SET_VIDEO_MODE, cjmain_set_video_mode},
};

static u32 cjmain_dcg_ratio_table_ratio4[] = {4000};
static u32 cjmain_dcg_ratio_table_ratio16[] = {16000};
static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 4000,
	.OB_pedestal = 64,
	.saturation_level = 3900,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_14bit = {
	.OB_pedestal = 64,
	.adc_bit = 10,
	.ob_bm = 64,
};

/*
static struct mtk_sensor_saturation_info imgsensor_saturation_info_for_vs = {
	.OB_pedestal = 64,
	.adc_bit = 10,
	.ob_bm = 64,
	.bit_depth = 14,
	.valid_bit = 10,
	.dummy_padding = LSB_PADDING,
};
*/

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01AF0144,/* cal_layout_table */
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.qsc_support = TRUE,
		.qsc_size = 0x0C00,
		.addr_qsc = 0x4300,
		.sensor_reg_addr_qsc = 0xC000, /*QSC_GAIN_TABLE_R_0*/

		.pdc_support = TRUE,
		.pdc_size = 0x180,
		.addr_pdc = 0x5000,
		.sensor_reg_addr_pdc = 0xD200, /* SPC_GAIN_TABLE_0_0 */
	},
};

#define pd_i4Crop { \
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */ \
		{0, 0}, {0, 0}, {0, 256}, {0, 384}, {0, 384}, \
		/* <cust1> <cust2> <cust3> <cust4> <cust5> */ \
		{0, 192}, {0, 0}, {0, 0}, {0, 0}, {2048, 1536}, \
		/* <cust6> <cust7> <cust8> <cust9> <cust10>*/ \
		{0, 192}, {2048, 1536}, {1472, 1104}, {0, 256}, {0, 384}, \
		/* <cust11> <cust12> <cust13> <cust14> <cust15> */ \
		{992, 744}, {0, 0}, {0, 0}, {992, 744}, {0, 384}, \
		/* <cust16> <cust17> <cust18> <cust19> <cust20>*/ \
		{0, 384}, {0, 0}, {2048, 1536}, {0, 0}, {2048, 1920}, \
		/* <cust20> <cust21> <cust22> <cust23> <cust24>*/ \
		{128, 456}, \
}/*(i4FullRawW - output_size_W) / 2, (i4FullRawH - output_size_H) / 2*/

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = MIRROR_FLIP,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,/* all-pd */
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, /* R=1, L=0 */
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_v2h2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = MIRROR_FLIP,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,/* all-pd */
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, /* R=1, L=0 */
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = MIRROR_FLIP,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,/* all-pd */
		.i4BinFacX = 4,
		.i4BinFacY = 2,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, /* R=1, L=0 */
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cus9 = {
	.i4OffsetX = 16,
	.i4OffsetY = 32,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{16, 35}, {20, 37}, {19, 42}, {23, 44} },
	.i4PosR = {{18, 33}, {22, 39}, {17, 40}, {21, 46}},
	.i4BlockNumX = 508,
	.i4BlockNumY = 128,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,/* all-pd */
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, /* R=1, L=0 */
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cus10 = {
	.i4OffsetX = 16,
	.i4OffsetY = 32,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{16, 35}, {20, 37}, {19, 42}, {23, 44} },
	.i4PosR = {{18, 33}, {22, 39}, {17, 40}, {21, 46}},
	.i4BlockNumX = 508,
	.i4BlockNumY = 144,
	.i4LeFirst = 0,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,/* all-pd */
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, /* R=1, L=0 */
	},
};


/*
static struct SET_PD_BLOCK_INFO_T imgsensor_partial_pd_info = {
	.i4OffsetX = 16,
	.i4OffsetY = 32,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{16, 35}, {20, 37}, {19, 42}, {23, 44}},
	.i4PosR = {{18, 33}, {22, 39}, {17, 40}, {21, 46}},
	.i4BlockNumX = 496,
	.i4BlockNumY = 144,
	.i4Crop = pd_i4Crop,
	.iMirrorFlip = MIRROR_FLIP,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 0,
	.i4VCPackNum = 1,
	.PDAF_Support = PDAF_SUPPORT_CAMSV,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 3,
		.i4PDRepetition = 4,
		.i4PDOrder = {1, 0, 0, 1},
	},
};
*/

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0A00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0280,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2c,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 5,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_SE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
			.force_remap_user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
			.force_remap_user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 1152,
			.vsize = 864,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 1152,
			.vsize = 216,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0A00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0A00,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0280,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	/*HCG *all-pd*/
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
	/*LCG partial-pd*/
	/*{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x30,
			.hsize = 508,
			.vsize = 1152,
			.user_data_desc = VC_PDAF_STATS_ME_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},*/
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2112,
			.vsize = 1584,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_ONLY_ONE,
		},
	},
	/*{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2112,
			.vsize = 396,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},*/
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus14[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2112,
			.vsize = 1584,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2112,
			.vsize = 396,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus15[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus16[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus17[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus18[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 1536,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
			.force_remap_user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus19[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 1152,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus20[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2560,
			.user_data_desc = VC_STAGGER_ME,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 1280,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus21[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 3840,
			.vsize = 2160,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 3840,
			.vsize = 540,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};


#define REG2GAIN_ROUNDUP(_reg) ((16384 * BASEGAIN + (16384 - (_reg) - 1))/ (16384 - (_reg)))
#define REG2GAIN_ROUNDDOWN(_reg) (16384 * BASEGAIN / (16384 - (_reg)))

static struct subdrv_mode_struct mode_struct[] = {
	{/* reg B1-S1 4096x3072 @30FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = cjmain_preview_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_preview_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_preview),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12900,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{/* B3-S1 4096x3072 @60FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = cjmain_capture_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 6448,
		.max_framerate = 600,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{/*reg_B1-S3 4096x2560 @30FPS QBIN(VBIN) NOT VB_MAX*/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = cjmain_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_normal_video_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjmain_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_normal_video),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12696,
		.max_framerate = 300,
		.mipi_pixel_rate = 2502860000,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 512,
			.w0_size = 8192,
			.h0_size = 5120,
			.scale_w = 4096,
			.scale_h = 2560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2560,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_B6 4096x2304 @30FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = cjmain_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12900,
		.max_framerate = 300,
		.mipi_pixel_rate = 2530971428,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_G4 4096x2304 @60FPS QBIN(VBIN) DCG-HDR 1:4 VB_MAX*/
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = cjmain_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 12304,
		.framelength = 3960,
		.max_framerate = 600,
		.mipi_pixel_rate = 1586290000,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(13520),/*15.15dB(13520)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1778,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_B,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_HCG_BASE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_V1 2408x1152 @240FPS QBIN(VBIN)-V2H2 VB_MAX*/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = cjmain_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 6848,
		.framelength = 1752,
		.max_framerate = 2403,
		.mipi_pixel_rate = 1592914285,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1474,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {12},
	},
	{/* Reg_B-2 QBIN(VBIN)_4096x3072 24FPS */
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = cjmain_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom2_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_custom2_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_custom2_setting),
		.hdr_mode = HDR_RAW_DCG_RAW_VS,
		.raw_cnt = 3,
		.exp_cnt = 3,
		.pclk = 2928000000,
		.linelength = 14560,
		.framelength = 3316 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 3136,
		.fine_integ_line = 1495,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 3,
		.saturation_info = &imgsensor_saturation_info_14bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio16,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio16),
		},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_SE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_SE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFC-0x64),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFC-0x64),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_SE].max = (0xFFFC-0x64),
		.multiexp_s_info[IMGSENSOR_EXPOSURE_SE].fine_integ_line = 1375,
		.multiexp_s_info[IMGSENSOR_EXPOSURE_SE].belong_to_lut_id = IMGSENSOR_LUT_B,
		.mode_lut_s_info[IMGSENSOR_LUT_B].linelength = 7520,
		.mode_lut_s_info[IMGSENSOR_LUT_B].framelength = 6480,
		.mode_lut_s_info[IMGSENSOR_LUT_A].read_margin = 179,
		.mode_lut_s_info[IMGSENSOR_LUT_B].read_margin = 360,
		.mode_lut_s_info[IMGSENSOR_LUT_A].cit_loss = 1516,
		.mode_lut_s_info[IMGSENSOR_LUT_B].cit_loss = 0,
		.prohibited_exp = {8},
	},
	{/*reg_F1-S1 8192x6144 @30FPS Full(BAYER) All-PD VB_MAX*/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = cjmain_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 11632,
		.framelength = 8332,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{	/*reg_L1-S1 4096x3072 @30FPS QBIN(VBIN) LBMF_Manual VB_MAX, LB-MF LUTA*/
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = cjmain_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom4),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 6448 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 3254,/* min_fll:3258, min_fll - read_margin */
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFC,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFC,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.delay_frame = 3,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.prohibited_exp = {8},
	},
	{/*reg_F2-S1 4096x3072 @30FPS Full-RAW-Crop w/ All-PD VB_MAX*/
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = cjmain_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom5_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom5,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom5),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 9424,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFC,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.csi_param = {
		},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{	/*reg_V2 2048x1152 @480FPS QBIN-V2H2 w/o PD VB_MAX*/
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = cjmain_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 2928000000,
		.linelength = 4352,
		.framelength = 1388,
		.max_framerate = 4803,
		.mipi_pixel_rate = 2046171428,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {12},
	},
	{	/*reg_L2-S1 4096x3072 @30FPS Full(Quad Bayer)-Crop All-PD LBMF_manual VB_MAX, LB-MF LUTA*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = cjmain_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom7_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom7,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom7),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 9424,
		.framelength = 5128 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 3288, /*(4607-1536+1)/1+220-4=3288*/
		.read_margin = 4,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFC,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFC,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.csi_param = {
		},
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 3,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{/*reg_B4 1152x864 @30FPS QBIN(VBIN)-Crop VB_MAX*/
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = cjmain_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12900,
		.max_framerate = 300,
		.mipi_pixel_rate = 1006628571,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 2208,
			.w0_size = 8192,
			.h0_size = 1728,
			.scale_w = 4096,
			.scale_h = 864,
			.x1_offset = 1472,
			.y1_offset = 0,
			.w1_size = 1152,
			.h1_size = 864,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1152,
			.h2_tg_size = 864,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_G1-S3 4096x2560 @30FPS QBIN(VBIN) DCG(RAW) DirectMode NOT VB_MAX, HSG*/
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = cjmain_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom9_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom9,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom9),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 14560,
		.framelength = 6608,
		.max_framerate = 300,
		.mipi_pixel_rate = 2502860000,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 512,
			.w0_size = 8192,
			.h0_size = 5120,
			.scale_w = 4096,
			.scale_h = 2560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2560,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cus9,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1503,
		.saturation_info = &imgsensor_saturation_info_10bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = FALSE,
		.pdc_enabled = FALSE,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{/*reg_G3 4096x2304 @60FPS QBIN(VBIN) DCG(RAW) DirectMode VB_MAX, HSG*/
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = cjmain_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom10_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom10,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom10),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2196000000,
		.linelength = 14560,
		.framelength = 3332,
		.max_framerate = 600,
		.mipi_pixel_rate = 2511090000,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cus10,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1503,
		.saturation_info = &imgsensor_saturation_info_10bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = FALSE,
		.pdc_enabled = FALSE,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_B3 2112x1584 @30FPS QBIN(VBIN)-Crop RST 4ms VB_MAX*/
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = cjmain_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12900,
		.max_framerate = 300,
		.mipi_pixel_rate = 1483885714,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1472,
			.w0_size = 8192,
			.h0_size = 3200,
			.scale_w = 4096,
			.scale_h = 1600,
			.x1_offset = 992,
			.y1_offset = 8,
			.w1_size = 2112,
			.h1_size = 1584,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2112,
			.h2_tg_size = 1584,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{/*reg_F1-S1 8192x6144 @30FPS Full(Q BAYER) All-PD VB_MAX*/
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = cjmain_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom12_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_custom12_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_custom12_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 11632,
		.framelength = 8332,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{	/*reg_B1-S1 4096x3072 @30FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = cjmain_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 12900,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_B5 2112x1584 @30FPS QBIN(VBIN)-Crop Rst 12ms VB_MAX*/
		.frame_desc = frame_desc_cus14,
		.num_entries = ARRAY_SIZE(frame_desc_cus14),
		.mode_setting_table = cjmain_custom14_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom14_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 992000000,
		.linelength = 7520,
		.framelength = 4368,
		.max_framerate = 300,
		.mipi_pixel_rate = 486857142,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1472,
			.w0_size = 8192,
			.h0_size = 3200,
			.scale_w = 4096,
			.scale_h = 1600,
			.x1_offset = 992,
			.y1_offset = 8,
			.w1_size = 2112,
			.h1_size = 1584,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2112,
			.h2_tg_size = 1584,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{/*reg_B2-S1 4096x2304 @60FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_cus15,
		.num_entries = ARRAY_SIZE(frame_desc_cus15),
		.mode_setting_table = cjmain_custom15_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom15_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 6448,
		.max_framerate = 600,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*4096x2304 @120FPS QBIN(VBIN) Tline= 2.64us NOT VB MAX*/
		.frame_desc = frame_desc_cus16,
		.num_entries = ARRAY_SIZE(frame_desc_cus16),
		.mode_setting_table = cjmain_custom16_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom16_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 3172,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2530971428,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cus10,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.dpc_enabled = false,
		.pdc_enabled = false,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*reg_G1-S1 4096x3072 @30FPS QBIN(VBIN) DCG(RAW) DirectMode VB_MAX, HSG*/
		.frame_desc = frame_desc_cus17,
		.num_entries = ARRAY_SIZE(frame_desc_cus17),
		.mode_setting_table = cjmain_custom17_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom17_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom17,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom17),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 14560,
		.framelength = 6664,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(4928),/*3.11dB(4928)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1503,
		.saturation_info = &imgsensor_saturation_info_10bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
	{	/*not use, reg_L2-S1 4096x3072 @30FPS Full(Quad Bayer)-Crop All-PD LBMF_manual VB_MAX, LB-MF LUTA*/
		.frame_desc = frame_desc_cus18,
		.num_entries = ARRAY_SIZE(frame_desc_cus18),
		.mode_setting_table = cjmain_custom18_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom18_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 9424,
		.framelength = 5128 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 3021257143,
		.readout_length = 3288, /*(4607-1536+1)/1+220-4=3288*/
		.read_margin = 4,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFC,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFC,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.csi_param = {
		},
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 3,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{/*G3-1	DCG-HDR	4096x2304 @60FPS Full-RMSC-Crop DCG(RAW) DirectMode w/ All-PD VB_MAX seamless reg_G3*/
		.frame_desc = frame_desc_cus19,
		.num_entries = ARRAY_SIZE(frame_desc_cus19),
		.mode_setting_table = cjmain_custom19_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom19_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom19,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom19),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 18672,
		.framelength = 2596,
		.max_framerate = 600,
		.mipi_pixel_rate = 2511090000,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 6,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(15360),/*24.08dB(15360)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 2048,
			.y0_offset = 1920,
			.w0_size = 4096,
			.h0_size = 2304,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 748,
		.saturation_info = &imgsensor_saturation_info_10bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{/*reg_G2-S3 4096x2560 @30FPS Full-RMSC-Crop DCG(RAW) DirectMode NOT VB_MAX, HSG*/
		.frame_desc = frame_desc_cus20,
		.num_entries = ARRAY_SIZE(frame_desc_cus20),
		.mode_setting_table = cjmain_custom20_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom20_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjmain_seamless_custom20,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjmain_seamless_custom20),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 2928000000,
		.linelength = 18672,
		.framelength = 5160,
		.max_framerate = 300,
		.mipi_pixel_rate = 2502860000,
		.readout_length = 0,
		.read_margin = 4 * 2,
		.framelength_step = 2,
		.coarse_integ_step = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 6,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16128),/*36.12dB(16128)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = REG2GAIN_ROUNDUP(0),/*0dB(0)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = REG2GAIN_ROUNDDOWN(15360),/*24.08dB(15360)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1792,
			.w0_size = 8192,
			.h0_size = 2560,
			.scale_w = 8192,
			.scale_h = 2560,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2560,
		},
		.pdaf_cap = TRUE,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 748,
		.saturation_info = &imgsensor_saturation_info_10bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_DIRECT_MODE,
			.dcg_gain_ratio_min = 1000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjmain_dcg_ratio_table_ratio4,
			.dcg_gain_table_size = sizeof(cjmain_dcg_ratio_table_ratio4),
		},
		.dpc_enabled = true,
		.pdc_enabled = true,
		.delay_frame = 2,
		.awb_enabled = true,
		.prohibited_exp = {8, 12, 16, 20, 24, 28},
	},
	{/*reg_B7 3840x2160 @120FPS QBIN(VBIN) VB_MAX*/
		.frame_desc = frame_desc_cus21,
		.num_entries = ARRAY_SIZE(frame_desc_cus21),
		.mode_setting_table = cjmain_custom21_setting,
		.mode_setting_len = ARRAY_SIZE(cjmain_custom21_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2928000000,
		.linelength = 7520,
		.framelength = 3224,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2371885714,
		.readout_length = 0,
		.read_margin = 4,
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = REG2GAIN_ROUNDUP(4916),/*3.10dB(4916)*/
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = REG2GAIN_ROUNDDOWN(16320),/*48.16dB(16320)*/
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 896,
			.w0_size = 8192,
			.h0_size = 4352,
			.scale_w = 4096,
			.scale_h = 2176,
			.x1_offset = 128,
			.y1_offset = 8,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1382,
		.csi_param = {
			.cphy_settle = 59,
		},
		.dpc_enabled = false,
		.pdc_enabled = false,
		.delay_frame = 2,
		.prohibited_exp = {8},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = CJMAIN_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0x35, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.mirror = MIRROR_FLIP,

	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 256, /*24dB --> 10^(24/10) = 251.189*/
	.ana_gain_type = 0,
	.ana_gain_step = 4,
	.ana_gain_table = cjmain_ana_gain_table,
	.ana_gain_table_size = sizeof(cjmain_ana_gain_table),
	.dig_gain_min = BASE_DGAIN * 1, /* no need (only use ana gain)*/
	.dig_gain_max = BASEGAIN * 16, /* no need (only use ana gain)*/
	.dig_gain_step = 4, /* no need (only use ana gain)*/
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xFFFC - 64) << 8,
	.cit_lshift_max = 8,
	.exposure_step = 4,
	.exposure_margin = 64,

	.frame_length_max = 0xfffc,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1946160,
	.line_interleave_num = 2,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF|HDR_SUPPORT_DCG_VS,
	.seamless_switch_support = TRUE,
	.seamless_switch_type = SEAMLESS_SWITCH_CUT_VB_INIT_SHUT,
	.seamless_switch_hw_re_init_time_ns = 0,
	.seamless_switch_prsh_hw_fixed_value = 32,
	.seamless_switch_prsh_length_lc = 0,
	.reg_addr_prsh_length_lines = {0x3058, 0x3059, 0x305A, 0x305B},
	.reg_addr_prsh_mode = 0x3056,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
			{0x3162, 0x3163},
			{0x0224, 0x0225},
	},
	.reg_addr_exposure_in_lut = {
			{0x0E20, 0x0E21},
			{0x0E60, 0x0E61},
			{0x0EA0, 0x0EA1},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3160,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
			{0x3164, 0x3165},
			{0x0216, 0x0217},
	},
	.reg_addr_ana_gain_in_lut = {
			{0x0E22, 0x0E23}, /* DCG+VS HCG AGAIN */
			{0x0E62, 0x0E63}, /* DCG+VS VS AGAIN */
			{0x0E38, 0x0E39}, /* DCG+VS LCG AGAIN */
	},
	.reg_addr_dig_gain = {
			{0x020E, 0x020F},
			{0x3166, 0x3167},
			{0x0218, 0x0219},
	},
	.reg_addr_dig_gain_in_lut = {
			{0x0E24, 0x0E25},
			{0x0E64, 0x0E65},
			{0x0EA4, 0x0EA5},
	},
	.reg_addr_dcg_ratio = 0x3182,
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_frame_length_in_lut = {
			{0x0E28, 0x0E29},
			{0x0E68, 0x0E69},
			{0x0EA8, 0x0EA9},
	},
	.reg_addr_temp_en = 0x0138, /* TEMP_SEN_CTL */
	.reg_addr_temp_read = 0x013A, /* TEMP_SEN_OUT */
	.reg_addr_auto_extend = 0x0350, /* FRM_LENGTH_CTL */
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010,
	.reg_addr_fast_mode_in_lbmf = 0x31B7,
	.reg_addr_gph_delay = 0x3018,
	.seamless_switch_with_gph_delay = 0,

/*	.init_setting_table = cjmain_init_setting,
	.init_setting_len = ARRAY_SIZE(cjmain_init_setting),*/
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0xf10e5980,
	.cust_get_linetime_in_ns = cjmain_get_linetime_in_ns,
	.cycle_base_ratio = 16,
	.ctle_param = &cjmain_static_ctle_param,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
	.set_ctrl_locker = set_ctrl_locker,
	.disable_fast_mode = cjmain_disable_fast_mode,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, {24}, 0},
	{HW_ID_RST, {0}, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, {4}, 1000},
	{HW_ID_AVDD, {2804000, 2804000}, 1000},
	{HW_ID_AVDD1, {1804000, 1804000}, 1000},
	{HW_ID_AFVDD, {3300000, 3300000}, 3000},
	{HW_ID_DOVDD, {1800000, 1800000}, 1000},
	{HW_ID_DVDD, {832000, 832000}, 1000},
	{HW_ID_RST, {1}, 4000}
};

const struct subdrv_entry cjmain_mipi_raw_entry = {
	.name = "cjmain_mipi_raw",
	.id = CJMAIN_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_cjmain_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != cjmain_eeprom_info[meta_id].meta)
		return -1;

	if (size != cjmain_eeprom_info[meta_id].size)
		return -1;

	addr = cjmain_eeprom_info[meta_id].start;
	readsize = cjmain_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = CJMAIN_EEPROM_READ_ID,
	.i2c_write_id = CJMAIN_EEPROM_WRITE_ID,

	.addr_modinfo = CJMAIN_EEPROM_MOD_INFO,
	.addr_sensorid = CJMAIN_EEPROM_SENSOR_ID,
	.addr_lens = CJMAIN_EEPROM_LENS_ID,
	.addr_vcm = CJMAIN_EEPROM_VCM_ID,
	.addr_modinfoflag = CJMAIN_EEPROM_MOD_INFO_FLAG,

	.addr_af = CJMAIN_EEPROM_MOD_AF,
	.addr_afmacro = CJMAIN_EEPROM_MOD_AF_MACRO,
	.addr_afinf = CJMAIN_EEPROM_MOD_AF_INF,
	.addr_afflag = CJMAIN_EEPROM_MOD_AF_FLAG,

	.addr_qrcode = CJMAIN_EEPROM_MOD_QR_CODE,
	.addr_qrcodeflag = CJMAIN_EEPROM_MOD_QR_CODE_FLAG,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct oplus_eeprom_info_struct* infoPtr;
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	infoPtr = (struct oplus_eeprom_info_struct*)(para);
	*len = sizeof(oplus_eeprom_info);
	infoPtr->afInfo[0] = (kal_uint8)((infoPtr->afInfo[1] << 4) | (infoPtr->afInfo[0] >> 4));
	infoPtr->afInfo[1] = (kal_uint8)(infoPtr->afInfo[1] >> 4);
	infoPtr->afInfo[2] = (kal_uint8)((infoPtr->afInfo[3] << 4) | (infoPtr->afInfo[2] >> 4));
	infoPtr->afInfo[3] = (kal_uint8)(infoPtr->afInfo[3] >> 4);
	infoPtr->afInfo[4] = (kal_uint8)((infoPtr->afInfo[5] << 4) | (infoPtr->afInfo[4] >> 4));
	infoPtr->afInfo[5] = (kal_uint8)(infoPtr->afInfo[5] >> 4);

	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, CJMAIN_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static int cjmain_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static int cjmain_get_otp_qcom_pdaf_offset_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_OFFSET_DATA_START_ADDR, otp_qcom_pdaf_offset_data, OTP_QCOM_PDAF_OFFSET_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_offset_data, sizeof(otp_qcom_pdaf_offset_data));
	*len = sizeof(otp_qcom_pdaf_offset_data);

	return 0;
}

#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH	 (32)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
	ret = adaptor_i2c_wr_p8(ctx->i2c_client, CJMAIN_EEPROM_WRITE_ID >> 1,
			addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xA000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, CJMAIN_EEPROM_READ_ID >> 1, reg, 0x0E);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, CJMAIN_EEPROM_READ_ID >> 1, reg, 0x00);
	}

	return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
	ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;
	kal_uint32 checksum = 0;
	UINT32 i = 0;
	kal_uint16 offset = 0;
	if(pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
			pStereodata->uSensorId,
			pStereodata->uDeviceId,
			pStereodata->baseAddr,
			pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		for(i = 0; i < pStereodata->dataLength; i++) {
			checksum += pData[i];
		}
		pData[data_length] = 0x01;
		pData[data_length + 1] = checksum % 255;
		data_length = data_length + 2;
		offset = ALIGN(data_base, WRITE_DATA_MAX_LENGTH) - data_base;
		if (offset > data_length) {
			offset = data_length;
		}
		if (((pStereodata->uSensorId == CJMAIN_SENSOR_ID) && ((data_length - 2) == CALI_DATA_MASTER_LENGTH))
				&& (data_base == CJMAIN_STEREO_START_ADDR || data_base == CJMAIN_STEREO_MT_START_ADDR
				|| data_base == CJMAIN_STEREO_MT_105CM_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
			/* close write protect */
			write_eeprom_protect(ctx, 0);
			msleep(6);
			if (offset > 0) {
				ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: offset\n");
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
				data_base += offset;
				data_length -= offset;
				pData += offset;
			}
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			for (i = 0; i < idx; i++) {
				ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
					&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
				&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != ERROR_NONE) {
				LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(ctx, 1);
				msleep(6);
				return -1;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			msleep(6);
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
			LOG_INF("write_Module_data Write end\n");
		} else if ((pStereodata->uSensorId == CJMAIN_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
				&& (data_base == CJMAIN_AESYNC_START_ADDR)) {
			LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
			/* close write protect */
			write_eeprom_protect(ctx, 0);
			msleep(6);
			if (offset > 0) {
				ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: offset\n");
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
				data_base += offset;
				data_length -= offset;
				pData += offset;
			}
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			for (i = 0; i < idx; i++) {
				ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
					&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
				&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != ERROR_NONE) {
				LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(ctx, 1);
				msleep(6);
				return -1;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			msleep(6);
			/* LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+1),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+2),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+3),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+4),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+5),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+6),
				read_cmos_eeprom_8(ctx, CJMAIN_AESYNC_START_ADDR+7)); */
			LOG_INF("AESync write_Module_data Write end\n");
		} else {
			LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return -1;
		}
	} else {
		LOG_INF("cjmain write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

static int cjmain_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int ret = ERROR_NONE;
	ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
	if (ret != ERROR_NONE) {
		*len = (u32)-1; /*write eeprom failed*/
		LOG_INF("ret=%d\n", ret);
	}
	return 0;
}

static int cjmain_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_MASTER_LENGTH)
		*len = CALI_DATA_MASTER_LENGTH;
	LOG_INF("feature_data mode: %d", *feature_data_16);
	switch (*feature_data_16) {
	case EEPROM_STEREODATA_MT_MAIN:
		read_cjmain_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MT_MAIN_105CM:
		read_cjmain_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA_105CM,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MW_MAIN:
	default:
		read_cjmain_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	}
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, CJMAIN_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "cjmain read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "cjmain read_otp_info end\n");
}

static int cjmain_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	u32 length = sizeof(otp_data_checksum);

	if(*len < sizeof(otp_data_checksum)) {
		length = *len;
	}
	DRV_LOGE(ctx, "get otp data length:0x%x", length);

	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}

	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, length);
	return 0;
}

static int cjmain_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = GET_SENSOR_ID_RETRY_CNT;
	static bool first_read = KAL_TRUE;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOGE(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0xa24a) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOGE(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOGE(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = GET_SENSOR_ID_RETRY_CNT;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static u16 cjmain_feedback_awbgain[] = {
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B90, 0x02,
	0x0B91, 0x28,
	0x0B92, 0x01,
	0x0B93, 0x77,
	0x0B94, 0x01,
	0x0B95, 0x00,
};

/*write AWB gain to sensor*/
static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
	r_gain_int = r_gain / 512;
	b_gain_int = b_gain / 512;
	cjmain_feedback_awbgain[5] = r_gain_int;
	cjmain_feedback_awbgain[7] = (r_gain - r_gain_int * 512) / 2;
	cjmain_feedback_awbgain[9] = b_gain_int;
	cjmain_feedback_awbgain[11] = (b_gain - b_gain_int * 512) / 2;
	subdrv_i2c_wr_regs_u8(ctx, cjmain_feedback_awbgain,
		ARRAY_SIZE(cjmain_feedback_awbgain));
}

static int cjmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
	return 0;
}

static void cjmain_write_init_setting(struct subdrv_ctx *ctx)
{
	u8 module_info = 0;
	u64 time_boot_begin = 0;
	u64 ixc_time = 0;

	module_info = subdrv_i2c_rd_u8(ctx, 0x0018);
	DRV_LOG_MUST(ctx, "module_info (0x%x) write init setting +", module_info);

	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en))
		time_boot_begin = ktime_get_boottime_ns();

	if ((module_info | 0x0F) == 0x1F) {
		ixc_time = ixc_table_rewrite(ctx, cjmain_init_setting_short, ARRAY_SIZE(cjmain_init_setting_short));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(cjmain_init_setting_short);
	} else {
		ixc_time = ixc_table_rewrite(ctx, cjmain_init_setting_long, ARRAY_SIZE(cjmain_init_setting_long));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(cjmain_init_setting_long);
	}

	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en)) {
		ctx->sensor_pw_on_profile.i2c_init_period = ktime_get_boottime_ns() - time_boot_begin;
		ctx->sensor_pw_on_profile.i2c_init_table_len = ctx->s_ctx.init_setting_len;
	}
	DRV_LOG_MUST(ctx, "write init setting -, size:%u, time(us):%lld\n",
					ctx->sensor_pw_on_profile.i2c_init_table_len, ixc_time);

	if (ctx->s_ctx.temperature_support && ctx->s_ctx.reg_addr_temp_en)
		subdrv_ixc_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	/* enable mirror or flip */
	set_mirror_flip(ctx, ctx->mirror);
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	cjmain_write_init_setting(ctx);

	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
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

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			if (ctx->s_ctx.reg_addr_mirror_flip) {
				subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_mirror_flip, MIRROR_FLIP);
			}
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, QSC_EN_ADDR, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, QSC_EN_ADDR, 0x00);
		}
	}

	support = info[idx].pdc_support;
	pbuf = info[idx].preload_pdc_table;
	size = info[idx].pdc_size;
	addr = 0xD200;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size >> 1);
			addr = 0xD300;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf + (size >> 1), size >> 1);
			DRV_LOG(ctx, "set SPC data done.");
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature < 0x55)
		temperature_convert = temperature;
	else if (temperature < 0x80)
		temperature_convert = 85;
	else if (temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (int)temperature - 256;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (16384 - (16384 * BASEGAIN) / gain);
}

void cjmain_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u set default\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = 0;
	}
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
		case HDR_NONE:
		case HDR_RAW_LBMF:
		case HDR_RAW_DCG_RAW:
			if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
				ctx->s_ctx.mode[scenario_id].multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min) {
				*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
				*min_shutter = ctx->s_ctx.mode[scenario_id].multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min;
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

int cjmain_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	cjmain_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return 0;
}

static int cjmain_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;
	u64 shutters[3] = {0};
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;
	pre_seamless_scenario_id = ctx->current_scenario_id;

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	/*When seamless from DCG_VS, write the default OB setting*/
	if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM2) {
		ixc_table_write(ctx, cjmain_custom2_OB_default_setting, ARRAY_SIZE(cjmain_custom2_OB_default_setting));
		DRV_LOG_MUST(ctx, "SET DEFAULT OB SETTING -\n");
	}

	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_DCG_RAW_VS ||
	   (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF &&
		ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_DCG_RAW_VS)) {
		ctx->s_ctx.seamless_switch_with_gph_delay = 1;
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_gph_delay, 0x01);
		DRV_LOG_MUST(ctx, "gph_delay = 1\n");
	}

	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);

	update_mode_info_seamless_switch(ctx, scenario_id);

	ixc_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	ctx->ae_ctrl_gph_en = 1;
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		case HDR_RAW_DCG_RAW_VS:
		case HDR_RAW_DCG_COMPOSE_VS:
			/* DONE: DCG-VS */
			shutters[0] = ae_ctrl->exposure.le_exposure;
			shutters[1] = ae_ctrl->exposure.se_exposure;
			set_dcg_vs_multi_shutter_frame_length_in_lut(ctx,
				shutters, 2, 0, frame_length_in_lut);
			cjmain_set_dcg_vs_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}

	/*if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM2) {
		DRV_LOG_MUST(ctx, "SET V1 OB SETTING -\n");
		ixc_table_write(ctx, cjmain_custom2_OB_V1_setting, ARRAY_SIZE(cjmain_custom2_OB_V1_setting));
	}*/

	ctx->ae_ctrl_gph_en = 0;
	commit_i2c_buffer(ctx);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	mutex_lock(&ctx->fast_mode_lock);
	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	mutex_unlock(&ctx->fast_mode_lock);
	ctx->is_seamless = FALSE;

	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int cjmain_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (mode) {
	/* 1:Solid Color 2:Color Bar 5:Black */
		switch (mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x01);
			break;
		default:
			subdrv_i2c_wr_u8(ctx, 0x0601, mode);
			break;
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
	}
	ctx->test_pattern = mode;
	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int cjmain_disable_fast_mode(struct subdrv_ctx *ctx)
{
	if (ctx->fast_mode_on && (ctx->sof_cnt > ctx->ref_sof_cnt)) {
		DRV_LOG_MUST(ctx, "disabled_fast_mode sof_cnt(%u) ref_sof_cnt(%u) fast_mode_on(%d)",
			ctx->sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
		mutex_lock(&ctx->fast_mode_lock);
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		mutex_unlock(&ctx->fast_mode_lock);
		commit_i2c_buffer(ctx);
	}
	return ERROR_NONE;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	ctx->sof_cnt = sof_cnt;
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		/*if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM2) {
			ixc_table_write(ctx, cjmain_custom2_OB_V1_setting, ARRAY_SIZE(cjmain_custom2_OB_V1_setting));
		}*/
		DRV_LOG(ctx, "seamless_switch disabled.");
	/*When seamless to DCG_VS, write the DCG_VS OB setting*/
		ctx->s_ctx.seamless_switch_with_gph_delay = 0;
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_gph_delay, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

void get_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	size = info[idx].qsc_size;
	addr = info[idx].addr_qsc;
	buf = info[idx].qsc_table;
	if (support && size > 0) {
		/* Check QSC validation */
		if (info[idx].preload_qsc_table == NULL) {
			info[idx].preload_qsc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_qsc_table, size)) {
					DRV_LOGE(ctx, "preload QSC data failed");
				}
			} else {
				memcpy(info[idx].preload_qsc_table, buf, size);
			}
			DRV_LOG(ctx, "preload QSC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "QSC data is already preloaded %u bytes", size);
		}
	}

	support = info[idx].pdc_support;
	size = info[idx].pdc_size;
	addr = info[idx].addr_pdc;
	buf = info[idx].pdc_table;
	if (support && size > 0) {
		/* Check pdc validation */
		if (info[idx].preload_pdc_table == NULL) {
			info[idx].preload_pdc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_pdc_table, size)) {
					DRV_LOGE(ctx, "preload PDC data failed");
				}
			} else {
				memcpy(info[idx].preload_pdc_table, buf, size);
			}
			DRV_LOG(ctx, "preload PDC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "PDC data is already preloaded %u bytes", size);
		}
	}
	ctx->is_read_preload_eeprom = 1;
}

/*
static void calculate_prsh_length_lines(struct subdrv_ctx *ctx,
	struct mtk_hdr_ae *ae_ctrl,
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id)
{
	u32 ae_ctrl_cit;
	u32 prsh_length_lc = 0;
	u32 cit_step = 1;
	u8 hw_fixed_value = ctx->s_ctx.seamless_switch_prsh_hw_fixed_value;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = ctx->current_scenario_id;
	enum IMGSENSOR_HDR_MODE_ENUM hdr_mode;

	if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM4 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM7) {
		prsh_length_lc = 2000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM7 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM4) {
		prsh_length_lc = 2300;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM2 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM18) {
		prsh_length_lc = 5000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM18 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM2) {
		prsh_length_lc = 6000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM2 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM19) {
		prsh_length_lc = 6000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM19 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM2) {
		prsh_length_lc = 5000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW && scenario_id == SENSOR_SCENARIO_ID_CUSTOM4) {
		prsh_length_lc = 4400;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM4 && scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
		prsh_length_lc = 2265;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM4 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM12) {
		prsh_length_lc = 2000;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM12 && scenario_id == SENSOR_SCENARIO_ID_CUSTOM4) {
		prsh_length_lc = 2300;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW && scenario_id == SENSOR_SCENARIO_ID_CUSTOM3) {
		prsh_length_lc = 3600;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 && scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
		prsh_length_lc = 2300;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_NORMAL_CAPTURE && scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) {
		prsh_length_lc = 3600;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5 && scenario_id == SENSOR_SCENARIO_ID_NORMAL_CAPTURE) {
		prsh_length_lc = 3700;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW && scenario_id == SENSOR_SCENARIO_ID_CUSTOM7) {
		prsh_length_lc = 3600;
	} else if (pre_seamless_scenario_id == SENSOR_SCENARIO_ID_CUSTOM7 && scenario_id == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
		prsh_length_lc = 2300;
	} else {
		prsh_length_lc = 0;
	}

	hdr_mode = ctx->s_ctx.mode[scenario_id].hdr_mode;
	switch (hdr_mode) {
	case HDR_RAW_LBMF:
		if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
			IMGSENSOR_LBMF_EXPOSURE_SE_FIRST) {
			ae_ctrl_cit = ae_ctrl->exposure.me_exposure;
			DRV_LOG_MUST(ctx, "debug se %llu le %llu, me %llu", ae_ctrl->exposure.se_exposure, ae_ctrl->exposure.le_exposure, ae_ctrl->exposure.me_exposure);
		} else if (ctx->s_ctx.mode[scenario_id].exposure_order_in_lbmf ==
			IMGSENSOR_LBMF_EXPOSURE_LE_FIRST) {
			ae_ctrl_cit = ae_ctrl->exposure.le_exposure;
			DRV_LOG_MUST(ctx, "debug le\n");
		} else {
			DRV_LOGE(ctx, "pls assign exposure_order_in_lbmf value!\n");
			return;
		}
		break;
	case HDR_NONE:
	case HDR_RAW:
	case HDR_CAMSV:
	case HDR_RAW_ZHDR:
	case HDR_MultiCAMSV:
	case HDR_RAW_STAGGER:
	case HDR_RAW_DCG_RAW:
	case HDR_RAW_DCG_COMPOSE:
	default:
		ae_ctrl_cit = ae_ctrl->exposure.le_exposure;
		break;
	}
	ae_ctrl_cit = max(ae_ctrl_cit, ctx->s_ctx.exposure_min);
	ae_ctrl_cit = min(ae_ctrl_cit, ctx->s_ctx.exposure_max);
	cit_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step ?: 1;
	if (cit_step) {
		ae_ctrl_cit = round_up(ae_ctrl_cit, cit_step);
		prsh_length_lc = round_up(prsh_length_lc, cit_step);
	}
	DRV_LOG_MUST(ctx, "prsh_length_lc %u ae_ctrl_cit %u fine_integ_line %d\n",
					prsh_length_lc, ae_ctrl_cit, ctx->s_ctx.mode[scenario_id].fine_integ_line);
	if(hdr_mode != HDR_RAW_LBMF && ctx->s_ctx.mode[scenario_id].fine_integ_line != 0) {
		ae_ctrl_cit = ae_ctrl_cit / 1000;
	}
	prsh_length_lc = (prsh_length_lc > (ae_ctrl_cit + hw_fixed_value)) ? prsh_length_lc : 0;
	if (prsh_length_lc < (ae_ctrl_cit + hw_fixed_value)) {
		DRV_LOG_MUST(ctx,
			"pre-shutter no need: prsh_length_lc(%u) < (ae_ctrl_cit(%u(max=%u,min=%u)) + hw_fixed_value(%u))\n",
			prsh_length_lc, ae_ctrl_cit, ctx->s_ctx.exposure_max, ctx->s_ctx.exposure_min, hw_fixed_value);
		ctx->s_ctx.seamless_switch_prsh_length_lc = 0;
		return;
	}

	ctx->s_ctx.seamless_switch_prsh_length_lc = prsh_length_lc;
}
*/

static int cjmain_get_linetime_in_ns(void *arg,
	u32 scenario_id, u32 *linetime_in_ns, enum GET_LINETIME_ENUM linetime_type,
	enum IMGSENSOR_EXPOSURE exp_idx)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u32 ret = 0;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		*linetime_in_ns = 4200;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		*linetime_in_ns = 5000;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		*linetime_in_ns = 2620;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		*linetime_in_ns = 5040;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM20:
		*linetime_in_ns = 6450;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
	case SENSOR_SCENARIO_ID_CUSTOM2:
	case SENSOR_SCENARIO_ID_CUSTOM3:
	case SENSOR_SCENARIO_ID_CUSTOM4:
	case SENSOR_SCENARIO_ID_CUSTOM5:
	case SENSOR_SCENARIO_ID_CUSTOM6:
	case SENSOR_SCENARIO_ID_CUSTOM7:
	case SENSOR_SCENARIO_ID_CUSTOM8:
	case SENSOR_SCENARIO_ID_CUSTOM11:
	case SENSOR_SCENARIO_ID_CUSTOM12:
	case SENSOR_SCENARIO_ID_CUSTOM13:
	case SENSOR_SCENARIO_ID_CUSTOM14:
	case SENSOR_SCENARIO_ID_CUSTOM15:
	case SENSOR_SCENARIO_ID_CUSTOM16:
	case SENSOR_SCENARIO_ID_CUSTOM17:
	case SENSOR_SCENARIO_ID_CUSTOM18:
	case SENSOR_SCENARIO_ID_CUSTOM19:
	default:
		ret = common_get_cycle_base_v1_linetime_in_ns(ctx, scenario_id,
			linetime_in_ns, linetime_type, exp_idx);
		break;
	}

	DRV_LOG(ctx, "linetime(%d)ns\n", *linetime_in_ns);
	return ret;
}

static void cjmain_set_dcg_vs_multi_gain_in_lut(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt) {
	int i = 0;
	u16 ana_gain_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	struct subdrv_mode_struct *mode_info = &ctx->s_ctx.mode[ctx->current_scenario_id];
	enum IMGSENSOR_DCG_GAIN_MODE dcg_gain_mode = mode_info->dcg_info.dcg_gain_mode;

	if (exp_cnt > ARRAY_SIZE(ctx->ana_gain)) {
		DRV_LOGE(ctx,
			"invalid exp_cnt:%u>%lu\n",
			exp_cnt, ARRAY_SIZE(ctx->ana_gain));
		exp_cnt = ARRAY_SIZE(ctx->ana_gain);
	}
	for (i = 0; i < exp_cnt; i++) {
		DRV_LOG_MUST(ctx, "input gain[%d] = %u\n", i, gains[i]);

		/* check boundary of gain */
		gains[i] = max(gains[i],
			ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[i].min);
		gains[i] = min(gains[i],
			ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[i].max);
		/* mapping of gain to register value */
		if (ctx->s_ctx.g_gain2reg != NULL)
			gains[i] = ctx->s_ctx.g_gain2reg(gains[i]);
		else
			gains[i] = gain2reg(gains[i]);
	}
	for (i = 0; i < exp_cnt; i++) {
		/* update ana_gain_in_lut */
		/* 3exp: ana_gain_lut_a = DCG(HSG) / ana_gain_lut_b = DCG(LSG) / ana_gain_lut_c = VS */
		ana_gain_in_lut[i] = gains[i];
	}
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	for (i = 0; i < exp_cnt; i++)
		ctx->ana_gain[i] = gains[i];
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	/* check dcg ratio/direct mode */
	/* write gain: update ana gain addr and set gain for dcg */
	switch (dcg_gain_mode) {
	case IMGSENSOR_DCG_DIRECT_MODE:
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_ana_gain_in_lut[0].addr[0],
			(ana_gain_in_lut[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_ana_gain_in_lut[0].addr[1],
			ana_gain_in_lut[0] & 0xFF);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_ana_gain_in_lut[2].addr[0],
			(ana_gain_in_lut[1] >> 8) & 0xFF);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_ana_gain_in_lut[2].addr[1],
			ana_gain_in_lut[1] & 0xFF);
		break;
	case IMGSENSOR_DCG_RATIO_MODE:
		set_i2c_buffer(ctx,
			ctx->s_ctx.reg_addr_ana_gain_in_lut[0].addr[0],
			(ana_gain_in_lut[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,
			ctx->s_ctx.reg_addr_ana_gain_in_lut[0].addr[1],
			ana_gain_in_lut[0] & 0xFF);
		break;
	}

	/* write gain: update ana gain addr and set gain for vs */
	set_i2c_buffer(ctx,
		ctx->s_ctx.reg_addr_ana_gain_in_lut[1].addr[0],
		(ana_gain_in_lut[2] >> 8) & 0xFF);
	set_i2c_buffer(ctx,
		ctx->s_ctx.reg_addr_ana_gain_in_lut[1].addr[1],
		ana_gain_in_lut[2] & 0xFF);
	DRV_LOG(ctx,
		"sid:%u,dcg_gain_mode(%d)gain(input/lut):0x%x/%x/%x,%x/%x/%x\n",
		ctx->current_scenario_id,
		dcg_gain_mode,
		gains[0], gains[1], gains[2],
		ana_gain_in_lut[0], ana_gain_in_lut[1], ana_gain_in_lut[2]);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	/*commit_i2c_buffer(ctx);*/
	/* group hold end */
}

static void cjmain_set_hdr_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt) {
	int i = 0;
	u32 values[3] = {0};

	if (gains != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(gains + i);
	}

	switch(ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode) {
	case HDR_RAW_LBMF:
		set_multi_gain_in_lut(ctx, values, exp_cnt);
		break;
	case HDR_RAW_DCG_RAW_VS:
	case HDR_RAW_DCG_COMPOSE_VS:
		/* DONE: DCG-VS */
		cjmain_set_dcg_vs_multi_gain_in_lut(ctx, values, exp_cnt);
		break;
	default:
		set_multi_gain(ctx,	values, exp_cnt);
		break;
	}
}

static int cjmain_set_dual_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u64 *feature_data = (u64 *) para;
	/* DONE: DCG-VS modify in set_hdr_tri_gain */
	if (ctx->s_ctx.aov_sensor_support &&
		ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode &&
		(ctx->s_ctx.mode[ctx->current_scenario_id].ae_ctrl_support !=
			IMGSENSOR_AE_CONTROL_SUPPORT_VIEWING_MODE))
		DRV_LOG_MUST(ctx,
			"AOV mode not support ae gain control!\n");
	else {
		cjmain_set_hdr_gain(ctx, feature_data, 2);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static int cjmain_set_hdr_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u64 *feature_data = (u64 *) para;
	/* DONE: DCG-VS modify in set_hdr_tri_gain */
	if (ctx->s_ctx.aov_sensor_support &&
		ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode &&
		(ctx->s_ctx.mode[ctx->current_scenario_id].ae_ctrl_support !=
			IMGSENSOR_AE_CONTROL_SUPPORT_VIEWING_MODE))
		DRV_LOG_MUST(ctx,
			"AOV mode not support ae gain control!\n");
	else {
		cjmain_set_hdr_gain(ctx, feature_data, 3);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static int cjmain_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if (!ctx || !para || !len)
		return -1;

	u16 *framerate = (u16 *)para;
	set_max_framerate(ctx, *framerate, 1);
	set_auto_flicker(ctx, 1);
	set_dummy(ctx);
	DRV_LOG_MUST(ctx, "fps(input/max):%u/%u\n", *framerate, ctx->current_fps);

	return 0;
}
