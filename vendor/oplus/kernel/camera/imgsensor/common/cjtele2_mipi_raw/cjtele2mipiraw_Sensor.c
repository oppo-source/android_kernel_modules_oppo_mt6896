// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 cjtele2mipiraw_Sensor.c
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
#include "cjtele2mipiraw_Sensor.h"
// #include "adaptor-subdrv-ctrl.h"
// #include "adaptor-subdrv.h"
// #include "adaptor-ctrls.h"
// #include "cjtele2_ana_gain_table.h"
// #include <linux/of.h>
// #include "adaptor-i2c.h"
// #include <linux/regulator/consumer.h>

#define CJTELE2_EEPROM_READ_ID	0xA1
#define CJTELE2_EEPROM_WRITE_ID   0xA0
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "cjtele2_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x8000
#define OTP_QCOM_PDAF_DATA_LENGTH 0x03E8
#define OTP_QCOM_PDAF_DATA_START_ADDR 0x3C00
#define OTP_QCOM_PDAF_OFFSET_DATA_LENGTH 0x00C8
#define OTP_QCOM_PDAF_OFFSET_DATA_START_ADDR 0x3FF0
#define GET_SENSOR_ID_RETRY_CNT    5
#define SUBDRV_I2C_BUF_SIZE 256

static bool module_flag = FALSE;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static kal_uint8 otp_qcom_pdaf_data[OTP_QCOM_PDAF_DATA_LENGTH] = {0};
static kal_uint8 otp_qcom_pdaf_offset_data[OTP_QCOM_PDAF_OFFSET_DATA_LENGTH] = {0};
#define MAX_BURST_LEN  2048
static u8 * msg_buf = NULL;

static int group_hold_frame_count = 0;
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int cjtele2_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_sensor_temperature(void *arg);
static void get_sensor_cali(void* arg);
static void set_sensor_cali(void *arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static int cjtele2_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_i2c_burst_wr_regs_u16(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);
static int cjtele2_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_get_otp_qcom_pdaf_offset_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int cjtele2_chk_streaming_st(void *arg);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts);
static int cjtele2_get_otp_initsetting_info(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* STRUCT */

struct mtk_sensor_ctle_param cjtele2_static_ctle_param = {
	.cdr_delay = 0xC,
};

static struct eeprom_map_info cjtele2_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x009A, 0x009B, 6, true },
	{ EEPROM_META_AF_FLAG, 0x009A, 0x009A, 0x009B, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x51F0, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2780, 0x2D89, 0x2D8A, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x5000, 0x3859, 0x385A, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0x6000, 0x3859, 0x385A, CALI_DATA_SLAVE_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, cjtele2_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, cjtele2_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, cjtele2_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, get_eeprom_common_data},
	{SENSOR_FEATURE_SET_SENSOR_OTP, cjtele2_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, cjtele2_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, cjtele2_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_DATA, cjtele2_get_otp_qcom_pdaf_data},
	{SENSOR_FEATURE_GET_OTP_QCOM_PDAF_OFFSET_DATA, cjtele2_get_otp_qcom_pdaf_offset_data},
	{SENSOR_FEATURE_SET_AWB_GAIN, cjtele2_set_awb_gain},
	{SENSOR_FEATURE_SET_VIDEO_MODE, cjtele2_set_video_mode},
	{SENSOR_FEATURE_GET_OTP_INITSETTING_INFO, cjtele2_get_otp_initsetting_info},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01AC010F,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,
	},
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

// static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
// 	.gain_ratio = 4000,
// 	.OB_pedestal = 256,
// 	.saturation_level = 4095,
// };

static u32 cjtele2_dcg_ratio_table_10bit[] = {8000};
// static u32 cjtele2_dcg_ratio_table_12bit[] = {4000};

#define pd_i4Crop { \
	/* pre cap normal_video hs_video slim_video */\
	{0, 0}, {0, 0}, {0, 256}, {0, 384}, {0, 384},\
	/* cust1 cust2         cust3   cust4   cust5 */\
	{0, 0}, {2048,1536}, {0, 0}, {0, 0}, {6144, 4608},\
	/* cust6 cust7     cust8         cust9         cust10 */\
	{0, 0}, {0, 256}, {2048, 1792}, {6144, 4864}, {0, 192},\
	/* cust11  cust12  cust13      cust14  cust15 */\
	{0, 384}, {0, 0}, {2048, 1536}, {0, 256}, {5736, 4304}, \
	/* cust16  cust17      cust18       cust19  cust20 */\
	{0, 256}, {128, 456}, {2048, 1920}\
}

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_fullsize = {
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
	.iMirrorFlip = IMAGE_H_MIRROR,
	.i4FullRawW = 16384,
	.i4FullRawH = 12288,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 8,
		.i4BinFacY = 16,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_fullsize_fdsum = {
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
	.iMirrorFlip = IMAGE_H_MIRROR,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

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
	.iMirrorFlip = IMAGE_H_MIRROR,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_8Fdsum_4H2V = {
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
	.iMirrorFlip = IMAGE_H_MIRROR,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4ModeIndex = 0x3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		// .i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
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
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
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
	// {
	// 	.bus.csi2 = {
	// 		.channel = 0,
	// 		.data_type = 0x30,
	// 		.hsize = 4096,
	// 		.vsize = 768,
	// 		.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
	// 		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
	// 	},
	// },
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
			.force_remap_user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
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
	// {
	// 	.bus.csi2 = {
	// 		.channel = 0,
	// 		.data_type = 0x30,
	// 		.hsize = 2048,
	// 		.vsize = 384,
	// 		.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
	// 		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
	// 	},
	// },
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
			.force_remap_user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
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
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 8192,
			.vsize = 6144,
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
			.hsize = 1024,
			.vsize = 192,
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
			.hsize = 16384,
			.vsize = 12288,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
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
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 640,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
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
			.channel = 3,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 320,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
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
			.channel = 3,
			.data_type = 0x30,
			.hsize = 1024,
			.vsize = 160,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

/* 14_OceanDX4_03_JN5_Full_12.5Mp_Bypass_4096x3072_30fps_3056Msps QBC*/
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
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
		}
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
			.hsize = 2048,
			.vsize = 384,
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
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 640,
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
			.hsize = 4912,
			.vsize = 3680,
			.user_data_desc = VC_STAGGER_NE,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_FIRST,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 1224,
			.vsize = 230,
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
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 640,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus18[] = {
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
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.fs_seq = MTK_FRAME_DESC_FS_SEQ_LAST,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{/*00_4096x3072_30fps_2976Msps*/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = cjtele2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_preview_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_preview_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_preview_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = { /* BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/*00_4096x3072_30fps_2976Msps*/
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = cjtele2_capture_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {  /* BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/*07_4096x2560_30fps_2976Msps*/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = cjtele2_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_normal_video_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_normal_video_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2560*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/* 011_4096x2304_60fps_2976Msps */
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = cjtele2_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_hs_video_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_hs_video_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 5128,
		.max_framerate = 600,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {  /* 16:10 BIN */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/* 012_4096x2304_120fps_2976Msps */
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = cjtele2_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240 ,
		.framelength = 2564,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 57,
		},
	},
	{/* 01_4096x3072_30fps_2976Msps AEB*/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = cjtele2_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom1_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom1_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom1_setting),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 5128 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 3072,
		.read_margin = (4154 - 3072), /*lut_A_FLL_min=13.5ms*/
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFF - 0x28),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFF - 0x28),
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 256,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.csi_param = {
			.cphy_settle = 57,
		},
	},
	{/*06_4096*3072 30fps 2X2 AEB(AP merge)-izoom_QBC(AP RMSC)*/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = cjtele2_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom2_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom2_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom2_setting),
		.hdr_mode = HDR_RAW_LBMF,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 3205 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 3072,
		.read_margin = (3205 - 3072),
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* 2X crop 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.awb_enabled = 1,
		.exposure_order_in_lbmf = IMGSENSOR_LBMF_EXPOSURE_SE_FIRST,
		.mode_type_in_lbmf = IMGSENSOR_LBMF_MODE_MANUAL,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFF - 0x26),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFF - 0x26),
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 64,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.csi_param = {
			.cphy_settle = 57,
		},
	},
	{  /*02_4096*3072 30FPS iDCG(AP Merge) */
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = cjtele2_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom3_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom3_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom3_setting),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
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
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjtele2_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(cjtele2_dcg_ratio_table_10bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {78},
	},
	{ /* 03_8192*6144 2x2 Fullsize QBC (For 25M) */
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = cjtele2_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom4_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom4_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* FULL MODE 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-8192*2)/2,
			.y0_offset = (12288-6144*2)/2,
			.w0_size = 8192*2,
			.h0_size = 6144*2,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.csi_param = {
			.cphy_settle = 54,
		},
	},
	{ /* 04_4096*3072 30fps Fullsize-crop izoom-QBC(AP RMSC)*/
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = cjtele2_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom5_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom5_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom5_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 19968,
		.framelength = 3205,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096)/2,
			.y0_offset = (12288-3072)/2,
			.w0_size = 4096,
			.h0_size = 3072,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize,
		.csi_param = {
			.cphy_settle = 57,
		},
	},
	{ /* 05_16384x12288_7.5fps_2976Msps */
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = cjtele2_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom6_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom6_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom6_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 19968,
		.framelength = 12820,
		.max_framerate = 75,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46, /* HP9: full:0x46, 4Sum:0x26, 8Sum_2H1V:0x28 */
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 16384,
			.h0_size = 12288,
			.scale_w = 16384,
			.scale_h = 12288,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 16384,
			.h1_size = 12288,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 16384,
			.h2_tg_size = 12288,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{  /*08_4096x2560_30.01fps_2976Msps_iDCG*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = cjtele2_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom7_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_custom7_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom7_setting),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2560*4,
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
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjtele2_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(cjtele2_dcg_ratio_table_10bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{ /* 09_4096*2560 2X2 izoom-bayer */
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = cjtele2_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom8_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_custom8_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom8_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 6410,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* 2X crop 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-2560*2)/2,
			.w0_size = 4096*2,
			.h0_size = 2560*2,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/*010_4096*2560 Fullsize-crop izoom-QBC(AP RMSC)*/
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = cjtele2_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom9_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_custom9_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom9_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 19968,
		.framelength = 3205,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096)/2,
			.y0_offset = (12288-2560)/2,
			.w0_size = 4096,
			.h0_size = 2560,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize,
		.csi_param = {
			.cphy_settle = 56,
		},
	},
	{/*014_2048x1152_240.01fps_1760Msps*/
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = cjtele2_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 1282,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1206857142,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x18,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-2048*8)/2,
			.y0_offset = (12288-1152*8)/2,
			.w0_size = 2048*8,
			.h0_size = 1152*8,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 128,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_8Fdsum_4H2V,
		.csi_param = {
			.cphy_settle = 58,
		},
	},
	{/* 20_4096x2304_30fps_2976Msps */
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = cjtele2_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2304*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2304*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	},
	{/* 4096x3072_60fps_2976Msps */
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = cjtele2_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240,
		.framelength = 5128,
		.max_framerate = 600,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-3072*4)/2,
			.w0_size = 4096*4,
			.h0_size = 3072*4,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	},
	{  /*15_4096x3072_30fps_2976Msps FCM*/
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = cjtele2_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom13_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom13_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom13_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 5888,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_2X2, /* tetra */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-3072*2)/2,
			.w0_size = 4096*2,
			.h0_size = 3072*2,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
	},
	{  /*17_4096x2560_30.01fps_2976Msps 8Sum2Avg_iDCG1:4_HCGPD*/
		.frame_desc = frame_desc_cus14,
		.num_entries = ARRAY_SIZE(frame_desc_cus14),
		.mode_setting_table = cjtele2_custom14_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom14_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2560*4,
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
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjtele2_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(cjtele2_dcg_ratio_table_10bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFF - 0x26),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFF - 0x26),
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 128,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {78},
	},
	{  /*23_4912x3688_25fps_2976Msps 18M full crop*/
		.frame_desc = frame_desc_cus15,
		.num_entries = ARRAY_SIZE(frame_desc_cus15),
		.mode_setting_table = cjtele2_custom15_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom15_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = cjtele2_custom15_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom15_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 19968,
		.framelength = 3846,
		.max_framerate = 250,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x46, /* HP9: full:0x46, 4Sum:0x26, 8Sum_2H1V:0x28 */
		.sensor_output_dataformat_cell_type = SENSOR_OUTPUT_FORMAT_CELL_4X4, /* hexdeca */
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4912)/2,
			.y0_offset = (12288-3680)/2,
			.w0_size = 4912,
			.h0_size = 3680,
			.scale_w = 4912,
			.scale_h = 3680,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4912,
			.h1_size = 3680,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4912,
			.h2_tg_size = 3680,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize,
	},
	{  /*19_4096x2560_30.01fps_2976Msps 8Sum2Avg_iDCG1:8_HCGPD*/
		.frame_desc = frame_desc_cus16,
		.num_entries = ARRAY_SIZE(frame_desc_cus16),
		.mode_setting_table = cjtele2_custom16_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom16_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_custom16_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom16_setting),
		.hdr_mode = HDR_RAW_DCG_RAW,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.pclk = 1920000000,
		.linelength = 20376,
		.framelength = 3140,
		.max_framerate = 300,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x14,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*4)/2,
			.y0_offset = (12288-2560*4)/2,
			.w0_size = 4096*4,
			.h0_size = 2560*4,
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
		.saturation_info = &imgsensor_saturation_info,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_RAW,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_LCG_BASE,
			.dcg_gain_ratio_min = 8000,
			.dcg_gain_ratio_max = 8000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = cjtele2_dcg_ratio_table_10bit,
			.dcg_gain_table_size = sizeof(cjtele2_dcg_ratio_table_10bit),
		},
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = (0xFFFF - 0x26),
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = (0xFFFF - 0x26),
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 8,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 32,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {78},
	},
	{/*21_3840x2160_120fps_2976Msps*/
		.frame_desc = frame_desc_cus17,
		.num_entries = ARRAY_SIZE(frame_desc_cus17),
		.mode_setting_table = cjtele2_custom17_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom17_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 6240 ,
		.framelength = 2564,
		.max_framerate = 1200,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x28,
		.imgsensor_winsize_info = {
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-3840*4)/2,
			.y0_offset = (12288-2160*4)/2,
			.w0_size = 3840*4,
			.h0_size = 2160*4,
			.scale_w = 3840,
			.scale_h = 2160,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.coarse_integ_step = 2,
		.framelength_step = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
	},
	{ /* 22_4096x2304_60fps_2976Msps */
		.frame_desc = frame_desc_cus18,
		.num_entries = ARRAY_SIZE(frame_desc_cus18),
		.mode_setting_table = cjtele2_custom18_setting,
		.mode_setting_len = ARRAY_SIZE(cjtele2_custom18_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = cjtele2_custom18_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(cjtele2_custom18_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1920000000,
		.linelength = 9984,
		.framelength = 3205,
		.max_framerate = 600,
		.mipi_pixel_rate = 2040685714,
		.readout_length = 0,
		.read_margin = 0,
		.exposure_margin = 0x26,
		.imgsensor_winsize_info = { /* 2X crop 4cell */
			.full_w = 16384,
			.full_h = 12288,
			.x0_offset = (16384-4096*2)/2,
			.y0_offset = (12288-2304*2)/2,
			.w0_size = 4096*2,
			.h0_size = 2304*2,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.coarse_integ_step = 1,
		.framelength_step = 1,
		.awb_enabled = 1,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_fullsize_fdsum,
		/*.csi_param = {
			.cphy_settle = 56,
		},*/
	},
};

#ifdef CJTELE2_ISF_DBG
struct subdrv_static_ctx cjtele2_legacy_s_ctx = {
#else
static struct subdrv_static_ctx static_ctx = {
#endif
	.sensor_id = CJTELE2_SENSOR_ID,
	.reg_addr_sensor_id = { 0x0000, 0x0001 },
	.i2c_addr_table = {0x20, 0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),

	.mirror = IMAGE_H_MIRROR,

	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 256,
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 1,
	.ana_gain_table = cjtele2_ana_gain_table,
	.ana_gain_table_size = sizeof(cjtele2_ana_gain_table),
	.tuning_iso_base = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xFFFF - 0x46) << 7,
	.cit_lshift_max = 7,
	.exposure_step = 1,
	.exposure_margin = 0x28,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 1,
	.frame_length_max = 0xFFFF << 7,
	.frame_length_max_without_lshift = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1616100,
	.line_interleave_num = 1,
	/* stagger behavior by vendor type */
	.stagger_rg_order = IMGSENSOR_STAGGER_RG_SE_FIRST,
	.stagger_fl_type = IMGSENSOR_STAGGER_FL_MANUAL,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.saturation_info = &imgsensor_saturation_info,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101, /*general_setup_image_orientation*/
	.reg_addr_frame_length = {0x0340, 0x0341}, /*frame_length_lines*/
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020, /*General temperature*/
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005, /*api_rd_general_frame_count*/
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0704, /*coarse_integration_time_sh*/
	.reg_addr_frame_length_lshift = 0x0702, /*frame_length_lines_sh*/

	/*datasheet search "AE control guide"*/
	.reg_addr_exposure = {
		{ 0x0202, 0x0203 },
		{ 0x022C, 0x022D },
		{ 0x0226, 0x0227 },
	},
	.reg_addr_ana_gain = {
		{ 0x0204, 0x0205 },
		{ 0x0208, 0x0209 },
		{ 0x0206, 0x0207 },
	},

	/*datasheet search "Example of AEB Control"*/
	.reg_addr_exposure_in_lut = {
			{0x0E10, 0x0E11}, /* LUT_A_COARSE_INTEG_TIME */
			{0x0E1E, 0x0E1F}, /* LUT_B_COARSE_INTEG_TIME */
	},
	.reg_addr_ana_gain_in_lut = {
			{0x0E12, 0x0E13}, /* LUT_A_ANA_GAIN_GLOBAL */
			{0x0E20, 0x0E21}, /* LUT_B_ANA_GAIN_GLOBAL */
	},
	// .reg_addr_dig_gain_in_lut = {
	// 		{0x0E14, 0x0E15},
	// 		{0x0E20, 0x0E21},
	// },
	.reg_addr_frame_length_in_lut = {
			{0x0E14, 0x0E15},
			{0x0E22, 0x0E23},
	},

	// .init_setting_table = cjtele2_init_setting,
	// .init_setting_len = ARRAY_SIZE(cjtele2_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.chk_streaming_st = cjtele2_chk_streaming_st,

	.checksum_value = 0xc88841a,
	.ctle_param = &cjtele2_static_ctle_param,
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
	.update_sof_cnt = common_update_sof_cnt,
	.vsync_notify = vsync_notify,
	.set_ctrl_locker = set_ctrl_locker,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, {24}, 10000},
	{HW_ID_RST, {0}, 1000},
	{HW_ID_DOVDD, {1800000, 1800000}, 1000},
	{HW_ID_AVDD, {2204000, 2204000}, 1000},
	{HW_ID_DVDD, {1008000, 1008000}, 1000},
	{HW_ID_AFVDD, {3100000, 3100000}, 3000},
	{HW_ID_RST, {1}, 5000},
	{HW_ID_MCLK_DRIVING_CURRENT, {6}, 20000},
};

#ifdef CJTELE2_ISF_DBG
const struct subdrv_entry cjtele2_mipi_raw_entry_legacy = {
#else
const struct subdrv_entry cjtele2_mipi_raw_entry = {
#endif
	.name = "cjtele2_mipi_raw",
	.id = CJTELE2_SENSOR_ID, /* 0x1b73 */
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt, u64 sof_ts)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		// set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u16 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_ixc_rd_u16(ctx, ctx->s_ctx.reg_addr_temp_read);

	temperature_convert = (temperature>>8)&0xFF;

	temperature_convert = (temperature_convert > 100) ? 100: temperature_convert;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static unsigned int read_cjtele2_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != cjtele2_eeprom_info[meta_id].meta)
		return -1;

	if (size != cjtele2_eeprom_info[meta_id].size)
		return -1;

	addr = cjtele2_eeprom_info[meta_id].start;
	readsize = cjtele2_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static struct eeprom_addr_table_struct oplus_eeprom_addr_table = {
	.i2c_read_id = 0xA1,
	.i2c_write_id = 0xA0,

	.addr_modinfo = 0x0000,
	.addr_sensorid = 0x0006,
	.addr_lens = 0x0008,
	.addr_vcm = 0x000A,
	.addr_modinfoflag = 0x0010,

	.addr_af = 0x0092,
	.addr_afmacro = 0x0092,
	.addr_afinf = 0x0094,
	.addr_afflag = 0x009A,

	.addr_qrcode = 0x00B0,
	.addr_qrcodeflag = 0x00C7,
};

static struct oplus_eeprom_info_struct  oplus_eeprom_info = {0};

static int get_eeprom_common_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	memcpy(para, (u8*)(&oplus_eeprom_info), sizeof(oplus_eeprom_info));
	*len = sizeof(oplus_eeprom_info);
	return 0;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, CJTELE2_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH     (32)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, CJTELE2_EEPROM_WRITE_ID >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xA000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, CJTELE2_EEPROM_WRITE_ID >> 1, reg, 0x0E);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, CJTELE2_EEPROM_WRITE_ID >> 1, reg, 0x00);
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
    if (pStereodata->uSensorId != CJTELE2_SENSOR_ID) {
        LOG_INF("Invalid Sensor id:0x%x write eeprom, Real Sensor id:0x%x\n", 
            pStereodata->uSensorId, CJTELE2_SENSOR_ID);
    }
    if (pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x dev(%d) Addr(0x%x) Len(%d)\n",
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
        if (((data_length - 2 == CALI_DATA_SLAVE_TELE_LENGTH) && (data_base == CJTELE2_STEREO_START_ADDR))
            || ((data_length - 2 == CALI_DATA_SLAVE_TELE_LENGTH) && (data_base == CJTELE2_STEREO_105CM_START_ADDR))) {
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
            for (i = 0; i < idx; i++ ) {
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
        } else if ((data_length < AESYNC_DATA_LENGTH_TOTAL) && (data_base == CJTELE2_AESYNC_START_ADDR)) {
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
            for (i = 0; i < idx; i++ ) {
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
            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+1),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+2),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+3),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+4),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+5),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+6),
                read_cmos_eeprom_8(ctx, CJTELE2_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("EEprom addr or len is invalid, please check\n");
            return -1;
        }
    } else {
        LOG_INF("cjtele2 write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static int cjtele2_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
    return 0;
}

static int cjtele2_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	switch (*feature_data_16) {
	case EEPROM_STEREODATA_MT_MAIN:
		read_cjtele2_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MT_MAIN_105CM:
		read_cjtele2_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA_105CM,
				(BYTE *)feature_return_para_32, *len);
		break;
	default:
		DRV_LOG_MUST(ctx, "error meta_id (%d), only support EEPROM_STEREODATA_MT_MAIN, EEPROM_STEREODATA_MT_MAIN_105CM",
			*feature_data_16);
		break;
	}
	return 0;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, CJTELE2_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static int cjtele2_get_otp_qcom_pdaf_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_DATA_START_ADDR, otp_qcom_pdaf_data, OTP_QCOM_PDAF_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_data, sizeof(otp_qcom_pdaf_data));
	*len = sizeof(otp_qcom_pdaf_data);

	return 0;
}

static int cjtele2_get_otp_qcom_pdaf_offset_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;

	read_cmos_eeprom_p8(ctx, OTP_QCOM_PDAF_OFFSET_DATA_START_ADDR, otp_qcom_pdaf_offset_data, OTP_QCOM_PDAF_OFFSET_DATA_LENGTH);

	memcpy(feature_return_para_32, (UINT32 *)otp_qcom_pdaf_offset_data, sizeof(otp_qcom_pdaf_offset_data));
	*len = sizeof(otp_qcom_pdaf_offset_data);

	return 0;
}

static int cjtele2_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D82, awb_gain->ABS_GAIN_R * 2); //red 1024(1x)
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D86, awb_gain->ABS_GAIN_B * 2); //blue

	LOG_INF("[test] ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)", awb_gain->ABS_GAIN_GR, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B, awb_gain->ABS_GAIN_GB);
	LOG_INF("[test] 0x0D82(red) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D82));
	LOG_INF("[test] 0x0D84(green) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D84));
	LOG_INF("[test] 0x0D86(blue) = (0x%x)", subdrv_i2c_rd_u16(ctx, 0x0D86));
	return 0;
}

static int cjtele2_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if (!ctx || !para || !len)
		return -1;

	u16 *framerate = (u16 *)para;
	set_max_framerate(ctx, *framerate, 1);
	set_auto_flicker(ctx, 1);
	set_dummy(ctx);
	DRV_LOG(ctx, "fps(input/max):%u/%u\n", *framerate, ctx->current_fps);

	return 0;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "jn1 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "jn1 read_otp_info end\n");
}

static int cjtele2_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int cjtele2_get_otp_initsetting_info(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_data_8 = (u32 *) para;
	u32 module_info = 0;
	module_info = subdrv_i2c_rd_u16(ctx, 0x0010);
	DRV_LOG(ctx, "get otp initsetting info. module_info (0x%x)\n", module_info);
	if ((module_info | 0x000F) != 0x024F && (module_info | 0x000F) != 0x02FF) {
		feature_data_8[0] = 0;
	} else {
		feature_data_8[0] = 1;
	}
	DRV_LOG(ctx, "get otp initsetting info. whether burn (0x%x)\n", feature_data_8[0]);
	return 0;
}

static int cjtele2_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
	return 0;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = GET_SENSOR_ID_RETRY_CNT;
	static bool first_read = TRUE;
	u32 eeprom_time_year = 0, eeprom_time_m_d = 0;
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
			if (*sensor_id == 0x1B75) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx, &oplus_eeprom_info, oplus_eeprom_addr_table);

					first_read = FALSE;

					msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
					if(!msg_buf) {
						LOG_INF("boot stage, malloc msg_buf error");
					}
				}
				eeprom_time_year = (read_cmos_eeprom_8(ctx, 0x0004) << 8) | read_cmos_eeprom_8(ctx, 0x0005);
				eeprom_time_m_d = (read_cmos_eeprom_8(ctx, 0x0003) << 8) | read_cmos_eeprom_8(ctx, 0x0002);
				// sensor with eeprom data since 2024/01/27
				module_flag = (eeprom_time_year > 0x1814) || ((eeprom_time_year == 0x1814) && (eeprom_time_m_d >= 0x11B));
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

static void cjtele2_write_init_setting(struct subdrv_ctx *ctx)
{
	u32 module_info = 0;
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	module_info = subdrv_i2c_rd_u16(ctx, 0x0010);
	DRV_LOG_MUST(ctx, "module_info (0x%x) write init setting +", module_info);

	u64 time_boot_begin = 0;
	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en))
		time_boot_begin = ktime_get_boottime_ns();

	subdrv_i2c_wr_regs_u16(ctx, cjtele2_init_setting_part1, ARRAY_SIZE(cjtele2_init_setting_part1));
	mdelay(1);
	subdrv_i2c_wr_regs_u16(ctx, cjtele2_init_setting_part2, ARRAY_SIZE(cjtele2_init_setting_part2));
	mdelay(20);

	if ((module_info | 0x000F) != 0x024F && (module_info | 0x000F) != 0x02FF) {
		mdelay(20);
		cjtele2_i2c_burst_wr_regs_u16(ctx, cjtele2_init_setting_part3_long,
										ARRAY_SIZE(cjtele2_init_setting_part3_long));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(cjtele2_init_setting_part1)
										+ ARRAY_SIZE(cjtele2_init_setting_part2)
										+ ARRAY_SIZE(cjtele2_init_setting_part3_long);
	} else {
		mdelay(1);
		cjtele2_i2c_burst_wr_regs_u16(ctx, cjtele2_init_setting_part3_short,
										ARRAY_SIZE(cjtele2_init_setting_part3_short));
		ctx->s_ctx.init_setting_len = ARRAY_SIZE(cjtele2_init_setting_part1)
										+ ARRAY_SIZE(cjtele2_init_setting_part2)
										+ ARRAY_SIZE(cjtele2_init_setting_part3_short);
	}

	if ((ctx->power_on_profile_en != NULL) && (*ctx->power_on_profile_en)) {
		ctx->sensor_pw_on_profile.i2c_init_period = ktime_get_boottime_ns() - time_boot_begin;
		ctx->sensor_pw_on_profile.i2c_init_table_len = ctx->s_ctx.init_setting_len;
	}
	DRV_LOG_MUST(ctx, "write init setting -, size:%u, time(us):%lld\n", ctx->sensor_pw_on_profile.i2c_init_table_len,
		ctx->sensor_pw_on_profile.i2c_init_period);
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	cjtele2_write_init_setting(ctx);

	if (ctx->s_ctx.temperature_support && ctx->s_ctx.reg_addr_temp_en)
		subdrv_ixc_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	/* enable mirror or flip */
	set_mirror_flip(ctx, ctx->mirror);

	/* HW GGC*/
	set_sensor_cali(ctx);

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

static bool gph_enable = false;

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	if (group_hold_frame_count < 6) {
		DRV_LOGE(ctx, "group_hold_frame_count: %d", group_hold_frame_count);
		group_hold_frame_count++;
		return;
	}

	DRV_LOG(ctx, "CJTELE2 gph en: %d  gph_enable: %d", en, gph_enable);
	if (en && !gph_enable) {
		set_i2c_buffer(ctx, 0x0104, 0x01);
		gph_enable = true;
	} else if (!en && gph_enable) {
		set_i2c_buffer(ctx, 0x0104, 0x00);
		gph_enable = false;
	}
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int cjtele2_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	group_hold_frame_count = 0;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;
	u32 after_seamless_linetime_in_ns = 0;
	u16 preshutter = 0;

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
	after_seamless_linetime_in_ns = ((u64)ctx->s_ctx.mode[scenario_id].linelength * 1000000000)
									/ ctx->s_ctx.mode[scenario_id].pclk;
	preshutter = 10000000 / after_seamless_linetime_in_ns; /*10ms*/
	DRV_LOG(ctx, "tline:%u, preshutter:%u\n", after_seamless_linetime_in_ns, preshutter);

	subdrv_ixc_wr_u8(ctx, 0x0104, 0x01);

	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_ixc_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
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
			if (ae_ctrl->exposure.me_exposure < preshutter ) {
				preshutter = (preshutter - ae_ctrl->exposure.me_exposure) * after_seamless_linetime_in_ns / 1000;
				DRV_LOG(ctx, "+ LBMF preshutter:%u\n", preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x2003);
				subdrv_ixc_wr_u16(ctx, 0x0C8A, preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x4000);
			}
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			if (ae_ctrl->exposure.le_exposure < preshutter ) {
				preshutter = (preshutter - ae_ctrl->exposure.le_exposure) * after_seamless_linetime_in_ns / 1000;
				DRV_LOG(ctx, "+ default preshutter:%d\n", preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x2003);
				subdrv_ixc_wr_u16(ctx, 0x0C8A, preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x4000);
			}
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, 1, 0);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.me_gain);
			break;
		default:
			if (ae_ctrl->exposure.le_exposure < preshutter ) {
				preshutter = (preshutter - ae_ctrl->exposure.le_exposure) * after_seamless_linetime_in_ns / 1000;
				DRV_LOG(ctx, "+ default preshutter:%d\n", preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x2003);
				subdrv_ixc_wr_u16(ctx, 0x0C8A, preshutter);
				subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x4000);
			}
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, 1, 0);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	ctx->ae_ctrl_gph_en = 0;
	commit_i2c_buffer(ctx);
	subdrv_ixc_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int cjtele2_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
		if (mode == 5) {
			subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*black*/
			subdrv_i2c_wr_u16(ctx, 0x0620, 0x0001); /*black*/
			subdrv_i2c_wr_u16(ctx, 0x0636, 0x0001); /*black*/
		} else {
			subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
			subdrv_i2c_wr_u16(ctx, 0x0620, mode); /*100% Color bar*/
			subdrv_i2c_wr_u16(ctx, 0x0636, mode); /*100% Color bar*/
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/
		subdrv_i2c_wr_u16(ctx, 0x0620, 0x0000); /*No pattern*/
		subdrv_i2c_wr_u16(ctx, 0x0636, 0x0000); /*No pattern*/
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

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	// struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	ctx->is_read_preload_eeprom = 1;
}

static void set_sensor_cali(void *arg)
{
	//struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	return;
}

static bool dump_i2c_enable = false;

static void dump_i2c_buf(struct subdrv_ctx *ctx, u8 * buf, u32 length)
{
	int i;
	char *out_str = NULL;
	char *strptr = NULL;
	size_t buf_size = SUBDRV_I2C_BUF_SIZE * sizeof(char);
	size_t remind = buf_size;
	int num = 0;

	out_str = kzalloc(buf_size + 1, GFP_KERNEL);
	if (!out_str)
		return;

	strptr = out_str;
	memset(out_str, 0, buf_size + 1);

	num = snprintf(strptr, remind,"[ ");
	remind -= num;
	strptr += num;

	for (i = 0 ; i < length; i ++) {
		num = snprintf(strptr, remind,"0x%02x, ", buf[i]);

		if (num <= 0) {
			DRV_LOG(ctx, "snprintf return negative at line %d\n", __LINE__);
			kfree(out_str);
			return;
		}

		remind -= num;
		strptr += num;

		if (remind <= 20) {
			DRV_LOG(ctx, " write %s\n", out_str);
			memset(out_str, 0, buf_size + 1);
			strptr = out_str;
			remind = buf_size;
		}
	}

	num = snprintf(strptr, remind," ]");
	remind -= num;
	strptr += num;

	DRV_LOG(ctx, " write %s\n", out_str);
	strptr = out_str;
	remind = buf_size;

	kfree(out_str);
}

static int cjtele2_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	int retry = 3;
	int ret = 0;
	while (retry > 0) {
		ret = adapter_i2c_burst_wr_regs_u16(ctx, ctx->i2c_write_id >> 1, list, len);
		if (ret == 0) {
			break;
		}
		DRV_LOGE(ctx, "retry write init setting!");
		retry--;
	}
	return ret;
}

/* #define MAX_BUF_SIZE  4096
#define MAX_MSG_NUM_U16  MAX_BUF_SIZE/4

 struct cache_wr_regs_u16 {
	struct i2c_msg msg[MAX_MSG_NUM_U16];
}; */

static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx ,
		u16 addr, u16 *list, u32 len)
{
	struct i2c_client *i2c_client = ctx->i2c_client;
	struct i2c_msg  msg;
	struct i2c_msg *pmsg = &msg;

	u8 *pbuf = NULL;
	u16 *plist = NULL;
	u16 *plist_end = NULL;

	u32 sent = 0;
	u32 total = 0;
	u32 per_sent = 0;
	int ret, i;

	if(!msg_buf) {
		LOG_INF("malloc msg_buf retry");
		msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
		if(!msg_buf) {
			LOG_INF("malloc error");
			return -ENOMEM;
		}
	}

	/* each msg contains addr(u16) + val(u16 *) */
	sent = 0;
	total = len / 2;
	plist = list;
	plist_end = list + len - 2;

	DRV_LOG(ctx, "len(%u)  total(%u)", len, total);

	while (sent < total) {

		per_sent = 0;
		pmsg = &msg;
		pbuf = msg_buf;

		pmsg->addr = addr;
		pmsg->flags = i2c_client->flags;
		pmsg->buf = pbuf;

		pbuf[0] = plist[0] >> 8;    //address
		pbuf[1] = plist[0] & 0xff;

		pbuf[2] = plist[1] >> 8;  //data 1
		pbuf[3] = plist[1] & 0xff;

		pbuf += 4;
		pmsg->len = 4;
		per_sent += 1;

		for (i = 0; i < total - sent - 1; i++) {  //Maximum number of remaining cycles - 1
			if(plist[0] + 2 == plist[2] ) {  //Addresses are consecutive
				pbuf[0] = plist[3] >> 8;
				pbuf[1] = plist[3] & 0xff;

				pbuf += 2;
				pmsg->len += 2;
				per_sent += 1;
				plist += 2;

				if(pmsg->len >= MAX_BURST_LEN) {
					break;
				}
			}
		}
		plist += 2;

		if(dump_i2c_enable) {
			DRV_LOG(ctx, "pmsg->len(%d) buff: ", pmsg->len);
			dump_i2c_buf(ctx, msg_buf, pmsg->len);
		}

		ret = i2c_transfer(i2c_client->adapter, pmsg, 1);

		if (ret < 0) {
			dev_info(&i2c_client->dev,
				"i2c transfer failed (%d)\n", ret);
			return -EIO;
		}

		sent += per_sent;

		DRV_LOG(ctx, "sent(%u)  total(%u)  per_sent(%u)", sent, total, per_sent);
	}

	return 0;
}

static int cjtele2_chk_streaming_st(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int ret = ERROR_NONE;
	u32 i = 0, framecnt = 0;
	int timeout = ctx->current_fps ? (10000 / ctx->current_fps) + 1 : 101;

	if (!ctx->s_ctx.reg_addr_frame_count)
		return ret;
	subdrv_ixc_wr_u16(ctx, 0xFCFC, 0x4000);
	for (i = 0; i < timeout; i++) {
		framecnt = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_frame_count);
		DRV_LOG(ctx, "check_stream_on delay cnt:%d, framecnt:%d\n", i, framecnt);
		if (framecnt != 0xFF)
			return ret;
		mdelay(1);
	}
	DRV_LOGE(ctx, "stream on fail!\n");
	return ret;
}
