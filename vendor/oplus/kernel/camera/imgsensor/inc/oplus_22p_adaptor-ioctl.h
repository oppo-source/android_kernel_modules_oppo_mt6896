/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 MediaTek Inc. */

#ifndef __OPLUS_22P_ADAPTOR_IOCTL_H__
#define __OPLUS_22P_ADAPTOR_IOCTL_H__
static int g_get_unique_sensorid(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_get_unique_sensorid *data = arg;
	struct workbuf workbuf;
	int ret;

	ret = workbuf_get(&workbuf, data->p_buf, data->size,  F_ZERO | F_WRITE);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_UNIQUE_SENSORID,
		workbuf.kbuf, &(data->size));
	if (ret)
		return ret;

	ret = workbuf_put(&workbuf);
	if (ret)
		return ret;

	return 0;
}
static int g_get_camera_eeprom_common_data(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_get_eeprom_common_data *temp = arg;
	unsigned int  arg_length = sizeof(struct oplus_get_eeprom_common_data);
	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_EEPROM_COMDATA,
		temp->header, &arg_length);
	return 0;
}

static int g_get_camerasn(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_get_camera_sn *temp = arg;
	subdrv_call(ctx, feature_control,
	SENSOR_FEATURE_GET_EEPROM_COMDATA,
	temp->data,&(temp->len));
	return 0;
}


static int g_get_stereo_data(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_calc_eeprom_info *info = arg;
	struct workbuf workbuf;
	int ret;

	ret = workbuf_get(&workbuf, info->p_buf, info->size, F_READ | F_WRITE);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_EEPROM_STEREODATA,
		workbuf.kbuf, &(info->size));
	if (ret)
		return ret;

	ret = workbuf_put(&workbuf);
	if (ret)
		return ret;

	return 0;
}

static int g_set_cali_data(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_set_cali_data *info = arg;
	struct workbuf workbuf;
	int ret;
	ret = workbuf_get(&workbuf, info->p_buf, (info->size) + 2, F_READ);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_SET_CALI_DATA,
		workbuf.kbuf, &info->size);
	if (ret)
		return ret;

	ret = workbuf_put(&workbuf);
	if (ret)
		return ret;

	return 0;
}

static int g_cloud_otp_info(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_cloud_otp_info *info = arg;
	union feature_para para;
	u32 len;
	struct SENSOR_OTP_INFO_STRUCT cloudinfo;
	para.u64[0] = info->otp_type;
	para.u64[1] = (u64)&cloudinfo;
	memset(&cloudinfo, 0, sizeof(cloudinfo));

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_CLOUD_OTP_INFO,
		para.u8, &len);
	if (copy_to_user((void *)info->p_cloud_info, &cloudinfo, sizeof(cloudinfo)))
		return -EFAULT;
	return 0;
}

static int g_get_eeprom_probe_state(struct adaptor_ctx *ctx, void *arg)
{
	int ret;
	u32 len;
	int *eeprom_probe_state = arg;
	union feature_para para;
	para.u32[0] = 0;
	ret = subdrv_call(ctx, feature_control, SENSOR_FEATURE_GET_EEPROM_PROBE_STATE, para.u8, &len);
	*eeprom_probe_state = para.u32[0];

	if (ret)
		return ret;

	return 0;
}

static int g_get_otp_data(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_calc_eeprom_info *info = arg;
	struct workbuf workbuf;
	int ret;

	ret = workbuf_get(&workbuf, info->p_buf, info->size, F_ZERO | F_WRITE);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_SENSOR_OTP_ALL,
		workbuf.kbuf, &(info->size));
	if (ret)
		return ret;

	ret = workbuf_put(&workbuf);
	if (ret)
		return ret;

	return 0;
}

static int g_get_distortionparams_data(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_distortion_data *data = arg;
	struct workbuf workbuf;
	int ret;

	ret = workbuf_get(&workbuf, data->p_buf, data->size,  F_ZERO | F_WRITE);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_DISTORTIONPARAMS,
		workbuf.kbuf, &(data->size));
	if (ret)
		return ret;

	ret = workbuf_put(&workbuf);
	if (ret)
		return ret;

	return 0;
}

static int s_calibration_eeprom(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_calc_eeprom_info *info = arg;
	struct workbuf workbuf;
	int ret;

	ret = workbuf_get(&workbuf, info->p_buf, info->size, F_READ);
	if (ret)
		return ret;

	ret = subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_SET_SENSOR_OTP,
		workbuf.kbuf, &info->size);
	if (ret)
		return ret;

	return 0;
}


static int g_sensor_setting_info(struct adaptor_ctx *ctx, void *arg)
{
	struct oplus_sensor_setting_info *info = arg;
	union feature_para para;
	u32 len;
	struct SENSOR_SETTING_INFO_STRUCT sensorinfo;
	para.u64[0] = info->scenario_id;
	para.u64[1] = (u64)&sensorinfo;
	memset(&sensorinfo, 0, sizeof(sensorinfo));

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_SENSOR_SETTING_INFO,
		para.u8, &len);
	if (copy_to_user((void *)info->p_sensor_info, &sensorinfo, sizeof(sensorinfo)))
		return -EFAULT;
	return 0;
}

static const struct ioctl_entry oplus_ioctl_list[] = {
	{VIDIOC_MTK_S_CALIBRATION_EEPROM, s_calibration_eeprom},
	{VIDIOC_MTK_S_CALI_DATA, g_set_cali_data},
	{VIDIOC_MTK_G_CAMERA_SN,g_get_camerasn},
	{VIDIOC_MTK_G_STEREO_DATA, g_get_stereo_data},
	{VIDIOC_MTK_G_OTP_DATA, g_get_otp_data},
	{VIDIOC_MTK_G_DISTORTIONPARAMS_DATA, g_get_distortionparams_data},
	{VIDIOC_MTK_G_CAMERA_EEPROM_COMMON, g_get_camera_eeprom_common_data},
	{VIDIOC_MTK_G_SENSOR_SETTING_INFO, g_sensor_setting_info},
	{VIDIOC_MTK_G_UNIQUE_SENSORID, g_get_unique_sensorid},
	{VIDIOC_MTK_G_EEPROM_PROBE_STATE, g_get_eeprom_probe_state},
	{VIDIOC_MTK_G_CLOUD_OTP_INFO, g_cloud_otp_info},
};

void oplus_22p_adaptor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg, int* ret)
{
	int i;
	struct adaptor_ctx *ctx = sd_to_ctx(sd);
	if (*ret != -ENOIOCTLCMD) {
		return;
	}
	/* dispatch ioctl request */
	for (i = 0; i < ARRAY_SIZE(oplus_ioctl_list); i++) {
		if (oplus_ioctl_list[i].cmd == cmd) {
			*ret = oplus_ioctl_list[i].func(ctx, arg);
			break;
		}
	}
}

#endif /* __OPLUS_ADAPTOR_IOCTL_H__ */
