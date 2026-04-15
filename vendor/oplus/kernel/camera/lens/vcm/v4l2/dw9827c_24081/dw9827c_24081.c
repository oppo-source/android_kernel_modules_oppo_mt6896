// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <soc/oplus/system/oplus_project.h>

#define DRIVER_NAME "dw9827c_24081"
#define DW9827C_I2C_SLAVE_ADDR 0x18
#define DW9827C_PID_VERSION_ADDR 0x7E

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9827C_NAME				"dw9827c_24081"
#define DW9827C_MAX_FOCUS_POS			1023
#define DW9827C_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9827C_FOCUS_STEPS			1
#define DW9827C_SET_POSITION_ADDR		0x00

#define DW9827C_CMD_DELAY			0xff
#define DW9827C_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9827C_MOVE_STEPS			100
#define DW9827C_MOVE_DELAY_US			1000
#define DW9827C_INIT_DELAY_US			3000

#define DW9827C_MOVE_INIT_POS			230

/* dw9827c device structure */
struct dw9827c_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9827c_init_regs[] = {
	{0x02, 0x40, 1},
	{0x04, 0x01, 1},
	{0x00, ((DW9827C_MOVE_INIT_POS << 6) >> 8) & 0xff, 0},
	{0x01, (DW9827C_MOVE_INIT_POS << 6) & 0xff       , 0},
	{0x02, 0x00, 1},
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 4)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 5)

#define DW9827C_CURRENT_PID_VERSION 0x03
static struct regval_list dw9827c_pid[] = {
	{0x02, 0x40, 0},
	{0x34, 0x85, 0},
	{0x45, 0x4F, 0},
	{0x48, 0x10, 0},
	{0x49, 0x00, 0},
	{0x4A, 0x13, 0},
	{0x4B, 0x88, 0},
	{0x4C, 0x00, 0},
	{0x4D, 0xE6, 0},
	{0x4E, 0x17, 0},
	{0x4F, 0x70, 0},
	{0x50, 0x0D, 0},
	{0x51, 0xB4, 0},
	{0x52, 0x0B, 0},
	{0x53, 0xFC, 0},
	{0x54, 0x55, 0},
	{0x74, 0x03, 0},
	{0x7E, 0x03, 0},
	{0x80, 0xFD, 0},
	{0x82, 0x0E, 0},
	{0x83, 0x8A, 0},
	{0x86, 0x7D, 0},
	{0x87, 0x21, 0},
	{0x88, 0xC2, 0},
	{0x89, 0x1A, 0},
	{0x8A, 0x3F, 0},
	{0x8B, 0x44, 0},
	{0x8C, 0x82, 0},
	{0x8D, 0xDF, 0},
	{0x8E, 0x3E, 0},
	{0x8F, 0xA2, 0},
	{0x90, 0x00, 0},
	{0x91, 0x00, 0},
	{0x92, 0x00, 0},
	{0x93, 0x00, 0},
	{0x94, 0x00, 0},
	{0x95, 0x00, 0},
	{0x96, 0x00, 0},
	{0x97, 0x0A, 0},
	{0x98, 0x5A, 0},
	{0x99, 0x07, 0},
	{0x9A, 0x10, 0},
	{0x9B, 0x14, 0},
	{0x9C, 0xFF, 0},
	{0x9D, 0x00, 0},
	{0x9E, 0x18, 0},
	{0x9F, 0xDC, 0},
	{0xA0, 0xE6, 0},
	{0xA1, 0x00, 0},
	{0xA2, 0x80, 0},
	{0xA3, 0x80, 0},
	{0xA8, 0x06, 0},
	{0xA9, 0x0A, 0},
	{0xAA, 0x1E, 0},
	{0xAB, 0x00, 0},
	{0xAC, 0x10, 0},
};

extern unsigned int get_project(void);

static inline struct dw9827c_device *to_dw9827c_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9827c_device, ctrls);
}

static inline struct dw9827c_device *sd_to_dw9827c_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9827c_device, sd);
}

static int dw9827c_set_position(struct dw9827c_device *dw9827c, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);
	int retry = 3;
	int ret;
	int pos = -1;

	pos = i2c_smbus_read_word_data(client, 0x84);
	LOG_INF("dw9827c last postition:%d.", pos);

	LOG_INF("dw9827c Set postition:%u.", val);
	while (--retry > 0) {
        ret = i2c_smbus_write_word_data(client, DW9827C_SET_POSITION_ADDR,
                                        swab16(val << 6));
		if (ret < 0) {
			usleep_range(DW9827C_MOVE_DELAY_US,
				     DW9827C_MOVE_DELAY_US + 1000);
			LOG_INF("dw9827c Set postition:%u fail.", val);
		} else {
			break;
		}
	}
	return ret;
}

static int dw9827c_release(struct dw9827c_device *dw9827c)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);

	if (!dw9827c->active) {
		return 0;
	}
	pr_info("%s +\n", __func__);
	diff_dac = DW9827C_ORIGIN_FOCUS_POS - dw9827c->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9827C_MOVE_STEPS;

	val = dw9827c->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9827C_MOVE_STEPS*(-1)) : DW9827C_MOVE_STEPS);

		ret = dw9827c_set_position(dw9827c, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9827C_MOVE_DELAY_US,
			     DW9827C_MOVE_DELAY_US + 10);
	}

	// last step to origin
	ret = dw9827c_set_position(dw9827c, DW9827C_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9827c->active = false;

	pr_info("%s -\n", __func__);

	return 0;
}


static int dw9827c_write_array(struct dw9827c_device *dw9827c,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);

	for (i = 0; i < len; i++) {
		LOG_INF("write [0x%x, 0x%x, %d]", vals[i].reg_num, vals[i].value, vals[i].delay);
		ret = i2c_smbus_write_byte_data(client, vals[i].reg_num, vals[i].value);
		if (ret < 0)
			return ret;
		if(vals[i].delay) {
			usleep_range(vals[i].delay * 1000, vals[i].delay * 1000 + 1000);
		}
	}

	return 0;
}

static int dw9827c_init(struct dw9827c_device *dw9827c)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);
	client->addr = DW9827C_I2C_SLAVE_ADDR >> 1;

	if (dw9827c->active) {
		return 0;
	}
	pr_info("%s +\n", __func__);
	ret = dw9827c_write_array(dw9827c, dw9827c_init_regs,
                    ARRAY_SIZE(dw9827c_init_regs));
	usleep_range(1000, 1000 + 1000);
	LOG_INF("dw9827c_init position: %d", dw9827c->focus->val);
	ret = dw9827c_set_position(dw9827c, dw9827c->focus->val);

	dw9827c->active = true;

	pr_info("%s -\n", __func__);

	return 0;
}

/* Power handling */
static int dw9827c_power_off(struct dw9827c_device *dw9827c)
{
	int ret;

	LOG_INF("+\n");

	ret = dw9827c_release(dw9827c);
	if (ret)
		LOG_INF("dw9827c release failed!\n");

	ret = regulator_disable(dw9827c->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9827c->vdd);
	if (ret)
		return ret;

	if (dw9827c->vcamaf_pinctrl && dw9827c->vcamaf_off)
		ret = pinctrl_select_state(dw9827c->vcamaf_pinctrl,
					dw9827c->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9827c_power_on(struct dw9827c_device *dw9827c)
{
	int ret;

	LOG_INF("+\n");

	ret = regulator_set_voltage(dw9827c->vin, 3300000, 3300000);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9827c->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9827c->vdd);
	if (ret < 0)
		return ret;

	if (dw9827c->vcamaf_pinctrl && dw9827c->vcamaf_on)
		ret = pinctrl_select_state(dw9827c->vcamaf_pinctrl,
					dw9827c->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9827C_CTRL_DELAY_US, DW9827C_CTRL_DELAY_US + 100);

	ret = dw9827c_init(dw9827c);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9827c->vin);
	regulator_disable(dw9827c->vdd);
	if (dw9827c->vcamaf_pinctrl && dw9827c->vcamaf_off) {
		pinctrl_select_state(dw9827c->vcamaf_pinctrl,
				dw9827c->vcamaf_off);
	}

	return ret;
}

static int dw9827c_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9827c_device *dw9827c = to_dw9827c_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9827c_set_position(dw9827c, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9827c_vcm_ctrl_ops = {
	.s_ctrl = dw9827c_set_ctrl,
};

static int dw9827c_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9827c_device *dw9827c = sd_to_dw9827c_vcm(sd);

	ret = dw9827c_power_on(dw9827c);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9827c_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9827c_device *dw9827c = sd_to_dw9827c_vcm(sd);

	dw9827c_power_off(dw9827c);

	return 0;
}

static int dw9827c_vcm_resume(struct dw9827c_device *dw9827c)
{
    int ret = 0;
    ret = dw9827c_init(dw9827c);
    pr_info("%s exit stand by mode, active:%d\n", __func__, dw9827c->active);
    return ret;
}

static int dw9827c_vcm_suspend(struct dw9827c_device *dw9827c)
{
    int ret = 0;
    ret = dw9827c_release(dw9827c);
    pr_info("%s entry stand by mode, active:%d\n", __func__, dw9827c->active);
    return ret;
}

static long dw9827c_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9827c_device *dw9827c = sd_to_dw9827c_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);
	client->addr = DW9827C_I2C_SLAVE_ADDR >> 1;
	pr_info("%s +\n", __func__);

	switch (cmd) {
	case VCM_IOC_POWER_ON:
        ret = dw9827c_vcm_resume(dw9827c);
        pr_info("%s VCM_IOC_POWER_ON, cmd:%d, ret: %d\n", __func__, cmd, ret);
        break;
    case VCM_IOC_POWER_OFF:
        ret = dw9827c_vcm_suspend(dw9827c);
        pr_info("%s VCM_IOC_POWER_OFF, cmd:%d, ret:%d\n", __func__, cmd, ret);
        break;
	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	pr_info("%s -\n", __func__);
	return ret;
}

static const struct v4l2_subdev_internal_ops dw9827c_int_ops = {
	.open = dw9827c_open,
	.close = dw9827c_close,
};

static struct v4l2_subdev_core_ops dw9827c_ops_core = {
	.ioctl = dw9827c_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9827c_ops = {
	.core = &dw9827c_ops_core,
};

static void dw9827c_subdev_cleanup(struct dw9827c_device *dw9827c)
{
	v4l2_async_unregister_subdev(&dw9827c->sd);
	v4l2_ctrl_handler_free(&dw9827c->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9827c->sd.entity);
#endif
}

static int dw9827c_init_controls(struct dw9827c_device *dw9827c)
{
	struct v4l2_ctrl_handler *hdl = &dw9827c->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9827c_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9827c->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9827C_MAX_FOCUS_POS, DW9827C_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9827c->sd.ctrl_handler = hdl;

	return 0;
}

static void dw9827c_update_pid(struct dw9827c_device *dw9827c)
{
	int ret = -1;
	uint8_t value = 0xFF;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c->sd);

	value = i2c_smbus_read_byte_data(client, DW9827C_PID_VERSION_ADDR);
	LOG_INF("current PID version: %#04X", value);
	if (value < DW9827C_CURRENT_PID_VERSION) {
		LOG_INF("update pid ...");
		ret = dw9827c_write_array(dw9827c, dw9827c_pid, ARRAY_SIZE(dw9827c_pid));
		if (ret < 0) {
			LOG_INF("update pid failed");
			return;
		}
		LOG_INF("store pid ...");
		i2c_smbus_write_byte_data(client, 0x02, 0x40);
		i2c_smbus_write_byte_data(client, 0x03, 0x01);
		mdelay(51);
		i2c_smbus_write_byte_data(client, 0x34, 0x52);
		dw9827c_power_off(dw9827c);
		mdelay(1);
		dw9827c_power_on(dw9827c);
		mdelay(6);
		LOG_INF("check pid ...");
		for (int i = 2; i < ARRAY_SIZE(dw9827c_pid); ++i) {
			value = i2c_smbus_read_byte_data(client, dw9827c_pid[i].reg_num);
			LOG_INF("check pid param addr: %#04X, value(exp/read): %#04X/%#04X", dw9827c_pid[i].reg_num, dw9827c_pid[i].value, value);
			if (dw9827c_pid[i].value != value) {
				LOG_INF("check pid failed");
				return;
			}
		}
		LOG_INF("check pid success");
	}

	return;
}

static int dw9827c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9827c_device *dw9827c;
	int ret;
	unsigned int prj_id = 0;

	LOG_INF("+\n");

	dw9827c = devm_kzalloc(dev, sizeof(*dw9827c), GFP_KERNEL);
	if (!dw9827c)
		return -ENOMEM;

	dw9827c->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9827c->vin)) {
		ret = PTR_ERR(dw9827c->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9827c->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9827c->vdd)) {
		ret = PTR_ERR(dw9827c->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9827c->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9827c->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9827c->vcamaf_pinctrl);
		dw9827c->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9827c->vcamaf_on = pinctrl_lookup_state(
			dw9827c->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9827c->vcamaf_on)) {
			ret = PTR_ERR(dw9827c->vcamaf_on);
			dw9827c->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9827c->vcamaf_off = pinctrl_lookup_state(
			dw9827c->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9827c->vcamaf_off)) {
			ret = PTR_ERR(dw9827c->vcamaf_off);
			dw9827c->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9827c->sd, client, &dw9827c_ops);
	dw9827c->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9827c->sd.internal_ops = &dw9827c_int_ops;

	ret = dw9827c_init_controls(dw9827c);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9827c->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9827c->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9827c->sd);
	if (ret < 0)
		goto err_cleanup;
{
	int value = 0x00;
	dw9827c_power_on(dw9827c);

	// change i2c addr from 0x1E to 0x18
	client->addr = 0x0c;  //0x18
	LOG_INF("[test] client->addr(0x%x)", client->addr);

	i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
	usleep_range(3 * 1000, 3 * 1000 + 1000);
	value = 0x00;
	value = i2c_smbus_read_byte_data(client, 0x7D);

	if((client->addr == 0x0c) && (value < 0)) {   //0x1E addr error
		LOG_INF("[test] i2c 0x1E  error  value(%d)", value);
		client->addr = 0x0f;   //0x1E
		value = i2c_smbus_read_byte_data(client, 0x7D);
		if(value >= 0) {
			LOG_INF("[test] i2c 0x18  correct  value(%d)", value);

		i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
		usleep_range(5 * 1000, 5 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x85);//pt off
		i2c_smbus_write_byte_data(client, 0x7D, 0x00);// States standby  // change i2c addr to 0x18
//		i2c_smbus_write_byte_data(client, 0x95, 0x00);// SAL OFF
		i2c_smbus_write_byte_data(client, 0x03, 0x01);// store
		usleep_range(10 * 1000, 10 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x52);// pt on
		i2c_smbus_write_byte_data(client, 0x04, 0x01);// sw reset
		usleep_range(5 * 1000, 5 * 1000 + 1000);

		value = i2c_smbus_read_byte_data(client, 0x03);
		LOG_INF("[test] 0x18 product_id: %x\n", value);

		client->addr = 0x0c;   //0x18
		value = i2c_smbus_read_byte_data(client, 0x03);
		LOG_INF("[test] 0x1E product_id: %x\n", value);
		} else {
			LOG_INF("[test] i2c 0x18  error   value(%d)", value);
		}
	}

	prj_id = get_project();
	LOG_INF("project: %d", prj_id);
	if ((24081 == prj_id) || (24082 == prj_id) || (24206 == prj_id)) {
		dw9827c_update_pid(dw9827c);
	}

	dw9827c_power_off(dw9827c);

}

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9827c_subdev_cleanup(dw9827c);
	return ret;
}

static void dw9827c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9827c_device *dw9827c = sd_to_dw9827c_vcm(sd);

	LOG_INF("+\n");

	dw9827c_subdev_cleanup(dw9827c);

	LOG_INF("-\n");
	//return 0;
}

static const struct i2c_device_id dw9827c_id_table[] = {
	{ DW9827C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9827c_id_table);

static const struct of_device_id dw9827c_of_table[] = {
	{ .compatible = "oplus,dw9827c_24081" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9827c_of_table);

static struct i2c_driver dw9827c_i2c_driver = {
	.driver = {
		.name = DW9827C_NAME,
		.of_match_table = dw9827c_of_table,
	},
	.probe  = dw9827c_probe,
	.remove = dw9827c_remove,
	.id_table = dw9827c_id_table,
};

module_i2c_driver(dw9827c_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9827C VCM driver");
MODULE_LICENSE("GPL v2");
