/**************************************************************
* Copyright (C), 2008-2024, OPPO Mobile Comm Corp., Ltd.
* VENDOR_EDIT
* File: ois_power.c
* Description: OIS Power Ctrl Function Implement
* Version: 1.0
* Date : 2024/02/28
* Author: Wang Jianwei
* ----------------------Revision History-----------------------
*   <author>       <data>      <version>     <desc>
* Wang Jianwei   2024/05/12       1.0         init
****************************************************************/

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "hf_manager.h"
#include "hf_sensor_io.h"
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/timekeeping.h>
#include <linux/spinlock_types.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define DRV_NAME "ois_power"

/* Log interfaces */
#define LOG_INF(fmt, args...) pr_info(DRV_NAME" I [%s] " fmt, __func__, ##args)
#define LOG_ERR(fmt, args...) pr_err(DRV_NAME" E [%s] " fmt, __func__, ##args)
#define LOG_DBG(cond, ...) do { if ((cond)) { LOG_INF(__VA_ARGS__); } }while(0)

enum OIS_PINCTRL_STATE {
	OIS_PINCTRL_STATE_GPIO = 0,
	OIS_PINCTRL_STATE_I2C,
	OIS_PINCLRL_STATE_MAX,
};

static const char * const ois1_pinctrl_names[] = {
	"ois1_gpio",
	"ois1_i2c",
};

static const char * const ois2_pinctrl_names[] = {
	"ois2_gpio",
	"ois2_i2c",
};

struct ois_power_device {
	int vin_vol;
	int iovdd_vol;
	int afvdd_vol;
	bool power_done;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev subdev;
	struct hf_device hf_dev;
	struct regulator *vin;
	struct regulator *iovdd;
	struct regulator *afvdd;
	const char * const *pinctrl_names;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state[OIS_PINCLRL_STATE_MAX];
	struct mutex enable_mutex;
};

enum OIS_POWER_CMD
{
	OIS_POWER_CMD_MIN = 0,
	OIS_POWER_CMD_GET_POWER_STATE,
	OIS_POWER_CMD_MAX,
};

struct ois_power_device *ois1_power_dev;
struct ois_power_device *ois2_power_dev;

static struct sensor_info support_sensors1[] = {
	{
		.sensor_type = SENSOR_TYPE_OIS1_POWER,
		.gain = 1,
		.name = "ois1_power",
		.vendor = "oplus"
	},
};

static struct sensor_info support_sensors2[] = {
	{
		.sensor_type = SENSOR_TYPE_OIS2_POWER,
		.gain = 1,
		.name = "ois2_power",
		.vendor = "oplus"
	},
};

static inline struct ois_power_device *subdev_to_ois_power(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ois_power_device, subdev);
}

static int ois_power_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	int ret;

	ret = pm_runtime_get_sync(subdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(subdev->dev);
		return ret;
	}

	return 0;
}

static int ois_power_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(subdev->dev);

	return 0;
}

static long ois_power_ops_core_ioctl(struct v4l2_subdev *subdev, unsigned int cmd, void *arg)
{
	return 0;
}

static int ois_power_batch(struct hf_device *hfdev, int sensor_type, int64_t delay, int64_t latency)
{
	return 0;
}

static int ois_power_disable_thread_func(void *data)
{
	int ret = 0;
	struct ois_power_device *ois_power_dev = (struct ois_power_device*)data;

	mutex_lock(&ois_power_dev->enable_mutex);
	mdelay(30);
	if (ois_power_dev->afvdd_vol > 0) {
		ret = regulator_disable(ois_power_dev->afvdd);
		if (ret) {
			LOG_ERR("disable ois_power_dev->afvdd failed!\n");
			mutex_unlock(&ois_power_dev->enable_mutex);
			return ret;
		}
		LOG_INF("disable ois_power_dev->afvdd success ret[%d]", ret);
	}
	ret = regulator_disable(ois_power_dev->vin);
	if (ret) {
		LOG_ERR("disable ois_power_dev->vin failed!\n");
		mutex_unlock(&ois_power_dev->enable_mutex);
		return ret;
	}
	LOG_INF("disable ois_power_dev->vin success ret[%d]", ret);
	LOG_ERR("ois_power debug1 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
	if (ois_power_dev->pinctrl && ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]) {
		LOG_ERR("ois_power debug2 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
		ret = pinctrl_select_state(ois_power_dev->pinctrl, ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]);
		if (ret < 0) {
			mutex_unlock(&ois_power_dev->enable_mutex);
			LOG_ERR("select %s pinctrl state failed, ret: %d", ois_power_dev->pinctrl_names[OIS_PINCTRL_STATE_GPIO], ret);
			return ret;
		}
		LOG_INF("set ois i2c to low success");
	} else {
		LOG_ERR("set ois i2c to low failed, pinctrl or pinctrl_state is NULL");
	}
	ret = regulator_disable(ois_power_dev->iovdd);
	if (ret) {
		LOG_ERR("disable ois_power_dev->iovdd failed!\n");
		mutex_unlock(&ois_power_dev->enable_mutex);
		return ret;
	}
	LOG_INF("disable ois_power_dev->iovdd success ret[%d]", ret);
	ois_power_dev->power_done = false;
	mutex_unlock(&ois_power_dev->enable_mutex);

	return 0;
}

static int ois1_power_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	struct ois_power_device *ois_power_dev;
	struct task_struct *disable_task = NULL;
	int ret = 0;
	int ret1 = 0;

	if (0 == hfdev) {
		LOG_ERR("hfdev is null!! return error");
		return -1;
	}

	ois_power_dev = ois1_power_dev;
	if (NULL == ois_power_dev) {
		LOG_ERR("ois_power_dev is null, return error");
		return -1;
	}

	LOG_INF("ois_power_dev: %p sensor_type[%d] en[%d]", ois_power_dev, sensor_type, en);
	if (en) {
		mutex_lock(&ois_power_dev->enable_mutex);
		ret1 = regulator_set_voltage(ois_power_dev->iovdd, ois_power_dev->iovdd_vol, ois_power_dev->iovdd_vol);
		ret = regulator_enable(ois_power_dev->iovdd);
		if (ret < 0) {
			mutex_unlock(&ois_power_dev->enable_mutex);
			LOG_ERR("ois_power_dev->iovdd set failed ret[%d]", ret);
			return -1;
		}
		LOG_INF("ois_power_dev->iovdd set success ret[%d], ret1[%d]", ret, ret1);
		LOG_ERR("ois_power debug3 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
		if (ois_power_dev->pinctrl && ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_I2C]) {
			LOG_ERR("ois_power debug4 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
			ret = pinctrl_select_state(ois_power_dev->pinctrl, ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_I2C]);
			if (ret < 0) {
				mutex_unlock(&ois_power_dev->enable_mutex);
				LOG_ERR("select %s pinctrl state failed, ret: %d", ois_power_dev->pinctrl_names[OIS_PINCTRL_STATE_I2C], ret);
				return ret;
			}
			LOG_INF("set ois i2c to scp_i2c success");
		} else {
			LOG_ERR("set ois i2c to scp_i2c failed, pinctrl or pinctrl_state is NULL");
		}
		ret1 = regulator_set_voltage(ois_power_dev->vin, ois_power_dev->vin_vol, ois_power_dev->vin_vol);
		ret = regulator_enable(ois_power_dev->vin);
		if (ret < 0) {
			mutex_unlock(&ois_power_dev->enable_mutex);
			LOG_ERR("ois_power_dev->vin set failed ret[%d] ret1[%d]", ret, ret1);
			return -1;
		}
		LOG_INF("ois_power_dev->vin set success ret[%d], ret1[%d]", ret, ret1);
		mdelay(5);
		ois_power_dev->power_done = true;
		mutex_unlock(&ois_power_dev->enable_mutex);
	} else {
		disable_task = kthread_create(ois_power_disable_thread_func, ois_power_dev, "ois1_power_disable_kthread");
		if (IS_ERR(disable_task)) {
			LOG_INF("unable to start ois1_power_disable thread");
			disable_task = NULL;
			ois_power_disable_thread_func(ois_power_dev);
			return 0;
		}
		wake_up_process(disable_task);
	}

	return 0;
}

static int ois2_power_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	struct ois_power_device *ois_power_dev;
	struct task_struct *disable_task = NULL;
	int ret = 0;
	int ret1 = 0;

	if (0 == hfdev) {
		LOG_ERR("hfdev is null, return error");
		return -1;
	}

	ois_power_dev = ois2_power_dev;
	if (NULL == ois_power_dev) {
		LOG_ERR("ois_power_dev is null, return error");
		return -1;
	}

	LOG_INF("ois_power_dev: %p sensor_type[%d] en[%d]", ois_power_dev, sensor_type, en);
	if (en) {
		mutex_lock(&ois_power_dev->enable_mutex);
		ret1 = regulator_set_voltage(ois_power_dev->iovdd, ois_power_dev->iovdd_vol, ois_power_dev->iovdd_vol);
		ret = regulator_enable(ois_power_dev->iovdd);
		if (ret < 0) {
			mutex_unlock(&ois_power_dev->enable_mutex);
			LOG_ERR("ois_power_dev->iovdd set failed ret[%d]", ret);
			return -1;
		}
		LOG_INF("ois_power_dev->iovdd set success ret[%d], ret1[%d]", ret, ret1);
		LOG_ERR("ois_power debug5 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
		if (ois_power_dev->pinctrl && ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_I2C]) {
			LOG_ERR("ois_power debug6 pinctrl: %p pinctrl_state: %p", ois_power_dev->pinctrl, ois_power_dev->pinctrl_state);
			ret = pinctrl_select_state(ois_power_dev->pinctrl, ois_power_dev->pinctrl_state[OIS_PINCTRL_STATE_I2C]);
			if (ret < 0) {
				mutex_unlock(&ois_power_dev->enable_mutex);
				LOG_ERR("select %s pinctrl state failed, ret: %d", ois_power_dev->pinctrl_names[OIS_PINCTRL_STATE_I2C], ret);
				return ret;
			}
			LOG_INF("set ois i2c to scp_i2c success");
		} else {
			LOG_ERR("set ois i2c to scp_i2c failed, pinctrl or pinctrl_state is NULL");
		}
		ret1 = regulator_set_voltage(ois_power_dev->vin, ois_power_dev->vin_vol, ois_power_dev->vin_vol);
		ret = regulator_enable(ois_power_dev->vin);
		if (ret < 0) {
			mutex_unlock(&ois_power_dev->enable_mutex);
			LOG_ERR("ois_power_dev->vin set failed ret[%d] ret1[%d]", ret, ret1);
			return -1;
		}
		LOG_INF("ois_power_dev->vin set success ret[%d], ret1[%d]", ret, ret1);
		if (ois_power_dev->afvdd_vol > 0) {
			ret1 = regulator_set_voltage(ois_power_dev->afvdd, ois_power_dev->afvdd_vol, ois_power_dev->afvdd_vol);
			ret = regulator_enable(ois_power_dev->afvdd);
			if (ret < 0) {
				mutex_unlock(&ois_power_dev->enable_mutex);
				LOG_ERR("ois_power_dev->afvdd set failed ret[%d] ret1[%d]", ret, ret1);
				return -1;
			}
			LOG_INF("ois_power_dev->afvdd set success ret[%d], ret1[%d]", ret, ret1);
		}
		mdelay(11);
		ois_power_dev->power_done = true;
		mutex_unlock(&ois_power_dev->enable_mutex);
	} else {
		disable_task = kthread_create(ois_power_disable_thread_func, ois_power_dev, "ois2_power_disable_kthread");
		if (IS_ERR(disable_task)) {
			LOG_INF("unable to start ois2_power_disable thread");
			disable_task = NULL;
			ois_power_disable_thread_func(ois_power_dev);
			return 0;
		}
		wake_up_process(disable_task);
	}

	return 0;
}

int ois_power_config_cali(struct hf_device *hfdev, int sensor_type, void *data, uint8_t length)
{
	return 0;
}

static int ois_power_custom_cmd(struct hf_device *hfdev, int sensor_type, struct custom_cmd *cust_cmd)
{
	struct ois_power_device *ois_power_dev = NULL;

	if (sensor_type == SENSOR_TYPE_OIS1_POWER) {
		ois_power_dev = ois1_power_dev;
	} else if (sensor_type == SENSOR_TYPE_OIS2_POWER) {
		ois_power_dev = ois2_power_dev;
	}
	if (ois_power_dev == NULL) {
		LOG_ERR("ois_power_dev is NULL");
		return 0;
	}

	switch (cust_cmd->command) {
	case OIS_POWER_CMD_GET_POWER_STATE: {
		cust_cmd->rx_len = sizeof(int32_t);
		cust_cmd->data[0] = ois_power_dev->power_done;
		LOG_INF("sensor_type: %d, ois_power_dev: %p, power_done: %d", sensor_type, ois_power_dev, ois_power_dev->power_done);
	} break;
	default:
		break;
	}

	return 0;
}

static const struct v4l2_subdev_internal_ops ois_power_int_ops = {
	.open = ois_power_open,
	.close = ois_power_close,
};

static struct v4l2_subdev_core_ops ois_power_ops_core = {
	.ioctl = ois_power_ops_core_ioctl,
};

static const struct v4l2_subdev_ops ois_power_ops = {
	.core = &ois_power_ops_core,
};

static void ois_power_subdev_cleanup(struct ois_power_device *ois_power_dev)
{
	v4l2_async_unregister_subdev(&ois_power_dev->subdev);
	v4l2_ctrl_handler_free(&ois_power_dev->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&ois_power_dev->subdev.entity);
#endif
}

static int ois_power_init_controls(struct ois_power_device *ois_power_dev)
{
	struct v4l2_ctrl_handler *hdl = &ois_power_dev->ctrls;

	v4l2_ctrl_handler_init(hdl, 1);

	if (hdl->error) {
		LOG_ERR("v4l2_ctrl_handler_init failed, hdl->error: %d", hdl->error);
		return hdl->error;
	}

	ois_power_dev->subdev.ctrl_handler = hdl;

	return 0;
}

static int ois_power_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int ret;

	LOG_INF("E");

	ois1_power_dev = devm_kzalloc(dev, sizeof(*ois1_power_dev), GFP_KERNEL);
	if (!ois1_power_dev) {
		return -ENOMEM;
	}
	ois2_power_dev = devm_kzalloc(dev, sizeof(*ois2_power_dev), GFP_KERNEL);
	if (!ois2_power_dev) {
		return -ENOMEM;
	}
	LOG_INF("ois1_power_dev: %p, ois2_power_dev: %p", ois1_power_dev, ois2_power_dev);

	mutex_init(&ois1_power_dev->enable_mutex);
	mutex_init(&ois2_power_dev->enable_mutex);

	/* get regulator for OIS1 */
	ois1_power_dev->vin = devm_regulator_get(dev, "ois1");
	if (IS_ERR(ois1_power_dev->vin)) {
		ret = PTR_ERR(ois1_power_dev->vin);
		if (ret != -EPROBE_DEFER)
			LOG_ERR("cannot get vin regulator\n");
		return ret;
	}
	ois1_power_dev->iovdd = devm_regulator_get(dev, "iovdd");
	if (IS_ERR(ois1_power_dev->iovdd)) {
		ret = PTR_ERR(ois1_power_dev->iovdd);
		if (ret != -EPROBE_DEFER)
			LOG_ERR("cannot get iovdd regulator\n");
		return ret;
	}
	ret = of_property_read_u32(dev->of_node, "ois1-supply-vol", &ois1_power_dev->vin_vol);
	if (ret < 0) {
		ois1_power_dev->vin_vol = 0;
		LOG_ERR("cannot get vin voltage");
	}
	ret = of_property_read_u32(dev->of_node, "iovdd-supply-vol", &ois1_power_dev->iovdd_vol);
	if (ret < 0) {
		ois1_power_dev->iovdd_vol = 0;
		LOG_ERR("cannot get iovdd voltage");
	}
	LOG_INF("ois1_power vin_vol: %d, iovdd_vol: %d", ois1_power_dev->vin_vol, ois1_power_dev->iovdd_vol);

	ois1_power_dev->pinctrl_names = ois1_pinctrl_names;
	ois1_power_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ois1_power_dev->pinctrl)) {
		LOG_ERR("get pinctrl failed");
		return -EINVAL;
	}
	for (int i = 0; i < OIS_PINCLRL_STATE_MAX; ++i) {
		ois1_power_dev->pinctrl_state[i] = pinctrl_lookup_state(ois1_power_dev->pinctrl, ois1_power_dev->pinctrl_names[i]);
		if (IS_ERR(ois1_power_dev->pinctrl_state[i])) {
			ois1_power_dev->pinctrl_state[i] = NULL;
			LOG_ERR("get pinctrl state failed %s", ois1_power_dev->pinctrl_names[i]);
			return -EINVAL;
		}
	}
	LOG_ERR("ois_power debug7 pinctrl: %p pinctrl_state: %p", ois1_power_dev->pinctrl, ois1_power_dev->pinctrl_state);
	if (ois1_power_dev->pinctrl && ois1_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]) {
		LOG_ERR("ois_power debug8 pinctrl: %p pinctrl_state: %p", ois1_power_dev->pinctrl, ois1_power_dev->pinctrl_state);
		ret = pinctrl_select_state(ois1_power_dev->pinctrl, ois1_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]);
		if (ret < 0) {
			LOG_ERR("select %s pinctrl state failed, ret: %d", ois1_power_dev->pinctrl_names[OIS_PINCTRL_STATE_GPIO], ret);
			return ret;
		}
		LOG_INF("set ois1 i2c to low success");
	}
	ois1_power_dev->power_done = false;

	/* get regulator for OIS2 */
	ois2_power_dev->vin = devm_regulator_get(dev, "ois2");
	if (IS_ERR(ois2_power_dev->vin)) {
		ret = PTR_ERR(ois2_power_dev->vin);
		if (ret != -EPROBE_DEFER)
			LOG_ERR("cannot get vin regulator\n");
		return ret;
	}
	ois2_power_dev->iovdd = devm_regulator_get(dev, "iovdd");
	if (IS_ERR(ois2_power_dev->iovdd)) {
		ret = PTR_ERR(ois2_power_dev->iovdd);
		if (ret != -EPROBE_DEFER)
			LOG_ERR("cannot get iovdd regulator\n");
		return ret;
	}
	ois2_power_dev->afvdd = devm_regulator_get(dev, "ois2_afvdd");
	if (IS_ERR(ois2_power_dev->afvdd)) {
		ret = PTR_ERR(ois2_power_dev->afvdd);
		if (ret != -EPROBE_DEFER)
			LOG_ERR("cannot get afvdd regulator\n");
		return ret;
	}
	ret = of_property_read_u32(dev->of_node, "ois2-supply-vol", &ois2_power_dev->vin_vol);
	if (ret < 0) {
		ois1_power_dev->vin_vol = 0;
		LOG_ERR("cannot get vin voltage");
	}
	ret = of_property_read_u32(dev->of_node, "iovdd-supply-vol", &ois2_power_dev->iovdd_vol);
	if (ret < 0) {
		ois1_power_dev->iovdd_vol = 0;
		LOG_ERR("cannot get iovdd voltage");
	}
	ret = of_property_read_u32(dev->of_node, "ois2_afvdd-supply-vol", &ois2_power_dev->afvdd_vol);
	if (ret < 0) {
		ois2_power_dev->afvdd_vol = 0;
		LOG_ERR("cannot get afvdd voltage");
	}
	LOG_INF("ois2_power vin_vol: %d, iovdd_vol: %d, afvdd_vol: %d", ois2_power_dev->vin_vol, ois2_power_dev->iovdd_vol, ois2_power_dev->afvdd_vol);

	ois2_power_dev->pinctrl_names = ois2_pinctrl_names;
	ois2_power_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ois2_power_dev->pinctrl)) {
		LOG_ERR("get pinctrl failed");
		return -EINVAL;
	}
	for (int i = 0; i < OIS_PINCLRL_STATE_MAX; i++) {
		ois2_power_dev->pinctrl_state[i] = pinctrl_lookup_state(ois2_power_dev->pinctrl, ois2_power_dev->pinctrl_names[i]);
		if (IS_ERR(ois2_power_dev->pinctrl_state[i])) {
			ois2_power_dev->pinctrl_state[i] = NULL;
			LOG_ERR("get pinctrl state failed %s", ois2_power_dev->pinctrl_names[i]);
			return -EINVAL;
		}
	}
	LOG_ERR("ois_power debug9 pinctrl: %p pinctrl_state: %p", ois2_power_dev->pinctrl, ois2_power_dev->pinctrl_state);
	if (ois2_power_dev->pinctrl && ois2_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]) {
		LOG_ERR("ois_power debug10 pinctrl: %p pinctrl_state: %p", ois2_power_dev->pinctrl, ois2_power_dev->pinctrl_state);
		ret = pinctrl_select_state(ois2_power_dev->pinctrl, ois2_power_dev->pinctrl_state[OIS_PINCTRL_STATE_GPIO]);
		if (ret < 0) {
			LOG_ERR("select %s pinctrl state failed, ret: %d", ois2_power_dev->pinctrl_names[OIS_PINCTRL_STATE_GPIO], ret);
			return ret;
		}
		LOG_INF("set ois2 i2c to low success");
	}
	ois2_power_dev->power_done = false;

	v4l2_i2c_subdev_init(&ois1_power_dev->subdev, client, &ois_power_ops);
	ois1_power_dev->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ois1_power_dev->subdev.internal_ops = &ois_power_int_ops;

	ret = ois_power_init_controls(ois1_power_dev);
	if (ret != 0) {
		LOG_ERR("ois_power_init_controls failed");
		goto err_cleanup;
	}

	ois1_power_dev->hf_dev.dev_name = "ois1_power";
	ois1_power_dev->hf_dev.device_poll = HF_DEVICE_IO_POLLING;
	ois1_power_dev->hf_dev.device_bus = HF_DEVICE_IO_SYNC;
	ois1_power_dev->hf_dev.support_list = support_sensors1;
	ois1_power_dev->hf_dev.support_size = ARRAY_SIZE(support_sensors1);
	ois1_power_dev->hf_dev.enable = ois1_power_enable;
	ois1_power_dev->hf_dev.batch = ois_power_batch;
	ois1_power_dev->hf_dev.custom_cmd = ois_power_custom_cmd;
	ois1_power_dev->hf_dev.config_cali = ois_power_config_cali;
	hf_device_set_private_data(&ois1_power_dev->hf_dev, client);
	ret = hf_device_register_manager_create(&ois1_power_dev->hf_dev);
	if (ret < 0) {
		LOG_ERR("hf_manager_create fail");
		goto err_cleanup;
	}

	ois2_power_dev->hf_dev.dev_name = "ois2_power";
	ois2_power_dev->hf_dev.device_poll = HF_DEVICE_IO_POLLING;
	ois2_power_dev->hf_dev.device_bus = HF_DEVICE_IO_SYNC;
	ois2_power_dev->hf_dev.support_list = support_sensors2;
	ois2_power_dev->hf_dev.support_size = ARRAY_SIZE(support_sensors2);
	ois2_power_dev->hf_dev.enable = ois2_power_enable;
	ois2_power_dev->hf_dev.batch = ois_power_batch;
	ois2_power_dev->hf_dev.custom_cmd = ois_power_custom_cmd;
	ois2_power_dev->hf_dev.config_cali = ois_power_config_cali;
	hf_device_set_private_data(&ois2_power_dev->hf_dev, client);
	ret = hf_device_register_manager_create(&ois2_power_dev->hf_dev);
	if (ret < 0) {
		LOG_ERR("hf_manager_create fail");
		goto err_cleanup;
	}

#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&ois1_power_dev->subdev.entity, 0, NULL);
	if (ret < 0) {
		goto err_cleanup;
	}

	ois1_power_dev->subdev.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&ois1_power_dev->subdev);
	if (ret < 0) {
		LOG_ERR("v4l2_async_register_subdev failed.");
		goto err_cleanup;
	}

	pm_runtime_enable(dev);

	LOG_INF("X");

	return 0;

err_cleanup:
	if (ois1_power_dev->hf_dev.manager) {
		hf_device_unregister_manager_destroy(&ois1_power_dev->hf_dev);
	}
	if (ois2_power_dev->hf_dev.manager) {
		hf_device_unregister_manager_destroy(&ois2_power_dev->hf_dev);
	}
	ois_power_subdev_cleanup(ois1_power_dev);
	LOG_ERR("ois_power err_cleanup");
	return ret;
}

static void ois_power_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ois_power_device *ois_power_dev = subdev_to_ois_power(subdev);
	LOG_ERR("remove ois_power");

	ois_power_subdev_cleanup(ois_power_dev);
	pm_runtime_disable(&client->dev);

	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&ois1_power_dev->enable_mutex);
	mutex_destroy(&ois2_power_dev->enable_mutex);

	return;
}

static int ois_power_suspend(struct device *dev)
{
	return 0;
}

static int ois_power_resume(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id ois_power_id_table[] = {
	{DRV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ois_power_id_table);

static const struct of_device_id ois_power_of_table[] = {
	{.compatible = "oplus,ois_power"},
	{},
};
MODULE_DEVICE_TABLE(of, ois_power_of_table);

static const struct dev_pm_ops ois_power_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(ois_power_suspend, ois_power_resume, NULL)
};

static struct i2c_driver ois_power_i2c_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ois_power_of_table,
	},
	.probe = ois_power_probe,
	.remove = ois_power_remove,
	.id_table = ois_power_id_table,
};
module_i2c_driver(ois_power_i2c_driver);

MODULE_AUTHOR("WangJianwei");
MODULE_DESCRIPTION("OIS power driver");
MODULE_LICENSE("GPL v2");
