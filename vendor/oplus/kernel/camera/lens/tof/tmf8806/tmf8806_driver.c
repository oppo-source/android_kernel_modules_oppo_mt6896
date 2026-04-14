/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                 *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

/*! \file tmf8806_driver.c - TMF8806 driver
 * \brief Device driver for measuring Distance in mm.
 */

/* -------------------------------- includes -------------------------------- */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include "tmf8806_driver.h"

/* -------------------------------- defines --------------------------------- */
#define TMF8806_NAME "tmf8806"
#define LOG_INF(format, args...) pr_info(TMF8806_NAME " [%s] " format "\n", __func__, ##args)
#define LOG_ERR(format, args...) pr_err(TMF8806_NAME " [%s] " format "\n", __func__, ##args)
#define LOG_IF(cond, ...) do { if ( (cond) ) { LOG_INF(__VA_ARGS__); } }while(0)

#define APP_LOG_LEVEL                                                          \
  TMF8806_LOG_LEVEL_ERROR // how much logging we want at initialization

#define TMF8806_APP_ID_REG 0x00
#define TMF8806_REQ_APP_ID_REG 0x02
#define TMF8806_CMD_DATA9_REG 0x06
#define TMF8806_APP_ID_BOOTLOADER 0x80
#define TMF8806_APP_ID_APP0 0xC0
#define TMF8806_RESULT_NUMBER_OFFSET 0x20

#define TMF8806_APP0_CMD_IDX 11
#define TMF8806_FACTORY_CAL_CMD_SIZE 14
#define TMF8806_FACTORY_CALIB_KITERS                                           \
  40960 // kilo iterations for factory calibration

#define TMF8806_OSC_MIN_TRIM_VAL -256
#define TMF8806_OSC_MAX_TRIM_VAL 255

#define TOF_HISTOGRAM_SUM 128
struct oplus_tof_info {
  int32_t num_of_rows; /* Max : 8 */
  int32_t num_of_cols; /* Max : 8 */
  int32_t ranging_distance;
  int32_t dmax_distance;
  int32_t error_status;
  int64_t timestamp;
  int32_t confidence;
  int32_t result_cnt;
  uint16_t histogram[TOF_HISTOGRAM_SUM];
};

#define AMS_TOF_TRIM_DATA_SIZE  5
#define AMS_TOF_XTALK_DATA_SIZE  20
#define AMS_TOF_CALIBRATION_DATA_SIZE  20
struct oplus_tof_data {
  char trim_data[AMS_TOF_TRIM_DATA_SIZE];
  char xtalk_data[AMS_TOF_XTALK_DATA_SIZE];
  char calibration_data[AMS_TOF_CALIBRATION_DATA_SIZE];
};

atomic_t g_data_valid = ATOMIC_INIT(0);
uint16_t g_thread_state = 0;

/* Control commnad */
#define TMF8806_MAGIC_NUMBER 'T'
#define VIDIOC_MTK_G_TOF_INIT                _IOWR(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 3, struct oplus_tof_info)
#define VIDIOC_MTK_G_TOF_INFO                _IOWR(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 4, struct oplus_tof_info)
#define VIDIOC_OPLUS_G_TOF_CALIB             _IOWR(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 10, struct oplus_tof_data)
#define VIDIOC_OPLUS_G_TOF_TRIM              _IOWR(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 12, struct oplus_tof_data)
#define VIDIOC_OPLUS_G_TOF_XTALK             _IOWR(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 13, struct oplus_tof_data)
#define VIDIOC_OPLUS_S_TOF_CAPTURE           _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 5, char *)
#define VIDIOC_OPLUS_S_TOF_PERIOD            _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 6, char *)
#define VIDIOC_OPLUS_S_TOF_ITERATIONS        _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 7, char *)
#define VIDIOC_OPLUS_S_TOF_ALGSETTING        _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 8, char *)
#define VIDIOC_OPLUS_S_TOF_CALIB             _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 9, char [20])
#define VIDIOC_OPLUS_S_TOF_STOP              _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 14, unsigned int)
#define VIDIOC_OPLUS_S_TOF_DEBUG             _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 15, char *)
#define VIDIOC_OPLUS_S_TOF_CHIP_ENABLE       _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 16, char *)
#define VIDIOC_OPLUS_S_TOF_HISTOGRAM_ENABLE  _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 17, unsigned int)
#define VIDIOC_OPLUS_S_TOF_TRIM              _IOW(TMF8806_MAGIC_NUMBER, BASE_VIDIOC_PRIVATE + 11, char *)

/* -------------------------------- variables ------------------------------- */
static struct tmf8806_platform_data tmf8806_pdata = {
  .tof_name = TMF8806_NAME,
};

/**************************************************************************/
/*  Functions                                                             */
/**************************************************************************/
static ssize_t app0_apply_fac_calib_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count);
/**
 * tmf8806_switch_apps - Switch to Bootloader/App0
 *
 * @tmf8806_chip: tmf8806_chip pointer
 * @req_app_id: required application
 *
 * Returns 0 for No Error, -EIO for Error
 */
static int tmf8806_switch_apps(tmf8806_chip *chip, char req_app_id) {
  int error = 0;
  int current_app_id = 0;
  if ((req_app_id != TMF8806_APP_ID_BOOTLOADER) &&
      (req_app_id != TMF8806_APP_ID_APP0))
    return -EIO;

  current_app_id =
      i2cRxReg(chip, 0, TMF8806_APP_ID_REG, 1, chip->tof_core.dataBuffer);

  if (req_app_id == current_app_id)
    return 0;

  error = i2cTxReg(chip, 0, TMF8806_REQ_APP_ID_REG, 1, &req_app_id);
  if (error) {
    dev_err(&chip->client->dev, "Error setting REQ_APP_ID register.\n");
    error = -EIO;
  }

  switch (req_app_id) {
  case TMF8806_APP_ID_APP0:
    if (error) {
      dev_err(&chip->client->dev, "Error switch to App0.\n");
      /* Hard reset back to bootloader if error */
      enablePinLow(chip);
      delayInMicroseconds(5 * 1000);
      enablePinHigh(chip);
      error = tmf8806IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS);
      if (error) {
        dev_err(&chip->client->dev, "Error waiting for CPU ready flag.\n");
      }
      tmf8806ReadDeviceInfo(&chip->tof_core);
    } else {
      error = tmf8806IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS);
      if (error == 0) {
        dev_err(&chip->client->dev, "Cpu not Ready.\n");
      }
      error = tmf8806ReadDeviceInfo(&chip->tof_core);
      dev_info(&chip->client->dev, "Running app_id: 0x%02x\n",
               chip->tof_core.device.appVersion[0]);
      if (chip->tof_core.device.appVersion[0] != TMF8806_APP_ID_APP0) {
        dev_err(&chip->client->dev, "Error: Unrecognized application.\n");
        return APP_ERROR_CMD;
      }
      tmf8806ClrAndEnableInterrupts(&chip->tof_core,
                                    TMF8806_INTERRUPT_RESULT |
                                        TMF8806_INTERRUPT_DIAGNOSTIC);
    }
    break;
  case TMF8806_APP_ID_BOOTLOADER:
    if (!error) {
      error = tmf8806IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS);
      if (error == 0) {
        dev_err(&chip->client->dev, "Cpu not Ready.\n");
      }
      i2cRxReg(chip, 0, TMF8806_APP_ID_REG, 1, chip->tof_core.dataBuffer);

      (chip->tof_core.dataBuffer[0] == TMF8806_APP_ID_BOOTLOADER)
          ? (error = APP_SUCCESS_OK)
          : (error = APP_ERROR_CMD);
    }
    break;
  default:
    error = -EIO;
    break;
  }
  return error;
}

/**************************************************************************/
/* Sysfs callbacks                                                        */
/**************************************************************************/
/******** Common show/store functions ********/
static ssize_t program_show(struct device *dev, struct device_attribute *attr,
                            char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int error;
  dev_info(dev, "%s\n", __func__);

  AMS_MUTEX_LOCK(&chip->lock);
  error = i2cRxReg(chip, 0, TMF8806_APP_ID_REG, 1, chip->tof_core.dataBuffer);
  AMS_MUTEX_UNLOCK(&chip->lock);
  if (error) {
    return -EIO;
  }
  return scnprintf(buf, PAGE_SIZE, "%#x\n", chip->tof_core.dataBuffer[0]);
}

static ssize_t program_store(struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  char req_app_id;
  int error;
  sscanf(buf, "%hhx", &req_app_id);
  dev_info(dev, "%s: requested app: %#x\n", __func__, req_app_id);

  AMS_MUTEX_LOCK(&chip->lock);
  error = tmf8806_switch_apps(chip, req_app_id);
  AMS_MUTEX_UNLOCK(&chip->lock);

  return error ? -EIO : count;
}

static ssize_t chip_enable_show(struct device *dev,
                                struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int state;
  if (NULL == chip) {
    LOG_ERR("chip is NULL!!!!");
    return -EIO;
  }
  LOG_INF("chip: %p, dev: %p", chip, dev);
  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);
  if (NULL == chip->pdata) {
    LOG_ERR("chip->pdata is NULL!!!!");
    return -EIO;
  }
  if (!chip->pdata->gpiod_enable) {
    AMS_MUTEX_UNLOCK(&chip->lock);
    dev_info(dev, "%s !chip->pdata->gpiod_enable\n", __func__);
    return -EIO;
  }
  state = gpiod_get_value(chip->pdata->gpiod_enable) ? 1 : 0;
  AMS_MUTEX_UNLOCK(&chip->lock);
  LOG_INF("chip_enable: %d", state);

  return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}

static ssize_t chip_enable_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf,
                                 size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int req_state;
  int error;
  dev_info(dev, "%s\n", __func__);
  error = sscanf(buf, "%d", &req_state);
  if (error != 1)
    return -EINVAL;
  AMS_MUTEX_LOCK(&chip->lock);
  if (!chip->pdata->gpiod_enable) {
    AMS_MUTEX_UNLOCK(&chip->lock);
    return -EIO;
  }

  if (req_state == 0) {
    if (chip->tof_core.device.appVersion[0] == TMF8806_APP_ID_APP0) {
      if (tmf8806StopMeasurement(&chip->tof_core) != APP_SUCCESS_OK) {
        dev_err(dev, "Stop Measurement");
      }
    }
    if (enablePinLow(chip)) {
        LOG_ERR("enablePinLow fail");
    }
  } else {
    if (enablePinHigh(chip)) {
        LOG_ERR("enablePinHigh fail");
    }
    delayInMicroseconds(ENABLE_TIME_MS * 1000);
    tmf8806Wakeup(&chip->tof_core);
    error = tmf8806IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS);

    if (error == 0) {
      dev_err(dev, "CPU not ready");
      AMS_MUTEX_UNLOCK(&chip->lock);
      return -EIO;
    }

    uint8_t app_id[1] = {TMF8806_APP_ID_APP0};
    if (i2cTxReg(chip, 0, TMF8806_REQ_APP_ID_REG, 1, app_id)) {
        LOG_INF("write REQ_APP_ID fail");
    }
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return count;
}

static ssize_t driver_debug_show(struct device *dev,
                                 struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  dev_info(dev, "%s\n", __func__);
  return scnprintf(buf, PAGE_SIZE, "%d\n", chip->tof_core.logLevel);
}

static ssize_t driver_debug_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  dev_info(dev, "%s\n", __func__);
  sscanf(buf, "%hhx", &chip->tof_core.logLevel);
  return count;
}

static ssize_t program_version_show(struct device *dev,
                                    struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int len = -1;
  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);

  if (tmf8806ReadDeviceInfo(&chip->tof_core) == APP_SUCCESS_OK) {
    len = scnprintf(buf, PAGE_SIZE, "%#x %#x %#x %#x %#x %#x\n",
                    chip->tof_core.device.appVersion[0],
                    chip->tof_core.device.appVersion[1],
                    chip->tof_core.device.appVersion[2],
                    chip->tof_core.device.appVersion[3],
                    chip->tof_core.device.chipVersion[0],
                    chip->tof_core.device.chipVersion[1]);
  }

  AMS_MUTEX_UNLOCK(&chip->lock);
  return len;
}

static ssize_t register_write_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  char preg;
  char pval;
  char pmask = -1;
  int numparams;
  int rc = 0;

  dev_info(dev, "%s\n", __func__);

  numparams = sscanf(buf, "%hhx:%hhx:%hhx", &preg, &pval, &pmask);
  if ((numparams < 2) || (numparams > 3))
    return -EINVAL;
  if ((numparams >= 1) && (preg < 0))
    return -EINVAL;
  if ((numparams >= 2) && (preg < 0))
    return -EINVAL;

  AMS_MUTEX_LOCK(&chip->lock);
  if (pmask == -1) {
    rc = i2cTxReg(chip, 0, preg, 1, &pval);
  } else {
    rc = i2c_write_mask(chip->client, preg, pval, pmask);
  }
  AMS_MUTEX_UNLOCK(&chip->lock);

  return rc ? rc : count;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr,
                              char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int per_line = 4; // dumped register values per line
  int len = 0;
  int idx, per_line_idx;
  int bufsize = PAGE_SIZE;
  int error;

  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);

  memset(chip->shadow, 0, MAX_REGS);
  error = i2cRxReg(chip, 0, 0x00, MAX_REGS, chip->shadow);

  if (error) {
    dev_err(dev, "Read all registers failed: %d\n", error);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error;
  }

  for (idx = 0; idx < MAX_REGS; idx += per_line) {
    len += scnprintf(buf + len, bufsize - len, "%#02x:", idx);
    for (per_line_idx = 0; per_line_idx < per_line; per_line_idx++) {
      len += scnprintf(buf + len, bufsize - len, " ");
      len += scnprintf(buf + len, bufsize - len, "%02x",
                       chip->shadow[idx + per_line_idx]);
    }
    len += scnprintf(buf + len, bufsize - len, "\n");
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return len;
}

static ssize_t request_ram_patch_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int error = -1;
  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);

  if (error == BL_SUCCESS_OK) {
    tmf8806ClrAndEnableInterrupts(&chip->tof_core,
                                  TMF8806_INTERRUPT_RESULT |
                                      TMF8806_INTERRUPT_DIAGNOSTIC);
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return error ? -EIO : count;
}

static ssize_t app0_command_show(struct device *dev,
                                 struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int error, i;
  int len = 0;
  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  error = i2cRxReg(chip, 0, TMF8806_CMD_DATA9_REG, TMF8806_APP0_CMD_IDX,
                   chip->tof_core.dataBuffer);
  AMS_MUTEX_UNLOCK(&chip->lock);
  if (error) {
    return -EIO;
  }
  for (i = 0; i < TMF8806_APP0_CMD_IDX; i++) {
    len += scnprintf(buf + len, PAGE_SIZE - len, "%#02x:%02x\n", i,
                     chip->tof_core.dataBuffer[i]);
  }

  return len;
}

/****** App0 cmd show/store functions  ******/
static ssize_t tmf8806_app0_meas_cmd_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  ssize_t ret = 0;
  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s cmd: %s buf %s \n", __func__, attr->attr.name, buf);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  if (!strncmp(attr->attr.name, "capture", strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhx\n",
                     chip->tof_core.measureConfig.data.command);
  } else if (!strncmp(attr->attr.name, "iterations", strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hu\n",
                     chip->tof_core.measureConfig.data.kIters);
  } else if (!strncmp(attr->attr.name, "period", strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhu\n",
                     chip->tof_core.measureConfig.data.repetitionPeriodMs);
  } else if (!strncmp(attr->attr.name, "snr", strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhx\n",
                     *((char *)&chip->tof_core.measureConfig.data.snr));
  } else if (!strncmp(attr->attr.name, "capture_delay",
                      strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhu\n",
                     chip->tof_core.measureConfig.data.daxDelay100us);
  } else if (!strncmp(attr->attr.name, "gpio_setting",
                      strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhx\n",
                     *((char *)&chip->tof_core.measureConfig.data.gpio));
  } else if (!strncmp(attr->attr.name, "alg_setting",
                      strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhx\n",
                     *((char *)&chip->tof_core.measureConfig.data.algo));
  } else if (!strncmp(attr->attr.name, "data_setting",
                      strlen(attr->attr.name))) {
    ret += scnprintf(buf, PAGE_SIZE, "%hhx\n",
                     *((char *)&chip->tof_core.measureConfig.data.data));
  } else if (!strncmp(attr->attr.name, "spreadSpecVcsel",
                      strlen(attr->attr.name))) {
    ret += scnprintf(
        buf, PAGE_SIZE, "%hhx\n",
        *((char *)&chip->tof_core.measureConfig.data.spreadSpecVcselChp));
  } else if (!strncmp(attr->attr.name, "spreadSpecSpad",
                      strlen(attr->attr.name))) {
    ret += scnprintf(
        buf, PAGE_SIZE, "%hhx\n",
        *((char *)&chip->tof_core.measureConfig.data.spreadSpecSpadChp));
  } else {
    ret += scnprintf(buf, PAGE_SIZE, "wrong cmd \n");
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return ret;
}

static ssize_t capture_show(struct device *dev, struct device_attribute *attr,
                            char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t iterations_show(struct device *dev,
                               struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t period_show(struct device *dev, struct device_attribute *attr,
                           char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t snr_show(struct device *dev, struct device_attribute *attr,
                        char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t capture_delay_show(struct device *dev,
                                  struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t gpio_setting_show(struct device *dev,
                                 struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t alg_setting_show(struct device *dev,
                                struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t data_setting_show(struct device *dev,
                                 struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t spreadSpecVcsel_show(struct device *dev,
                                    struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t spreadSpecSpad_show(struct device *dev,
                                   struct device_attribute *attr, char *buf) {
  return tmf8806_app0_meas_cmd_show(dev, attr, buf);
}

static ssize_t app0_command_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int len = 0;
  int error = 0;
  char bytes[TMF8806_APP0_CMD_IDX];
  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);

  len = sscanf(buf, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
               &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5],
               &bytes[6], &bytes[7], &bytes[8], &bytes[9], &bytes[10]);

  if ((len < 1) || (len > TMF8806_APP0_CMD_IDX))
    return -EINVAL;

  error = i2cTxReg(chip, 0, TMF8806_COM_CMD_REG + 1 - len, len, &bytes[0]);

  AMS_MUTEX_UNLOCK(&chip->lock);
  return count;
}

static ssize_t tmf8806_app0_cmd_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  ssize_t ret = count;
  int8_t error;
  char bytes[TMF8806_APP0_CMD_IDX];

  LOG_INF("cmd: %s buf: %s \n", attr->attr.name, buf);
  AMS_MUTEX_LOCK(&chip->lock);

  if (!strncmp(attr->attr.name, "capture", strlen(attr->attr.name))) {
    // only for capture attr start and stop measurement
    ret = sscanf(buf, "%hhx", &chip->tof_core.measureConfig.data.command);
    if (ret == 1) {
      tmf8806ClrAndEnableInterrupts(&chip->tof_core,
                                    TMF8806_INTERRUPT_RESULT |
                                        TMF8806_INTERRUPT_DIAGNOSTIC);
      if (chip->tof_core.measureConfig.data.command > 0) {
        error = tmf8806StartMeasurement(&chip->tof_core);
      } else {
        error = tmf8806StopMeasurement(&chip->tof_core);
      }
      if (error != APP_SUCCESS_OK) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
      }
    }
  } else if (!strncmp(attr->attr.name, "iterations", strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hu", &chip->tof_core.measureConfig.data.kIters);
  } else if (!strncmp(attr->attr.name, "period", strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhu",
                 &chip->tof_core.measureConfig.data.repetitionPeriodMs);
  } else if (!strncmp(attr->attr.name, "snr", strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.snr, bytes, 1);
  } else if (!strncmp(attr->attr.name, "capture_delay",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhu", &chip->tof_core.measureConfig.data.daxDelay100us);
  } else if (!strncmp(attr->attr.name, "gpio_setting",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.gpio, bytes, 1);
  } else if (!strncmp(attr->attr.name, "alg_setting",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.algo, bytes, 1);
  } else if (!strncmp(attr->attr.name, "data_setting",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.data, bytes, 1);
  } else if (!strncmp(attr->attr.name, "spreadSpecVcsel",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.spreadSpecVcselChp, bytes, 1);
  } else if (!strncmp(attr->attr.name, "spreadSpecSpad",
                      strlen(attr->attr.name))) {
    ret = sscanf(buf, "%hhx", &bytes[0]);
    memcpy(&chip->tof_core.measureConfig.data.spreadSpecSpadChp, bytes, 1);

  } else {
    dev_info(dev, " cmd not found\n");
  }

  AMS_MUTEX_UNLOCK(&chip->lock);
  return (ret != 1) ? -EINVAL : count;
}

static ssize_t capture_store(struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t iterations_store(struct device *dev,
                                struct device_attribute *attr, const char *buf,
                                size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t period_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t snr_store(struct device *dev, struct device_attribute *attr,
                         const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t capture_delay_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t gpio_setting_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t alg_setting_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf,
                                 size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t data_setting_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t spreadSpecVcsel_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

static ssize_t spreadSpecSpad_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count) {
  return tmf8806_app0_cmd_store(dev, attr, buf, count);
}

/******** App0 functions  ********/
static ssize_t app0_fac_calib_show(struct device *dev,
                                   struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  ssize_t ret = -1;
  char byte[TMF8806_FACTORY_CAL_CMD_SIZE];

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }
  app0_apply_fac_calib_store(dev, attr, buf, 14);

  AMS_MUTEX_LOCK(&chip->lock);
  memcpy(byte, &chip->tof_core.factoryCalib, TMF8806_FACTORY_CAL_CMD_SIZE);
  ret = scnprintf(
      buf, PAGE_SIZE,
      "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx\n",
      byte[0], byte[1], byte[2], byte[3], byte[4], byte[5], byte[6], byte[7],
      byte[8], byte[9], byte[10], byte[11], byte[12], byte[13]);

  AMS_MUTEX_UNLOCK(&chip->lock);

  return ret;
}

static ssize_t distance_thresholds_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  ssize_t ret = -EIO;
  uint8_t pers = 0;
  uint16_t lowThr, highThr = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  if (APP_SUCCESS_OK ==
      tmf8806GetThresholds(&chip->tof_core, &pers, &lowThr, &highThr)) {
    ret = scnprintf(buf, PAGE_SIZE, "%hhx %hx %hx \n", pers, lowThr, highThr);
  }
  AMS_MUTEX_UNLOCK(&chip->lock);

  return ret;
}

static ssize_t app0_fac_calib_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  char byte[TMF8806_FACTORY_CAL_CMD_SIZE] = {0};

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  ret = sscanf(
      buf,
      "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
      &byte[0], &byte[1], &byte[2], &byte[3], &byte[4], &byte[5], &byte[6],
      &byte[7], &byte[8], &byte[9], &byte[10], &byte[11], &byte[12], &byte[13]);

  if (ret == TMF8806_FACTORY_CAL_CMD_SIZE) {
    tmf8806SetFactoryCalibration(&chip->tof_core,
                                 (tmf8806FactoryCalibData *)byte);
  }
  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != TMF8806_FACTORY_CAL_CMD_SIZE) ? -EINVAL : count;
}

static ssize_t app0_state_data_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  char byte[TMF8806_COM_STATE_DATA_COMPRESSED];

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  ret = sscanf(buf, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
               &byte[0], &byte[1], &byte[2], &byte[3], &byte[4], &byte[5],
               &byte[6], &byte[7], &byte[8], &byte[9], &byte[10]);

  if (ret == TMF8806_COM_STATE_DATA_COMPRESSED) {
    tmf8806SetStateData(&chip->tof_core, (tmf8806StateData *)byte);
  }
  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != TMF8806_COM_STATE_DATA_COMPRESSED) ? -EINVAL : count;
}

static ssize_t distance_thresholds_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  uint8_t pers = 0;
  uint16_t lowThr, highThr = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  ret = sscanf(buf, "%hhx %hx %hx", &pers, &lowThr, &highThr);
  if (ret != 3)
    return -EINVAL;

  AMS_MUTEX_LOCK(&chip->lock);
  ret = tmf8806SetThresholds(&chip->tof_core, pers, lowThr, highThr);
  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != APP_SUCCESS_OK) ? -EIO : count;
}

static ssize_t app0_apply_fac_calib_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int8_t ret;
  uint32_t timeout = 500;
  uint8_t irqs = 0;
  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  tmf8806ClrAndEnableInterrupts(&chip->tof_core, TMF8806_INTERRUPT_RESULT);
  ret =
      tmf8806FactoryCalibration(&chip->tof_core, TMF8806_FACTORY_CALIB_KITERS);

  if (ret == APP_SUCCESS_OK) {
    delayInMicroseconds(1000000); // needs more then 1 sec.
    while (irqs == 0 && timeout-- > 0) {
      delayInMicroseconds(10000);
      irqs =
          tmf8806GetAndClrInterrupts(&chip->tof_core, TMF8806_INTERRUPT_RESULT);
    }

    ret = tmf8806ReadFactoryCalibration(&chip->tof_core);
    if (ret != APP_SUCCESS_OK) {
      dev_info(dev, "No factory calibration page \n");
    }
    if (timeout == 0) {
      ret = -1;
    }
    dev_info(dev, "factory calibration success(timeout %d)\n", timeout);
    char debug[120];
    u32 strsize = 0;
    for (int idx = 0; idx < 14; ++idx) {
      strsize += scnprintf(debug + strsize, sizeof(debug) - strsize, "%02x ",
        chip->tof_core.dataBuffer[(TMF8806_COM_CONFIG-TMF8806_COM_CONTENT) + idx]);
    }
    dev_info(dev, "%s", debug);
  }

  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != BL_SUCCESS_OK) ? -EIO : count;
}

static ssize_t app0_clk_correction_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  uint8_t clkenable = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  ret = sscanf(buf, "%hhx ", &clkenable);
  if (ret != 1)
    return -EINVAL;

  AMS_MUTEX_LOCK(&chip->lock);
  tmf8806ClkCorrection(&chip->tof_core, clkenable);
  AMS_MUTEX_UNLOCK(&chip->lock);

  return count;
}

static ssize_t app0_histogram_readout_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  uint8_t histograms = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  ret = sscanf(buf, "%hhx", &histograms);
  if (ret != 1)
    return -EINVAL;

  AMS_MUTEX_LOCK(&chip->lock);
  ret = tmf8806ConfigureHistograms(&chip->tof_core, histograms);
  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != APP_SUCCESS_OK) ? -EIO : count;
}

static ssize_t app0_ctrl_reg_show(struct device *dev,
                                  struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int error, i;
  int len = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  error = i2cRxReg(chip, 0, TMF8806_APP_ID_REG, TMF8806_RESULT_NUMBER_OFFSET,
                   chip->tof_core.dataBuffer);
  AMS_MUTEX_UNLOCK(&chip->lock);
  if (error) {
    return -EIO;
  }
  for (i = 0; i < TMF8806_RESULT_NUMBER_OFFSET; i++) {
    len += scnprintf(buf + len, PAGE_SIZE - len, "%#02x:%02x\n", i,
                     chip->tof_core.dataBuffer[i]);
  }

  return len;
}

static ssize_t app0_osc_trim_show(struct device *dev,
                                  struct device_attribute *attr, char *buf) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int error, trim = 0;
  int len = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  AMS_MUTEX_LOCK(&chip->lock);
  error = tmf8806_oscillator_trim(&chip->tof_core, &trim, 0);
  AMS_MUTEX_UNLOCK(&chip->lock);

  if (error) {
    return -EIO;
  }

  len += scnprintf(buf, PAGE_SIZE, "%d\n", trim);

  return len;
}

static ssize_t app0_osc_trim_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count) {
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int ret;
  int trim = 0;

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(dev, "%s\n", __func__);
  }

  ret = sscanf(buf, "%d", &trim);
  if (ret != 1)
    return -EINVAL;

  if ((trim > TMF8806_OSC_MAX_TRIM_VAL) || (trim < TMF8806_OSC_MIN_TRIM_VAL)) {
    dev_err(dev, "%s: Error clk trim setting is out of range [%d,%d]\n",
            __func__, -256, 255);
    return -EINVAL;
  }

  AMS_MUTEX_LOCK(&chip->lock);
  ret = tmf8806_oscillator_trim(&chip->tof_core, &trim, 1);
  AMS_MUTEX_UNLOCK(&chip->lock);

  return (ret != APP_SUCCESS_OK) ? -EIO : count;
}

/******** OUTPUT DATA ********/
static ssize_t app0_tof_output_read(struct file *fp, struct kobject *kobj,
                                    struct bin_attribute *attr, char *buf,
                                    loff_t off, size_t size) {
  struct device *dev = kobj_to_dev(kobj);
  tmf8806_chip *chip = container_of((struct v4l2_subdev *)dev_get_drvdata(dev), tmf8806_chip, subdev);
  int read;
  u32 elem_len;

  AMS_MUTEX_LOCK(&chip->lock);
  elem_len = kfifo_peek_len(&chip->tof_output_fifo);
  dev_dbg(dev, "%s size: %u\n", __func__, (unsigned int)size);
  if (kfifo_len(&chip->tof_output_fifo)) {
    dev_dbg(dev, "fifo read elem_len: %u\n", elem_len);
    read = kfifo_out(&chip->tof_output_fifo, buf, elem_len);
    dev_dbg(dev, "fifo_len: %u\n", kfifo_len(&chip->tof_output_fifo));
    AMS_MUTEX_UNLOCK(&chip->lock);
    return elem_len;
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return 0;
}
/**************************************************************************/
/* System File System                                                     */
/**************************************************************************/
/* Common Sysfs */
static DEVICE_ATTR_RW(program);
static DEVICE_ATTR_RW(chip_enable);
static DEVICE_ATTR_RW(driver_debug);
static DEVICE_ATTR_RO(program_version);
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_RO(registers);
static DEVICE_ATTR_WO(request_ram_patch);

static struct attribute *tmf8806_common_attrs[] = {
    &dev_attr_program.attr,           &dev_attr_chip_enable.attr,
    &dev_attr_driver_debug.attr,      &dev_attr_program_version.attr,
    &dev_attr_register_write.attr,    &dev_attr_registers.attr,
    &dev_attr_request_ram_patch.attr, NULL,
};
static const struct attribute_group tmf8806_common_attr_group = {
    .name = "tmf8806_common",
    .attrs = tmf8806_common_attrs,
};
/* App0 Sysfs */
static DEVICE_ATTR_RW(app0_command);
static DEVICE_ATTR_RW(capture);
static DEVICE_ATTR_RW(iterations);
static DEVICE_ATTR_RW(period);
static DEVICE_ATTR_RW(snr);
static DEVICE_ATTR_RW(capture_delay);
static DEVICE_ATTR_RW(gpio_setting);
static DEVICE_ATTR_RW(alg_setting);
static DEVICE_ATTR_RW(data_setting);
static DEVICE_ATTR_RW(spreadSpecVcsel);
static DEVICE_ATTR_RW(spreadSpecSpad);
static DEVICE_ATTR_RW(app0_fac_calib);
static DEVICE_ATTR_WO(app0_state_data);
static DEVICE_ATTR_RW(distance_thresholds);
static DEVICE_ATTR_WO(app0_apply_fac_calib);
static DEVICE_ATTR_WO(app0_clk_correction);
static DEVICE_ATTR_WO(app0_histogram_readout);
static DEVICE_ATTR_RO(app0_ctrl_reg);
static DEVICE_ATTR_RW(app0_osc_trim);
static BIN_ATTR_RO(app0_tof_output, 0);

static struct bin_attribute *tof_app0_bin_attrs[] = {
    &bin_attr_app0_tof_output,
    NULL,
};
static struct attribute *tmf8806_app0_attrs[] = {
    &dev_attr_app0_command.attr,
    &dev_attr_capture.attr,
    &dev_attr_iterations.attr,
    &dev_attr_period.attr,
    &dev_attr_snr.attr,
    &dev_attr_capture_delay.attr,
    &dev_attr_gpio_setting.attr,
    &dev_attr_alg_setting.attr,
    &dev_attr_data_setting.attr,
    &dev_attr_spreadSpecVcsel.attr,
    &dev_attr_spreadSpecSpad.attr,
    &dev_attr_app0_fac_calib.attr,
    &dev_attr_app0_state_data.attr,
    &dev_attr_distance_thresholds.attr,
    &dev_attr_app0_apply_fac_calib.attr,
    &dev_attr_app0_clk_correction.attr,
    &dev_attr_app0_histogram_readout.attr,
    &dev_attr_app0_ctrl_reg.attr,
    &dev_attr_app0_osc_trim.attr,
    NULL,
};
static const struct attribute_group tmf8806_app0_attr_group = {
    .name = "tmf8806_app0",
    .attrs = tmf8806_app0_attrs,
    .bin_attrs = tof_app0_bin_attrs,
};
/*All groups*/
static const struct attribute_group *tmf8806_attr_groups[] = {
    &tmf8806_common_attr_group,
    &tmf8806_app0_attr_group,
    NULL,
};

/**************************************************************************/
/* Probe Remove Functions                                                 */
/**************************************************************************/
int tof_queue_frame(tmf8806_chip *chip) {
  int result = 0;
  int size = (chip->tof_output_frame.frame.payload_msb << 8) +
             chip->tof_output_frame.frame.payload_lsb + 4;

  result = kfifo_in(&chip->tof_output_fifo, chip->tof_output_frame.buf, size);
  if (chip->tof_core.logLevel >= TMF8806_LOG_LEVEL_INFO) {

    dev_info(&chip->client->dev, "Size %x\n", size);
    dev_info(&chip->client->dev, "Fr Num %x\n",
             chip->tof_output_frame.frame.frameNumber);
  }
  if (result == 0) {
    if (chip->tof_core.logLevel >= TMF8806_LOG_LEVEL_INFO) {
      dev_info(&chip->client->dev, "Reset output frame.\n");
    }
    kfifo_reset(&chip->tof_output_fifo);
    result = kfifo_in(&chip->tof_output_fifo, chip->tof_output_frame.buf, size);
    if (result == 0) {
      dev_err(&chip->client->dev, "Error: queueing ToF output frame.\n");
    }
    if (result != size) {
      dev_err(&chip->client->dev, "Error: queueing ToF output frame Size.\n");
    }
  }
  chip->tof_output_frame.frame.frameId = 0;
  chip->tof_output_frame.frame.payload_lsb = 0;
  chip->tof_output_frame.frame.payload_msb = 0;
  chip->tof_output_frame.frame.frameNumber++;

  return (result == size) ? 0 : -1;
}

/**
 * tof_get_gpio_config - Get GPIO config from Device-Managed API
 *
 * @tmf8806_chip: tmf8806_chip pointer
 */
static int tof_get_gpio_config(tmf8806_chip *tof_chip) {
  int error;
  struct device *dev;
  struct gpio_desc *gpiod;

  if (!tof_chip->client) {
    return -EINVAL;
  }
  dev = &tof_chip->client->dev;

  /* Get the enable line GPIO pin number */
  gpiod = devm_gpiod_get(dev, TOF_GPIO_ENABLE_NAME, GPIOD_OUT_HIGH);
  if (IS_ERR(gpiod)) {
    error = PTR_ERR(gpiod);
    return error;
  }
  tof_chip->pdata->gpiod_enable = gpiod;

  /* Get the interrupt GPIO pin number */
  gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_INT_NAME, GPIOD_IN);

  if (IS_ERR(gpiod)) {
    error = PTR_ERR(gpiod);
    dev_info(&tof_chip->client->dev, "Error: Irq. %d \n", error);
    if (PTR_ERR(gpiod) != -EBUSY) { // for ICAM on this pin there is an Error
                                    // (from ACPI), but seems to be working
      return error;
    }
  }
  tof_chip->pdata->gpiod_interrupt = gpiod;

  return 0;
}

/**
 * tof_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 *
 * Returns IRQ_HANDLED
 */
static irqreturn_t tof_irq_handler(int irq, void *dev_id) {
  tmf8806_chip *chip = (tmf8806_chip *)dev_id;
  int size;
  AMS_MUTEX_LOCK(&chip->lock);

  if (chip->tof_core.logLevel & TMF8806_LOG_LEVEL_VERBOSE) {
    dev_info(&chip->client->dev, "irq_handler");
  }

  size = tmf8806_app_process_irq(&chip->tof_core);

  if (size == 0) {
    AMS_MUTEX_UNLOCK(&chip->lock);
    return IRQ_HANDLED;
  }

  if (atomic_read(&g_data_valid) == 0) {
    dev_info(&chip->client->dev, "data status is true\n");
    atomic_set(&g_data_valid, 1);
  }

  /* Alert user space of changes */
  sysfs_notify(&chip->client->dev.kobj, tmf8806_app0_attr_group.name,
               bin_attr_app0_tof_output.attr.name);

  AMS_MUTEX_UNLOCK(&chip->lock);
  return IRQ_HANDLED;
}

/**
 * tof_request_irq - request IRQ for given gpio
 *
 * @tof_chip: tmf8806_chip pointer
 *
 * Returns status of function devm_request_threaded_irq
 */
static int tof_request_irq(tmf8806_chip *tof_chip) {
  int irq = tof_chip->client->irq;
  unsigned long default_trigger = irqd_get_trigger_type(irq_get_irq_data(irq));
  dev_info(&tof_chip->client->dev, "irq: %d, trigger_type: %lu", irq,
           default_trigger);
  return devm_request_threaded_irq(&tof_chip->client->dev,
                                   tof_chip->client->irq, NULL, tof_irq_handler,
                                   default_trigger | IRQF_SHARED | IRQF_ONESHOT,
                                   tof_chip->client->name, tof_chip);
}

/**
 * tmf8806_app0_poll_irq_thread -
 *
 * @tof_chip: tmf8806_chip pointer
 *
 * Returns 0
 */

static int tmf8806_app0_poll_irq_thread(void *tof_chip) {
  tmf8806_chip *chip = (tmf8806_chip *)tof_chip;
  int us_sleep = 0;

  // Poll period is interpreted in units of 100 usec
  us_sleep = chip->tof_core.measureConfig.data.repetitionPeriodMs * 1000;
  dev_info(&chip->client->dev, "Starting ToF irq polling thread, period: %u us\n", us_sleep);
  while (!kthread_should_stop()) {
    (void)tof_irq_handler(0, tof_chip);
    LOG_INF("thread is running!");
    delayInMicroseconds(us_sleep);
  }
  dev_err(&chip->client->dev, "ToF irq polling thread stopped\n");

  return 0;
}

static int thread_store(struct v4l2_subdev *subdev, uint8_t start_or_stop)
{
  int ret = 0;
  tmf8806_chip *chip = container_of(subdev, tmf8806_chip, subdev);
  if (start_or_stop) {
    if (g_thread_state == 0) {
      chip->app0_poll_irq = kthread_run(tmf8806_app0_poll_irq_thread, (void *)chip, "tof-irq_poll");
      if (IS_ERR(chip->app0_poll_irq)) {
          LOG_ERR("start irq polling thread failed");
          ret = PTR_ERR(chip->app0_poll_irq);
      } else {
        g_thread_state = 1;
      }
    } else {
      LOG_INF("thread is already exist\n");
    }
  } else {
    if (g_thread_state == 1) {
      if (chip->poll_period) {
        (void)kthread_stop(chip->app0_poll_irq);
      }
      g_thread_state = 0;
    } else {
      LOG_INF("now has no thread exist\n");
    }
  }
  return ret;
}

static int tmf8806_init_controls(tmf8806_chip *chip)
{
  struct v4l2_ctrl_handler *hdl = &chip->ctrl_handler;

  v4l2_ctrl_handler_init(hdl, 1);

  if (hdl->error) {
    return hdl->error;
  }

  chip->subdev.ctrl_handler = hdl;

  return 0;
}

static long tmf8806_ops_core_ioctl(struct v4l2_subdev *subdev, unsigned int cmd, void *arg)
{
  int ret = 0;
  tmf8806_chip *chip = container_of(subdev, tmf8806_chip, subdev);

  switch (cmd) {
  case VIDIOC_MTK_G_TOF_INIT: {
    LOG_INF("VIDIOC_MTK_G_TOF_INIT\n");
    struct oplus_tof_info* u_tof_info = arg;
    memset(u_tof_info, 0, sizeof(struct oplus_tof_info));
    chip_enable_store(subdev->dev, &dev_attr_chip_enable, "1", 1);
    capture_store(subdev->dev, &dev_attr_capture, "1", 1);
    thread_store(subdev, 1);
  } break;
  case VIDIOC_MTK_G_TOF_INFO: {
    LOG_INF("VIDIOC_MTK_G_TOF_INFO\n");
    struct oplus_tof_info* u_tof_info = arg;

    if (atomic_read(&g_data_valid) == 0) {
      return -EFAULT;
    }
    u_tof_info->num_of_rows = 1;
    u_tof_info->num_of_cols = 1; /* Unit : mm */
    u_tof_info->dmax_distance = 2500;
    u_tof_info->timestamp = 0;
    u_tof_info->error_status = 0;
    AMS_MUTEX_LOCK(&chip->datalock);
    u_tof_info->ranging_distance = chip->ranging_distance;
    u_tof_info->confidence = chip->confidence;
    u_tof_info->result_cnt = chip->result_cnt;
    LOG_INF("distance = %d, confidence = %d, result count = %d\n",
      u_tof_info->ranging_distance, u_tof_info->confidence, u_tof_info->result_cnt);
    AMS_MUTEX_UNLOCK(&chip->datalock);
  } break;
  case VIDIOC_OPLUS_G_TOF_CALIB: {
    LOG_INF("VIDIOC_OPLUS_G_TOF_CALIB\n");
    struct oplus_tof_data *u_tof_info = arg;

    thread_store(subdev, 0);
    ret = app0_fac_calib_show(subdev->dev, &dev_attr_app0_fac_calib, u_tof_info->calibration_data);
    thread_store(subdev, 1);
    if (ret < 0) {
      LOG_INF("app0_get_fac_calib_show fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_G_TOF_TRIM: {
    LOG_INF("VIDIOC_OPLUS_G_TOF_TRIM is not used\n");
  } break;
  case VIDIOC_OPLUS_G_TOF_XTALK: {
    LOG_INF("VIDIOC_OPLUS_G_TOF_XTALK\n");
    struct oplus_tof_data *u_tof_data = arg;
    uint16_t xtalk_peak = chip->xtalk;
    snprintf(u_tof_data->xtalk_data, sizeof(u_tof_data->xtalk_data), "%u", xtalk_peak);
    LOG_INF("xtalk peak = %d\n", xtalk_peak);
  } break;
  case VIDIOC_OPLUS_S_TOF_CAPTURE: {
    uint8_t *data = arg;
    LOG_INF("VIDIOC_OPLUS_S_TOF_CAPTURE %d\n", *data);
    capture_store(subdev->dev, &dev_attr_capture, (const char *)arg, 1);
    thread_store(subdev, *data);
  } break;
  case VIDIOC_OPLUS_S_TOF_PERIOD: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_PERIOD\n");
    ret = period_store(subdev->dev, &dev_attr_period, (const char *)arg, 1);
    if (ret < 0) {
      LOG_INF("period_store fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_S_TOF_ITERATIONS: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_ITERATIONS\n");
    ret = iterations_store(subdev->dev, &dev_attr_iterations, (const char *)arg, 1);
    if (ret < 0) {
      LOG_INF("iterations_store fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_S_TOF_ALGSETTING: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_ALGSETTING\n");
    ret = alg_setting_store(subdev->dev, &dev_attr_alg_setting, (const char *)arg, 1);
    if (ret < 0) {
      LOG_INF("alg_setting_store fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_S_TOF_CALIB: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_CALIB\n");
    thread_store(subdev, 0);
    ret = app0_fac_calib_store(subdev->dev, &dev_attr_app0_apply_fac_calib, (const char *)arg, 14);
    thread_store(subdev, 1);
    if (ret < 0) {
      LOG_INF("app0_fac_calib_store fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_S_TOF_STOP: {
    unsigned int *data = arg;
    LOG_INF("VIDIOC_OPLUS_S_TOF_STOP %d\n", *data);
    capture_store(subdev->dev, &dev_attr_capture, (const char *)arg, 1);
  } break;
  case VIDIOC_OPLUS_S_TOF_DEBUG: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_DEBUG\n");
    ret = driver_debug_store(subdev->dev, &dev_attr_driver_debug, (const char *)arg, 1);
    if (ret < 0) {
      LOG_INF("driver_debug_store fail\n");
      return ret;
    }
  } break;
  case VIDIOC_OPLUS_S_TOF_CHIP_ENABLE: {
    LOG_INF("VIDIOC_OPLUS_S_TOF_CHIP_ENABLE\n");
    chip_enable_store(subdev->dev, &dev_attr_chip_enable, (const char *)arg, 1);
  } break;
  case VIDIOC_OPLUS_S_TOF_HISTOGRAM_ENABLE: {
    unsigned int *data = arg;
    chip->histogram_enable = *data;
    LOG_INF("VIDIOC_OPLUS_S_TOF_HISTOGRAM_ENABLE %d\n", chip->histogram_enable);
  } break;
  case VIDIOC_OPLUS_S_TOF_TRIM: {
    char *data = arg;
    LOG_INF("VIDIOC_OPLUS_S_TOF_TRIM\n");
    thread_store(subdev, 0);
    ret = app0_clk_correction_store(subdev->dev, &dev_attr_app0_clk_correction, (const char *)data, 1);
    thread_store(subdev, 1);
    if (ret < 0) {
      LOG_INF("app0_clk_trim_set_store fail\n");
      return ret;
    }
  } break;
  default: {
    ret = -ENOIOCTLCMD;
  } break;
  }

  return ret;
}

static struct v4l2_subdev_core_ops tmf8806_ops_core = {
  .ioctl = tmf8806_ops_core_ioctl,
};

static const struct v4l2_subdev_ops tmf8806_ops = {
  .core = &tmf8806_ops_core,
};

static int tmf8806_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
  int ret = 0;
  LOG_INF("tmf8806_open");
//   thread_store(subdev, 1);
  return ret;
}

static int tmf8806_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
  int ret = 0;
  LOG_INF("tmf8806_close");
  atomic_set(&g_data_valid, 0);
  thread_store(subdev, 0);
  return ret;
}

static const struct v4l2_subdev_internal_ops tmf8806_int_ops = {
  .open = tmf8806_open,
  .close = tmf8806_close,
};

static int tmf8806_probe(struct i2c_client *client) {
  tmf8806_chip *chip;
  int error = 0;
  void *poll_prop_ptr = NULL;
  uint8_t app_id[1] = {TMF8806_APP_ID_APP0};
  int ret = -1;

  LOG_INF("%s", __func__);

  /* Check I2C functionality */
  LOG_INF("I2C Address: %#04x\n", client->addr);
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    dev_err(&client->dev, "I2C check functionality failed.\n");
    return -ENXIO;
  }

  /* Memory Alloc */
  chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
  if (!chip) {
    dev_err(&client->dev, "Mem kzalloc failed. \n");
    return -ENOMEM;
  }

  /* Platform Setup */
  mutex_init(&chip->lock);
  mutex_init(&chip->datalock);

  chip->client = client;
  chip->pdata = &tmf8806_pdata;

  /* v4l2 device register */
  v4l2_i2c_subdev_init(&chip->subdev, client, &tmf8806_ops);
  chip->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
  chip->subdev.internal_ops = &tmf8806_int_ops;
  ret = tmf8806_init_controls(chip);
  if (ret) {
    goto v4l2_init_err;
  }

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
  ret = media_entity_pads_init(&chip->subdev.entity, 0, NULL);
  if (ret < 0) {
    goto v4l2_init_err;
  }

  chip->subdev.entity.function = MEDIA_ENT_F_LENS;
#endif

  ret = v4l2_async_register_subdev(&chip->subdev);
  if (ret < 0) {
    goto v4l2_init_err;
  }

  error = tof_get_gpio_config(chip);
  if (error) {
    dev_err(&client->dev, "Error gpio config.\n");
    goto gpio_err;
  }


  if (writePin(chip->pdata->gpiod_enable, 1)) {
    dev_err(&client->dev, "Chip enable failed.\n");
    goto gpio_err;
  }

  /* Setup IRQ Handling */
  poll_prop_ptr = (void *)of_get_property(chip->client->dev.of_node, TOF_PROP_NAME_POLLIO, NULL);
  chip->poll_period = poll_prop_ptr ? be32_to_cpup(poll_prop_ptr) : 10;
  LOG_INF("poll_period: %d", chip->poll_period);

  /* initialize kfifo for frame output */
  INIT_KFIFO(chip->tof_output_fifo);
  chip->tof_output_frame.frame.frameId = 0;
  chip->tof_output_frame.frame.frameNumber = 0;
  chip->tof_output_frame.frame.payload_lsb = 0;
  chip->tof_output_frame.frame.payload_msb = 0;

  /* TMF8806 Setup */
  AMS_MUTEX_LOCK(&chip->lock);

  tmf8806Initialise(&chip->tof_core, APP_LOG_LEVEL);
  delayInMicroseconds(ENABLE_TIME_MS * 1000);
  tmf8806Wakeup(&chip->tof_core);

  if (tmf8806IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS) == 0) {
    dev_err(&client->dev, "CPU is not ready.\n");
    goto cpu_err;
  }

  if (i2cTxReg(chip, 0, TMF8806_REQ_APP_ID_REG, 1, app_id)) {
    dev_err(&chip->client->dev, "Error setting REQ_APP_ID register.\n");
  }

  if (tmf8806IsApp0Ready(&chip->tof_core, 20) != 1) {
    dev_err(&client->dev, "App0 not ready.\n");
    error = -EIO; // in case of error
    goto app_err;
  }
  LOG_INF("App0 is ready");

  if (chip->poll_period == 0) {
    tmf8806ClrAndEnableInterrupts(&chip->tof_core,
                                  TMF8806_INTERRUPT_RESULT |
                                      TMF8806_INTERRUPT_DIAGNOSTIC);
  }

  AMS_MUTEX_UNLOCK(&chip->lock);

  /* Sysfs Setup */
  error = sysfs_create_groups(&client->dev.kobj, tmf8806_attr_groups);
  if (error) {
    dev_err(&client->dev, "Error creating sysfs attribute group.\n");
    goto sysfs_err;
  }

  pm_runtime_enable(&client->dev);

  dev_err(&client->dev, "probed\n");

  return 0;

  /* Probe error handling */
sysfs_err:
  sysfs_remove_groups(&client->dev.kobj, tmf8806_attr_groups);
app_err:
cpu_err:
gpio_err:
  enablePinLow(chip);
  i2c_set_clientdata(client, NULL);
  AMS_MUTEX_UNLOCK(&chip->lock);

v4l2_init_err:
    v4l2_async_unregister_subdev(&chip->subdev);
    v4l2_ctrl_handler_free(&chip->ctrl_handler);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&chip->subdev.entity);
#endif

  LOG_ERR("probe failed");
  return 0;
}

static void tmf8806_remove(struct i2c_client *client) {
  tmf8806_chip *chip = i2c_get_clientdata(client);

  tmf8806StopMeasurement(&chip->tof_core);

  if (chip->pdata->gpiod_interrupt != 0 &&
      (PTR_ERR(chip->pdata->gpiod_interrupt) != -EBUSY)) {
    dev_info(&client->dev, "clear gpio irqdata %s\n", __func__);
    devm_free_irq(&client->dev, client->irq, chip);
    dev_info(&client->dev, "put %s\n", __func__);
    devm_gpiod_put(&client->dev, chip->pdata->gpiod_interrupt);
  }
  if (chip->poll_period) {
    (void)kthread_stop(chip->app0_poll_irq);
  }
  if (chip->pdata->gpiod_enable) {
    dev_info(&client->dev, "clear gpio enable %s\n", __func__);
    gpiod_direction_output(chip->pdata->gpiod_enable, 0);
    devm_gpiod_put(&client->dev, chip->pdata->gpiod_enable);
  }
  dev_info(&client->dev, "clear sys attr %s\n", __func__);
  sysfs_remove_groups(&client->dev.kobj, tmf8806_attr_groups);
  dev_info(&client->dev, "%s\n", __func__);
  i2c_set_clientdata(client, NULL);

  v4l2_async_unregister_subdev(&chip->subdev);
    v4l2_ctrl_handler_free(&chip->ctrl_handler);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&chip->subdev.entity);
#endif
  pm_runtime_disable(&client->dev);
  pm_runtime_set_suspended(&client->dev);

  return;
}

/**************************************************************************/
/* Linux Driver Specific Code                                             */
/**************************************************************************/
static struct i2c_device_id tmf8806_idtable[] = {{TMF8806_NAME, 0}, {}};

static const struct of_device_id tmf8806_of_match[] = {
    {.compatible = "ams,tmf8806"}, {}};

MODULE_DEVICE_TABLE(i2c, tmf8806_idtable);
MODULE_DEVICE_TABLE(of, tmf8806_of_match);

static struct i2c_driver tmf8806_driver = {
    .driver =
        {
            .name = TMF8806_NAME,
            .of_match_table = of_match_ptr(tmf8806_of_match),
        },
    .id_table = tmf8806_idtable,
    .probe = tmf8806_probe,
    .remove = tmf8806_remove,
};

module_i2c_driver(tmf8806_driver);

MODULE_AUTHOR("Wangjianwei");
MODULE_DESCRIPTION("ams-OSRAM AG TMF8806 ToF sensor driver");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
