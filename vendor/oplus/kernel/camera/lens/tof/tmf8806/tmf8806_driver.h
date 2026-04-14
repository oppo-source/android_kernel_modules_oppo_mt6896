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

/*
 * History of Versions:
 *    1.0 ... initial version
 *    1.1 ... clock correction changed, output fifo buffer increased
 *        ... histogram scaled for output data in uint_32
 *        ... check output buffer size before histograms are dumped
 *        ... app0_apply_fac_calib_store wait for interrupt
 *    1.2 ... histogram dumping not scaled
 *        ... output data with header
 *        ... fix serial number readout
 */

/*! \file tmf8806_driver.h - TMF8806 driver
 * \brief Device driver for measuring Distance in mm.
 */

#ifndef TMF8806_DRIVER_H
#define TMF8806_DRIVER_H

/* -------------------------------- includes -------------------------------- */
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "tmf8806.h"
#include "tmf8806_shim.h"
#include "tmf8806_regs.h"
#include "tmf8806.h"

/* -------------------------------- defines --------------------------------- */
#define MAX_REGS 256                                // i2c max accessable registers
#define TOF_GPIO_INT_NAME     "irq"                 // pin from overlay file
#define TOF_GPIO_ENABLE_NAME  "enable"              // pin from overlay file
#define TOF_PROP_NAME_POLLIO  "poll_period"         // property in overlay file
#define TOF_XTALK_PEAK_COUNT        5

/* -------------------------------- macros ---------------------------------- */
#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
    pr_info("%s: Mutex Lock\n", __func__); \
    mutex_lock_interruptible(m); \
  }
#define AMS_MUTEX_UNLOCK(m) { \
    pr_info("%s: Mutex Unlock\n", __func__); \
    mutex_unlock(m); \
  }
#else
#define AMS_MUTEX_LOCK(m) { \
    mutex_lock(m); \
  }
#define AMS_MUTEX_TRYLOCK(m) ({ \
    mutex_trylock(m); \
})
#define AMS_MUTEX_UNLOCK(m) { \
    mutex_unlock(m); \
  }
#endif

/* -------------------------------- structures ------------------------------ */
struct tmf8806_output {
  char frameId;
  char frameNumber;
  char payload_lsb;
  char payload_msb;
  char payload[TMF8806_NUMBER_HISTOGRAMS*TMF8806_DATA_BUFFER_SIZE*2]; /* 2 bytes per bin */
};

union tmf8806_output_frame {
  struct tmf8806_output frame;
  char buf[sizeof(struct tmf8806_output)];
}__attribute__((packed));

struct tmf8806_platform_data {
  const char *tof_name;
  struct gpio_desc *gpiod_interrupt;
  struct gpio_desc *gpiod_enable;
  const char *ram_patch_fname[]; //FIXME Firmware download
};

typedef struct _tmf8806_chip
{
  tmf8806Driver tof_core;

  struct i2c_client *client;
  struct mutex lock;
  struct mutex datalock;
  struct tmf8806_platform_data *pdata;
  u8 shadow[256];
  STRUCT_KFIFO_REC_2(PAGE_SIZE*4) tof_output_fifo;
  struct task_struct *app0_poll_irq;
  int poll_period;
  union tmf8806_output_frame tof_output_frame;
  /* v4l2 */
  bool is_inited;
  struct v4l2_ctrl_handler ctrl_handler;
  struct v4l2_subdev subdev;
  int histogram_enable;
  uint16_t xtalk;
  uint16_t xtalk_count;
  int32_t ranging_distance;
  int32_t confidence;
  int32_t result_cnt;
}tmf8806_chip;

/* -------------------------------- functions ------------------------------- */
/**
 * tof_queue_frame - queue data of buffer into output fifo
 *
 * @tmf8806_chip: tmf8806_chip pointer
 *
 * Returns 0 for no Error, -1 for Error
 */
int tof_queue_frame(tmf8806_chip *chip);

#endif