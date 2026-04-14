/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 OPLUS Inc.
 */

#ifndef __OPLUS_CAM_CAL_LIST_H
#define __OPLUS_CAM_CAL_LIST_H

#include "oplus_kd_imgsensor.h"

#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_oplusCamCalList[] = {
	{HHFRONT_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HHMAIN_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{HHTELE_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{HHUWIDE_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{CJFRONT_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{CJMAIN_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{CJTELE_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{CJTELE2_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{OMEGAC2MAIN_SENSOR_ID_23081, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{OMEGAC2FRONT_SENSOR_ID_23081, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OMEGAC2TELE_SENSOR_ID_23081, 0xA1, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OMEGAC2WIDE_SENSOR_ID_23081, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX890_SENSOR_ID_23251, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX882_SENSOR_ID_23021, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX709TELE_SENSOR_ID_23021, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_23021, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX709_SENSOR_ID_23021, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
    /*  ADD before this line */
    {0, 0, 0}       /*end of list */
};

#endif        /* __OPLUS_CAM_CAL_LIST_H */
