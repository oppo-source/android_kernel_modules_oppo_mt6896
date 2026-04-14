// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */
#ifndef __OPLUS_KD_IMGSENSOR_H
#define __OPLUS_KD_IMGSENSOR_H

#define SENSOR_ID_OFFSET_HH                  0x2000
#define HHMAIN_SENSOR_ID                     (0x0966 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHMAIN_MIPI_RAW       "hhmain_mipi_raw"
#define HHTELE_SENSOR_ID                     (0x0882 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHTELE_MIPI_RAW       "hhtele_mipi_raw"
#define HHUTELE_SENSOR_ID                    (0x0858 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHUTELE_MIPI_RAW      "hhutele_mipi_raw"
#define HHUWIDE_SENSOR_ID                    (0x38E5 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHUWIDE_MIPI_RAW      "hhuwide_mipi_raw"
#define HHFRONT_SENSOR_ID                    (0x0615 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHFRONT_MIPI_RAW      "hhfront_mipi_raw"
#define HHMWCS_SENSOR_ID                     (0x002B + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_HHMWCS_MIPI_RAW       "hhmwcs_mipi_raw"

#define CJMAIN_SENSOR_ID                     (0xa24a + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_CJMAIN_MIPI_RAW       "cjmain_mipi_raw"
#define CJFRONT_SENSOR_ID                    (0x38E6 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_CJFRONT_MIPI_RAW      "cjfront_mipi_raw"
#define CJTELE_SENSOR_ID                     (0x1B75 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_CJTELE_MIPI_RAW       "cjtele_mipi_raw"
#define CJTELE2_SENSOR_ID                     (0x1B76 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_CJTELE2_MIPI_RAW       "cjtele2_mipi_raw"
#define YALAMAIN_SENSOR_ID                      (0x0906 + SENSOR_ID_OFFSET_HH)
#define SENSOR_DRVNAME_YALAMAIN_MIPI_RAW        "yalamain_mipi_raw"

#define OMEGAC2WIDE_SENSOR_ID                        0x0355
#define SENSOR_ID_OFFSET_23081                       0x0A00
#define OMEGAC2MAIN_SENSOR_ID_23081                  (0x0882 + SENSOR_ID_OFFSET_23081)
#define SENSOR_DRVNAME_OMEGAC2MAIN_MIPI_RAW_23081    "omegac2main_mipi_raw_23081"
#define OMEGAC2FRONT_SENSOR_ID_23081                 (0x010A + SENSOR_ID_OFFSET_23081)
#define SENSOR_DRVNAME_OMEGAC2FRONT_MIPI_RAW_23081   "omegac2front_mipi_raw_23081"
#define OMEGAC2WIDE_SENSOR_ID_23081                  (0x0355 + SENSOR_ID_OFFSET_23081)
#define SENSOR_DRVNAME_OMEGAC2WIDE_MIPI_RAW_23081    "omegac2wide_mipi_raw_23081"
#define OMEGAC2TELE_SENSOR_ID_23081                  (0x010B + SENSOR_ID_OFFSET_23081 + 1)
#define SENSOR_DRVNAME_OMEGAC2TELE_MIPI_RAW_23081    "omegac2tele_mipi_raw_23081"

#define SENSOR_ID_OFFSET_23021                       0xf000
#define IMX882_SENSOR_ID_23021                       (0x0882 + SENSOR_ID_OFFSET_23021)
#define SENSOR_DRVNAME_IMX882_MIPI_RAW_23021         "imx882_mipi_raw_23021"
#define IMX709_SENSOR_ID_23021                       (0x0709 + SENSOR_ID_OFFSET_23021)
#define SENSOR_DRVNAME_IMX709_MIPI_RAW_23021         "imx709_mipi_raw_23021"
#define IMX355_SENSOR_ID_23021                       (0x0355 + SENSOR_ID_OFFSET_23021)
#define SENSOR_DRVNAME_IMX355_MIPI_RAW_23021         "imx355_mipi_raw_23021"
#define IMX709TELE_SENSOR_ID_23021                   (0x0709 + SENSOR_ID_OFFSET_23021 + 1)
#define SENSOR_DRVNAME_IMX709TELE_MIPI_RAW_23021      "imx709tele_mipi_raw_23021"
#define IMX890_SENSOR_ID_23251                       (0x0766 + SENSOR_ID_OFFSET_23021)
#define SENSOR_DRVNAME_IMX890_MIPI_RAW_23251         "imx890_mipi_raw_23251"

#endif    /* __OPLUS_KD_IMGSENSOR_H */
