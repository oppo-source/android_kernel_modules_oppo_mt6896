/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*! \file   rtt.h
 *  \brief  The rtt related define, macro and structure are described here.
 */

#ifndef _RTT_H
#define _RTT_H

#if (CFG_SUPPORT_RTT == 1)
/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define RTT_REQUEST_DONE_TIMEOUT_SEC 4
#define RTT_REQUEST_CONT_TIMEOUT_SEC 2

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/* RTT state */
enum ENUM_RTT_STATE {
	RTT_STATE_IDLE,
	RTT_STATE_START,
	RTT_STATE_DONE,
	RTT_STATE_NUM
};

enum ENUM_RTT_EVENT_TYPE {
	RTT_EVENT_PER_PACKET = 0x1,
	RTT_EVENT_PER_BURST  = 0x2,
	RTT_EVENT_NUM
};

struct RTT_INFO {
	uint8_t ucBssIndex;
	uint8_t fgIsRunning;
	uint8_t fgIsContRunning; /* for continuous RTT requests */
	uint8_t ucSeqNum;
	enum ENUM_RTT_STATE ucState;
	enum ENUM_RTT_PEER_TYPE eRttPeerType;
	struct LINK rResultList;
	struct TIMER rRttDoneTimer;
	struct TIMER rRttContTimer; /* for continuous RTT requests */
};

struct RTT_RESULT_ENTRY {
	struct LINK_ENTRY rLinkEntry;
	struct RTT_RESULT rResult;
	uint16_t u2IELen;
	/* Keep it last */
	uint8_t aucIE[];
};

struct PARAM_RTT_REQUEST {
	uint8_t fgEnable;
	uint8_t ucConfigNum;
	struct RTT_CONFIG arRttConfigs[CFG_RTT_MAX_CANDIDATES];
};

struct CMD_RTT_REQUEST {
	uint8_t ucSeqNum;
	uint8_t fgEnable;              /* request or cancel */
	uint8_t ucConfigNum;
	uint8_t ucPaddings[5];
	struct RTT_CONFIG arRttConfigs[CFG_RTT_MAX_CANDIDATES];
};

struct EVENT_RTT_RESULT {
	struct RTT_RESULT rResult;
	uint16_t u2IELen;
	/* Keep it last */
	uint8_t aucIE[];
};

struct EVENT_RTT_DONE {
	uint8_t ucSeqNum;
};
/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
struct RTT_INFO *rttGetInfo(struct ADAPTER *prAdapter);

void rttInit(struct ADAPTER *prAdapter);

void rttUninit(struct ADAPTER *prAdapter);

uint8_t rttIsRunning(struct ADAPTER *prAdapter);

uint8_t rttBwToBssBw(uint8_t eRttBw);

uint8_t rttBssBwToRttBw(uint8_t ucBssBw);

uint8_t rttMaxBwToRttBw(uint8_t ucMaxBw);

uint8_t rttRttBwToMaxBw(uint8_t eRttBw);

uint32_t rttHandleRttRequest(struct ADAPTER *prAdapter,
	struct PARAM_RTT_REQUEST *prRequest,
	uint8_t ucBssIndex);

void rttEventGetCapabilities(struct ADAPTER *prAdapter,
	struct CMD_INFO *prCmdInfo, uint8_t *pucEventBuf);

void rttEventDone(struct ADAPTER *prAdapter,
	struct EVENT_RTT_DONE *prEvent);

void rttEventResult(struct ADAPTER *prAdapter,
	struct EVENT_RTT_RESULT *prEvent);

#if (CFG_SUPPORT_RTT_RSTA == 1)
void rttProcessPublicAction(struct ADAPTER *prAdapter,
		struct SW_RFB *prSwRfb);
#endif /* CFG_SUPPORT_RTT_RSTA */

#endif /* CFG_SUPPORT_RTT */
#endif /* _RTT_H */
