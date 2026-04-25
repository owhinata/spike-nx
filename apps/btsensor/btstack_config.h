/* btstack_config.h for SPIKE Prime Hub (STM32F413 + TI CC2564C) SPP IMU
 * streaming application.  Classic BT + RFCOMM + SDP only; BLE features are
 * disabled.  See /Users/ouwa/.claude/plans/ssp-bt-lexical-pudding.md and
 * GitHub Issue #52.
 */

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

/* Port / platform features */
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_MALLOC

/* Logging */
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_PRINTF_HEXDUMP

/* Protocol features */
#define ENABLE_CLASSIC
#define ENABLE_CC256X_BAUDRATE_CHANGE_FLOWCONTROL_BUG_WORKAROUND

/* BLE is disabled in Step B–F.  Leave the macros undef'd so the
 * corresponding btstack sources compile out or link as stubs.
 */

/* Sizing.  CC2564C supports up to (1691 + 4) byte ACL payload. */
#define HCI_ACL_PAYLOAD_SIZE            (1691 + 4)
#define HCI_INCOMING_PRE_BUFFER_SIZE    14

/* GATT Server DB is not used (GATT needs ENABLE_BLE). */
#define MAX_ATT_DB_SIZE                 0

/* 1 PC connection, 1 RFCOMM channel. */
#define MAX_NR_HCI_CONNECTIONS          1
#define MAX_NR_L2CAP_SERVICES           2
#define MAX_NR_L2CAP_CHANNELS           2
#define MAX_NR_RFCOMM_MULTIPLEXERS      1
#define MAX_NR_RFCOMM_SERVICES          1
#define MAX_NR_RFCOMM_CHANNELS          1
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES  2
#define MAX_NR_SERVICE_RECORD_ITEMS     1

/* Silence btstack_memory.c "not defined" warnings for audio/HID/network
 * features we do not enable.
 */
#define MAX_NR_AVDTP_CONNECTIONS        0
#define MAX_NR_AVDTP_STREAM_ENDPOINTS   0
#define MAX_NR_AVRCP_BROWSING_CONNECTIONS 0
#define MAX_NR_AVRCP_CONNECTIONS        0
#define MAX_NR_BNEP_CHANNELS            0
#define MAX_NR_BNEP_SERVICES            0
#define MAX_NR_GATT_CLIENTS             0
#define MAX_NR_GOEP_SERVER_CONNECTIONS  0
#define MAX_NR_GOEP_SERVER_SERVICES     0
#define MAX_NR_HFP_CONNECTIONS          0
#define MAX_NR_HIDS_CLIENTS             0
#define MAX_NR_HID_HOST_CONNECTIONS     0
#define MAX_NR_LE_DEVICE_DB_ENTRIES     0
#define MAX_NR_SM_LOOKUP_ENTRIES        0
#define MAX_NR_WHITELIST_ENTRIES        0

#endif /* BTSTACK_CONFIG_H */
