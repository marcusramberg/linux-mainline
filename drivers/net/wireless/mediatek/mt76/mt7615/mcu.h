/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2019 MediaTek Inc. */

#ifndef __MT7615_MCU_H
#define __MT7615_MCU_H

#include "../mt76_connac_mcu.h"

struct mt7615_mcu_txd {
	__le32 txd[8];

	__le16 len;
	__le16 pq_id;

	u8 cid;
	u8 pkt_type;
	u8 set_query; /* FW don't care */
	u8 seq;

	u8 uc_d2b0_rev;
	u8 ext_cid;
	u8 s2d_index;
	u8 ext_cid_ack;

	u32 reserved[5];
} __packed __aligned(4);

/**
 * struct mt7615_uni_txd - mcu command descriptor for firmware v3
 * @txd: hardware descriptor
 * @len: total length not including txd
 * @cid: command identifier
 * @pkt_type: must be 0xa0 (cmd packet by long format)
 * @frag_n: fragment number
 * @seq: sequence number
 * @checksum: 0 mean there is no checksum
 * @s2d_index: index for command source and destination
 *  Definition              | value | note
 *  CMD_S2D_IDX_H2N         | 0x00  | command from HOST to WM
 *  CMD_S2D_IDX_C2N         | 0x01  | command from WA to WM
 *  CMD_S2D_IDX_H2C         | 0x02  | command from HOST to WA
 *  CMD_S2D_IDX_H2N_AND_H2C | 0x03  | command from HOST to WA and WM
 *
 * @option: command option
 *  BIT[0]: UNI_CMD_OPT_BIT_ACK
 *          set to 1 to request a fw reply
 *          if UNI_CMD_OPT_BIT_0_ACK is set and UNI_CMD_OPT_BIT_2_SET_QUERY
 *          is set, mcu firmware will send response event EID = 0x01
 *          (UNI_EVENT_ID_CMD_RESULT) to the host.
 *  BIT[1]: UNI_CMD_OPT_BIT_UNI_CMD
 *          0: original command
 *          1: unified command
 *  BIT[2]: UNI_CMD_OPT_BIT_SET_QUERY
 *          0: QUERY command
 *          1: SET command
 */
struct mt7615_uni_txd {
	__le32 txd[8];

	/* DW1 */
	__le16 len;
	__le16 cid;

	/* DW2 */
	u8 reserved;
	u8 pkt_type;
	u8 frag_n;
	u8 seq;

	/* DW3 */
	__le16 checksum;
	u8 s2d_index;
	u8 option;

	/* DW4 */
	u8 reserved2[4];
} __packed __aligned(4);

enum {
    MT_SKU_CCK_1_2 = 0,
    MT_SKU_CCK_55_11,
    MT_SKU_OFDM_6_9,
    MT_SKU_OFDM_12_18,
    MT_SKU_OFDM_24_36,
    MT_SKU_OFDM_48,
    MT_SKU_OFDM_54,
    MT_SKU_HT20_0_8,
    MT_SKU_HT20_32,
    MT_SKU_HT20_1_2_9_10,
    MT_SKU_HT20_3_4_11_12,
    MT_SKU_HT20_5_13,
    MT_SKU_HT20_6_14,
    MT_SKU_HT20_7_15,
    MT_SKU_HT40_0_8,
    MT_SKU_HT40_32,
    MT_SKU_HT40_1_2_9_10,
    MT_SKU_HT40_3_4_11_12,
    MT_SKU_HT40_5_13,
    MT_SKU_HT40_6_14,
    MT_SKU_HT40_7_15,
    MT_SKU_VHT20_0,
    MT_SKU_VHT20_1_2,
    MT_SKU_VHT20_3_4,
    MT_SKU_VHT20_5_6,
    MT_SKU_VHT20_7,
    MT_SKU_VHT20_8,
    MT_SKU_VHT20_9,
    MT_SKU_VHT40_0,
    MT_SKU_VHT40_1_2,
    MT_SKU_VHT40_3_4,
    MT_SKU_VHT40_5_6,
    MT_SKU_VHT40_7,
    MT_SKU_VHT40_8,
    MT_SKU_VHT40_9,
    MT_SKU_VHT80_0,
    MT_SKU_VHT80_1_2,
    MT_SKU_VHT80_3_4,
    MT_SKU_VHT80_5_6,
    MT_SKU_VHT80_7,
    MT_SKU_VHT80_8,
    MT_SKU_VHT80_9,
    MT_SKU_VHT160_0,
    MT_SKU_VHT160_1_2,
    MT_SKU_VHT160_3_4,
    MT_SKU_VHT160_5_6,
    MT_SKU_VHT160_7,
    MT_SKU_VHT160_8,
    MT_SKU_VHT160_9,
    MT_SKU_1SS_DELTA,
    MT_SKU_2SS_DELTA,
    MT_SKU_3SS_DELTA,
    MT_SKU_4SS_DELTA,
};

struct mt7615_mcu_rxd {
	__le32 rxd[4];

	__le16 len;
	__le16 pkt_type_id;

	u8 eid;
	u8 seq;
	__le16 __rsv;

	u8 ext_eid;
	u8 __rsv1[2];
	u8 s2d_index;
};

struct mt7615_mcu_csa_notify {
	struct mt7615_mcu_rxd rxd;

	u8 omac_idx;
	u8 csa_count;
	u8 rsv[2];
} __packed;

struct mt7615_mcu_rdd_report {
	struct mt7615_mcu_rxd rxd;

	u8 band_idx;
	u8 long_detected;
	u8 constant_prf_detected;
	u8 staggered_prf_detected;
	u8 radar_type_idx;
	u8 periodic_pulse_num;
	u8 long_pulse_num;
	u8 hw_pulse_num;

	u8 out_lpn;
	u8 out_spn;
	u8 out_crpn;
	u8 out_crpw;
	u8 out_crbn;
	u8 out_stgpn;
	u8 out_stgpw;

	u8 _rsv[2];

	__le32 out_pri_const;
	__le32 out_pri_stg[3];

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
	} long_pulse[32];

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
	} periodic_pulse[32];

	struct {
		__le32 start;
		__le16 pulse_width;
		__le16 pulse_power;
		u8 sc_pass;
		u8 sw_reset;
	} hw_pulse[32];
};

enum {
	MCU_ATE_SET_FREQ_OFFSET = 0xa,
	MCU_ATE_SET_TX_POWER_CONTROL = 0x15,
};

struct mt7615_roc_tlv {
	u8 bss_idx;
	u8 token;
	u8 active;	/* 1 releases the channel */
	u8 primary_chan;
	u8 sco;
	u8 band;
	u8 width;	/* To support 80/160MHz bandwidth */
	u8 freq_seg1;	/* To support 80/160MHz bandwidth */
	u8 freq_seg2;	/* To support 80/160MHz bandwidth */
	u8 req_type;
	u8 dbdc_band;
	u8 rsv0;
	__le32 max_interval;	/* ms */
	u8 rsv1[8];
} __packed;

enum {
	FW_STATE_PWR_ON = 1,
	FW_STATE_N9_RDY = 2,
};

enum {
	DBDC_TYPE_WMM,
	DBDC_TYPE_MGMT,
	DBDC_TYPE_BSS,
	DBDC_TYPE_MBSS,
	DBDC_TYPE_REPEATER,
	DBDC_TYPE_MU,
	DBDC_TYPE_BF,
	DBDC_TYPE_PTA,
	__DBDC_TYPE_MAX,
};

/* MT6768: the legacy CE command set, all its firmware has */
#define MT6768_STA_TYPE_AP		0x41
#define MT6768_STA_TYPE_CLIENT		0x21
#define MT6768_EVENT_RX_ADDBA		0x0a
#define MT6768_EVENT_RX_DELBA		0x0b

struct mt6768_tx_done {
	u8 pid;
	u8 status;
	__le16 sn;
	u8 wlan_idx;
	u8 tx_count;
	__le16 tx_rate;
	u8 rsv[80];
} __packed;

#define MT6768_STA_STATE_1		0
#define MT6768_STA_STATE_3		2
#define MT6768_STA_ALL			27

#define MT6768_PHY_HR_DSSS		BIT(0)
#define MT6768_PHY_ERP			BIT(1)
#define MT6768_PHY_OFDM			BIT(3)
#define MT6768_PHY_HT			BIT(4)
#define MT6768_PHY_VHT			BIT(5)

#define MT6768_BAND_2G			1
#define MT6768_BAND_5G			2
#define MT6768_SCA			1
#define MT6768_SCB			3
#define MT6768_DBDC_AUTO		4
#define MT6768_CH_REQ_JOIN		0

#define MT6768_RATE_CCK_1M		0x0000
#define MT6768_RATE_OFDM_6M		0x004b

struct mt6768_bss_activate {
	u8 bss_idx;
	u8 active;
	u8 net_type;
	u8 omac_idx;
	u8 addr[ETH_ALEN];
	u8 bmc_wlan_idx;
	u8 rsv;
} __packed;

struct mt6768_bss_rlm {
	u8 bss_idx;
	u8 band;
	u8 primary_ch;
	u8 sco;
	u8 erp_prot;
	u8 ht_prot;
	u8 gf_mode;
	u8 tx_rifs;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 short_preamble;
	u8 short_slot;
	u8 vht_width;
	u8 vht_seg1;
	u8 vht_seg2;
	__le16 vht_basic_mcs;
	u8 tx_nss;
	u8 rx_nss;
} __packed;

struct mt6768_bss_info {
	u8 bss_idx;
	u8 conn_state;		/* 0 connected, 1 disconnected */
	u8 op_mode;		/* 0 infra, 2 AP */
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];
	u8 qos;
	u8 version;
	__le16 rates;
	__le16 basic_rates;
	u8 ap_sta_idx;
	u8 rsv0;
	__le16 fixed_rate;
	u8 non_ht_phy;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_types;
	u8 wapi;
	u8 ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid;
	u8 disconnect_threshold;
	u8 iot_act;
	u8 rsv1[2];
	__le32 private;
	struct mt6768_bss_rlm rlm;
	u8 dbdc_band;
	u8 wmm_set;
	u8 dbdc_action;
	u8 nss;
	u8 rsv2[22];
} __packed;

struct mt6768_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_types;
	__le16 rates;
	__le16 basic_rates;
	u8 qos;
	u8 uapsd;
	u8 state;
	u8 mcs;
	u8 mcs32;
	u8 version;
	u8 rx_mcs_mask[10];
	__le16 rx_highest;
	__le32 tx_rate_info;
	__le16 ht_cap;
	__le16 ht_ext_cap;
	__le32 txbf_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap;
	__le16 vht_rx_mcs_map;
	__le16 vht_rx_highest;
	__le16 vht_tx_mcs_map;
	__le16 vht_tx_highest;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_type;
	u8 tx_gf;
	u8 tx_sgi;
	u8 tx_stbc;
	__le16 fixed_rate;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 rsv0[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	u8 tx_ba_size;
	u8 rx_ba_size;
	u8 rsv1[34];
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 rsv2[2];
	__le32 max_amsdu_len;
	u8 rsv3[32];
} __packed;

struct mt6768_sta_remove {
	u8 action;		/* 0 one STA, 1 all of the BSS */
	u8 sta_idx;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct mt6768_key {
	u8 add;
	u8 tx_key;
	u8 pairwise;
	u8 authenticator;
	u8 peer_addr[ETH_ALEN];
	u8 bss_idx;
	u8 cipher;
	u8 key_id;
	u8 key_len;
	u8 wlan_idx;
	u8 mgmt_prot;
	u8 key[32];
	u8 rsc[16];
} __packed;

struct mt6768_ac_params {
	__le16 cw_min;
	__le16 cw_max;
	__le16 txop;
	__le16 aifs;
	u8 guard_time;
	u8 acm;
} __packed;

struct mt6768_wmm {
	struct mt6768_ac_params ac[IEEE80211_NUM_ACS];	/* BE, BK, VI, VO */
	u8 bss_idx;
	u8 qos;
	u8 wmm_set;
	u8 rsv;
} __packed;

struct mt6768_subband {
	u8 reg_class;
	u8 band;
	u8 span;
	u8 first_ch;
	u8 n_ch;
	u8 rsv[3];
} __packed;

struct mt6768_domain_info {
	__le16 country;
	__le16 passive;
	struct mt6768_subband sub[6];
	u8 bw_2g;
	u8 bw_5g;
	u8 rsv[2];
} __packed;

#endif
