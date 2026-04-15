#load("@mgk_info//:dict.bzl", "ANDROID_TOP","TARGET_BUILD_VARIANT")
OS = "linux"

def generate_wlan_module(config_opts):
    copts_def =[
        "-Werror",
        "-Wall",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/include".format(OS),
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/nic",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/mgmt",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/chips",
        "-Wno-error=unused-but-set-variable",
        "-Wno-error=tautological-overlap-compare",
        "-Wno-cast-function-type-strict",
    ]

    copts_def += [
        "-I/usr/include/"]if OS == "none" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/nan",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/nan/wpa_supp",
    ]if config_opts["CONFIG_MTK_WIFI_NAN"] == "y" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/include/dvt",
    ]if config_opts["CFG_SUPPORT_WIFI_SYSDVT"] == "1" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/sdio/include".format(OS),
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "sdio" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/pcie/include".format(OS),
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/common/include".format(OS),
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "pcie" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/axi/include".format(OS),
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/common/include".format(OS),
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "axi" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/usb/include".format(OS),
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "usb" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/test",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/test/lib/include",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/test/testcases",
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/test/lib/hif",
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "ut" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/os/{}/hif/none/include".format(OS),
    ]if config_opts["CONFIG_MTK_COMBO_WIFI_HIF"] == "none" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/prealloc/include",
    ]if config_opts["CONFIG_MTK_PREALLOC_MEMORY"] == "y" else []
    copts_def += [
        "-I$(TOP)/vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4-mt79xx/reset/include",
    ]if config_opts["CONFIG_CHIP_RESET_KO_SUPPORT"] == "y" else []

    srcs_diff = []

    srcs_diff += [
        "nic/nic_uni_cmd_event.c",
    ] if config_opts["CONFIG_MTK_WIFI_UNIFIED_COMMND_SUPPORT"] == "y" else []
    srcs_diff += [
        "os/linux/gl_fw_dev.c",
    ] if config_opts["CONFIG_SUPPORT_FW_IDX_LOG_SAVE"] == "y" else []
    srcs_diff += [
        "os/linux/gl_csi.c",
    ] if config_opts["CONFIG_MTK_WIFI_CSI_VNI"] == "y" else []
    srcs_diff += [
        "os/linux/gl_fmcw.c",
    ] if config_opts["CONFIG_MTK_WIFI_FMCW_VNI"] == "y" else []
    #srcs_diff += [
    #    "os/linux/plat/mt8195/plat_priv.c",
    #    ] if CONFIG_MTK_PLATFORM == "mt8195" else []
    srcs_diff += [
        "mgmt/he_ie.c",
        "mgmt/he_rlm.c"
    ] if config_opts["CONFIG_MTK_WIFI_11AX_SUPPORT"] == "y" else []
    srcs_diff += [
        "mgmt/twt_req_fsm.c",
        "mgmt/twt.c",
        "mgmt/twt_planner.c",
    ] if config_opts["CONFIG_MTK_WIFI_TWT_SUPPORT"] == "y" else []
    srcs_diff += [
        "chips/common/fw_log_parser.c",
    ] if config_opts["CONFIG_SUPPORT_FW_IDX_LOG_TRANS"] == "y" else []
    srcs_diff += [
        "dvt/dvt_common.c",
    ] if config_opts["CFG_SUPPORT_WIFI_SYSDVT"] == "1" else []
    srcs_diff += [
        "dvt/dvt_dmashdl.c",
    ] if config_opts["CFG_SUPPORT_DMASHDL_SYSDVT"] == "1" else []
    srcs_diff += [
        "nic/nic_ext_cmd_event.c",
        "nic/nic_txd_v2.c",
        "nic/nic_rxd_v2.c",
        "chips/common/dbg_connac2x.c",
        "chips/common/cmm_asic_connac2x.c",
    ] if config_opts["CONFIG_MTK_WIFI_CONNAC2X"] == "y" else []
    srcs_diff += [
        "nic/nic_ext_cmd_event.c",
        "nic/nic_txd_v3.c",
        "nic/nic_rxd_v3.c",
    ] if config_opts["CONFIG_MTK_WIFI_CONNAC3X"] == "y" else []
    srcs_diff += [
        "os/linux/gl_nan.c",
        "os/linux/gl_vendor_nan.c",
        "os/linux/gl_vendor_ndp.c",
        "nan/nan_dev.c",
        "nan/nanDiscovery.c",
        "nan/nanScheduler.c",
        "nan/nanReg.c",
        "nan/nan_data_engine.c",
        "nan/nan_data_engine_util.c",
        "nan/nan_ranging.c",
        "nan/nan_txm.c",
        "nan/nan_pairing.c",
        "nan/nan_sec.c",
        "nan/wpa_supp/FourWayHandShake.c",
        "nan/wpa_supp/src/ap/wpa_auth_ie.c",
        "nan/wpa_supp/src/ap/wpa_auth.c",
        "nan/wpa_supp/src/crypto/sha1.c",
        "nan/wpa_supp/src/crypto/sha1-internal.c",
        "nan/wpa_supp/src/crypto/sha1-prf.c",
        "nan/wpa_supp/src/crypto/aes-wrap.c",
        "nan/wpa_supp/src/crypto/aes-internal.c",
        "nan/wpa_supp/src/common/wpa_common.c",
        "nan/wpa_supp/src/utils/common.c",
        "nan/wpa_supp/src/rsn_supp/wpa.c",
        "nan/wpa_supp/src/crypto/aes-unwrap.c",
        "nan/wpa_supp/src/crypto/aes-internal-enc.c",
        "nan/wpa_supp/src/crypto/aes-internal-dec.c",
        "nan/wpa_supp/src/crypto/sha256.c",
        "nan/wpa_supp/src/crypto/sha256-prf.c",
        "nan/wpa_supp/src/crypto/sha256-internal.c",
        "nan/wpa_supp/wpa_supplicant/wpas_glue.c",
        "nan/wpa_supp/wpa_supplicant/wpa_supplicant.c",
        "nan/wpa_supp/src/ap/wpa_auth_glue.c",
        "nan/wpa_supp/src/crypto/pbkdf2-sha256.c",
        "nan/wpa_supp/src/crypto/sha384-internal.c",
        "nan/wpa_supp/src/crypto/sha512-internal.c",
        "nan/wpa_supp/src/crypto/sha384-prf.c",
        "nan/wpa_supp/src/crypto/sha384.c",
    ] if config_opts["CONFIG_MTK_WIFI_NAN"] == "y" else []
    return  copts_def, srcs_diff
