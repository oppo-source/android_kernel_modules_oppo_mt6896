load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")
load("@mgk_info//:kernel_version.bzl", "kernel_version")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_camera_wl28681c_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/wl28681c-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_24085",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_24085/jd5516w_24085.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7316m_24085",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316m_24085/ak7316m_24085.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo".format(kernel_version),
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7316t_24085",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316t_24085/ak7316t_24085.c",
        ]),
        includes = ["."],
    )
    define_oplus_ddk_module(
        name = "oplus_camera_ak7316t_24081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316t_24081/ak7316t_24081.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24081/dw9800s_24081.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9827c_24081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_24081/dw9827c_24081.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo".format(kernel_version),
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9827c_23081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_23081/dw9827c_23081.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_23081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_23081/dw9800s_23081.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_tele_23081",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_tele_23081/dw9800s_tele_23081.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_23021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_23021/dw9800s_23021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_tele_23021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_tele_23021/dw9800s_tele_23021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9827c_23021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_23021/dw9827c_23021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_23251",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_23251/jd5516w_23251.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ois_power",
        srcs = native.glob([
            "**/*.h",
            "lens/ois/ois_power/ois_power.c",
        ]),
        includes = ["."],
        ko_deps = [
            "//kernel_device_modules-{}/drivers/misc/mediatek/sensor/2.0/core:hf_manager".format(kernel_version),
        ],
        copts = [
            "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/sensor/2.0/core",
        ],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_tmf8806",
        srcs = native.glob([
            "**/*.h",
            "lens/tof/tmf8806/core_driver/tmf8806.c",
            "lens/tof/tmf8806/ams_i2c.c",
            "lens/tof/tmf8806/tmf8806_shim.c",
            "lens/tof/tmf8806/tmf8806_driver.c",
        ]),
        includes = [
            ".",
            "lens/tof/tmf8806",
            "lens/tof/tmf8806/core_driver",
        ],
        copts = ["-Wno-unused-function"],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_camera",
        module_list = [
            "oplus_camera_wl28681c_regulator.ko",
            "oplus_camera_tmf8806.ko",
            "oplus_camera_ois_power.ko",
            "oplus_camera_ak7316m_24085.ko",
            "oplus_camera_ak7316t_24085.ko",
            "oplus_camera_jd5516w_24085.ko",
            "oplus_camera_dw9827c_24081.ko",
            "oplus_camera_dw9800s_24081.ko",
            "oplus_camera_ak7316t_24081.ko",
            "oplus_camera_dw9827c_23081.ko",
            "oplus_camera_dw9800s_23081.ko",
            "oplus_camera_dw9800s_tele_23081.ko",
            "oplus_camera_dw9800s_23021.ko",
            "oplus_camera_dw9800s_tele_23021.ko",
            "oplus_camera_dw9827c_23021.ko",
            "oplus_camera_jd5516w_23251.ko",
        ],
    )

    ddk_headers(
        name = "oplus_camera_header",
        hdrs = native.glob([
            "cam_cal/common/*.h",
            "imgsensor/inc/*.h",
        ]),
        includes = ["cam_cal/common", "imgsensor/inc"],
        visibility = ["//visibility:public"],
    )
