load(":configs/oplus_k6991v1_64_config.bzl", "oplus_k6991v1_64_config")
load(":oplus_platform.bzl", "COMMON_OPLUS_MTK_PLATFORM")

oplus_config = {
    "k6991v1_64": oplus_k6991v1_64_config
}

def oplus_modules_get_config(target):
    if target not in oplus_config:
        fail("target: \"{}\" not support".format(target))
        return {}

    return oplus_config[target]

def oplus_modules_get_target_list():
    return [COMMON_OPLUS_MTK_PLATFORM]
