def _impl(repository_ctx):
    repository_content = ""
    keys = ["DEFCONFIG_OVERLAYS","KERNEL_VERSION","SOURCE_DATE_EPOCH","SUPPORT_PLATFORM","KERNEL_BUILD_VARIANT","OPLUS_PLATFORM_INFO","OPLUS_FEATURES","OPLUS_USE_PREBUILT_BOOTIMAGE"]
    for key in keys:
      if key in repository_ctx.os.environ:
          value = repository_ctx.os.environ[key].strip()
      else:
          value = ""
      repository_content += '{} = "{}"\n'.format(key, value)

    for key, value in repository_ctx.attr.additional_values.items():
        repository_content += '{} = "{}"\n'.format(key, value)

    repository_ctx.file("BUILD", """
load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
bzl_library(
    name = "dict",
    srcs = ["dict.bzl"],
    visibility = ["//visibility:public"],
)
""", executable = False)
    repository_ctx.file("dict.bzl", repository_content, executable = False)

    kernel_version = repository_ctx.os.environ["KERNEL_VERSION"].split("-")[1]
    repository_ctx.file("kernel_version.bzl", """
kernel_version = "{kernel_version}"
""".format(kernel_version = kernel_version), executable = False)

    device_module_dir = "kernel_device_modules-" + kernel_version
    repository_ctx.file("kleaf_device_modules.bzl", """
load("@//{device_module_dir}:kernel/kleaf/mgk_64.bzl",
    _mgk_64_kleaf_device_modules_srcs="mgk_64_kleaf_device_modules_srcs",
    _mgk_64_kleaf_device_modules="mgk_64_kleaf_device_modules",
    _mgk_64_kleaf_eng_device_modules="mgk_64_kleaf_eng_device_modules",
    _mgk_64_kleaf_userdebug_device_modules="mgk_64_kleaf_userdebug_device_modules",
    _mgk_64_kleaf_user_device_modules="mgk_64_kleaf_user_device_modules",
    _mgk_64_kleaf_ack_device_modules="mgk_64_kleaf_user_device_modules"
)
kleaf_device_modules_srcs = _mgk_64_kleaf_device_modules_srcs
kleaf_eng_device_modules = _mgk_64_kleaf_device_modules + _mgk_64_kleaf_eng_device_modules
kleaf_userdebug_device_modules = _mgk_64_kleaf_device_modules + _mgk_64_kleaf_userdebug_device_modules
kleaf_user_device_modules = _mgk_64_kleaf_device_modules + _mgk_64_kleaf_user_device_modules
kleaf_ack_device_modules = _mgk_64_kleaf_device_modules + _mgk_64_kleaf_ack_device_modules
""".format(device_module_dir = device_module_dir), executable = False)

key_value_repo = repository_rule(
    implementation = _impl,
    local = True,
    environ = [
        "DEFCONFIG_OVERLAYS",
        "KERNEL_VERSION",
        "SOURCE_DATE_EPOCH",
        "OPLUS_PLATFORM_INFO",
        "OPLUS_FEATURES",
        "SUPPORT_PLATFORM",
        "KERNEL_BUILD_VARIANT",
        "OPLUS_USE_PREBUILT_BOOTIMAGE",
    ],
    attrs = {
        "additional_values": attr.string_dict(),
    },
)
