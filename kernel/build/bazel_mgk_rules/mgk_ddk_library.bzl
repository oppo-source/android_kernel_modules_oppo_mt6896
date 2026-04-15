load("@kleaf//build/kernel/kleaf:kernel.bzl", "ddk_library")
load(
    ":mgk.bzl",
    "kernel_versions_and_projects",
)

def define_mgk_ddk_library(
    name,
    srcs,
    pkvm_el2,
    copts = [],
    header_deps = [],
    ko_deps = [],
    includes = None,
    local_defines = None,
    linux_includes = None,
):
    for version,projects in kernel_versions_and_projects.items():
        for project in projects.split(" "):
            for build in ["eng", "userdebug", "user", "ack"]:
                ack_dir = "kernel-" + version
                if build == "ack":
                    ack_dir = "common-" + version
                device_modules_dir = "kernel_device_modules-" + version
                real_copts=[]
                if copts != None:
                    for opt in copts:
                        if "$(srctree)" in opt:
                            real_copts.append(opt.replace("$(srctree)","$(ROOT_DIR)/{}".format(ack_dir)))
                        else:
                            if "$(DEVICE_MODULES_PATH)" in opt:
                                real_copts.append(opt.replace("$(DEVICE_MODULES_PATH)","$(ROOT_DIR)/{}".format(device_modules_dir)))
                            else:
                                real_copts.append(opt)

                ddk_library(
                    name = "{}.{}.{}.{}".format(name, project, version, build),
                    srcs = srcs,
                    kernel_build = "//{}:{}.{}".format(device_modules_dir, project, build),

                    # Indicate that this builds EL2 hypervisor code.
                    pkvm_el2 = pkvm_el2,
                    copts = real_copts,
                    deps = ["//{}:headers".format(device_modules_dir),]
                            + ["{}.{}.{}.{}".format(m, project, version, build) for m in ko_deps]
                            + header_deps,
                    includes = includes,
                    local_defines = local_defines,
                    linux_includes = linux_includes,
                )