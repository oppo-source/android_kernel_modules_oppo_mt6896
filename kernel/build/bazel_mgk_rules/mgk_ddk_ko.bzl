load(
    "//build/kernel/kleaf:kernel.bzl",
    "ddk_module",
    "ddk_submodule",
)
load(
    ":mgk.bzl",
    "kernel_versions_and_projects",
)
#distinguish different platform info
load("@mgk_info//:dict.bzl", "OPLUS_PLATFORM_INFO")

def define_mgk_ddk_ko(
        name,
        module_type = None,
        srcs = None,
        header_deps = [],
        prebuilt_srcs = [],
        ko_deps = [],
        hdrs = None,
        includes = None,
        generate_btf = None,
        conditional_srcs = None,
        linux_includes = None,
        out = None,
        local_defines = None,
        copts = None,
        kconfig = None,
        defconfig = None,
        conditional_defines = None,
        **kwargs):
    if srcs == None:
        srcs = native.glob(
            [
                "**/*.c",
                "**/*.h",
            ],
            exclude = [
                ".*",
                ".*/**",
            ],
        )
    if out == None:
        out = name + ".ko"

    flattened_conditional_defines = None
    if conditional_defines:
        for config_vendor, config_defines in conditional_defines.items():
            if config_vendor == OPLUS_PLATFORM_INFO:
                if flattened_conditional_defines:
                    flattened_conditional_defines = flattened_conditional_defines + config_defines
                else:
                    flattened_conditional_defines = config_defines

    if flattened_conditional_defines:
        if local_defines:
            local_defines =  local_defines + flattened_conditional_defines
        else:
            local_defines = flattened_conditional_defines

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
                        elif "$(objtree)" in opt:
                            opt = opt.replace("$(objtree)", "$(ROOT_DIR)/out/android-{}/{}".format(version,ack_dir))
                            real_copts.append(opt)
                        else:
                            if "$(DEVICE_MODULES_PATH)" in opt:
                                real_copts.append(opt.replace("$(DEVICE_MODULES_PATH)","$(ROOT_DIR)/{}".format(device_modules_dir)))
                            else:
                                real_copts.append(opt)

                if module_type != "sub":
                    if module_type != "main":
                        ddk_module(
                            name = "{}.{}.{}.{}".format(name, project, version, build),
                            kernel_build = "//{}:{}.{}".format(device_modules_dir, project, build),
                            srcs = srcs,
                            deps = [
                                "//{}:headers".format(device_modules_dir),
                            ] 
                            #+ (["//{}:{}_modules.{}".format(device_modules_dir, project, build)] if native.package_name().startswith("vendor") else [])
                            + ["{}.{}.{}.{}".format(m, project, version, build) for m in ko_deps]
                            + header_deps
                            + prebuilt_srcs,
                            hdrs = hdrs,
                            includes = includes,
                            generate_btf = generate_btf,
                            conditional_srcs = conditional_srcs,
                            linux_includes = linux_includes,
                            out = out,
                            local_defines = local_defines,
                            copts = real_copts,
                            kconfig = kconfig,
                            defconfig = defconfig,
                            **kwargs
                    )
                    else:
                        ddk_module(
                            name = "{}.{}.{}.{}".format(name, project, version, build),
                            kernel_build = "//{}:{}.{}".format(device_modules_dir, project, build),
                            deps = ["{}.{}.{}.{}".format(m, project, version, build) for m in ko_deps],
                            linux_includes = linux_includes,
                            kconfig = kconfig,
                            defconfig = defconfig,
                            **kwargs
                        )
                else:
                    ddk_submodule(
                        name = "{}.{}.{}.{}".format(name, project, version, build),
                        srcs = srcs,
                        deps = [
                            "//{}:headers".format(device_modules_dir),
                        ]
                        #+ (["//{}:{}_modules.{}".format(device_modules_dir, project, build)] if native.package_name().startswith("vendor") else [])
                        + ["{}.{}.{}.{}".format(m, project, version, build) for m in ko_deps]
                        + header_deps
                        + prebuilt_srcs,
                        hdrs = hdrs,
                        includes = includes,
                        conditional_srcs = conditional_srcs,
                        out = out,
                        local_defines = local_defines,
                        copts = real_copts,
                   )

