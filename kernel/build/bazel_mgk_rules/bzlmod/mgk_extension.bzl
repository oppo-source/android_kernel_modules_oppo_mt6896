load("//build/bazel_mgk_rules:kleaf/key_value_repo.bzl", mtk_key_value_repo = "key_value_repo")
load("//build/kernel/kleaf:key_value_repo.bzl", "key_value_repo")
load("//prebuilts/clang/host/linux-x86/kleaf:clang_toolchain_repository.bzl", "clang_toolchain_repository")

visibility("public")

def mgk_module_impl(module_ctx):

    mtk_key_value_repo(
        name = "mgk_info",
    )

def _mgk_kernel_toolchain_ext_impl(module_ctx):
    kernel_version = module_ctx.os.environ["KERNEL_VERSION"]
    toolchain_constants = "//" + kernel_version + ":build.config.constants"
    key_value_repo(
        name = "kernel_toolchain_info",
        srcs = [toolchain_constants],
    )
    clang_toolchain_repository(
        name = "kleaf_clang_toolchain",
    )



mgk_module = module_extension(
    implementation = mgk_module_impl,
)




mgk_kernel_toolchain_ext = module_extension(
    implementation = _mgk_kernel_toolchain_ext_impl,
)
