#!/bin/bash

source kernel/oplus/build/oplus_setup.sh $1 $2
init_build_environment

if [ "$3" = "" ]; then
    echo "Please enter the ko directory:"
    print_module_help
    read ko_path
    echo
else
    ko_path=$3
    echo $4
fi

if [ "$4" != "" ]; then
    target_path=$4
fi

function build_in_tree_ko () {
    echo "enter build_in_tree_ko"
    cd ${TOPDIR}/kernel
    build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //kernel_device_modules-${VERSION}:${KRN_MGK}_modules.${variants_type}/${ko_path})
    if [ -n "$build_target" ]
    then
        echo "find the ko successfully!!!"
        tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} build //kernel_device_modules-${VERSION}:${KRN_MGK}_modules.${variants_type}/${ko_path}
    else
        echo "the directory you input is not exit or not right, please exit and try again!!!"
    fi
    file_name=`basename $ko_path`
    echo "you can push below ko or repack *.img by tools"
    ls -lh ${INTREE_MODULE_OUT}/${file_name}
    ls -lh ${INTREE_MODULE_OUT}/${ko_path}
    echo "last ko will copy to dist dir"
    cp ${INTREE_MODULE_OUT}/${file_name}  ${VENDOR_MODULES_DIR}
    ls -lh ${VENDOR_MODULES_DIR}/${file_name}
    echo "exit build_in_tree_ko"
}

function build_out_of_tree_ko () {
    echo "enter build_out_of_tree_ko"
    cd ${TOPDIR}/kernel
    build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //${ko_path}.${KRN_MGK}.${VERSION}.${variants_type})
    if [ -n "$build_target" ]
    then
        echo "find the ko successfully!!!"

    tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} build \
        --experimental_writable_outputs --allow_ddk_unsafe_headers=1 \
        --nokmi_symbol_list_violations_check \
        --//build/bazel_mgk_rules:kernel_version=${VERSION} //${ko_path}.${KRN_MGK}.${VERSION}.${variants_type} --sandbox_debug
    else
        echo "the directory you input is not exit or not right, please exit and try again!!!"
    fi

    ko_line=(${ko_path//:/ })
    ko_path=${ko_line[0]}
    ko_name=${ko_line[1]}

    echo "module path is $ko_path"
    echo "module name is $ko_name"

    echo "you can push below ko or repack *.img by tools"
    ls -lh ${TOPDIR}/kernel/bazel-bin/${ko_path}/${ko_name}.${KRN_MGK}.${VERSION}.${variants_type}/*.ko
    echo "this ko will copy to dist dir"
    cp ${TOPDIR}/kernel/bazel-bin/${ko_path}/${ko_name}.${KRN_MGK}.${VERSION}.${variants_type}/*.ko \
    ${VENDOR_MODULES_DIR}/
    echo "last module copy to ${VENDOR_MODULES_DIR}"
    echo "exit build_out_of_tree_ko"
}

function query_target () {
    echo "enter query_target"
    cd ${TOPDIR}/kernel
    echo build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //${target_path})
    build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //${target_path})
    echo $(build_target)
}

function build_target () {
    echo "enter build_target"
    cd ${TOPDIR}/kernel
    tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} build \
            --experimental_writable_outputs --allow_ddk_unsafe_headers=1 \
            --nokmi_symbol_list_violations_check \
            --//build/bazel_mgk_rules:kernel_version=${VERSION} \
            //${target_path} --sandbox_debug
}

function build_ko {
    if [[ "$ko_path" == *".ko" ]]; then
        echo "-------------------------this is in tree ko-----------------------"
        build_in_tree_ko
    elif [[ "$ko_path" == *":"* ]]; then
        echo "-------------------------this is out of tree ko-------------------"
        build_out_of_tree_ko
    elif [[ "$ko_path" == "query" ]]; then
        query_target
    elif [[ "$ko_path" == "build" ]]; then
        build_target
    else
        echo "[ERROR]: Invalid parameter, please provide a path to a .ko file or a directory!!!"
        print_module_help
        exit 1
    fi
}

#print_module_help
build_ko
