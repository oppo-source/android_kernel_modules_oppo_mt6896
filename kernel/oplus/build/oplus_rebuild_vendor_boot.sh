#!/bin/bash

if [ $# -lt 2 ]; then
    echo "***********************************************************"
    echo "参数必须至少为2个"
    echo "dx5开始所有模块都进行了ddk改造，编译如下，ko名字可以在mgk_64_kleaf_device_modules中找到"
    echo "./kernel/oplus/build/oplus_rebuild_vendor_boot.sh platform build_type ko1 ko2 ko3..."
    echo "例：./kernel/oplus/build/oplus_rebuild_vendor_boot.sh mt6991 user //kernel_device_modules-6.12/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo 
    //kernel_device_modules-6.12/drivers/misc/mediatek/mtprof:bootprof"
    echo "如果模块位于mgk_64_device_modules（ddk改造完成之后这里将不存在）, 这里不需要输入模块名"
    echo "./kernel/oplus/build/oplus_rebuild_vendor_boot.sh platform build_type"
    echo "例：./kernel/oplus/build/oplus_rebuild_vendor_boot.sh mt6991 user"
    echo "***********************************************************"
    exit
fi

source kernel/oplus/build/oplus_setup.sh $1 $2
init_build_environment $2
IS_INTRANET="no"
is_rebulid_all_ko=""
args=("$@")
ko_input=${args[@]:2}
ko_input_num=$#
for ko in $ko_input
do
echo "input ko is ${ko}"
done

source kernel/oplus/build/oplus_rebuild_img_function.sh

rebuild_dtb_image() {
    echo "**********rebuild dtb.img start $(date +%H:%M:%S)**********"
    cp ${MAINDTB_PATH}/mtk.dtb ${VENDOR_BOOT_TMP_IMAGE}/origin/dtb
    echo "**********rebuild dtb.img end $(date +%H:%M:%S)**********"
}

vendor_boot_modules_all_update() {

    echo "vendor_boot module update"

    mkdir -p ${VENDOR_BOOT_TMP_IMAGE}/dist/
    mkdir -p ${VENDOR_BOOT_TMP_IMAGE}/tmp/

    mv ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/modules.load \
       ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/modules.load_bak

    cp ${ACKDIR}/oplus/prebuild/vendor_boot.load \
       ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/modules.load

    ko_list=`cat ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/modules.load | xargs -L 1 basename`

    if [ $ko_input_num -lt 3 ]; then
        # for module in mgk_64_device_modules(will be delete when all modules complete ddk)
        echo "mgk_64_device_modules module"
        for ko in  $ko_list
        do
            current=`find ${VENDOR_INTREE_MODULES_DIR} -maxdepth 1 -name ${ko}`
            if [ -n "${current}" ]; then
                echo "current is ${current}"
                cp ${current} ${VENDOR_BOOT_TMP_IMAGE}/dist/
                ${STRIP} -S ${VENDOR_BOOT_TMP_IMAGE}/dist/${ko} -o ${VENDOR_BOOT_TMP_IMAGE}/tmp/${ko}
                cp ${VENDOR_BOOT_TMP_IMAGE}/tmp/${ko} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/
            fi
        done
    else
        # for ddk module
        echo "ddk module"
        for ko_input_tmp in $ko_input
            do
            para1="$(echo "$ko_input_tmp" | cut -d: -f1)"
            para1="${para1#//}"
            ko_name="$(echo "$ko_input_tmp" | cut -d: -f2).ko"
            echo "para1 is ${para1}, ko_name is ${ko_name}"
            for ko in  $ko_list
            do
                if [ "${ko_name}" = "${ko}" ]; then
                    echo "ko_name(${ko_name}) is in ko_list"
                    ko_name_full_path=$(printf "%s%s" "$VENDOR_ORIGIN_MODULES_DIR" "$para1")
                    current=`find ${ko_name_full_path} -maxdepth 2 -name ${ko}`
                    if [ -n "${current}" ]; then
                        cp ${current} ${VENDOR_BOOT_TMP_IMAGE}/dist/
                        ${STRIP} -S ${VENDOR_BOOT_TMP_IMAGE}/dist/${ko} -o ${VENDOR_BOOT_TMP_IMAGE}/tmp/${ko}
                        cp ${VENDOR_BOOT_TMP_IMAGE}/tmp/${ko} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk00/lib/modules/
                    fi
                fi
            done
        done
    fi
}

rebuild_several_ko() {
    echo "**********rebuild out-tree ko $(date +%H:%M:%S)**********"
    cd ${TOPDIR}/kernel

    tools/bazel \
    --output_root=${KLEAF_OBJ} \
    --output_base=${OUTPUT_BASE} \
    build \
    --action_env=PATH=${ACKDIR}/build/kernel/build-tools/path/linux-x86:/usr/bin:/bin \
    --//build/bazel_mgk_rules:kernel_version=${VERSION} \
    --//build/kernel/kleaf:oplus_platform_name=mtk \
    --experimental_writable_outputs --allow_ddk_unsafe_headers=1 \
    --workaround_btrfs_b292212788 --experimental_optimize_ddk_config_actions --config=stamp \
    --user_ddk_unsafe_headers=//kernel_device_modules-${VERSION}:mtk_use_gki_unsafe_headers \
    --repo_manifest=${TOPDIR}/kernel/kernel_device_modules-${VERSION}/fake_manifest.xml \
    $1.${KRN_MGK}.${VERSION}.${variants_type} 2>&1 |tee ${TOPDIR}/LOGDIR/build_several_${CURRENT_LOG}.log

    cd ${TOPDIR}
    echo "**********rebuild out-tree ko end $(date +%H:%M:%S)**********"
}

rebuild_all_ko() {
    echo "**********rebuild in-tree ko $(date +%H:%M:%S)**********"
    cd ${TOPDIR}/kernel

    tools/bazel \
    --output_root=${KLEAF_OBJ} \
    --output_base=${OUTPUT_BASE} \
    build \
    --action_env=PATH=${ACKDIR}/build/kernel/build-tools/path/linux-x86:/usr/bin:/bin \
    --//build/bazel_mgk_rules:kernel_version=${VERSION} \
    --//build/kernel/kleaf:oplus_platform_name=mtk \
    --experimental_writable_outputs --allow_ddk_unsafe_headers=1 \
    --workaround_btrfs_b292212788 --experimental_optimize_ddk_config_actions --config=stamp \
    --user_ddk_unsafe_headers=//kernel_device_modules-${VERSION}:mtk_use_gki_unsafe_headers \
    --repo_manifest=${TOPDIR}/kernel/kernel_device_modules-${VERSION}/fake_manifest.xml \
    //kernel_device_modules-${VERSION}:${KRN_MGK}_customer_modules_install.${variants_type} 2>&1 |tee ${TOPDIR}/LOGDIR/build_all_${CURRENT_LOG}.log

    cd ${TOPDIR}
    echo "**********rebuild in-tree ko end $(date +%H:%M:%S)**********"
}

rebuild_vendor_boot_image() {
    echo "**********rebuild vendor_boot.img start $(date +%H:%M:%S)**********"
    rm -rf ${VENDOR_BOOT_TMP_IMAGE}/*
    boot_mkargs=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img ${ORIGIN_IMAGE}/vendor_boot.img --out ${VENDOR_BOOT_TMP_IMAGE}/origin --format=mkbootimg)
    rebuild_dtb_image
    index="00"
    for index in  $index
    do
        echo " index  $index "
        mv ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        #touch ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        ${LZ4} -d -f ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4 ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        mkdir -p ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}
        pushd  ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        ${CPIO} -idu < ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        popd
        rm ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        vendor_boot_modules_all_update
        ${MKBOOTFS} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index} > ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        #touch ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        ${LZ4} -l -f -12 --favor-decSpeed ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4 ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index}
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
    done
    bash -c "${PYTHON_TOOL} ${MKBOOTIMG_PATH} ${boot_mkargs} --vendor_boot ${IMAGE_OUT}/vendor_boot.img"
    #sign_vendor_boot_image     #tmp del sign for it's not ready
    echo "**********rebuild vendor_boot.img end $(date +%H:%M:%S)**********"
}

build_start_time
is_intranet
download_prebuild_image
get_image_info
get_modules_list
if [ -n "$3" ]; then
is_rebulid_all_ko="no"
for ko in $ko_input
do
echo "**********build ko , path is ${ko}**********"
rebuild_several_ko ${ko}
done
else
is_rebulid_all_ko="yes"
rebuild_all_ko
fi
rebuild_vendor_boot_image
print_end_help
build_end_time