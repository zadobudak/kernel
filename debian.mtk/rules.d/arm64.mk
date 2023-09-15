human_arch	= ARMv8
build_arch	= arm64
header_arch	= arm64
defconfig	= defconfig
flavours	= mtk
build_image	= Image.gz
kernel_file	= arch/$(build_arch)/boot/Image.gz
install_file	= vmlinuz
no_dumpfile = true

loader		= grub
vdso		= vdso_install

do_extras_package = false
do_tools_usbip  = true
do_tools_cpupower = true
do_tools_perf   = true
do_tools_perf_jvmti = true
do_tools_bpftool = true
do_tools_host = false

do_dtbs		= true
do_zfs		= false
do_dkms_wireguard = false
do_dkms_nvidia = false

disable_d_i = true
do_source_package = false
do_common_headers_indep = false
do_libc_dev_package = false
do_odm_drivers  = true
