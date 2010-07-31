KERN_IDENT=SENTRY5
MODULES_OVERRIDE=""
DEBUG=-g
MIPS_LITTLE_ENDIAN=defined
# Makefile.mips
# $FreeBSD: src/sys/conf/Makefile.mips,v 1.2 2010/01/10 05:34:46 imp Exp $
#
# Makefile for FreeBSD
#
# This makefile is constructed from a machine description:
#	config machineid
# Most changes should be made in the machine description
#	/sys/mips/conf/``machineid''
# after which you should do
#	 config machineid
# Generic makefile changes should be made in
#	/sys/conf/Makefile.mips
# after which config should be rerun for all machines.
#

# Which version of config(8) is required.

STD8X16FONT?=	iso

.if !defined(S)
.if exists(./@/.)
S=	./@
.else
S=	../../..
.endif
.endif
.include "$S/conf/kern.pre.mk"

LDSCRIPT_NAME?=ldscript.$M
SYSTEM_LD:= ${SYSTEM_LD:$S/conf/${LDSCRIPT_NAME}=${LDSCRIPT_NAME}}
SYSTEM_DEP:= ${SYSTEM_DEP:$S/conf/${LDSCRIPT_NAME}=${LDSCRIPT_NAME}}

# XXX: Such sweeping assumptions...
MACHINE=mips
MACHINE_ARCH=mips
KERNLOADADDR?=0x80001000
# This obscure value is defined by CFE for WR160N
# To be changed later
TRAMPLOADADDR?=0x807963c0

MKMODULESENV+=	MACHINE=${MACHINE} MACHINE_ARCH=${MACHINE_ARCH}

# We default to the MIPS32 ISA, if none specified in the
# kernel configuration file.
ARCH_FLAGS?=-march=mips32
EXTRA_FLAGS=-fno-pic -mno-abicalls -G0

HACK_EXTRA_FLAGS=-shared
.if defined(TARGET_BIG_ENDIAN)
CFLAGS+=-EB
SYSTEM_LD+=-EB
EXTRA_FLAGS+=-EB 
TRAMP_LDFLAGS+=-Wl,-EB 
HACK_EXTRA_FLAGS+=-EB -Wl,-EB
.if defined(TARGET_64BIT)
SYSTEM_LD+=-m elf64btsmip_fbsd
HACK_EXTRA_FLAGS+=-Wl,-m,elf64btsmip_fbsd
.endif
.else
CFLAGS+=-EL
SYSTEM_LD+=-EL
EXTRA_FLAGS+=-EL
TRAMP_LDFLAGS+=-Wl,-EL
HACK_EXTRA_FLAGS+=-EL -Wl,-EL
.if defined(TARGET_64BIT)
SYSTEM_LD+=-m elf64ltsmip_fbsd
HACK_EXTRA_FLAGS+=-Wl,-m,elf64ltsmip_fbsd
.endif
.endif


# We add the -fno-pic flag to kernels because otherwise performance
# is extremely poor, as well as -mno-abicalls to force no ABI usage.
CFLAGS+=${EXTRA_FLAGS} $(ARCH_FLAGS)
HACK_EXTRA_FLAGS+=${EXTRA_FLAGS} $(ARCH_FLAGS)

# XXX hardcoded kernel entry point
ASM_CFLAGS+=${CFLAGS} -D_LOCORE -DLOCORE

KERNEL_EXTRA=trampoline
trampoline: ${KERNEL_KO}.tramp.bin
${KERNEL_KO}.tramp.bin: ${KERNEL_KO} $S/$M/$M/elf_trampoline.c \
	$S/$M/$M/inckern.S 
	${OBJCOPY} --strip-symbol '$$d' --strip-symbol '$$a' \
	-g --strip-symbol '$$t' ${FULLKERNEL} ${KERNEL_KO}.tmp
	sed s/${KERNLOADADDR}/${TRAMPLOADADDR}/ ${LDSCRIPT_NAME} | \
		sed s/" + SIZEOF_HEADERS"//  > ${LDSCRIPT_NAME}.tramp.noheader
	# Generate .S file that setups stack and jumps to trampoline
	echo "#include <machine/asm.h>" >tmphack.S
	echo "ENTRY(_start)" >>tmphack.S
	echo "PTR_LA t0, kernel_end" >>tmphack.S
	echo "move sp, t0" >>tmphack.S
	echo "add sp, 0x2000" >>tmphack.S
	echo "and sp, ~0x7" >>tmphack.S
	echo "PTR_LA t0, _startC" >>tmphack.S
	echo "j t0" >>tmphack.S
	echo "END(_start)" >>tmphack.S
	echo "#define KERNNAME \"${KERNEL_KO}.tmp\""  >opt_kernname.h 
	${CC} -O -nostdlib -I. -I$S ${HACK_EXTRA_FLAGS} ${TRAMP_LDFLAGS} -Xlinker \
		-T -Xlinker ${LDSCRIPT_NAME}.tramp.noheader tmphack.S \
		$S/$M/$M/elf_trampoline.c $S/$M/$M/inckern.S \
		-o ${KERNEL_KO}.tramp.noheader 
	${OBJCOPY} -S -O binary ${KERNEL_KO}.tramp.noheader \
		${KERNEL_KO}.tramp.bin \

BEFORE_DEPEND=pccarddevs.h usbdevs.h usbdevs_data.h 

OBJS=db_access.o db_break.o db_capture.o db_command.o db_examine.o \
	db_expr.o db_input.o db_lex.o db_main.o db_output.o db_print.o \
	db_ps.o db_run.o db_script.o db_sym.o db_thread.o db_textdump.o \
	db_variables.o db_watch.o db_write_cmd.o eisa_if.o led.o md.o \
	memdev.o mmcbr_if.o mmcbus_if.o null.o card_if.o power_if.o \
	fixup_pci.o hostb_pci.o ignore_pci.o pci.o pci_if.o pci_pci.o \
	pci_user.o pcib_if.o vga_pci.o harvest.o ehci.o ehci_pci.o \
	uhci.o uhci_pci.o usb_controller.o usb_busdma.o \
	usb_compat_linux.o usb_core.o usb_debug.o usb_dev.o usb_device.o \
	usb_dynamic.o usb_error.o usb_generic.o usb_handle_request.o \
	usb_hid.o usb_hub.o usb_if.o usb_lookup.o usb_mbuf.o \
	usb_msctest.o usb_parse.o usb_process.o usb_request.o \
	usb_transfer.o usb_util.o usb_quirk.o watchdog.o dead_vnops.o \
	devfs_devs.o devfs_rule.o devfs_vfsops.o devfs_vnops.o \
	fifo_vnops.o pseudofs.o pseudofs_fileno.o pseudofs_vncache.o \
	pseudofs_vnops.o geom_ctl.o geom_dev.o geom_disk.o geom_dump.o \
	geom_event.o geom_io.o geom_kern.o geom_slice.o geom_subr.o \
	geom_vfs.o g_part.o g_part_if.o g_part_bsd.o g_part_mbr.o \
	isa_if.o bus_if.o clock_if.o cpufreq_if.o device_if.o \
	imgact_elf.o imgact_shell.o init_main.o init_sysent.o ksched.o \
	kern_acct.o kern_clock.o kern_condvar.o kern_conf.o kern_cons.o \
	kern_cpu.o kern_cpuset.o kern_context.o kern_descrip.o \
	kern_environment.o kern_event.o kern_exec.o kern_exit.o \
	kern_fail.o kern_fork.o kern_idle.o kern_intr.o kern_jail.o \
	kern_kthread.o kern_ktrace.o kern_linker.o kern_lock.o \
	kern_lockf.o kern_malloc.o kern_mbuf.o kern_mib.o kern_module.o \
	kern_mtxpool.o kern_mutex.o kern_ntptime.o kern_osd.o \
	kern_physio.o kern_pmc.o kern_priv.o kern_proc.o kern_prot.o \
	kern_resource.o kern_rmlock.o kern_rwlock.o kern_sema.o \
	kern_shutdown.o kern_sig.o kern_subr.o kern_switch.o kern_sx.o \
	kern_synch.o kern_syscalls.o kern_sysctl.o kern_tc.o kern_thr.o \
	kern_thread.o kern_time.o kern_timeout.o kern_umtx.o kern_uuid.o \
	kern_xxx.o link_elf.o linker_if.o md5c.o p1003_1b.o posix4_mib.o \
	sched_4bsd.o serdev_if.o stack_protector.o subr_acl_nfs4.o \
	subr_acl_posix1e.o subr_autoconf.o subr_blist.o subr_bus.o \
	subr_bufring.o subr_clock.o subr_devstat.o subr_disk.o \
	subr_eventhandler.o subr_fattime.o subr_hints.o subr_kdb.o \
	subr_kobj.o subr_lock.o subr_log.o subr_module.o subr_msgbuf.o \
	subr_param.o subr_pcpu.o subr_power.o subr_prf.o subr_prof.o \
	subr_rman.o subr_rtc.o subr_sbuf.o subr_scanf.o subr_sglist.o \
	subr_sleepqueue.o subr_smp.o subr_stack.o subr_taskqueue.o \
	subr_trap.o subr_turnstile.o subr_unit.o sys_generic.o \
	sys_pipe.o sys_process.o sys_socket.o syscalls.o sysv_ipc.o \
	tty.o tty_info.o tty_inq.o tty_outq.o tty_pts.o tty_tty.o \
	tty_ttydisc.o uipc_accf.o uipc_debug.o uipc_domain.o uipc_mbuf.o \
	uipc_mbuf2.o uipc_shm.o uipc_sockbuf.o uipc_socket.o \
	uipc_syscalls.o uipc_usrreq.o vfs_acl.o vfs_bio.o vfs_cache.o \
	vfs_cluster.o vfs_default.o vfs_export.o vfs_extattr.o \
	vfs_hash.o vfs_init.o vfs_lookup.o vfs_mount.o vfs_subr.o \
	vfs_syscalls.o vfs_vnops.o arc4random.o bcd.o bsearch.o crc32.o \
	fnmatch.o gets.o index.o inet_aton.o inet_ntoa.o memcmp.o \
	qsort.o qsort_r.o random.o rindex.o scanc.o skpc.o strcasecmp.o \
	strcat.o strcmp.o strcpy.o strcspn.o strdup.o strlcat.o \
	strlcpy.o strlen.o strncmp.o strncpy.o strsep.o strspn.o \
	strstr.o strtol.o strtoq.o strtoul.o strtouq.o strvalid.o bpf.o \
	if.o if_clone.o if_dead.o if_ethersubr.o if_loop.o if_llatbl.o \
	if_media.o if_mib.o netisr.o pfil.o radix.o radix_mpath.o \
	raw_cb.o raw_usrreq.o route.o rtsock.o if_ether.o igmp.o in.o \
	ip_id.o in_mcast.o in_pcb.o in_proto.o in_rmx.o ip_ecn.o \
	ip_encap.o ip_fastfwd.o ip_icmp.o ip_input.o ip_options.o \
	ip_output.o raw_ip.o tcp_hostcache.o tcp_input.o tcp_lro.o \
	tcp_output.o tcp_offload.o tcp_reass.o tcp_sack.o tcp_subr.o \
	tcp_syncache.o tcp_timer.o tcp_timewait.o tcp_usrreq.o \
	udp_usrreq.o nfs_common.o nfs_bio.o nfs_diskless.o nfs_node.o \
	nfs_krpc.o nfs_subs.o nfs_nfsiod.o nfs_vfsops.o nfs_vnops.o \
	nfs_lock.o auth_none.o auth_unix.o authunix_prot.o clnt_dg.o \
	clnt_rc.o clnt_vc.o getnetconfig.o inet_ntop.o inet_pton.o \
	rpc_callmsg.o rpc_generic.o rpc_prot.o rpcb_clnt.o rpcb_prot.o \
	audit_syscalls.o mac_syscalls.o default_pager.o device_pager.o \
	phys_pager.o sg_pager.o swap_pager.o uma_core.o uma_dbg.o \
	vm_contig.o vm_fault.o vm_glue.o vm_init.o vm_kern.o vm_map.o \
	vm_meter.o vm_mmap.o vm_object.o vm_page.o vm_pageout.o \
	vm_pager.o vm_phys.o vm_reserv.o vm_unix.o vm_zeroidle.o \
	vnode_pager.o xdr.o xdr_array.o xdr_mbuf.o xdr_mem.o \
	xdr_reference.o xdr_sizeof.o machdep.o psraccess.o autoconf.o \
	cpu.o elf_machdep.o exception.o gdb_machdep.o pmap.o trap.o \
	vm_machdep.o fp.o pm_machdep.o swtch.o tlb.o bus_space_generic.o \
	busdma_machdep.o cache.o cache_mipsNN.o db_disasm.o \
	db_interface.o db_trace.o dump_machdep.o in_cksum.o mem.o \
	nexus.o stack_machdep.o support.o sys_machdep.o uio_machdep.o \
	ashldi3.o ashrdi3.o divdi3.o ffsl.o fls.o flsl.o lshrdi3.o \
	memmove.o moddi3.o qdivrem.o udivdi3.o umoddi3.o cfe_api.o \
	cfe_console.o siba.o siba_pcib.o siba_cc.o s5_machdep.o \
	intr_machdep.o tick.o 

CFILES=$S/ddb/db_access.c $S/ddb/db_break.c $S/ddb/db_capture.c \
	$S/ddb/db_command.c $S/ddb/db_examine.c $S/ddb/db_expr.c \
	$S/ddb/db_input.c $S/ddb/db_lex.c $S/ddb/db_main.c \
	$S/ddb/db_output.c $S/ddb/db_print.c $S/ddb/db_ps.c \
	$S/ddb/db_run.c $S/ddb/db_script.c $S/ddb/db_sym.c \
	$S/ddb/db_thread.c $S/ddb/db_textdump.c $S/ddb/db_variables.c \
	$S/ddb/db_watch.c $S/ddb/db_write_cmd.c $S/dev/led/led.c \
	$S/dev/md/md.c $S/dev/mem/memdev.c $S/dev/null/null.c \
	$S/dev/pci/fixup_pci.c $S/dev/pci/hostb_pci.c \
	$S/dev/pci/ignore_pci.c $S/dev/pci/pci.c $S/dev/pci/pci_pci.c \
	$S/dev/pci/pci_user.c $S/dev/pci/vga_pci.c \
	$S/dev/random/harvest.c $S/dev/usb/controller/ehci.c \
	$S/dev/usb/controller/ehci_pci.c $S/dev/usb/controller/uhci.c \
	$S/dev/usb/controller/uhci_pci.c \
	$S/dev/usb/controller/usb_controller.c $S/dev/usb/usb_busdma.c \
	$S/dev/usb/usb_compat_linux.c $S/dev/usb/usb_core.c \
	$S/dev/usb/usb_debug.c $S/dev/usb/usb_dev.c \
	$S/dev/usb/usb_device.c $S/dev/usb/usb_dynamic.c \
	$S/dev/usb/usb_error.c $S/dev/usb/usb_generic.c \
	$S/dev/usb/usb_handle_request.c $S/dev/usb/usb_hid.c \
	$S/dev/usb/usb_hub.c $S/dev/usb/usb_lookup.c \
	$S/dev/usb/usb_mbuf.c $S/dev/usb/usb_msctest.c \
	$S/dev/usb/usb_parse.c $S/dev/usb/usb_process.c \
	$S/dev/usb/usb_request.c $S/dev/usb/usb_transfer.c \
	$S/dev/usb/usb_util.c $S/dev/usb/quirk/usb_quirk.c \
	$S/dev/watchdog/watchdog.c $S/fs/deadfs/dead_vnops.c \
	$S/fs/devfs/devfs_devs.c $S/fs/devfs/devfs_rule.c \
	$S/fs/devfs/devfs_vfsops.c $S/fs/devfs/devfs_vnops.c \
	$S/fs/fifofs/fifo_vnops.c $S/fs/pseudofs/pseudofs.c \
	$S/fs/pseudofs/pseudofs_fileno.c \
	$S/fs/pseudofs/pseudofs_vncache.c \
	$S/fs/pseudofs/pseudofs_vnops.c $S/geom/geom_ctl.c \
	$S/geom/geom_dev.c $S/geom/geom_disk.c $S/geom/geom_dump.c \
	$S/geom/geom_event.c $S/geom/geom_io.c $S/geom/geom_kern.c \
	$S/geom/geom_slice.c $S/geom/geom_subr.c $S/geom/geom_vfs.c \
	$S/geom/part/g_part.c $S/geom/part/g_part_bsd.c \
	$S/geom/part/g_part_mbr.c $S/kern/imgact_elf.c \
	$S/kern/imgact_shell.c $S/kern/init_main.c $S/kern/init_sysent.c \
	$S/kern/ksched.c $S/kern/kern_acct.c $S/kern/kern_clock.c \
	$S/kern/kern_condvar.c $S/kern/kern_conf.c $S/kern/kern_cons.c \
	$S/kern/kern_cpu.c $S/kern/kern_cpuset.c $S/kern/kern_context.c \
	$S/kern/kern_descrip.c $S/kern/kern_environment.c \
	$S/kern/kern_event.c $S/kern/kern_exec.c $S/kern/kern_exit.c \
	$S/kern/kern_fail.c $S/kern/kern_fork.c $S/kern/kern_idle.c \
	$S/kern/kern_intr.c $S/kern/kern_jail.c $S/kern/kern_kthread.c \
	$S/kern/kern_ktrace.c $S/kern/kern_linker.c $S/kern/kern_lock.c \
	$S/kern/kern_lockf.c $S/kern/kern_malloc.c $S/kern/kern_mbuf.c \
	$S/kern/kern_mib.c $S/kern/kern_module.c $S/kern/kern_mtxpool.c \
	$S/kern/kern_mutex.c $S/kern/kern_ntptime.c $S/kern/kern_osd.c \
	$S/kern/kern_physio.c $S/kern/kern_pmc.c $S/kern/kern_priv.c \
	$S/kern/kern_proc.c $S/kern/kern_prot.c $S/kern/kern_resource.c \
	$S/kern/kern_rmlock.c $S/kern/kern_rwlock.c $S/kern/kern_sema.c \
	$S/kern/kern_shutdown.c $S/kern/kern_sig.c $S/kern/kern_subr.c \
	$S/kern/kern_switch.c $S/kern/kern_sx.c $S/kern/kern_synch.c \
	$S/kern/kern_syscalls.c $S/kern/kern_sysctl.c $S/kern/kern_tc.c \
	$S/kern/kern_thr.c $S/kern/kern_thread.c $S/kern/kern_time.c \
	$S/kern/kern_timeout.c $S/kern/kern_umtx.c $S/kern/kern_uuid.c \
	$S/kern/kern_xxx.c $S/kern/link_elf.c $S/kern/md5c.c \
	$S/kern/p1003_1b.c $S/kern/posix4_mib.c $S/kern/sched_4bsd.c \
	$S/kern/stack_protector.c $S/kern/subr_acl_nfs4.c \
	$S/kern/subr_acl_posix1e.c $S/kern/subr_autoconf.c \
	$S/kern/subr_blist.c $S/kern/subr_bus.c $S/kern/subr_bufring.c \
	$S/kern/subr_clock.c $S/kern/subr_devstat.c $S/kern/subr_disk.c \
	$S/kern/subr_eventhandler.c $S/kern/subr_fattime.c \
	$S/kern/subr_hints.c $S/kern/subr_kdb.c $S/kern/subr_kobj.c \
	$S/kern/subr_lock.c $S/kern/subr_log.c $S/kern/subr_module.c \
	$S/kern/subr_msgbuf.c $S/kern/subr_param.c $S/kern/subr_pcpu.c \
	$S/kern/subr_power.c $S/kern/subr_prf.c $S/kern/subr_prof.c \
	$S/kern/subr_rman.c $S/kern/subr_rtc.c $S/kern/subr_sbuf.c \
	$S/kern/subr_scanf.c $S/kern/subr_sglist.c \
	$S/kern/subr_sleepqueue.c $S/kern/subr_smp.c \
	$S/kern/subr_stack.c $S/kern/subr_taskqueue.c \
	$S/kern/subr_trap.c $S/kern/subr_turnstile.c $S/kern/subr_unit.c \
	$S/kern/sys_generic.c $S/kern/sys_pipe.c $S/kern/sys_process.c \
	$S/kern/sys_socket.c $S/kern/syscalls.c $S/kern/sysv_ipc.c \
	$S/kern/tty.c $S/kern/tty_info.c $S/kern/tty_inq.c \
	$S/kern/tty_outq.c $S/kern/tty_pts.c $S/kern/tty_tty.c \
	$S/kern/tty_ttydisc.c $S/kern/uipc_accf.c $S/kern/uipc_debug.c \
	$S/kern/uipc_domain.c $S/kern/uipc_mbuf.c $S/kern/uipc_mbuf2.c \
	$S/kern/uipc_shm.c $S/kern/uipc_sockbuf.c $S/kern/uipc_socket.c \
	$S/kern/uipc_syscalls.c $S/kern/uipc_usrreq.c $S/kern/vfs_acl.c \
	$S/kern/vfs_bio.c $S/kern/vfs_cache.c $S/kern/vfs_cluster.c \
	$S/kern/vfs_default.c $S/kern/vfs_export.c $S/kern/vfs_extattr.c \
	$S/kern/vfs_hash.c $S/kern/vfs_init.c $S/kern/vfs_lookup.c \
	$S/kern/vfs_mount.c $S/kern/vfs_subr.c $S/kern/vfs_syscalls.c \
	$S/kern/vfs_vnops.c $S/libkern/arc4random.c $S/libkern/bcd.c \
	$S/libkern/bsearch.c $S/libkern/crc32.c $S/libkern/fnmatch.c \
	$S/libkern/gets.c $S/libkern/index.c $S/libkern/inet_aton.c \
	$S/libkern/inet_ntoa.c $S/libkern/memcmp.c $S/libkern/qsort.c \
	$S/libkern/qsort_r.c $S/libkern/random.c $S/libkern/rindex.c \
	$S/libkern/scanc.c $S/libkern/skpc.c $S/libkern/strcasecmp.c \
	$S/libkern/strcat.c $S/libkern/strcmp.c $S/libkern/strcpy.c \
	$S/libkern/strcspn.c $S/libkern/strdup.c $S/libkern/strlcat.c \
	$S/libkern/strlcpy.c $S/libkern/strlen.c $S/libkern/strncmp.c \
	$S/libkern/strncpy.c $S/libkern/strsep.c $S/libkern/strspn.c \
	$S/libkern/strstr.c $S/libkern/strtol.c $S/libkern/strtoq.c \
	$S/libkern/strtoul.c $S/libkern/strtouq.c $S/libkern/strvalid.c \
	$S/net/bpf.c $S/net/if.c $S/net/if_clone.c $S/net/if_dead.c \
	$S/net/if_ethersubr.c $S/net/if_loop.c $S/net/if_llatbl.c \
	$S/net/if_media.c $S/net/if_mib.c $S/net/netisr.c $S/net/pfil.c \
	$S/net/radix.c $S/net/radix_mpath.c $S/net/raw_cb.c \
	$S/net/raw_usrreq.c $S/net/route.c $S/net/rtsock.c \
	$S/netinet/if_ether.c $S/netinet/igmp.c $S/netinet/in.c \
	$S/netinet/ip_id.c $S/netinet/in_mcast.c $S/netinet/in_pcb.c \
	$S/netinet/in_proto.c $S/netinet/in_rmx.c $S/netinet/ip_ecn.c \
	$S/netinet/ip_encap.c $S/netinet/ip_fastfwd.c \
	$S/netinet/ip_icmp.c $S/netinet/ip_input.c \
	$S/netinet/ip_options.c $S/netinet/ip_output.c \
	$S/netinet/raw_ip.c $S/netinet/tcp_hostcache.c \
	$S/netinet/tcp_input.c $S/netinet/tcp_lro.c \
	$S/netinet/tcp_output.c $S/netinet/tcp_offload.c \
	$S/netinet/tcp_reass.c $S/netinet/tcp_sack.c \
	$S/netinet/tcp_subr.c $S/netinet/tcp_syncache.c \
	$S/netinet/tcp_timer.c $S/netinet/tcp_timewait.c \
	$S/netinet/tcp_usrreq.c $S/netinet/udp_usrreq.c \
	$S/nfs/nfs_common.c $S/nfsclient/nfs_bio.c \
	$S/nfsclient/nfs_diskless.c $S/nfsclient/nfs_node.c \
	$S/nfsclient/nfs_krpc.c $S/nfsclient/nfs_subs.c \
	$S/nfsclient/nfs_nfsiod.c $S/nfsclient/nfs_vfsops.c \
	$S/nfsclient/nfs_vnops.c $S/nfsclient/nfs_lock.c \
	$S/rpc/auth_none.c $S/rpc/auth_unix.c $S/rpc/authunix_prot.c \
	$S/rpc/clnt_dg.c $S/rpc/clnt_rc.c $S/rpc/clnt_vc.c \
	$S/rpc/getnetconfig.c $S/rpc/inet_ntop.c $S/rpc/inet_pton.c \
	$S/rpc/rpc_callmsg.c $S/rpc/rpc_generic.c $S/rpc/rpc_prot.c \
	$S/rpc/rpcb_clnt.c $S/rpc/rpcb_prot.c \
	$S/security/audit/audit_syscalls.c \
	$S/security/mac/mac_syscalls.c $S/vm/default_pager.c \
	$S/vm/device_pager.c $S/vm/phys_pager.c $S/vm/sg_pager.c \
	$S/vm/swap_pager.c $S/vm/uma_core.c $S/vm/uma_dbg.c \
	$S/vm/vm_contig.c $S/vm/vm_fault.c $S/vm/vm_glue.c \
	$S/vm/vm_init.c $S/vm/vm_kern.c $S/vm/vm_map.c $S/vm/vm_meter.c \
	$S/vm/vm_mmap.c $S/vm/vm_object.c $S/vm/vm_page.c \
	$S/vm/vm_pageout.c $S/vm/vm_pager.c $S/vm/vm_phys.c \
	$S/vm/vm_reserv.c $S/vm/vm_unix.c $S/vm/vm_zeroidle.c \
	$S/vm/vnode_pager.c $S/xdr/xdr.c $S/xdr/xdr_array.c \
	$S/xdr/xdr_mbuf.c $S/xdr/xdr_mem.c $S/xdr/xdr_reference.c \
	$S/xdr/xdr_sizeof.c $S/mips/mips/machdep.c \
	$S/mips/mips/autoconf.c $S/mips/mips/cpu.c \
	$S/mips/mips/elf_machdep.c $S/mips/mips/gdb_machdep.c \
	$S/mips/mips/pmap.c $S/mips/mips/trap.c \
	$S/mips/mips/vm_machdep.c $S/mips/mips/pm_machdep.c \
	$S/mips/mips/bus_space_generic.c $S/mips/mips/busdma_machdep.c \
	$S/mips/mips/cache.c $S/mips/mips/cache_mipsNN.c \
	$S/mips/mips/db_disasm.c $S/mips/mips/db_interface.c \
	$S/mips/mips/db_trace.c $S/mips/mips/dump_machdep.c \
	$S/mips/mips/in_cksum.c $S/mips/mips/mem.c $S/mips/mips/nexus.c \
	$S/mips/mips/stack_machdep.c $S/mips/mips/sys_machdep.c \
	$S/mips/mips/uio_machdep.c $S/libkern/ashldi3.c \
	$S/libkern/ashrdi3.c $S/libkern/divdi3.c $S/libkern/ffsl.c \
	$S/libkern/fls.c $S/libkern/flsl.c $S/libkern/lshrdi3.c \
	$S/libkern/memmove.c $S/libkern/moddi3.c $S/libkern/qdivrem.c \
	$S/libkern/udivdi3.c $S/libkern/umoddi3.c $S/dev/cfe/cfe_api.c \
	$S/dev/cfe/cfe_console.c $S/dev/siba/siba.c \
	$S/dev/siba/siba_pcib.c $S/dev/siba/siba_cc.c \
	$S/mips/sentry5/s5_machdep.c $S/mips/mips/intr_machdep.c \
	$S/mips/mips/tick.c 

SFILES=$S/mips/mips/psraccess.S $S/mips/mips/exception.S \
	$S/mips/mips/fp.S $S/mips/mips/swtch.S $S/mips/mips/tlb.S \
	$S/mips/mips/locore.S $S/mips/mips/support.S 

MFILES=$S/dev/eisa/eisa_if.m $S/dev/mmc/mmcbr_if.m \
	$S/dev/mmc/mmcbus_if.m $S/dev/pccard/card_if.m \
	$S/dev/pccard/power_if.m $S/dev/pci/pci_if.m \
	$S/dev/pci/pcib_if.m $S/dev/usb/usb_if.m \
	$S/geom/part/g_part_if.m $S/isa/isa_if.m $S/kern/bus_if.m \
	$S/kern/clock_if.m $S/kern/cpufreq_if.m $S/kern/device_if.m \
	$S/kern/linker_if.m $S/kern/serdev_if.m 

CLEAN=pccarddevs.h usbdevs.h usbdevs_data.h 

CLEAN+=	${LDSCRIPT_NAME} ${LDSCRIPT_NAME}.tramp.noheader \
	${KERNEL_KO}.tramp.noheader ${KERNEL_KO}.tramp.bin

${LDSCRIPT_NAME}: $S/conf/${LDSCRIPT_NAME}
	cat $S/conf/${LDSCRIPT_NAME}|sed s/KERNLOADADDR/${KERNLOADADDR}/g \
		> ${LDSCRIPT_NAME}
pccarddevs.h: $S/tools/pccarddevs2h.awk $S/dev/pccard/pccarddevs
	${AWK} -f $S/tools/pccarddevs2h.awk $S/dev/pccard/pccarddevs

usbdevs.h: $S/tools/usbdevs2h.awk $S/dev/usb/usbdevs
	${AWK} -f $S/tools/usbdevs2h.awk $S/dev/usb/usbdevs -h

usbdevs_data.h: $S/tools/usbdevs2h.awk $S/dev/usb/usbdevs
	${AWK} -f $S/tools/usbdevs2h.awk $S/dev/usb/usbdevs -d

db_access.ln: $S/ddb/db_access.c
	${NORMAL_LINT}

db_access.o: $S/ddb/db_access.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_break.ln: $S/ddb/db_break.c
	${NORMAL_LINT}

db_break.o: $S/ddb/db_break.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_capture.ln: $S/ddb/db_capture.c
	${NORMAL_LINT}

db_capture.o: $S/ddb/db_capture.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_command.ln: $S/ddb/db_command.c
	${NORMAL_LINT}

db_command.o: $S/ddb/db_command.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_examine.ln: $S/ddb/db_examine.c
	${NORMAL_LINT}

db_examine.o: $S/ddb/db_examine.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_expr.ln: $S/ddb/db_expr.c
	${NORMAL_LINT}

db_expr.o: $S/ddb/db_expr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_input.ln: $S/ddb/db_input.c
	${NORMAL_LINT}

db_input.o: $S/ddb/db_input.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_lex.ln: $S/ddb/db_lex.c
	${NORMAL_LINT}

db_lex.o: $S/ddb/db_lex.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_main.ln: $S/ddb/db_main.c
	${NORMAL_LINT}

db_main.o: $S/ddb/db_main.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_output.ln: $S/ddb/db_output.c
	${NORMAL_LINT}

db_output.o: $S/ddb/db_output.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_print.ln: $S/ddb/db_print.c
	${NORMAL_LINT}

db_print.o: $S/ddb/db_print.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_ps.ln: $S/ddb/db_ps.c
	${NORMAL_LINT}

db_ps.o: $S/ddb/db_ps.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_run.ln: $S/ddb/db_run.c
	${NORMAL_LINT}

db_run.o: $S/ddb/db_run.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_script.ln: $S/ddb/db_script.c
	${NORMAL_LINT}

db_script.o: $S/ddb/db_script.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_sym.ln: $S/ddb/db_sym.c
	${NORMAL_LINT}

db_sym.o: $S/ddb/db_sym.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_thread.ln: $S/ddb/db_thread.c
	${NORMAL_LINT}

db_thread.o: $S/ddb/db_thread.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_textdump.ln: $S/ddb/db_textdump.c
	${NORMAL_LINT}

db_textdump.o: $S/ddb/db_textdump.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_variables.ln: $S/ddb/db_variables.c
	${NORMAL_LINT}

db_variables.o: $S/ddb/db_variables.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_watch.ln: $S/ddb/db_watch.c
	${NORMAL_LINT}

db_watch.o: $S/ddb/db_watch.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_write_cmd.ln: $S/ddb/db_write_cmd.c
	${NORMAL_LINT}

db_write_cmd.o: $S/ddb/db_write_cmd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

eisa_if.ln: $S/dev/eisa/eisa_if.m
	${NORMAL_LINT}

eisa_if.o: $S/dev/eisa/eisa_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

led.ln: $S/dev/led/led.c
	${NORMAL_LINT}

led.o: $S/dev/led/led.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

md.ln: $S/dev/md/md.c
	${NORMAL_LINT}

md.o: $S/dev/md/md.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

memdev.ln: $S/dev/mem/memdev.c
	${NORMAL_LINT}

memdev.o: $S/dev/mem/memdev.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

mmcbr_if.ln: $S/dev/mmc/mmcbr_if.m
	${NORMAL_LINT}

mmcbr_if.o: $S/dev/mmc/mmcbr_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

mmcbus_if.ln: $S/dev/mmc/mmcbus_if.m
	${NORMAL_LINT}

mmcbus_if.o: $S/dev/mmc/mmcbus_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

null.ln: $S/dev/null/null.c
	${NORMAL_LINT}

null.o: $S/dev/null/null.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

card_if.ln: $S/dev/pccard/card_if.m
	${NORMAL_LINT}

card_if.o: $S/dev/pccard/card_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

power_if.ln: $S/dev/pccard/power_if.m
	${NORMAL_LINT}

power_if.o: $S/dev/pccard/power_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

fixup_pci.ln: $S/dev/pci/fixup_pci.c
	${NORMAL_LINT}

fixup_pci.o: $S/dev/pci/fixup_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

hostb_pci.ln: $S/dev/pci/hostb_pci.c
	${NORMAL_LINT}

hostb_pci.o: $S/dev/pci/hostb_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ignore_pci.ln: $S/dev/pci/ignore_pci.c
	${NORMAL_LINT}

ignore_pci.o: $S/dev/pci/ignore_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pci.ln: $S/dev/pci/pci.c
	${NORMAL_LINT}

pci.o: $S/dev/pci/pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pci_if.ln: $S/dev/pci/pci_if.m
	${NORMAL_LINT}

pci_if.o: $S/dev/pci/pci_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pci_pci.ln: $S/dev/pci/pci_pci.c
	${NORMAL_LINT}

pci_pci.o: $S/dev/pci/pci_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pci_user.ln: $S/dev/pci/pci_user.c
	${NORMAL_LINT}

pci_user.o: $S/dev/pci/pci_user.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pcib_if.ln: $S/dev/pci/pcib_if.m
	${NORMAL_LINT}

pcib_if.o: $S/dev/pci/pcib_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vga_pci.ln: $S/dev/pci/vga_pci.c
	${NORMAL_LINT}

vga_pci.o: $S/dev/pci/vga_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

harvest.ln: $S/dev/random/harvest.c
	${NORMAL_LINT}

harvest.o: $S/dev/random/harvest.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ehci.ln: $S/dev/usb/controller/ehci.c
	${NORMAL_LINT}

ehci.o: $S/dev/usb/controller/ehci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ehci_pci.ln: $S/dev/usb/controller/ehci_pci.c
	${NORMAL_LINT}

ehci_pci.o: $S/dev/usb/controller/ehci_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uhci.ln: $S/dev/usb/controller/uhci.c
	${NORMAL_LINT}

uhci.o: $S/dev/usb/controller/uhci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uhci_pci.ln: $S/dev/usb/controller/uhci_pci.c
	${NORMAL_LINT}

uhci_pci.o: $S/dev/usb/controller/uhci_pci.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_controller.ln: $S/dev/usb/controller/usb_controller.c
	${NORMAL_LINT}

usb_controller.o: $S/dev/usb/controller/usb_controller.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_busdma.ln: $S/dev/usb/usb_busdma.c
	${NORMAL_LINT}

usb_busdma.o: $S/dev/usb/usb_busdma.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_compat_linux.ln: $S/dev/usb/usb_compat_linux.c
	${NORMAL_LINT}

usb_compat_linux.o: $S/dev/usb/usb_compat_linux.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_core.ln: $S/dev/usb/usb_core.c
	${NORMAL_LINT}

usb_core.o: $S/dev/usb/usb_core.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_debug.ln: $S/dev/usb/usb_debug.c
	${NORMAL_LINT}

usb_debug.o: $S/dev/usb/usb_debug.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_dev.ln: $S/dev/usb/usb_dev.c
	${NORMAL_LINT}

usb_dev.o: $S/dev/usb/usb_dev.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_device.ln: $S/dev/usb/usb_device.c
	${NORMAL_LINT}

usb_device.o: $S/dev/usb/usb_device.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_dynamic.ln: $S/dev/usb/usb_dynamic.c
	${NORMAL_LINT}

usb_dynamic.o: $S/dev/usb/usb_dynamic.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_error.ln: $S/dev/usb/usb_error.c
	${NORMAL_LINT}

usb_error.o: $S/dev/usb/usb_error.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_generic.ln: $S/dev/usb/usb_generic.c
	${NORMAL_LINT}

usb_generic.o: $S/dev/usb/usb_generic.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_handle_request.ln: $S/dev/usb/usb_handle_request.c
	${NORMAL_LINT}

usb_handle_request.o: $S/dev/usb/usb_handle_request.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_hid.ln: $S/dev/usb/usb_hid.c
	${NORMAL_LINT}

usb_hid.o: $S/dev/usb/usb_hid.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_hub.ln: $S/dev/usb/usb_hub.c
	${NORMAL_LINT}

usb_hub.o: $S/dev/usb/usb_hub.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_if.ln: $S/dev/usb/usb_if.m
	${NORMAL_LINT}

usb_if.o: $S/dev/usb/usb_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_lookup.ln: $S/dev/usb/usb_lookup.c
	${NORMAL_LINT}

usb_lookup.o: $S/dev/usb/usb_lookup.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_mbuf.ln: $S/dev/usb/usb_mbuf.c
	${NORMAL_LINT}

usb_mbuf.o: $S/dev/usb/usb_mbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_msctest.ln: $S/dev/usb/usb_msctest.c
	${NORMAL_LINT}

usb_msctest.o: $S/dev/usb/usb_msctest.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_parse.ln: $S/dev/usb/usb_parse.c
	${NORMAL_LINT}

usb_parse.o: $S/dev/usb/usb_parse.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_process.ln: $S/dev/usb/usb_process.c
	${NORMAL_LINT}

usb_process.o: $S/dev/usb/usb_process.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_request.ln: $S/dev/usb/usb_request.c
	${NORMAL_LINT}

usb_request.o: $S/dev/usb/usb_request.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_transfer.ln: $S/dev/usb/usb_transfer.c
	${NORMAL_LINT}

usb_transfer.o: $S/dev/usb/usb_transfer.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_util.ln: $S/dev/usb/usb_util.c
	${NORMAL_LINT}

usb_util.o: $S/dev/usb/usb_util.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

usb_quirk.ln: $S/dev/usb/quirk/usb_quirk.c
	${NORMAL_LINT}

usb_quirk.o: $S/dev/usb/quirk/usb_quirk.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

watchdog.ln: $S/dev/watchdog/watchdog.c
	${NORMAL_LINT}

watchdog.o: $S/dev/watchdog/watchdog.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

dead_vnops.ln: $S/fs/deadfs/dead_vnops.c
	${NORMAL_LINT}

dead_vnops.o: $S/fs/deadfs/dead_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

devfs_devs.ln: $S/fs/devfs/devfs_devs.c
	${NORMAL_LINT}

devfs_devs.o: $S/fs/devfs/devfs_devs.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

devfs_rule.ln: $S/fs/devfs/devfs_rule.c
	${NORMAL_LINT}

devfs_rule.o: $S/fs/devfs/devfs_rule.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

devfs_vfsops.ln: $S/fs/devfs/devfs_vfsops.c
	${NORMAL_LINT}

devfs_vfsops.o: $S/fs/devfs/devfs_vfsops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

devfs_vnops.ln: $S/fs/devfs/devfs_vnops.c
	${NORMAL_LINT}

devfs_vnops.o: $S/fs/devfs/devfs_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

fifo_vnops.ln: $S/fs/fifofs/fifo_vnops.c
	${NORMAL_LINT}

fifo_vnops.o: $S/fs/fifofs/fifo_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pseudofs.ln: $S/fs/pseudofs/pseudofs.c
	${NORMAL_LINT}

pseudofs.o: $S/fs/pseudofs/pseudofs.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pseudofs_fileno.ln: $S/fs/pseudofs/pseudofs_fileno.c
	${NORMAL_LINT}

pseudofs_fileno.o: $S/fs/pseudofs/pseudofs_fileno.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pseudofs_vncache.ln: $S/fs/pseudofs/pseudofs_vncache.c
	${NORMAL_LINT}

pseudofs_vncache.o: $S/fs/pseudofs/pseudofs_vncache.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pseudofs_vnops.ln: $S/fs/pseudofs/pseudofs_vnops.c
	${NORMAL_LINT}

pseudofs_vnops.o: $S/fs/pseudofs/pseudofs_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_ctl.ln: $S/geom/geom_ctl.c
	${NORMAL_LINT}

geom_ctl.o: $S/geom/geom_ctl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_dev.ln: $S/geom/geom_dev.c
	${NORMAL_LINT}

geom_dev.o: $S/geom/geom_dev.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_disk.ln: $S/geom/geom_disk.c
	${NORMAL_LINT}

geom_disk.o: $S/geom/geom_disk.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_dump.ln: $S/geom/geom_dump.c
	${NORMAL_LINT}

geom_dump.o: $S/geom/geom_dump.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_event.ln: $S/geom/geom_event.c
	${NORMAL_LINT}

geom_event.o: $S/geom/geom_event.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_io.ln: $S/geom/geom_io.c
	${NORMAL_LINT}

geom_io.o: $S/geom/geom_io.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_kern.ln: $S/geom/geom_kern.c
	${NORMAL_LINT}

geom_kern.o: $S/geom/geom_kern.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_slice.ln: $S/geom/geom_slice.c
	${NORMAL_LINT}

geom_slice.o: $S/geom/geom_slice.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_subr.ln: $S/geom/geom_subr.c
	${NORMAL_LINT}

geom_subr.o: $S/geom/geom_subr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

geom_vfs.ln: $S/geom/geom_vfs.c
	${NORMAL_LINT}

geom_vfs.o: $S/geom/geom_vfs.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

g_part.ln: $S/geom/part/g_part.c
	${NORMAL_LINT}

g_part.o: $S/geom/part/g_part.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

g_part_if.ln: $S/geom/part/g_part_if.m
	${NORMAL_LINT}

g_part_if.o: $S/geom/part/g_part_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

g_part_bsd.ln: $S/geom/part/g_part_bsd.c
	${NORMAL_LINT}

g_part_bsd.o: $S/geom/part/g_part_bsd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

g_part_mbr.ln: $S/geom/part/g_part_mbr.c
	${NORMAL_LINT}

g_part_mbr.o: $S/geom/part/g_part_mbr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

isa_if.ln: $S/isa/isa_if.m
	${NORMAL_LINT}

isa_if.o: $S/isa/isa_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

bus_if.ln: $S/kern/bus_if.m
	${NORMAL_LINT}

bus_if.o: $S/kern/bus_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

clock_if.ln: $S/kern/clock_if.m
	${NORMAL_LINT}

clock_if.o: $S/kern/clock_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cpufreq_if.ln: $S/kern/cpufreq_if.m
	${NORMAL_LINT}

cpufreq_if.o: $S/kern/cpufreq_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

device_if.ln: $S/kern/device_if.m
	${NORMAL_LINT}

device_if.o: $S/kern/device_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

imgact_elf.ln: $S/kern/imgact_elf.c
	${NORMAL_LINT}

imgact_elf.o: $S/kern/imgact_elf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

imgact_shell.ln: $S/kern/imgact_shell.c
	${NORMAL_LINT}

imgact_shell.o: $S/kern/imgact_shell.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

init_main.ln: $S/kern/init_main.c
	${NORMAL_LINT}

init_main.o: $S/kern/init_main.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

init_sysent.ln: $S/kern/init_sysent.c
	${NORMAL_LINT}

init_sysent.o: $S/kern/init_sysent.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ksched.ln: $S/kern/ksched.c
	${NORMAL_LINT}

ksched.o: $S/kern/ksched.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_acct.ln: $S/kern/kern_acct.c
	${NORMAL_LINT}

kern_acct.o: $S/kern/kern_acct.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_clock.ln: $S/kern/kern_clock.c
	${NORMAL_LINT}

kern_clock.o: $S/kern/kern_clock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_condvar.ln: $S/kern/kern_condvar.c
	${NORMAL_LINT}

kern_condvar.o: $S/kern/kern_condvar.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_conf.ln: $S/kern/kern_conf.c
	${NORMAL_LINT}

kern_conf.o: $S/kern/kern_conf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_cons.ln: $S/kern/kern_cons.c
	${NORMAL_LINT}

kern_cons.o: $S/kern/kern_cons.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_cpu.ln: $S/kern/kern_cpu.c
	${NORMAL_LINT}

kern_cpu.o: $S/kern/kern_cpu.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_cpuset.ln: $S/kern/kern_cpuset.c
	${NORMAL_LINT}

kern_cpuset.o: $S/kern/kern_cpuset.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_context.ln: $S/kern/kern_context.c
	${NORMAL_LINT}

kern_context.o: $S/kern/kern_context.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_descrip.ln: $S/kern/kern_descrip.c
	${NORMAL_LINT}

kern_descrip.o: $S/kern/kern_descrip.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_environment.ln: $S/kern/kern_environment.c
	${NORMAL_LINT}

kern_environment.o: $S/kern/kern_environment.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_event.ln: $S/kern/kern_event.c
	${NORMAL_LINT}

kern_event.o: $S/kern/kern_event.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_exec.ln: $S/kern/kern_exec.c
	${NORMAL_LINT}

kern_exec.o: $S/kern/kern_exec.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_exit.ln: $S/kern/kern_exit.c
	${NORMAL_LINT}

kern_exit.o: $S/kern/kern_exit.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_fail.ln: $S/kern/kern_fail.c
	${NORMAL_LINT}

kern_fail.o: $S/kern/kern_fail.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_fork.ln: $S/kern/kern_fork.c
	${NORMAL_LINT}

kern_fork.o: $S/kern/kern_fork.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_idle.ln: $S/kern/kern_idle.c
	${NORMAL_LINT}

kern_idle.o: $S/kern/kern_idle.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_intr.ln: $S/kern/kern_intr.c
	${NORMAL_LINT}

kern_intr.o: $S/kern/kern_intr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_jail.ln: $S/kern/kern_jail.c
	${NORMAL_LINT}

kern_jail.o: $S/kern/kern_jail.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_kthread.ln: $S/kern/kern_kthread.c
	${NORMAL_LINT}

kern_kthread.o: $S/kern/kern_kthread.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_ktrace.ln: $S/kern/kern_ktrace.c
	${NORMAL_LINT}

kern_ktrace.o: $S/kern/kern_ktrace.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_linker.ln: $S/kern/kern_linker.c
	${NORMAL_LINT}

kern_linker.o: $S/kern/kern_linker.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_lock.ln: $S/kern/kern_lock.c
	${NORMAL_LINT}

kern_lock.o: $S/kern/kern_lock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_lockf.ln: $S/kern/kern_lockf.c
	${NORMAL_LINT}

kern_lockf.o: $S/kern/kern_lockf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_malloc.ln: $S/kern/kern_malloc.c
	${NORMAL_LINT}

kern_malloc.o: $S/kern/kern_malloc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_mbuf.ln: $S/kern/kern_mbuf.c
	${NORMAL_LINT}

kern_mbuf.o: $S/kern/kern_mbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_mib.ln: $S/kern/kern_mib.c
	${NORMAL_LINT}

kern_mib.o: $S/kern/kern_mib.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_module.ln: $S/kern/kern_module.c
	${NORMAL_LINT}

kern_module.o: $S/kern/kern_module.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_mtxpool.ln: $S/kern/kern_mtxpool.c
	${NORMAL_LINT}

kern_mtxpool.o: $S/kern/kern_mtxpool.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_mutex.ln: $S/kern/kern_mutex.c
	${NORMAL_LINT}

kern_mutex.o: $S/kern/kern_mutex.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_ntptime.ln: $S/kern/kern_ntptime.c
	${NORMAL_LINT}

kern_ntptime.o: $S/kern/kern_ntptime.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_osd.ln: $S/kern/kern_osd.c
	${NORMAL_LINT}

kern_osd.o: $S/kern/kern_osd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_physio.ln: $S/kern/kern_physio.c
	${NORMAL_LINT}

kern_physio.o: $S/kern/kern_physio.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_pmc.ln: $S/kern/kern_pmc.c
	${NORMAL_LINT}

kern_pmc.o: $S/kern/kern_pmc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_priv.ln: $S/kern/kern_priv.c
	${NORMAL_LINT}

kern_priv.o: $S/kern/kern_priv.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_proc.ln: $S/kern/kern_proc.c
	${NORMAL_LINT}

kern_proc.o: $S/kern/kern_proc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_prot.ln: $S/kern/kern_prot.c
	${NORMAL_LINT}

kern_prot.o: $S/kern/kern_prot.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_resource.ln: $S/kern/kern_resource.c
	${NORMAL_LINT}

kern_resource.o: $S/kern/kern_resource.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_rmlock.ln: $S/kern/kern_rmlock.c
	${NORMAL_LINT}

kern_rmlock.o: $S/kern/kern_rmlock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_rwlock.ln: $S/kern/kern_rwlock.c
	${NORMAL_LINT}

kern_rwlock.o: $S/kern/kern_rwlock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_sema.ln: $S/kern/kern_sema.c
	${NORMAL_LINT}

kern_sema.o: $S/kern/kern_sema.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_shutdown.ln: $S/kern/kern_shutdown.c
	${NORMAL_LINT}

kern_shutdown.o: $S/kern/kern_shutdown.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_sig.ln: $S/kern/kern_sig.c
	${NORMAL_LINT}

kern_sig.o: $S/kern/kern_sig.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_subr.ln: $S/kern/kern_subr.c
	${NORMAL_LINT}

kern_subr.o: $S/kern/kern_subr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_switch.ln: $S/kern/kern_switch.c
	${NORMAL_LINT}

kern_switch.o: $S/kern/kern_switch.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_sx.ln: $S/kern/kern_sx.c
	${NORMAL_LINT}

kern_sx.o: $S/kern/kern_sx.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_synch.ln: $S/kern/kern_synch.c
	${NORMAL_LINT}

kern_synch.o: $S/kern/kern_synch.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_syscalls.ln: $S/kern/kern_syscalls.c
	${NORMAL_LINT}

kern_syscalls.o: $S/kern/kern_syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_sysctl.ln: $S/kern/kern_sysctl.c
	${NORMAL_LINT}

kern_sysctl.o: $S/kern/kern_sysctl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_tc.ln: $S/kern/kern_tc.c
	${NORMAL_LINT}

kern_tc.o: $S/kern/kern_tc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_thr.ln: $S/kern/kern_thr.c
	${NORMAL_LINT}

kern_thr.o: $S/kern/kern_thr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_thread.ln: $S/kern/kern_thread.c
	${NORMAL_LINT}

kern_thread.o: $S/kern/kern_thread.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_time.ln: $S/kern/kern_time.c
	${NORMAL_LINT}

kern_time.o: $S/kern/kern_time.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_timeout.ln: $S/kern/kern_timeout.c
	${NORMAL_LINT}

kern_timeout.o: $S/kern/kern_timeout.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_umtx.ln: $S/kern/kern_umtx.c
	${NORMAL_LINT}

kern_umtx.o: $S/kern/kern_umtx.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_uuid.ln: $S/kern/kern_uuid.c
	${NORMAL_LINT}

kern_uuid.o: $S/kern/kern_uuid.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

kern_xxx.ln: $S/kern/kern_xxx.c
	${NORMAL_LINT}

kern_xxx.o: $S/kern/kern_xxx.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

link_elf.ln: $S/kern/link_elf.c
	${NORMAL_LINT}

link_elf.o: $S/kern/link_elf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

linker_if.ln: $S/kern/linker_if.m
	${NORMAL_LINT}

linker_if.o: $S/kern/linker_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

md5c.ln: $S/kern/md5c.c
	${NORMAL_LINT}

md5c.o: $S/kern/md5c.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

p1003_1b.ln: $S/kern/p1003_1b.c
	${NORMAL_LINT}

p1003_1b.o: $S/kern/p1003_1b.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

posix4_mib.ln: $S/kern/posix4_mib.c
	${NORMAL_LINT}

posix4_mib.o: $S/kern/posix4_mib.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sched_4bsd.ln: $S/kern/sched_4bsd.c
	${NORMAL_LINT}

sched_4bsd.o: $S/kern/sched_4bsd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

serdev_if.ln: $S/kern/serdev_if.m
	${NORMAL_LINT}

serdev_if.o: $S/kern/serdev_if.m
	${NORMAL_M}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

stack_protector.ln: $S/kern/stack_protector.c
	${NORMAL_LINT}

stack_protector.o: $S/kern/stack_protector.c
	${NORMAL_C:N-fstack-protector*}

subr_acl_nfs4.ln: $S/kern/subr_acl_nfs4.c
	${NORMAL_LINT}

subr_acl_nfs4.o: $S/kern/subr_acl_nfs4.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_acl_posix1e.ln: $S/kern/subr_acl_posix1e.c
	${NORMAL_LINT}

subr_acl_posix1e.o: $S/kern/subr_acl_posix1e.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_autoconf.ln: $S/kern/subr_autoconf.c
	${NORMAL_LINT}

subr_autoconf.o: $S/kern/subr_autoconf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_blist.ln: $S/kern/subr_blist.c
	${NORMAL_LINT}

subr_blist.o: $S/kern/subr_blist.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_bus.ln: $S/kern/subr_bus.c
	${NORMAL_LINT}

subr_bus.o: $S/kern/subr_bus.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_bufring.ln: $S/kern/subr_bufring.c
	${NORMAL_LINT}

subr_bufring.o: $S/kern/subr_bufring.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_clock.ln: $S/kern/subr_clock.c
	${NORMAL_LINT}

subr_clock.o: $S/kern/subr_clock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_devstat.ln: $S/kern/subr_devstat.c
	${NORMAL_LINT}

subr_devstat.o: $S/kern/subr_devstat.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_disk.ln: $S/kern/subr_disk.c
	${NORMAL_LINT}

subr_disk.o: $S/kern/subr_disk.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_eventhandler.ln: $S/kern/subr_eventhandler.c
	${NORMAL_LINT}

subr_eventhandler.o: $S/kern/subr_eventhandler.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_fattime.ln: $S/kern/subr_fattime.c
	${NORMAL_LINT}

subr_fattime.o: $S/kern/subr_fattime.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_hints.ln: $S/kern/subr_hints.c
	${NORMAL_LINT}

subr_hints.o: $S/kern/subr_hints.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_kdb.ln: $S/kern/subr_kdb.c
	${NORMAL_LINT}

subr_kdb.o: $S/kern/subr_kdb.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_kobj.ln: $S/kern/subr_kobj.c
	${NORMAL_LINT}

subr_kobj.o: $S/kern/subr_kobj.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_lock.ln: $S/kern/subr_lock.c
	${NORMAL_LINT}

subr_lock.o: $S/kern/subr_lock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_log.ln: $S/kern/subr_log.c
	${NORMAL_LINT}

subr_log.o: $S/kern/subr_log.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_module.ln: $S/kern/subr_module.c
	${NORMAL_LINT}

subr_module.o: $S/kern/subr_module.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_msgbuf.ln: $S/kern/subr_msgbuf.c
	${NORMAL_LINT}

subr_msgbuf.o: $S/kern/subr_msgbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_param.ln: $S/kern/subr_param.c
	${NORMAL_LINT}

subr_param.o: $S/kern/subr_param.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_pcpu.ln: $S/kern/subr_pcpu.c
	${NORMAL_LINT}

subr_pcpu.o: $S/kern/subr_pcpu.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_power.ln: $S/kern/subr_power.c
	${NORMAL_LINT}

subr_power.o: $S/kern/subr_power.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_prf.ln: $S/kern/subr_prf.c
	${NORMAL_LINT}

subr_prf.o: $S/kern/subr_prf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_prof.ln: $S/kern/subr_prof.c
	${NORMAL_LINT}

subr_prof.o: $S/kern/subr_prof.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_rman.ln: $S/kern/subr_rman.c
	${NORMAL_LINT}

subr_rman.o: $S/kern/subr_rman.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_rtc.ln: $S/kern/subr_rtc.c
	${NORMAL_LINT}

subr_rtc.o: $S/kern/subr_rtc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_sbuf.ln: $S/kern/subr_sbuf.c
	${NORMAL_LINT}

subr_sbuf.o: $S/kern/subr_sbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_scanf.ln: $S/kern/subr_scanf.c
	${NORMAL_LINT}

subr_scanf.o: $S/kern/subr_scanf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_sglist.ln: $S/kern/subr_sglist.c
	${NORMAL_LINT}

subr_sglist.o: $S/kern/subr_sglist.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_sleepqueue.ln: $S/kern/subr_sleepqueue.c
	${NORMAL_LINT}

subr_sleepqueue.o: $S/kern/subr_sleepqueue.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_smp.ln: $S/kern/subr_smp.c
	${NORMAL_LINT}

subr_smp.o: $S/kern/subr_smp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_stack.ln: $S/kern/subr_stack.c
	${NORMAL_LINT}

subr_stack.o: $S/kern/subr_stack.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_taskqueue.ln: $S/kern/subr_taskqueue.c
	${NORMAL_LINT}

subr_taskqueue.o: $S/kern/subr_taskqueue.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_trap.ln: $S/kern/subr_trap.c
	${NORMAL_LINT}

subr_trap.o: $S/kern/subr_trap.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_turnstile.ln: $S/kern/subr_turnstile.c
	${NORMAL_LINT}

subr_turnstile.o: $S/kern/subr_turnstile.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

subr_unit.ln: $S/kern/subr_unit.c
	${NORMAL_LINT}

subr_unit.o: $S/kern/subr_unit.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sys_generic.ln: $S/kern/sys_generic.c
	${NORMAL_LINT}

sys_generic.o: $S/kern/sys_generic.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sys_pipe.ln: $S/kern/sys_pipe.c
	${NORMAL_LINT}

sys_pipe.o: $S/kern/sys_pipe.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sys_process.ln: $S/kern/sys_process.c
	${NORMAL_LINT}

sys_process.o: $S/kern/sys_process.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sys_socket.ln: $S/kern/sys_socket.c
	${NORMAL_LINT}

sys_socket.o: $S/kern/sys_socket.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

syscalls.ln: $S/kern/syscalls.c
	${NORMAL_LINT}

syscalls.o: $S/kern/syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sysv_ipc.ln: $S/kern/sysv_ipc.c
	${NORMAL_LINT}

sysv_ipc.o: $S/kern/sysv_ipc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty.ln: $S/kern/tty.c
	${NORMAL_LINT}

tty.o: $S/kern/tty.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_info.ln: $S/kern/tty_info.c
	${NORMAL_LINT}

tty_info.o: $S/kern/tty_info.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_inq.ln: $S/kern/tty_inq.c
	${NORMAL_LINT}

tty_inq.o: $S/kern/tty_inq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_outq.ln: $S/kern/tty_outq.c
	${NORMAL_LINT}

tty_outq.o: $S/kern/tty_outq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_pts.ln: $S/kern/tty_pts.c
	${NORMAL_LINT}

tty_pts.o: $S/kern/tty_pts.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_tty.ln: $S/kern/tty_tty.c
	${NORMAL_LINT}

tty_tty.o: $S/kern/tty_tty.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tty_ttydisc.ln: $S/kern/tty_ttydisc.c
	${NORMAL_LINT}

tty_ttydisc.o: $S/kern/tty_ttydisc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_accf.ln: $S/kern/uipc_accf.c
	${NORMAL_LINT}

uipc_accf.o: $S/kern/uipc_accf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_debug.ln: $S/kern/uipc_debug.c
	${NORMAL_LINT}

uipc_debug.o: $S/kern/uipc_debug.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_domain.ln: $S/kern/uipc_domain.c
	${NORMAL_LINT}

uipc_domain.o: $S/kern/uipc_domain.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_mbuf.ln: $S/kern/uipc_mbuf.c
	${NORMAL_LINT}

uipc_mbuf.o: $S/kern/uipc_mbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_mbuf2.ln: $S/kern/uipc_mbuf2.c
	${NORMAL_LINT}

uipc_mbuf2.o: $S/kern/uipc_mbuf2.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_shm.ln: $S/kern/uipc_shm.c
	${NORMAL_LINT}

uipc_shm.o: $S/kern/uipc_shm.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_sockbuf.ln: $S/kern/uipc_sockbuf.c
	${NORMAL_LINT}

uipc_sockbuf.o: $S/kern/uipc_sockbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_socket.ln: $S/kern/uipc_socket.c
	${NORMAL_LINT}

uipc_socket.o: $S/kern/uipc_socket.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_syscalls.ln: $S/kern/uipc_syscalls.c
	${NORMAL_LINT}

uipc_syscalls.o: $S/kern/uipc_syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uipc_usrreq.ln: $S/kern/uipc_usrreq.c
	${NORMAL_LINT}

uipc_usrreq.o: $S/kern/uipc_usrreq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_acl.ln: $S/kern/vfs_acl.c
	${NORMAL_LINT}

vfs_acl.o: $S/kern/vfs_acl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_bio.ln: $S/kern/vfs_bio.c
	${NORMAL_LINT}

vfs_bio.o: $S/kern/vfs_bio.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_cache.ln: $S/kern/vfs_cache.c
	${NORMAL_LINT}

vfs_cache.o: $S/kern/vfs_cache.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_cluster.ln: $S/kern/vfs_cluster.c
	${NORMAL_LINT}

vfs_cluster.o: $S/kern/vfs_cluster.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_default.ln: $S/kern/vfs_default.c
	${NORMAL_LINT}

vfs_default.o: $S/kern/vfs_default.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_export.ln: $S/kern/vfs_export.c
	${NORMAL_LINT}

vfs_export.o: $S/kern/vfs_export.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_extattr.ln: $S/kern/vfs_extattr.c
	${NORMAL_LINT}

vfs_extattr.o: $S/kern/vfs_extattr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_hash.ln: $S/kern/vfs_hash.c
	${NORMAL_LINT}

vfs_hash.o: $S/kern/vfs_hash.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_init.ln: $S/kern/vfs_init.c
	${NORMAL_LINT}

vfs_init.o: $S/kern/vfs_init.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_lookup.ln: $S/kern/vfs_lookup.c
	${NORMAL_LINT}

vfs_lookup.o: $S/kern/vfs_lookup.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_mount.ln: $S/kern/vfs_mount.c
	${NORMAL_LINT}

vfs_mount.o: $S/kern/vfs_mount.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_subr.ln: $S/kern/vfs_subr.c
	${NORMAL_LINT}

vfs_subr.o: $S/kern/vfs_subr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_syscalls.ln: $S/kern/vfs_syscalls.c
	${NORMAL_LINT}

vfs_syscalls.o: $S/kern/vfs_syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vfs_vnops.ln: $S/kern/vfs_vnops.c
	${NORMAL_LINT}

vfs_vnops.o: $S/kern/vfs_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

arc4random.ln: $S/libkern/arc4random.c
	${NORMAL_LINT}

arc4random.o: $S/libkern/arc4random.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

bcd.ln: $S/libkern/bcd.c
	${NORMAL_LINT}

bcd.o: $S/libkern/bcd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

bsearch.ln: $S/libkern/bsearch.c
	${NORMAL_LINT}

bsearch.o: $S/libkern/bsearch.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

crc32.ln: $S/libkern/crc32.c
	${NORMAL_LINT}

crc32.o: $S/libkern/crc32.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

fnmatch.ln: $S/libkern/fnmatch.c
	${NORMAL_LINT}

fnmatch.o: $S/libkern/fnmatch.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

gets.ln: $S/libkern/gets.c
	${NORMAL_LINT}

gets.o: $S/libkern/gets.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

index.ln: $S/libkern/index.c
	${NORMAL_LINT}

index.o: $S/libkern/index.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

inet_aton.ln: $S/libkern/inet_aton.c
	${NORMAL_LINT}

inet_aton.o: $S/libkern/inet_aton.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

inet_ntoa.ln: $S/libkern/inet_ntoa.c
	${NORMAL_LINT}

inet_ntoa.o: $S/libkern/inet_ntoa.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

memcmp.ln: $S/libkern/memcmp.c
	${NORMAL_LINT}

memcmp.o: $S/libkern/memcmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

qsort.ln: $S/libkern/qsort.c
	${NORMAL_LINT}

qsort.o: $S/libkern/qsort.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

qsort_r.ln: $S/libkern/qsort_r.c
	${NORMAL_LINT}

qsort_r.o: $S/libkern/qsort_r.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

random.ln: $S/libkern/random.c
	${NORMAL_LINT}

random.o: $S/libkern/random.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rindex.ln: $S/libkern/rindex.c
	${NORMAL_LINT}

rindex.o: $S/libkern/rindex.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

scanc.ln: $S/libkern/scanc.c
	${NORMAL_LINT}

scanc.o: $S/libkern/scanc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

skpc.ln: $S/libkern/skpc.c
	${NORMAL_LINT}

skpc.o: $S/libkern/skpc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strcasecmp.ln: $S/libkern/strcasecmp.c
	${NORMAL_LINT}

strcasecmp.o: $S/libkern/strcasecmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strcat.ln: $S/libkern/strcat.c
	${NORMAL_LINT}

strcat.o: $S/libkern/strcat.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strcmp.ln: $S/libkern/strcmp.c
	${NORMAL_LINT}

strcmp.o: $S/libkern/strcmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strcpy.ln: $S/libkern/strcpy.c
	${NORMAL_LINT}

strcpy.o: $S/libkern/strcpy.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strcspn.ln: $S/libkern/strcspn.c
	${NORMAL_LINT}

strcspn.o: $S/libkern/strcspn.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strdup.ln: $S/libkern/strdup.c
	${NORMAL_LINT}

strdup.o: $S/libkern/strdup.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strlcat.ln: $S/libkern/strlcat.c
	${NORMAL_LINT}

strlcat.o: $S/libkern/strlcat.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strlcpy.ln: $S/libkern/strlcpy.c
	${NORMAL_LINT}

strlcpy.o: $S/libkern/strlcpy.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strlen.ln: $S/libkern/strlen.c
	${NORMAL_LINT}

strlen.o: $S/libkern/strlen.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strncmp.ln: $S/libkern/strncmp.c
	${NORMAL_LINT}

strncmp.o: $S/libkern/strncmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strncpy.ln: $S/libkern/strncpy.c
	${NORMAL_LINT}

strncpy.o: $S/libkern/strncpy.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strsep.ln: $S/libkern/strsep.c
	${NORMAL_LINT}

strsep.o: $S/libkern/strsep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strspn.ln: $S/libkern/strspn.c
	${NORMAL_LINT}

strspn.o: $S/libkern/strspn.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strstr.ln: $S/libkern/strstr.c
	${NORMAL_LINT}

strstr.o: $S/libkern/strstr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strtol.ln: $S/libkern/strtol.c
	${NORMAL_LINT}

strtol.o: $S/libkern/strtol.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strtoq.ln: $S/libkern/strtoq.c
	${NORMAL_LINT}

strtoq.o: $S/libkern/strtoq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strtoul.ln: $S/libkern/strtoul.c
	${NORMAL_LINT}

strtoul.o: $S/libkern/strtoul.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strtouq.ln: $S/libkern/strtouq.c
	${NORMAL_LINT}

strtouq.o: $S/libkern/strtouq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

strvalid.ln: $S/libkern/strvalid.c
	${NORMAL_LINT}

strvalid.o: $S/libkern/strvalid.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

bpf.ln: $S/net/bpf.c
	${NORMAL_LINT}

bpf.o: $S/net/bpf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if.ln: $S/net/if.c
	${NORMAL_LINT}

if.o: $S/net/if.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_clone.ln: $S/net/if_clone.c
	${NORMAL_LINT}

if_clone.o: $S/net/if_clone.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_dead.ln: $S/net/if_dead.c
	${NORMAL_LINT}

if_dead.o: $S/net/if_dead.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_ethersubr.ln: $S/net/if_ethersubr.c
	${NORMAL_LINT}

if_ethersubr.o: $S/net/if_ethersubr.c
	${NORMAL_C} -I$S/contrib/pf

if_loop.ln: $S/net/if_loop.c
	${NORMAL_LINT}

if_loop.o: $S/net/if_loop.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_llatbl.ln: $S/net/if_llatbl.c
	${NORMAL_LINT}

if_llatbl.o: $S/net/if_llatbl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_media.ln: $S/net/if_media.c
	${NORMAL_LINT}

if_media.o: $S/net/if_media.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_mib.ln: $S/net/if_mib.c
	${NORMAL_LINT}

if_mib.o: $S/net/if_mib.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

netisr.ln: $S/net/netisr.c
	${NORMAL_LINT}

netisr.o: $S/net/netisr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pfil.ln: $S/net/pfil.c
	${NORMAL_LINT}

pfil.o: $S/net/pfil.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

radix.ln: $S/net/radix.c
	${NORMAL_LINT}

radix.o: $S/net/radix.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

radix_mpath.ln: $S/net/radix_mpath.c
	${NORMAL_LINT}

radix_mpath.o: $S/net/radix_mpath.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

raw_cb.ln: $S/net/raw_cb.c
	${NORMAL_LINT}

raw_cb.o: $S/net/raw_cb.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

raw_usrreq.ln: $S/net/raw_usrreq.c
	${NORMAL_LINT}

raw_usrreq.o: $S/net/raw_usrreq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

route.ln: $S/net/route.c
	${NORMAL_LINT}

route.o: $S/net/route.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rtsock.ln: $S/net/rtsock.c
	${NORMAL_LINT}

rtsock.o: $S/net/rtsock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

if_ether.ln: $S/netinet/if_ether.c
	${NORMAL_LINT}

if_ether.o: $S/netinet/if_ether.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

igmp.ln: $S/netinet/igmp.c
	${NORMAL_LINT}

igmp.o: $S/netinet/igmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

in.ln: $S/netinet/in.c
	${NORMAL_LINT}

in.o: $S/netinet/in.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_id.ln: $S/netinet/ip_id.c
	${NORMAL_LINT}

ip_id.o: $S/netinet/ip_id.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

in_mcast.ln: $S/netinet/in_mcast.c
	${NORMAL_LINT}

in_mcast.o: $S/netinet/in_mcast.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

in_pcb.ln: $S/netinet/in_pcb.c
	${NORMAL_LINT}

in_pcb.o: $S/netinet/in_pcb.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

in_proto.ln: $S/netinet/in_proto.c
	${NORMAL_LINT}

in_proto.o: $S/netinet/in_proto.c
	${NORMAL_C} -I$S/contrib/pf

in_rmx.ln: $S/netinet/in_rmx.c
	${NORMAL_LINT}

in_rmx.o: $S/netinet/in_rmx.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_ecn.ln: $S/netinet/ip_ecn.c
	${NORMAL_LINT}

ip_ecn.o: $S/netinet/ip_ecn.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_encap.ln: $S/netinet/ip_encap.c
	${NORMAL_LINT}

ip_encap.o: $S/netinet/ip_encap.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_fastfwd.ln: $S/netinet/ip_fastfwd.c
	${NORMAL_LINT}

ip_fastfwd.o: $S/netinet/ip_fastfwd.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_icmp.ln: $S/netinet/ip_icmp.c
	${NORMAL_LINT}

ip_icmp.o: $S/netinet/ip_icmp.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_input.ln: $S/netinet/ip_input.c
	${NORMAL_LINT}

ip_input.o: $S/netinet/ip_input.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_options.ln: $S/netinet/ip_options.c
	${NORMAL_LINT}

ip_options.o: $S/netinet/ip_options.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ip_output.ln: $S/netinet/ip_output.c
	${NORMAL_LINT}

ip_output.o: $S/netinet/ip_output.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

raw_ip.ln: $S/netinet/raw_ip.c
	${NORMAL_LINT}

raw_ip.o: $S/netinet/raw_ip.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_hostcache.ln: $S/netinet/tcp_hostcache.c
	${NORMAL_LINT}

tcp_hostcache.o: $S/netinet/tcp_hostcache.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_input.ln: $S/netinet/tcp_input.c
	${NORMAL_LINT}

tcp_input.o: $S/netinet/tcp_input.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_lro.ln: $S/netinet/tcp_lro.c
	${NORMAL_LINT}

tcp_lro.o: $S/netinet/tcp_lro.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_output.ln: $S/netinet/tcp_output.c
	${NORMAL_LINT}

tcp_output.o: $S/netinet/tcp_output.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_offload.ln: $S/netinet/tcp_offload.c
	${NORMAL_LINT}

tcp_offload.o: $S/netinet/tcp_offload.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_reass.ln: $S/netinet/tcp_reass.c
	${NORMAL_LINT}

tcp_reass.o: $S/netinet/tcp_reass.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_sack.ln: $S/netinet/tcp_sack.c
	${NORMAL_LINT}

tcp_sack.o: $S/netinet/tcp_sack.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_subr.ln: $S/netinet/tcp_subr.c
	${NORMAL_LINT}

tcp_subr.o: $S/netinet/tcp_subr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_syncache.ln: $S/netinet/tcp_syncache.c
	${NORMAL_LINT}

tcp_syncache.o: $S/netinet/tcp_syncache.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_timer.ln: $S/netinet/tcp_timer.c
	${NORMAL_LINT}

tcp_timer.o: $S/netinet/tcp_timer.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_timewait.ln: $S/netinet/tcp_timewait.c
	${NORMAL_LINT}

tcp_timewait.o: $S/netinet/tcp_timewait.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tcp_usrreq.ln: $S/netinet/tcp_usrreq.c
	${NORMAL_LINT}

tcp_usrreq.o: $S/netinet/tcp_usrreq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

udp_usrreq.ln: $S/netinet/udp_usrreq.c
	${NORMAL_LINT}

udp_usrreq.o: $S/netinet/udp_usrreq.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_common.ln: $S/nfs/nfs_common.c
	${NORMAL_LINT}

nfs_common.o: $S/nfs/nfs_common.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_bio.ln: $S/nfsclient/nfs_bio.c
	${NORMAL_LINT}

nfs_bio.o: $S/nfsclient/nfs_bio.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_diskless.ln: $S/nfsclient/nfs_diskless.c
	${NORMAL_LINT}

nfs_diskless.o: $S/nfsclient/nfs_diskless.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_node.ln: $S/nfsclient/nfs_node.c
	${NORMAL_LINT}

nfs_node.o: $S/nfsclient/nfs_node.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_krpc.ln: $S/nfsclient/nfs_krpc.c
	${NORMAL_LINT}

nfs_krpc.o: $S/nfsclient/nfs_krpc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_subs.ln: $S/nfsclient/nfs_subs.c
	${NORMAL_LINT}

nfs_subs.o: $S/nfsclient/nfs_subs.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_nfsiod.ln: $S/nfsclient/nfs_nfsiod.c
	${NORMAL_LINT}

nfs_nfsiod.o: $S/nfsclient/nfs_nfsiod.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_vfsops.ln: $S/nfsclient/nfs_vfsops.c
	${NORMAL_LINT}

nfs_vfsops.o: $S/nfsclient/nfs_vfsops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_vnops.ln: $S/nfsclient/nfs_vnops.c
	${NORMAL_LINT}

nfs_vnops.o: $S/nfsclient/nfs_vnops.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nfs_lock.ln: $S/nfsclient/nfs_lock.c
	${NORMAL_LINT}

nfs_lock.o: $S/nfsclient/nfs_lock.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

auth_none.ln: $S/rpc/auth_none.c
	${NORMAL_LINT}

auth_none.o: $S/rpc/auth_none.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

auth_unix.ln: $S/rpc/auth_unix.c
	${NORMAL_LINT}

auth_unix.o: $S/rpc/auth_unix.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

authunix_prot.ln: $S/rpc/authunix_prot.c
	${NORMAL_LINT}

authunix_prot.o: $S/rpc/authunix_prot.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

clnt_dg.ln: $S/rpc/clnt_dg.c
	${NORMAL_LINT}

clnt_dg.o: $S/rpc/clnt_dg.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

clnt_rc.ln: $S/rpc/clnt_rc.c
	${NORMAL_LINT}

clnt_rc.o: $S/rpc/clnt_rc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

clnt_vc.ln: $S/rpc/clnt_vc.c
	${NORMAL_LINT}

clnt_vc.o: $S/rpc/clnt_vc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

getnetconfig.ln: $S/rpc/getnetconfig.c
	${NORMAL_LINT}

getnetconfig.o: $S/rpc/getnetconfig.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

inet_ntop.ln: $S/rpc/inet_ntop.c
	${NORMAL_LINT}

inet_ntop.o: $S/rpc/inet_ntop.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

inet_pton.ln: $S/rpc/inet_pton.c
	${NORMAL_LINT}

inet_pton.o: $S/rpc/inet_pton.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rpc_callmsg.ln: $S/rpc/rpc_callmsg.c
	${NORMAL_LINT}

rpc_callmsg.o: $S/rpc/rpc_callmsg.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rpc_generic.ln: $S/rpc/rpc_generic.c
	${NORMAL_LINT}

rpc_generic.o: $S/rpc/rpc_generic.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rpc_prot.ln: $S/rpc/rpc_prot.c
	${NORMAL_LINT}

rpc_prot.o: $S/rpc/rpc_prot.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rpcb_clnt.ln: $S/rpc/rpcb_clnt.c
	${NORMAL_LINT}

rpcb_clnt.o: $S/rpc/rpcb_clnt.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

rpcb_prot.ln: $S/rpc/rpcb_prot.c
	${NORMAL_LINT}

rpcb_prot.o: $S/rpc/rpcb_prot.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

audit_syscalls.ln: $S/security/audit/audit_syscalls.c
	${NORMAL_LINT}

audit_syscalls.o: $S/security/audit/audit_syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

mac_syscalls.ln: $S/security/mac/mac_syscalls.c
	${NORMAL_LINT}

mac_syscalls.o: $S/security/mac/mac_syscalls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

default_pager.ln: $S/vm/default_pager.c
	${NORMAL_LINT}

default_pager.o: $S/vm/default_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

device_pager.ln: $S/vm/device_pager.c
	${NORMAL_LINT}

device_pager.o: $S/vm/device_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

phys_pager.ln: $S/vm/phys_pager.c
	${NORMAL_LINT}

phys_pager.o: $S/vm/phys_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sg_pager.ln: $S/vm/sg_pager.c
	${NORMAL_LINT}

sg_pager.o: $S/vm/sg_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

swap_pager.ln: $S/vm/swap_pager.c
	${NORMAL_LINT}

swap_pager.o: $S/vm/swap_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uma_core.ln: $S/vm/uma_core.c
	${NORMAL_LINT}

uma_core.o: $S/vm/uma_core.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uma_dbg.ln: $S/vm/uma_dbg.c
	${NORMAL_LINT}

uma_dbg.o: $S/vm/uma_dbg.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_contig.ln: $S/vm/vm_contig.c
	${NORMAL_LINT}

vm_contig.o: $S/vm/vm_contig.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_fault.ln: $S/vm/vm_fault.c
	${NORMAL_LINT}

vm_fault.o: $S/vm/vm_fault.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_glue.ln: $S/vm/vm_glue.c
	${NORMAL_LINT}

vm_glue.o: $S/vm/vm_glue.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_init.ln: $S/vm/vm_init.c
	${NORMAL_LINT}

vm_init.o: $S/vm/vm_init.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_kern.ln: $S/vm/vm_kern.c
	${NORMAL_LINT}

vm_kern.o: $S/vm/vm_kern.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_map.ln: $S/vm/vm_map.c
	${NORMAL_LINT}

vm_map.o: $S/vm/vm_map.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_meter.ln: $S/vm/vm_meter.c
	${NORMAL_LINT}

vm_meter.o: $S/vm/vm_meter.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_mmap.ln: $S/vm/vm_mmap.c
	${NORMAL_LINT}

vm_mmap.o: $S/vm/vm_mmap.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_object.ln: $S/vm/vm_object.c
	${NORMAL_LINT}

vm_object.o: $S/vm/vm_object.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_page.ln: $S/vm/vm_page.c
	${NORMAL_LINT}

vm_page.o: $S/vm/vm_page.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_pageout.ln: $S/vm/vm_pageout.c
	${NORMAL_LINT}

vm_pageout.o: $S/vm/vm_pageout.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_pager.ln: $S/vm/vm_pager.c
	${NORMAL_LINT}

vm_pager.o: $S/vm/vm_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_phys.ln: $S/vm/vm_phys.c
	${NORMAL_LINT}

vm_phys.o: $S/vm/vm_phys.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_reserv.ln: $S/vm/vm_reserv.c
	${NORMAL_LINT}

vm_reserv.o: $S/vm/vm_reserv.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_unix.ln: $S/vm/vm_unix.c
	${NORMAL_LINT}

vm_unix.o: $S/vm/vm_unix.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_zeroidle.ln: $S/vm/vm_zeroidle.c
	${NORMAL_LINT}

vm_zeroidle.o: $S/vm/vm_zeroidle.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vnode_pager.ln: $S/vm/vnode_pager.c
	${NORMAL_LINT}

vnode_pager.o: $S/vm/vnode_pager.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr.ln: $S/xdr/xdr.c
	${NORMAL_LINT}

xdr.o: $S/xdr/xdr.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr_array.ln: $S/xdr/xdr_array.c
	${NORMAL_LINT}

xdr_array.o: $S/xdr/xdr_array.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr_mbuf.ln: $S/xdr/xdr_mbuf.c
	${NORMAL_LINT}

xdr_mbuf.o: $S/xdr/xdr_mbuf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr_mem.ln: $S/xdr/xdr_mem.c
	${NORMAL_LINT}

xdr_mem.o: $S/xdr/xdr_mem.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr_reference.ln: $S/xdr/xdr_reference.c
	${NORMAL_LINT}

xdr_reference.o: $S/xdr/xdr_reference.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

xdr_sizeof.ln: $S/xdr/xdr_sizeof.c
	${NORMAL_LINT}

xdr_sizeof.o: $S/xdr/xdr_sizeof.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

machdep.ln: $S/mips/mips/machdep.c
	${NORMAL_LINT}

machdep.o: $S/mips/mips/machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

psraccess.ln: $S/mips/mips/psraccess.S
	${NORMAL_LINT}

psraccess.o: $S/mips/mips/psraccess.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

autoconf.ln: $S/mips/mips/autoconf.c
	${NORMAL_LINT}

autoconf.o: $S/mips/mips/autoconf.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cpu.ln: $S/mips/mips/cpu.c
	${NORMAL_LINT}

cpu.o: $S/mips/mips/cpu.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

elf_machdep.ln: $S/mips/mips/elf_machdep.c
	${NORMAL_LINT}

elf_machdep.o: $S/mips/mips/elf_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

exception.ln: $S/mips/mips/exception.S
	${NORMAL_LINT}

exception.o: $S/mips/mips/exception.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

gdb_machdep.ln: $S/mips/mips/gdb_machdep.c
	${NORMAL_LINT}

gdb_machdep.o: $S/mips/mips/gdb_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pmap.ln: $S/mips/mips/pmap.c
	${NORMAL_LINT}

pmap.o: $S/mips/mips/pmap.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

trap.ln: $S/mips/mips/trap.c
	${NORMAL_LINT}

trap.o: $S/mips/mips/trap.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

vm_machdep.ln: $S/mips/mips/vm_machdep.c
	${NORMAL_LINT}

vm_machdep.o: $S/mips/mips/vm_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

fp.ln: $S/mips/mips/fp.S
	${NORMAL_LINT}

fp.o: $S/mips/mips/fp.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

pm_machdep.ln: $S/mips/mips/pm_machdep.c
	${NORMAL_LINT}

pm_machdep.o: $S/mips/mips/pm_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

swtch.ln: $S/mips/mips/swtch.S
	${NORMAL_LINT}

swtch.o: $S/mips/mips/swtch.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tlb.ln: $S/mips/mips/tlb.S
	${NORMAL_LINT}

tlb.o: $S/mips/mips/tlb.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

bus_space_generic.ln: $S/mips/mips/bus_space_generic.c
	${NORMAL_LINT}

bus_space_generic.o: $S/mips/mips/bus_space_generic.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

busdma_machdep.ln: $S/mips/mips/busdma_machdep.c
	${NORMAL_LINT}

busdma_machdep.o: $S/mips/mips/busdma_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cache.ln: $S/mips/mips/cache.c
	${NORMAL_LINT}

cache.o: $S/mips/mips/cache.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cache_mipsNN.ln: $S/mips/mips/cache_mipsNN.c
	${NORMAL_LINT}

cache_mipsNN.o: $S/mips/mips/cache_mipsNN.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_disasm.ln: $S/mips/mips/db_disasm.c
	${NORMAL_LINT}

db_disasm.o: $S/mips/mips/db_disasm.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_interface.ln: $S/mips/mips/db_interface.c
	${NORMAL_LINT}

db_interface.o: $S/mips/mips/db_interface.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

db_trace.ln: $S/mips/mips/db_trace.c
	${NORMAL_LINT}

db_trace.o: $S/mips/mips/db_trace.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

dump_machdep.ln: $S/mips/mips/dump_machdep.c
	${NORMAL_LINT}

dump_machdep.o: $S/mips/mips/dump_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

in_cksum.ln: $S/mips/mips/in_cksum.c
	${NORMAL_LINT}

in_cksum.o: $S/mips/mips/in_cksum.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

locore.ln: $S/mips/mips/locore.S
	${NORMAL_LINT}

locore.o: $S/mips/mips/locore.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

mem.ln: $S/mips/mips/mem.c
	${NORMAL_LINT}

mem.o: $S/mips/mips/mem.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

nexus.ln: $S/mips/mips/nexus.c
	${NORMAL_LINT}

nexus.o: $S/mips/mips/nexus.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

stack_machdep.ln: $S/mips/mips/stack_machdep.c
	${NORMAL_LINT}

stack_machdep.o: $S/mips/mips/stack_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

support.ln: $S/mips/mips/support.S
	${NORMAL_LINT}

support.o: $S/mips/mips/support.S
	${NORMAL_S}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

sys_machdep.ln: $S/mips/mips/sys_machdep.c
	${NORMAL_LINT}

sys_machdep.o: $S/mips/mips/sys_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

uio_machdep.ln: $S/mips/mips/uio_machdep.c
	${NORMAL_LINT}

uio_machdep.o: $S/mips/mips/uio_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ashldi3.ln: $S/libkern/ashldi3.c
	${NORMAL_LINT}

ashldi3.o: $S/libkern/ashldi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ashrdi3.ln: $S/libkern/ashrdi3.c
	${NORMAL_LINT}

ashrdi3.o: $S/libkern/ashrdi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

divdi3.ln: $S/libkern/divdi3.c
	${NORMAL_LINT}

divdi3.o: $S/libkern/divdi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

ffsl.ln: $S/libkern/ffsl.c
	${NORMAL_LINT}

ffsl.o: $S/libkern/ffsl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

fls.ln: $S/libkern/fls.c
	${NORMAL_LINT}

fls.o: $S/libkern/fls.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

flsl.ln: $S/libkern/flsl.c
	${NORMAL_LINT}

flsl.o: $S/libkern/flsl.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

lshrdi3.ln: $S/libkern/lshrdi3.c
	${NORMAL_LINT}

lshrdi3.o: $S/libkern/lshrdi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

memmove.ln: $S/libkern/memmove.c
	${NORMAL_LINT}

memmove.o: $S/libkern/memmove.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

moddi3.ln: $S/libkern/moddi3.c
	${NORMAL_LINT}

moddi3.o: $S/libkern/moddi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

qdivrem.ln: $S/libkern/qdivrem.c
	${NORMAL_LINT}

qdivrem.o: $S/libkern/qdivrem.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

udivdi3.ln: $S/libkern/udivdi3.c
	${NORMAL_LINT}

udivdi3.o: $S/libkern/udivdi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

umoddi3.ln: $S/libkern/umoddi3.c
	${NORMAL_LINT}

umoddi3.o: $S/libkern/umoddi3.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cfe_api.ln: $S/dev/cfe/cfe_api.c
	${NORMAL_LINT}

cfe_api.o: $S/dev/cfe/cfe_api.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

cfe_console.ln: $S/dev/cfe/cfe_console.c
	${NORMAL_LINT}

cfe_console.o: $S/dev/cfe/cfe_console.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

siba.ln: $S/dev/siba/siba.c
	${NORMAL_LINT}

siba.o: $S/dev/siba/siba.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

siba_pcib.ln: $S/dev/siba/siba_pcib.c
	${NORMAL_LINT}

siba_pcib.o: $S/dev/siba/siba_pcib.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

siba_cc.ln: $S/dev/siba/siba_cc.c
	${NORMAL_LINT}

siba_cc.o: $S/dev/siba/siba_cc.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

s5_machdep.ln: $S/mips/sentry5/s5_machdep.c
	${NORMAL_LINT}

s5_machdep.o: $S/mips/sentry5/s5_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

intr_machdep.ln: $S/mips/mips/intr_machdep.c
	${NORMAL_LINT}

intr_machdep.o: $S/mips/mips/intr_machdep.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif

tick.ln: $S/mips/mips/tick.c
	${NORMAL_LINT}

tick.o: $S/mips/mips/tick.c
	${NORMAL_C}
.if defined(NORMAL_CTFCONVERT) && !empty(NORMAL_CTFCONVERT)
	${NORMAL_CTFCONVERT}
.endif


.include "$S/conf/kern.post.mk"
