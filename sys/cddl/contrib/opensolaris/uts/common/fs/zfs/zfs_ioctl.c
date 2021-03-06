/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2009 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/errno.h>
#include <sys/uio.h>
#include <sys/buf.h>
#include <sys/file.h>
#include <sys/kmem.h>
#include <sys/conf.h>
#include <sys/cmn_err.h>
#include <sys/stat.h>
#include <sys/zfs_ioctl.h>
#include <sys/zfs_vfsops.h>
#include <sys/zfs_znode.h>
#include <sys/zap.h>
#include <sys/spa.h>
#include <sys/spa_impl.h>
#include <sys/vdev.h>
#include <sys/vdev_impl.h>
#include <sys/dmu.h>
#include <sys/dsl_dir.h>
#include <sys/dsl_dataset.h>
#include <sys/dsl_prop.h>
#include <sys/dsl_deleg.h>
#include <sys/dmu_objset.h>
#include <sys/sunddi.h>
#include <sys/policy.h>
#include <sys/zone.h>
#include <sys/nvpair.h>
#include <sys/mount.h>
#include <sys/taskqueue.h>
#include <sys/sdt.h>
#include <sys/varargs.h>
#include <sys/fs/zfs.h>
#include <sys/zfs_ctldir.h>
#include <sys/zfs_dir.h>
#include <sys/zvol.h>
#include <sys/dmu_objset.h>

#include "zfs_namecheck.h"
#include "zfs_prop.h"
#include "zfs_deleg.h"

CTASSERT(sizeof(zfs_cmd_t) <= PAGE_SIZE);

static struct cdev *zfsdev;

extern void zfs_init(void);
extern void zfs_fini(void);

typedef int zfs_ioc_func_t(zfs_cmd_t *);
typedef int zfs_secpolicy_func_t(zfs_cmd_t *, cred_t *);

typedef enum {
	NO_NAME,
	POOL_NAME,
	DATASET_NAME
} zfs_ioc_namecheck_t;

typedef struct zfs_ioc_vec {
	zfs_ioc_func_t		*zvec_func;
	zfs_secpolicy_func_t	*zvec_secpolicy;
	zfs_ioc_namecheck_t	zvec_namecheck;
	boolean_t		zvec_his_log;
	boolean_t		zvec_pool_check;
} zfs_ioc_vec_t;

/* This array is indexed by zfs_userquota_prop_t */
static const char *userquota_perms[] = {
	ZFS_DELEG_PERM_USERUSED,
	ZFS_DELEG_PERM_USERQUOTA,
	ZFS_DELEG_PERM_GROUPUSED,
	ZFS_DELEG_PERM_GROUPQUOTA,
};

static int zfs_ioc_userspace_upgrade(zfs_cmd_t *zc);
static void clear_props(char *dataset, nvlist_t *props, nvlist_t *newprops);
static int zfs_fill_zplprops_root(uint64_t, nvlist_t *, nvlist_t *,
    boolean_t *);
int zfs_set_prop_nvlist(const char *, nvlist_t *);

/* _NOTE(PRINTFLIKE(4)) - this is printf-like, but lint is too whiney */
void
__dprintf(const char *file, const char *func, int line, const char *fmt, ...)
{
	const char *newfile;
	char buf[256];
	va_list adx;

	/*
	 * Get rid of annoying "../common/" prefix to filename.
	 */
	newfile = strrchr(file, '/');
	if (newfile != NULL) {
		newfile = newfile + 1; /* Get rid of leading / */
	} else {
		newfile = file;
	}

	va_start(adx, fmt);
	(void) vsnprintf(buf, sizeof (buf), fmt, adx);
	va_end(adx);

	/*
	 * To get this data, use the zfs-dprintf probe as so:
	 * dtrace -q -n 'zfs-dprintf \
	 *	/stringof(arg0) == "dbuf.c"/ \
	 *	{printf("%s: %s", stringof(arg1), stringof(arg3))}'
	 * arg0 = file name
	 * arg1 = function name
	 * arg2 = line number
	 * arg3 = message
	 */
	DTRACE_PROBE4(zfs__dprintf,
	    char *, newfile, char *, func, int, line, char *, buf);
}

static void
history_str_free(char *buf)
{
	kmem_free(buf, HIS_MAX_RECORD_LEN);
}

static char *
history_str_get(zfs_cmd_t *zc)
{
	char *buf;

	if (zc->zc_history == 0)
		return (NULL);

	buf = kmem_alloc(HIS_MAX_RECORD_LEN, KM_SLEEP);
	if (copyinstr((void *)(uintptr_t)zc->zc_history,
	    buf, HIS_MAX_RECORD_LEN, NULL) != 0) {
		history_str_free(buf);
		return (NULL);
	}

	buf[HIS_MAX_RECORD_LEN -1] = '\0';

	return (buf);
}

/*
 * Check to see if the named dataset is currently defined as bootable
 */
static boolean_t
zfs_is_bootfs(const char *name)
{
	spa_t *spa;
	boolean_t ret = B_FALSE;

	if (spa_open(name, &spa, FTAG) == 0) {
		if (spa->spa_bootfs) {
			objset_t *os;

			if (dmu_objset_open(name, DMU_OST_ZFS,
			    DS_MODE_USER | DS_MODE_READONLY, &os) == 0) {
				ret = (dmu_objset_id(os) == spa->spa_bootfs);
				dmu_objset_close(os);
			}
		}
		spa_close(spa, FTAG);
	}
	return (ret);
}

/*
 * zfs_earlier_version
 *
 *	Return non-zero if the spa version is less than requested version.
 */
static int
zfs_earlier_version(const char *name, int version)
{
	spa_t *spa;

	if (spa_open(name, &spa, FTAG) == 0) {
		if (spa_version(spa) < version) {
			spa_close(spa, FTAG);
			return (1);
		}
		spa_close(spa, FTAG);
	}
	return (0);
}

/*
 * zpl_earlier_version
 *
 * Return TRUE if the ZPL version is less than requested version.
 */
static boolean_t
zpl_earlier_version(const char *name, int version)
{
	objset_t *os;
	boolean_t rc = B_TRUE;

	if (dmu_objset_open(name, DMU_OST_ANY,
	    DS_MODE_USER | DS_MODE_READONLY, &os) == 0) {
		uint64_t zplversion;

		if (zfs_get_zplprop(os, ZFS_PROP_VERSION, &zplversion) == 0)
			rc = zplversion < version;
		dmu_objset_close(os);
	}
	return (rc);
}

static void
zfs_log_history(zfs_cmd_t *zc)
{
	spa_t *spa;
	char *buf;

	if ((buf = history_str_get(zc)) == NULL)
		return;

	if (spa_open(zc->zc_name, &spa, FTAG) == 0) {
		if (spa_version(spa) >= SPA_VERSION_ZPOOL_HISTORY)
			(void) spa_history_log(spa, buf, LOG_CMD_NORMAL);
		spa_close(spa, FTAG);
	}
	history_str_free(buf);
}

/*
 * Policy for top-level read operations (list pools).  Requires no privileges,
 * and can be used in the local zone, as there is no associated dataset.
 */
/* ARGSUSED */
static int
zfs_secpolicy_none(zfs_cmd_t *zc, cred_t *cr)
{
	return (0);
}

/*
 * Policy for dataset read operations (list children, get statistics).  Requires
 * no privileges, but must be visible in the local zone.
 */
/* ARGSUSED */
static int
zfs_secpolicy_read(zfs_cmd_t *zc, cred_t *cr)
{
	if (INGLOBALZONE(curthread) ||
	    zone_dataset_visible(zc->zc_name, NULL))
		return (0);

	return (ENOENT);
}

static int
zfs_dozonecheck(const char *dataset, cred_t *cr)
{
	uint64_t zoned;
	int writable = 1;

	/*
	 * The dataset must be visible by this zone -- check this first
	 * so they don't see EPERM on something they shouldn't know about.
	 */
	if (!INGLOBALZONE(curthread) &&
	    !zone_dataset_visible(dataset, &writable))
		return (ENOENT);

	if (dsl_prop_get_integer(dataset, "jailed", &zoned, NULL))
		return (ENOENT);

	if (INGLOBALZONE(curthread)) {
		/*
		 * If the fs is zoned, only root can access it from the
		 * global zone.
		 */
		if (secpolicy_zfs(cr) && zoned)
			return (EPERM);
	} else {
		/*
		 * If we are in a local zone, the 'zoned' property must be set.
		 */
		if (!zoned)
			return (EPERM);

		/* must be writable by this zone */
		if (!writable)
			return (EPERM);
	}
	return (0);
}

int
zfs_secpolicy_write_perms(const char *name, const char *perm, cred_t *cr)
{
	int error;

	error = zfs_dozonecheck(name, cr);
	if (error == 0) {
		error = secpolicy_zfs(cr);
		if (error)
			error = dsl_deleg_access(name, perm, cr);
	}
	return (error);
}

static int
zfs_secpolicy_setprop(const char *name, zfs_prop_t prop, cred_t *cr)
{
	/*
	 * Check permissions for special properties.
	 */
	switch (prop) {
	case ZFS_PROP_ZONED:
		/*
		 * Disallow setting of 'zoned' from within a local zone.
		 */
		if (!INGLOBALZONE(curthread))
			return (EPERM);
		break;

	case ZFS_PROP_QUOTA:
		if (!INGLOBALZONE(curthread)) {
			uint64_t zoned;
			char setpoint[MAXNAMELEN];
			/*
			 * Unprivileged users are allowed to modify the
			 * quota on things *under* (ie. contained by)
			 * the thing they own.
			 */
			if (dsl_prop_get_integer(name, "zoned", &zoned,
			    setpoint))
				return (EPERM);
			if (!zoned || strlen(name) <= strlen(setpoint))
				return (EPERM);
		}
		break;
	}

	return (zfs_secpolicy_write_perms(name, zfs_prop_to_name(prop), cr));
}

int
zfs_secpolicy_fsacl(zfs_cmd_t *zc, cred_t *cr)
{
	int error;

	error = zfs_dozonecheck(zc->zc_name, cr);
	if (error)
		return (error);

	/*
	 * permission to set permissions will be evaluated later in
	 * dsl_deleg_can_allow()
	 */
	return (0);
}

int
zfs_secpolicy_rollback(zfs_cmd_t *zc, cred_t *cr)
{
	int error;
	error = zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_ROLLBACK, cr);
	if (error == 0)
		error = zfs_secpolicy_write_perms(zc->zc_name,
		    ZFS_DELEG_PERM_MOUNT, cr);
	return (error);
}

int
zfs_secpolicy_send(zfs_cmd_t *zc, cred_t *cr)
{
	return (zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_SEND, cr));
}

static int
zfs_secpolicy_deleg_share(zfs_cmd_t *zc, cred_t *cr)
{
	vnode_t *vp;
	int error;

	if ((error = lookupname(zc->zc_value, UIO_SYSSPACE,
	    NO_FOLLOW, NULL, &vp)) != 0)
		return (error);

	/* Now make sure mntpnt and dataset are ZFS */

	if (strcmp(vp->v_vfsp->mnt_stat.f_fstypename, "zfs") != 0 ||
	    (strcmp((char *)refstr_value(vp->v_vfsp->vfs_resource),
	    zc->zc_name) != 0)) {
		VN_RELE(vp);
		return (EPERM);
	}

	VN_RELE(vp);
	return (dsl_deleg_access(zc->zc_name,
	    ZFS_DELEG_PERM_SHARE, cr));
}

int
zfs_secpolicy_share(zfs_cmd_t *zc, cred_t *cr)
{
	if (!INGLOBALZONE(curthread))
		return (EPERM);

	if (secpolicy_nfs(cr) == 0) {
		return (0);
	} else {
		return (zfs_secpolicy_deleg_share(zc, cr));
	}
}

int
zfs_secpolicy_smb_acl(zfs_cmd_t *zc, cred_t *cr)
{
	if (!INGLOBALZONE(curthread))
		return (EPERM);

	if (secpolicy_smb(cr) == 0) {
		return (0);
	} else {
		return (zfs_secpolicy_deleg_share(zc, cr));
	}
}

static int
zfs_get_parent(const char *datasetname, char *parent, int parentsize)
{
	char *cp;

	/*
	 * Remove the @bla or /bla from the end of the name to get the parent.
	 */
	(void) strncpy(parent, datasetname, parentsize);
	cp = strrchr(parent, '@');
	if (cp != NULL) {
		cp[0] = '\0';
	} else {
		cp = strrchr(parent, '/');
		if (cp == NULL)
			return (ENOENT);
		cp[0] = '\0';
	}

	return (0);
}

int
zfs_secpolicy_destroy_perms(const char *name, cred_t *cr)
{
	int error;

	if ((error = zfs_secpolicy_write_perms(name,
	    ZFS_DELEG_PERM_MOUNT, cr)) != 0)
		return (error);

	return (zfs_secpolicy_write_perms(name, ZFS_DELEG_PERM_DESTROY, cr));
}

static int
zfs_secpolicy_destroy(zfs_cmd_t *zc, cred_t *cr)
{
	return (zfs_secpolicy_destroy_perms(zc->zc_name, cr));
}

/*
 * Must have sys_config privilege to check the iscsi permission
 */
/* ARGSUSED */
static int
zfs_secpolicy_iscsi(zfs_cmd_t *zc, cred_t *cr)
{
	return (secpolicy_zfs(cr));
}

int
zfs_secpolicy_rename_perms(const char *from, const char *to, cred_t *cr)
{
	char 	parentname[MAXNAMELEN];
	int	error;

	if ((error = zfs_secpolicy_write_perms(from,
	    ZFS_DELEG_PERM_RENAME, cr)) != 0)
		return (error);

	if ((error = zfs_secpolicy_write_perms(from,
	    ZFS_DELEG_PERM_MOUNT, cr)) != 0)
		return (error);

	if ((error = zfs_get_parent(to, parentname,
	    sizeof (parentname))) != 0)
		return (error);

	if ((error = zfs_secpolicy_write_perms(parentname,
	    ZFS_DELEG_PERM_CREATE, cr)) != 0)
		return (error);

	if ((error = zfs_secpolicy_write_perms(parentname,
	    ZFS_DELEG_PERM_MOUNT, cr)) != 0)
		return (error);

	return (error);
}

static int
zfs_secpolicy_rename(zfs_cmd_t *zc, cred_t *cr)
{
	return (zfs_secpolicy_rename_perms(zc->zc_name, zc->zc_value, cr));
}

static int
zfs_secpolicy_promote(zfs_cmd_t *zc, cred_t *cr)
{
	char 	parentname[MAXNAMELEN];
	objset_t *clone;
	int error;

	error = zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_PROMOTE, cr);
	if (error)
		return (error);

	error = dmu_objset_open(zc->zc_name, DMU_OST_ANY,
	    DS_MODE_USER | DS_MODE_READONLY, &clone);

	if (error == 0) {
		dsl_dataset_t *pclone = NULL;
		dsl_dir_t *dd;
		dd = clone->os->os_dsl_dataset->ds_dir;

		rw_enter(&dd->dd_pool->dp_config_rwlock, RW_READER);
		error = dsl_dataset_hold_obj(dd->dd_pool,
		    dd->dd_phys->dd_origin_obj, FTAG, &pclone);
		rw_exit(&dd->dd_pool->dp_config_rwlock);
		if (error) {
			dmu_objset_close(clone);
			return (error);
		}

		error = zfs_secpolicy_write_perms(zc->zc_name,
		    ZFS_DELEG_PERM_MOUNT, cr);

		dsl_dataset_name(pclone, parentname);
		dmu_objset_close(clone);
		dsl_dataset_rele(pclone, FTAG);
		if (error == 0)
			error = zfs_secpolicy_write_perms(parentname,
			    ZFS_DELEG_PERM_PROMOTE, cr);
	}
	return (error);
}

static int
zfs_secpolicy_receive(zfs_cmd_t *zc, cred_t *cr)
{
	int error;

	if ((error = zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_RECEIVE, cr)) != 0)
		return (error);

	if ((error = zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_MOUNT, cr)) != 0)
		return (error);

	return (zfs_secpolicy_write_perms(zc->zc_name,
	    ZFS_DELEG_PERM_CREATE, cr));
}

int
zfs_secpolicy_snapshot_perms(const char *name, cred_t *cr)
{
	int error;

	if ((error = zfs_secpolicy_write_perms(name,
	    ZFS_DELEG_PERM_SNAPSHOT, cr)) != 0)
		return (error);

	error = zfs_secpolicy_write_perms(name,
	    ZFS_DELEG_PERM_MOUNT, cr);

	return (error);
}

static int
zfs_secpolicy_snapshot(zfs_cmd_t *zc, cred_t *cr)
{

	return (zfs_secpolicy_snapshot_perms(zc->zc_name, cr));
}

static int
zfs_secpolicy_create(zfs_cmd_t *zc, cred_t *cr)
{
	char 	parentname[MAXNAMELEN];
	int 	error;

	if ((error = zfs_get_parent(zc->zc_name, parentname,
	    sizeof (parentname))) != 0)
		return (error);

	if (zc->zc_value[0] != '\0') {
		if ((error = zfs_secpolicy_write_perms(zc->zc_value,
		    ZFS_DELEG_PERM_CLONE, cr)) != 0)
			return (error);
	}

	if ((error = zfs_secpolicy_write_perms(parentname,
	    ZFS_DELEG_PERM_CREATE, cr)) != 0)
		return (error);

	error = zfs_secpolicy_write_perms(parentname,
	    ZFS_DELEG_PERM_MOUNT, cr);

	return (error);
}

static int
zfs_secpolicy_umount(zfs_cmd_t *zc, cred_t *cr)
{
	int error;

	error = secpolicy_fs_unmount(cr, NULL);
	if (error) {
		error = dsl_deleg_access(zc->zc_name, ZFS_DELEG_PERM_MOUNT, cr);
	}
	return (error);
}

/*
 * Policy for pool operations - create/destroy pools, add vdevs, etc.  Requires
 * SYS_CONFIG privilege, which is not available in a local zone.
 */
/* ARGSUSED */
static int
zfs_secpolicy_config(zfs_cmd_t *zc, cred_t *cr)
{
	if (secpolicy_sys_config(cr, B_FALSE) != 0)
		return (EPERM);

	return (0);
}

/*
 * Just like zfs_secpolicy_config, except that we will check for
 * mount permission on the dataset for permission to create/remove
 * the minor nodes.
 */
static int
zfs_secpolicy_minor(zfs_cmd_t *zc, cred_t *cr)
{
	if (secpolicy_sys_config(cr, B_FALSE) != 0) {
		return (dsl_deleg_access(zc->zc_name,
		    ZFS_DELEG_PERM_MOUNT, cr));
	}

	return (0);
}

/*
 * Policy for fault injection.  Requires all privileges.
 */
/* ARGSUSED */
static int
zfs_secpolicy_inject(zfs_cmd_t *zc, cred_t *cr)
{
	return (secpolicy_zinject(cr));
}

static int
zfs_secpolicy_inherit(zfs_cmd_t *zc, cred_t *cr)
{
	zfs_prop_t prop = zfs_name_to_prop(zc->zc_value);

	if (prop == ZPROP_INVAL) {
		if (!zfs_prop_user(zc->zc_value))
			return (EINVAL);
		return (zfs_secpolicy_write_perms(zc->zc_name,
		    ZFS_DELEG_PERM_USERPROP, cr));
	} else {
		if (!zfs_prop_inheritable(prop))
			return (EINVAL);
		return (zfs_secpolicy_setprop(zc->zc_name, prop, cr));
	}
}

/*
 * Policy for dataset backup operations (sendbackup).
 * Requires SYS_MOUNT privilege, and must be writable in the local zone.
 */
static int
zfs_secpolicy_operator(const char *dataset, cred_t *cr)
{
	int writable = 1;

	if (!INGLOBALZONE(curthread) && !zone_dataset_visible(dataset, &writable))
		return (ENOENT);
	if (secpolicy_zfs(cr) != 0 && !groupmember(GID_OPERATOR, cr))
		return (EPERM);
	return (0);
}

static int
zfs_secpolicy_userspace_one(zfs_cmd_t *zc, cred_t *cr)
{
	int err = zfs_secpolicy_read(zc, cr);
	if (err)
		return (err);

	if (zc->zc_objset_type >= ZFS_NUM_USERQUOTA_PROPS)
		return (EINVAL);

	if (zc->zc_value[0] == 0) {
		/*
		 * They are asking about a posix uid/gid.  If it's
		 * themself, allow it.
		 */
		if (zc->zc_objset_type == ZFS_PROP_USERUSED ||
		    zc->zc_objset_type == ZFS_PROP_USERQUOTA) {
			if (zc->zc_guid == crgetuid(cr))
				return (0);
		} else {
			if (groupmember(zc->zc_guid, cr))
				return (0);
		}
	}

	return (zfs_secpolicy_write_perms(zc->zc_name,
	    userquota_perms[zc->zc_objset_type], cr));
}

static int
zfs_secpolicy_userspace_many(zfs_cmd_t *zc, cred_t *cr)
{
	int err = zfs_secpolicy_read(zc, cr);
	if (err)
		return (err);

	if (zc->zc_objset_type >= ZFS_NUM_USERQUOTA_PROPS)
		return (EINVAL);

	return (zfs_secpolicy_write_perms(zc->zc_name,
	    userquota_perms[zc->zc_objset_type], cr));
}

static int
zfs_secpolicy_userspace_upgrade(zfs_cmd_t *zc, cred_t *cr)
{
	return (zfs_secpolicy_setprop(zc->zc_name, ZFS_PROP_VERSION, cr));
}

/*
 * Returns the nvlist as specified by the user in the zfs_cmd_t.
 */
static int
get_nvlist(uint64_t nvl, uint64_t size, nvlist_t **nvp)
{
	char *packed;
	int error;
	nvlist_t *list = NULL;

	/*
	 * Read in and unpack the user-supplied nvlist.
	 */
	if (size == 0)
		return (EINVAL);

	packed = kmem_alloc(size, KM_SLEEP);

	if ((error = xcopyin((void *)(uintptr_t)nvl, packed, size)) != 0) {
		kmem_free(packed, size);
		return (error);
	}

	if ((error = nvlist_unpack(packed, size, &list, 0)) != 0) {
		kmem_free(packed, size);
		return (error);
	}

	kmem_free(packed, size);

	*nvp = list;
	return (0);
}

static int
put_nvlist(zfs_cmd_t *zc, nvlist_t *nvl)
{
	char *packed = NULL;
	size_t size;
	int error;

	VERIFY(nvlist_size(nvl, &size, NV_ENCODE_NATIVE) == 0);

	if (size > zc->zc_nvlist_dst_size) {
		/*
		 * Solaris returns ENOMEM here, because even if an error is
		 * returned from an ioctl(2), new zc_nvlist_dst_size will be
		 * passed to the userland. This is not the case for FreeBSD.
		 * We need to return 0, so the kernel will copy the
		 * zc_nvlist_dst_size back and the userland can discover that a
		 * bigger buffer is needed.
		 */
		error = 0;
	} else {
		packed = kmem_alloc(size, KM_SLEEP);
		VERIFY(nvlist_pack(nvl, &packed, &size, NV_ENCODE_NATIVE,
		    KM_SLEEP) == 0);
		error = xcopyout(packed, (void *)(uintptr_t)zc->zc_nvlist_dst,
		    size);
		kmem_free(packed, size);
	}

	zc->zc_nvlist_dst_size = size;
	return (error);
}

static int
getzfsvfs(const char *dsname, zfsvfs_t **zvp)
{
	objset_t *os;
	int error;

	error = dmu_objset_open(dsname, DMU_OST_ZFS,
	    DS_MODE_USER | DS_MODE_READONLY, &os);
	if (error)
		return (error);

	mutex_enter(&os->os->os_user_ptr_lock);
	*zvp = dmu_objset_get_user(os);
	if (*zvp) {
		VFS_HOLD((*zvp)->z_vfs);
	} else {
		error = ESRCH;
	}
	mutex_exit(&os->os->os_user_ptr_lock);
	dmu_objset_close(os);
	return (error);
}

/*
 * Find a zfsvfs_t for a mounted filesystem, or create our own, in which
 * case its z_vfs will be NULL, and it will be opened as the owner.
 */
static int
zfsvfs_hold(const char *name, boolean_t readonly, void *tag, zfsvfs_t **zvp)
{
	int error = 0;
	int mode = DS_MODE_OWNER | (readonly ? DS_MODE_READONLY : 0);

	if (getzfsvfs(name, zvp) != 0)
		error = zfsvfs_create(name, mode, zvp);
	if (error == 0) {
		rrw_enter(&(*zvp)->z_teardown_lock, RW_READER, tag);
		if ((*zvp)->z_unmounted) {
			/*
			 * XXX we could probably try again, since the unmounting
			 * thread should be just about to disassociate the
			 * objset from the zfsvfs.
			 */
			rrw_exit(&(*zvp)->z_teardown_lock, tag);
			return (EBUSY);
		}
	}
	return (error);
}

static void
zfsvfs_rele(zfsvfs_t *zfsvfs, void *tag)
{
	rrw_exit(&zfsvfs->z_teardown_lock, tag);

	if (zfsvfs->z_vfs) {
		VFS_RELE(zfsvfs->z_vfs);
	} else {
		dmu_objset_close(zfsvfs->z_os);
		zfsvfs_free(zfsvfs);
	}
}

static int
zfs_ioc_pool_create(zfs_cmd_t *zc)
{
	int error;
	nvlist_t *config, *props = NULL;
	nvlist_t *rootprops = NULL;
	nvlist_t *zplprops = NULL;
	char *buf;

	if (error = get_nvlist(zc->zc_nvlist_conf, zc->zc_nvlist_conf_size,
	    &config))
		return (error);

	if (zc->zc_nvlist_src_size != 0 && (error =
	    get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size, &props))) {
		nvlist_free(config);
		return (error);
	}

	if (props) {
		nvlist_t *nvl = NULL;
		uint64_t version = SPA_VERSION;

		(void) nvlist_lookup_uint64(props,
		    zpool_prop_to_name(ZPOOL_PROP_VERSION), &version);
		if (version < SPA_VERSION_INITIAL || version > SPA_VERSION) {
			error = EINVAL;
			goto pool_props_bad;
		}
		(void) nvlist_lookup_nvlist(props, ZPOOL_ROOTFS_PROPS, &nvl);
		if (nvl) {
			error = nvlist_dup(nvl, &rootprops, KM_SLEEP);
			if (error != 0) {
				nvlist_free(config);
				nvlist_free(props);
				return (error);
			}
			(void) nvlist_remove_all(props, ZPOOL_ROOTFS_PROPS);
		}
		VERIFY(nvlist_alloc(&zplprops, NV_UNIQUE_NAME, KM_SLEEP) == 0);
		error = zfs_fill_zplprops_root(version, rootprops,
		    zplprops, NULL);
		if (error)
			goto pool_props_bad;
	}

	buf = history_str_get(zc);

	error = spa_create(zc->zc_name, config, props, buf, zplprops);

	/*
	 * Set the remaining root properties
	 */
	if (!error &&
	    (error = zfs_set_prop_nvlist(zc->zc_name, rootprops)) != 0)
		(void) spa_destroy(zc->zc_name);

	if (buf != NULL)
		history_str_free(buf);

pool_props_bad:
	nvlist_free(rootprops);
	nvlist_free(zplprops);
	nvlist_free(config);
	nvlist_free(props);

	return (error);
}

static int
zfs_ioc_pool_destroy(zfs_cmd_t *zc)
{
	int error;
	zfs_log_history(zc);
	error = spa_destroy(zc->zc_name);
	return (error);
}

static int
zfs_ioc_pool_import(zfs_cmd_t *zc)
{
	int error;
	nvlist_t *config, *props = NULL;
	uint64_t guid;

	if ((error = get_nvlist(zc->zc_nvlist_conf, zc->zc_nvlist_conf_size,
	    &config)) != 0)
		return (error);

	if (zc->zc_nvlist_src_size != 0 && (error =
	    get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size, &props))) {
		nvlist_free(config);
		return (error);
	}

	if (nvlist_lookup_uint64(config, ZPOOL_CONFIG_POOL_GUID, &guid) != 0 ||
	    guid != zc->zc_guid)
		error = EINVAL;
	else if (zc->zc_cookie)
		error = spa_import_verbatim(zc->zc_name, config,
		    props);
	else
		error = spa_import(zc->zc_name, config, props);

	nvlist_free(config);

	if (props)
		nvlist_free(props);

	return (error);
}

static int
zfs_ioc_pool_export(zfs_cmd_t *zc)
{
	int error;
	boolean_t force = (boolean_t)zc->zc_cookie;
	boolean_t hardforce = (boolean_t)zc->zc_guid;

	zfs_log_history(zc);
	error = spa_export(zc->zc_name, NULL, force, hardforce);
	return (error);
}

static int
zfs_ioc_pool_configs(zfs_cmd_t *zc)
{
	nvlist_t *configs;
	int error;

	if ((configs = spa_all_configs(&zc->zc_cookie)) == NULL)
		return (EEXIST);

	error = put_nvlist(zc, configs);

	nvlist_free(configs);

	return (error);
}

static int
zfs_ioc_pool_stats(zfs_cmd_t *zc)
{
	nvlist_t *config;
	int error;
	int ret = 0;

	error = spa_get_stats(zc->zc_name, &config, zc->zc_value,
	    sizeof (zc->zc_value));

	if (config != NULL) {
		ret = put_nvlist(zc, config);
		nvlist_free(config);

		/*
		 * The config may be present even if 'error' is non-zero.
		 * In this case we return success, and preserve the real errno
		 * in 'zc_cookie'.
		 */
		zc->zc_cookie = error;
	} else {
		ret = error;
	}

	return (ret);
}

/*
 * Try to import the given pool, returning pool stats as appropriate so that
 * user land knows which devices are available and overall pool health.
 */
static int
zfs_ioc_pool_tryimport(zfs_cmd_t *zc)
{
	nvlist_t *tryconfig, *config;
	int error;

	if ((error = get_nvlist(zc->zc_nvlist_conf, zc->zc_nvlist_conf_size,
	    &tryconfig)) != 0)
		return (error);

	config = spa_tryimport(tryconfig);

	nvlist_free(tryconfig);

	if (config == NULL)
		return (EINVAL);

	error = put_nvlist(zc, config);
	nvlist_free(config);

	return (error);
}

static int
zfs_ioc_pool_scrub(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	error = spa_scrub(spa, zc->zc_cookie);

	spa_close(spa, FTAG);

	return (error);
}

static int
zfs_ioc_pool_freeze(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;

	error = spa_open(zc->zc_name, &spa, FTAG);
	if (error == 0) {
		spa_freeze(spa);
		spa_close(spa, FTAG);
	}
	return (error);
}

static int
zfs_ioc_pool_upgrade(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	if (zc->zc_cookie < spa_version(spa) || zc->zc_cookie > SPA_VERSION) {
		spa_close(spa, FTAG);
		return (EINVAL);
	}

	spa_upgrade(spa, zc->zc_cookie);
	spa_close(spa, FTAG);

	return (error);
}

static int
zfs_ioc_pool_get_history(zfs_cmd_t *zc)
{
	spa_t *spa;
	char *hist_buf;
	uint64_t size;
	int error;

	if ((size = zc->zc_history_len) == 0)
		return (EINVAL);

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	if (spa_version(spa) < SPA_VERSION_ZPOOL_HISTORY) {
		spa_close(spa, FTAG);
		return (ENOTSUP);
	}

	hist_buf = kmem_alloc(size, KM_SLEEP);
	if ((error = spa_history_get(spa, &zc->zc_history_offset,
	    &zc->zc_history_len, hist_buf)) == 0) {
		error = xcopyout(hist_buf,
		    (char *)(uintptr_t)zc->zc_history,
		    zc->zc_history_len);
	}

	spa_close(spa, FTAG);
	kmem_free(hist_buf, size);
	return (error);
}

static int
zfs_ioc_dsobj_to_dsname(zfs_cmd_t *zc)
{
	int error;

	if (error = dsl_dsobj_to_dsname(zc->zc_name, zc->zc_obj, zc->zc_value))
		return (error);

	return (0);
}

static int
zfs_ioc_obj_to_path(zfs_cmd_t *zc)
{
	objset_t *osp;
	int error;

	if ((error = dmu_objset_open(zc->zc_name, DMU_OST_ZFS,
	    DS_MODE_USER | DS_MODE_READONLY, &osp)) != 0)
		return (error);
	error = zfs_obj_to_path(osp, zc->zc_obj, zc->zc_value,
	    sizeof (zc->zc_value));
	dmu_objset_close(osp);

	return (error);
}

static int
zfs_ioc_vdev_add(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;
	nvlist_t *config, **l2cache, **spares;
	uint_t nl2cache = 0, nspares = 0;

	error = spa_open(zc->zc_name, &spa, FTAG);
	if (error != 0)
		return (error);

	error = get_nvlist(zc->zc_nvlist_conf, zc->zc_nvlist_conf_size,
	    &config);
	(void) nvlist_lookup_nvlist_array(config, ZPOOL_CONFIG_L2CACHE,
	    &l2cache, &nl2cache);

	(void) nvlist_lookup_nvlist_array(config, ZPOOL_CONFIG_SPARES,
	    &spares, &nspares);

	/*
	 * A root pool with concatenated devices is not supported.
	 * Thus, can not add a device to a root pool.
	 *
	 * Intent log device can not be added to a rootpool because
	 * during mountroot, zil is replayed, a seperated log device
	 * can not be accessed during the mountroot time.
	 *
	 * l2cache and spare devices are ok to be added to a rootpool.
	 */
	if (spa->spa_bootfs != 0 && nl2cache == 0 && nspares == 0) {
		spa_close(spa, FTAG);
		return (EDOM);
	}

	if (error == 0) {
		error = spa_vdev_add(spa, config);
		nvlist_free(config);
	}
	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_remove(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;

	error = spa_open(zc->zc_name, &spa, FTAG);
	if (error != 0)
		return (error);
	error = spa_vdev_remove(spa, zc->zc_guid, B_FALSE);
	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_set_state(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;
	vdev_state_t newstate = VDEV_STATE_UNKNOWN;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);
	switch (zc->zc_cookie) {
	case VDEV_STATE_ONLINE:
		error = vdev_online(spa, zc->zc_guid, zc->zc_obj, &newstate);
		break;

	case VDEV_STATE_OFFLINE:
		error = vdev_offline(spa, zc->zc_guid, zc->zc_obj);
		break;

	case VDEV_STATE_FAULTED:
		error = vdev_fault(spa, zc->zc_guid);
		break;

	case VDEV_STATE_DEGRADED:
		error = vdev_degrade(spa, zc->zc_guid);
		break;

	default:
		error = EINVAL;
	}
	zc->zc_cookie = newstate;
	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_attach(zfs_cmd_t *zc)
{
	spa_t *spa;
	int replacing = zc->zc_cookie;
	nvlist_t *config;
	int error;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	if ((error = get_nvlist(zc->zc_nvlist_conf, zc->zc_nvlist_conf_size,
	    &config)) == 0) {
		error = spa_vdev_attach(spa, zc->zc_guid, config, replacing);
		nvlist_free(config);
	}

	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_detach(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	error = spa_vdev_detach(spa, zc->zc_guid, 0, B_FALSE);

	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_setpath(zfs_cmd_t *zc)
{
	spa_t *spa;
	char *path = zc->zc_value;
	uint64_t guid = zc->zc_guid;
	int error;

	error = spa_open(zc->zc_name, &spa, FTAG);
	if (error != 0)
		return (error);

	error = spa_vdev_setpath(spa, guid, path);
	spa_close(spa, FTAG);
	return (error);
}

static int
zfs_ioc_vdev_setfru(zfs_cmd_t *zc)
{
	spa_t *spa;
	char *fru = zc->zc_value;
	uint64_t guid = zc->zc_guid;
	int error;

	error = spa_open(zc->zc_name, &spa, FTAG);
	if (error != 0)
		return (error);

	error = spa_vdev_setfru(spa, guid, fru);
	spa_close(spa, FTAG);
	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_nvlist_dst_size	size of buffer for property nvlist
 *
 * outputs:
 * zc_objset_stats	stats
 * zc_nvlist_dst	property nvlist
 * zc_nvlist_dst_size	size of property nvlist
 */
static int
zfs_ioc_objset_stats(zfs_cmd_t *zc)
{
	objset_t *os = NULL;
	int error;
	nvlist_t *nv;

	if (error = dmu_objset_open(zc->zc_name,
	    DMU_OST_ANY, DS_MODE_USER | DS_MODE_READONLY, &os))
		return (error);

	dmu_objset_fast_stat(os, &zc->zc_objset_stats);

	if (zc->zc_nvlist_dst != 0 &&
	    (error = dsl_prop_get_all(os, &nv, FALSE)) == 0) {
		dmu_objset_stats(os, nv);
		/*
		 * NB: zvol_get_stats() will read the objset contents,
		 * which we aren't supposed to do with a
		 * DS_MODE_USER hold, because it could be
		 * inconsistent.  So this is a bit of a workaround...
		 */
		if (!zc->zc_objset_stats.dds_inconsistent) {
			if (dmu_objset_type(os) == DMU_OST_ZVOL)
				VERIFY(zvol_get_stats(os, nv) == 0);
		}
		error = put_nvlist(zc, nv);
		nvlist_free(nv);
	}

	dmu_objset_close(os);
	if (error == ENOMEM)
		error = 0;
	return (error);
}

static int
nvl_add_zplprop(objset_t *os, nvlist_t *props, zfs_prop_t prop)
{
	uint64_t value;
	int error;

	/*
	 * zfs_get_zplprop() will either find a value or give us
	 * the default value (if there is one).
	 */
	if ((error = zfs_get_zplprop(os, prop, &value)) != 0)
		return (error);
	VERIFY(nvlist_add_uint64(props, zfs_prop_to_name(prop), value) == 0);
	return (0);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_nvlist_dst_size	size of buffer for zpl property nvlist
 *
 * outputs:
 * zc_nvlist_dst	zpl property nvlist
 * zc_nvlist_dst_size	size of zpl property nvlist
 */
static int
zfs_ioc_objset_zplprops(zfs_cmd_t *zc)
{
	objset_t *os;
	int err;

	if (err = dmu_objset_open(zc->zc_name,
	    DMU_OST_ANY, DS_MODE_USER | DS_MODE_READONLY, &os))
		return (err);

	dmu_objset_fast_stat(os, &zc->zc_objset_stats);

	/*
	 * NB: nvl_add_zplprop() will read the objset contents,
	 * which we aren't supposed to do with a DS_MODE_USER
	 * hold, because it could be inconsistent.
	 */
	if (zc->zc_nvlist_dst != 0 &&
	    !zc->zc_objset_stats.dds_inconsistent &&
	    dmu_objset_type(os) == DMU_OST_ZFS) {
		nvlist_t *nv;

		VERIFY(nvlist_alloc(&nv, NV_UNIQUE_NAME, KM_SLEEP) == 0);
		if ((err = nvl_add_zplprop(os, nv, ZFS_PROP_VERSION)) == 0 &&
		    (err = nvl_add_zplprop(os, nv, ZFS_PROP_NORMALIZE)) == 0 &&
		    (err = nvl_add_zplprop(os, nv, ZFS_PROP_UTF8ONLY)) == 0 &&
		    (err = nvl_add_zplprop(os, nv, ZFS_PROP_CASE)) == 0)
			err = put_nvlist(zc, nv);
		nvlist_free(nv);
	} else {
		err = ENOENT;
	}
	dmu_objset_close(os);
	return (err);
}

static boolean_t
dataset_name_hidden(const char *name)
{
	/*
	 * Skip over datasets that are not visible in this zone,
	 * internal datasets (which have a $ in their name), and
	 * temporary datasets (which have a % in their name).
	 */
	if (strchr(name, '$') != NULL)
		return (B_TRUE);
	if (strchr(name, '%') != NULL)
		return (B_TRUE);
	if (!INGLOBALZONE(curthread) && !zone_dataset_visible(name, NULL))
		return (B_TRUE);
	return (B_FALSE);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_cookie		zap cursor
 * zc_nvlist_dst_size	size of buffer for property nvlist
 *
 * outputs:
 * zc_name		name of next filesystem
 * zc_cookie		zap cursor
 * zc_objset_stats	stats
 * zc_nvlist_dst	property nvlist
 * zc_nvlist_dst_size	size of property nvlist
 */
static int
zfs_ioc_dataset_list_next(zfs_cmd_t *zc)
{
	objset_t *os;
	int error;
	char *p;

	if (error = dmu_objset_open(zc->zc_name,
	    DMU_OST_ANY, DS_MODE_USER | DS_MODE_READONLY, &os)) {
		if (error == ENOENT)
			error = ESRCH;
		return (error);
	}

	p = strrchr(zc->zc_name, '/');
	if (p == NULL || p[1] != '\0')
		(void) strlcat(zc->zc_name, "/", sizeof (zc->zc_name));
	p = zc->zc_name + strlen(zc->zc_name);

	/*
	 * Pre-fetch the datasets.  dmu_objset_prefetch() always returns 0
	 * but is not declared void because its called by dmu_objset_find().
	 */
	if (zc->zc_cookie == 0) {
		uint64_t cookie = 0;
		int len = sizeof (zc->zc_name) - (p - zc->zc_name);

		while (dmu_dir_list_next(os, len, p, NULL, &cookie) == 0)
			(void) dmu_objset_prefetch(p, NULL);
	}

	do {
		error = dmu_dir_list_next(os,
		    sizeof (zc->zc_name) - (p - zc->zc_name), p,
		    NULL, &zc->zc_cookie);
		if (error == ENOENT)
			error = ESRCH;
	} while (error == 0 && dataset_name_hidden(zc->zc_name));
	dmu_objset_close(os);

	if (error == 0)
		error = zfs_ioc_objset_stats(zc); /* fill in the stats */

	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_cookie		zap cursor
 * zc_nvlist_dst_size	size of buffer for property nvlist
 *
 * outputs:
 * zc_name		name of next snapshot
 * zc_objset_stats	stats
 * zc_nvlist_dst	property nvlist
 * zc_nvlist_dst_size	size of property nvlist
 */
static int
zfs_ioc_snapshot_list_next(zfs_cmd_t *zc)
{
	objset_t *os;
	int error;

	error = dmu_objset_open(zc->zc_name,
	    DMU_OST_ANY, DS_MODE_USER | DS_MODE_READONLY, &os);
	if (error)
		return (error == ENOENT ? ESRCH : error);

	if (zc->zc_cookie == 0) {
		(void) dmu_objset_find(zc->zc_name, dmu_objset_prefetch,
		    NULL, DS_FIND_SNAPSHOTS);
	}
	/*
	 * A dataset name of maximum length cannot have any snapshots,
	 * so exit immediately.
	 */
	if (strlcat(zc->zc_name, "@", sizeof (zc->zc_name)) >= MAXNAMELEN) {
		dmu_objset_close(os);
		return (ESRCH);
	}

	error = dmu_snapshot_list_next(os,
	    sizeof (zc->zc_name) - strlen(zc->zc_name),
	    zc->zc_name + strlen(zc->zc_name), NULL, &zc->zc_cookie, NULL);
	dmu_objset_close(os);
	if (error == 0)
		error = zfs_ioc_objset_stats(zc); /* fill in the stats */
	else if (error == ENOENT)
		error = ESRCH;

	/* if we failed, undo the @ that we tacked on to zc_name */
	if (error)
		*strchr(zc->zc_name, '@') = '\0';
	return (error);
}

int
zfs_set_prop_nvlist(const char *name, nvlist_t *nvl)
{
	nvpair_t *elem;
	int error = 0;
	uint64_t intval;
	char *strval;
	nvlist_t *genericnvl;
	boolean_t issnap = (strchr(name, '@') != NULL);

	/*
	 * First validate permission to set all of the properties
	 */
	VERIFY(nvlist_alloc(&genericnvl, NV_UNIQUE_NAME, KM_SLEEP) == 0);
	elem = NULL;
	while ((elem = nvlist_next_nvpair(nvl, elem)) != NULL) {
		const char *propname = nvpair_name(elem);
		zfs_prop_t prop = zfs_name_to_prop(propname);

		if (prop == ZPROP_INVAL) {
			/*
			 * If this is a user-defined property, it must be a
			 * string, and there is no further validation to do.
			 */
			if (zfs_prop_user(propname) &&
			    nvpair_type(elem) == DATA_TYPE_STRING) {
				if (error = zfs_secpolicy_write_perms(name,
				    ZFS_DELEG_PERM_USERPROP, CRED()))
					return (error);
				continue;
			}

			if (!issnap && zfs_prop_userquota(propname) &&
			    nvpair_type(elem) == DATA_TYPE_UINT64_ARRAY) {
				const char *perm;
				const char *up = zfs_userquota_prop_prefixes
				    [ZFS_PROP_USERQUOTA];
				if (strncmp(propname, up, strlen(up)) == 0)
					perm = ZFS_DELEG_PERM_USERQUOTA;
				else
					perm = ZFS_DELEG_PERM_GROUPQUOTA;
				if (error = zfs_secpolicy_write_perms(name,
				    perm, CRED()))
					return (error);
				continue;
			}

			return (EINVAL);
		}

		if (issnap)
			return (EINVAL);

		if ((error = zfs_secpolicy_setprop(name, prop, CRED())) != 0)
			return (error);

		/*
		 * Check that this value is valid for this pool version
		 */
		switch (prop) {
		case ZFS_PROP_COMPRESSION:
			/*
			 * If the user specified gzip compression, make sure
			 * the SPA supports it. We ignore any errors here since
			 * we'll catch them later.
			 */
			if (nvpair_type(elem) == DATA_TYPE_UINT64 &&
			    nvpair_value_uint64(elem, &intval) == 0) {
				if (intval >= ZIO_COMPRESS_GZIP_1 &&
				    intval <= ZIO_COMPRESS_GZIP_9 &&
				    zfs_earlier_version(name,
				    SPA_VERSION_GZIP_COMPRESSION))
					return (ENOTSUP);

				/*
				 * If this is a bootable dataset then
				 * verify that the compression algorithm
				 * is supported for booting. We must return
				 * something other than ENOTSUP since it
				 * implies a downrev pool version.
				 */
				if (zfs_is_bootfs(name) &&
				    !BOOTFS_COMPRESS_VALID(intval))
					return (ERANGE);
			}
			break;

		case ZFS_PROP_COPIES:
			if (zfs_earlier_version(name, SPA_VERSION_DITTO_BLOCKS))
				return (ENOTSUP);
			break;

		case ZFS_PROP_SHARESMB:
			if (zpl_earlier_version(name, ZPL_VERSION_FUID))
				return (ENOTSUP);
			break;

		case ZFS_PROP_ACLINHERIT:
			if (nvpair_type(elem) == DATA_TYPE_UINT64 &&
			    nvpair_value_uint64(elem, &intval) == 0)
				if (intval == ZFS_ACL_PASSTHROUGH_X &&
				    zfs_earlier_version(name,
				    SPA_VERSION_PASSTHROUGH_X))
					return (ENOTSUP);
		}
	}

	elem = NULL;
	while ((elem = nvlist_next_nvpair(nvl, elem)) != NULL) {
		const char *propname = nvpair_name(elem);
		zfs_prop_t prop = zfs_name_to_prop(propname);

		if (prop == ZPROP_INVAL) {
			if (zfs_prop_userquota(propname)) {
				uint64_t *valary;
				unsigned int vallen;
				const char *domain;
				zfs_userquota_prop_t type;
				uint64_t rid;
				uint64_t quota;
				zfsvfs_t *zfsvfs;

				VERIFY(nvpair_value_uint64_array(elem,
				    &valary, &vallen) == 0);
				VERIFY(vallen == 3);
				type = valary[0];
				rid = valary[1];
				quota = valary[2];
				domain = propname +
				    strlen(zfs_userquota_prop_prefixes[type]);

				error = zfsvfs_hold(name, B_FALSE, FTAG,
				    &zfsvfs);
				if (error == 0) {
					error = zfs_set_userquota(zfsvfs,
					    type, domain, rid, quota);
					zfsvfs_rele(zfsvfs, FTAG);
				}
				if (error == 0)
					continue;
				else
					goto out;
			} else if (zfs_prop_user(propname)) {
				VERIFY(nvpair_value_string(elem, &strval) == 0);
				error = dsl_prop_set(name, propname, 1,
				    strlen(strval) + 1, strval);
				if (error == 0)
					continue;
				else
					goto out;
			}
		}

		switch (prop) {
		case ZFS_PROP_QUOTA:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = dsl_dir_set_quota(name, intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_REFQUOTA:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = dsl_dataset_set_quota(name, intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_RESERVATION:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = dsl_dir_set_reservation(name,
			    intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_REFRESERVATION:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = dsl_dataset_set_reservation(name,
			    intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_VOLSIZE:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = zvol_set_volsize(name,
			    ddi_driver_major(zfs_dip), intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_VOLBLOCKSIZE:
			if ((error = nvpair_value_uint64(elem, &intval)) != 0 ||
			    (error = zvol_set_volblocksize(name, intval)) != 0)
				goto out;
			break;

		case ZFS_PROP_VERSION:
		{
			zfsvfs_t *zfsvfs;

			if ((error = nvpair_value_uint64(elem, &intval)) != 0)
				goto out;
			if ((error = zfsvfs_hold(name, B_FALSE, FTAG,
			    &zfsvfs)) != 0)
			        goto out;
			error = zfs_set_version(zfsvfs, intval);
			zfsvfs_rele(zfsvfs, FTAG);

			if (error == 0 && intval >= ZPL_VERSION_USERSPACE) {
				zfs_cmd_t zc = { 0 };
				(void) strcpy(zc.zc_name, name);
				(void) zfs_ioc_userspace_upgrade(&zc);
			}
			if (error)
				goto out;
			break;
		}

		default:
			if (nvpair_type(elem) == DATA_TYPE_STRING) {
				if (zfs_prop_get_type(prop) !=
				    PROP_TYPE_STRING) {
					error = EINVAL;
					goto out;
				}
			} else if (nvpair_type(elem) == DATA_TYPE_UINT64) {
				const char *unused;

				VERIFY(nvpair_value_uint64(elem, &intval) == 0);

				switch (zfs_prop_get_type(prop)) {
				case PROP_TYPE_NUMBER:
					break;
				case PROP_TYPE_STRING:
					error = EINVAL;
					goto out;
				case PROP_TYPE_INDEX:
					if (zfs_prop_index_to_string(prop,
					    intval, &unused) != 0) {
						error = EINVAL;
						goto out;
					}
					break;
				default:
					cmn_err(CE_PANIC,
					    "unknown property type");
					break;
				}
			} else {
				error = EINVAL;
				goto out;
			}
			if ((error = nvlist_add_nvpair(genericnvl, elem)) != 0)
				goto out;
		}
	}

	if (nvlist_next_nvpair(genericnvl, NULL) != NULL) {
		error = dsl_props_set(name, genericnvl);
	}
out:
	nvlist_free(genericnvl);
	return (error);
}

/*
 * Check that all the properties are valid user properties.
 */
static int
zfs_check_userprops(char *fsname, nvlist_t *nvl)
{
	nvpair_t *elem = NULL;
	int error = 0;

	while ((elem = nvlist_next_nvpair(nvl, elem)) != NULL) {
		const char *propname = nvpair_name(elem);
		char *valstr;

		if (!zfs_prop_user(propname) ||
		    nvpair_type(elem) != DATA_TYPE_STRING)
			return (EINVAL);

		if (error = zfs_secpolicy_write_perms(fsname,
		    ZFS_DELEG_PERM_USERPROP, CRED()))
			return (error);

		if (strlen(propname) >= ZAP_MAXNAMELEN)
			return (ENAMETOOLONG);

		VERIFY(nvpair_value_string(elem, &valstr) == 0);
		if (strlen(valstr) >= ZAP_MAXVALUELEN)
			return (E2BIG);
	}
	return (0);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_value		name of property to set
 * zc_nvlist_src{_size}	nvlist of properties to apply
 * zc_cookie		clear existing local props?
 *
 * outputs:		none
 */
static int
zfs_ioc_set_prop(zfs_cmd_t *zc)
{
	nvlist_t *nvl;
	int error;

	if ((error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &nvl)) != 0)
		return (error);

	if (zc->zc_cookie) {
		nvlist_t *origprops;
		objset_t *os;

		if (dmu_objset_open(zc->zc_name, DMU_OST_ANY,
		    DS_MODE_USER | DS_MODE_READONLY, &os) == 0) {
			if (dsl_prop_get_all(os, &origprops, TRUE) == 0) {
				clear_props(zc->zc_name, origprops, nvl);
				nvlist_free(origprops);
			}
			dmu_objset_close(os);
		}

	}

	error = zfs_set_prop_nvlist(zc->zc_name, nvl);

	nvlist_free(nvl);
	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_value		name of property to inherit
 *
 * outputs:		none
 */
static int
zfs_ioc_inherit_prop(zfs_cmd_t *zc)
{
	/* the property name has been validated by zfs_secpolicy_inherit() */
	return (dsl_prop_set(zc->zc_name, zc->zc_value, 0, 0, NULL));
}

static int
zfs_ioc_pool_set_props(zfs_cmd_t *zc)
{
	nvlist_t *props;
	spa_t *spa;
	int error;
	nvpair_t *elem;

	if ((error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &props)))
		return (error);

	/*
	 * If the only property is the configfile, then just do a spa_lookup()
	 * to handle the faulted case.
	 */
	elem = nvlist_next_nvpair(props, NULL);
	if (elem != NULL && strcmp(nvpair_name(elem),
	    zpool_prop_to_name(ZPOOL_PROP_CACHEFILE)) == 0 &&
	    nvlist_next_nvpair(props, elem) == NULL) {
		mutex_enter(&spa_namespace_lock);
		if ((spa = spa_lookup(zc->zc_name)) != NULL) {
			spa_configfile_set(spa, props, B_FALSE);
			spa_config_sync(spa, B_FALSE, B_TRUE);
		}
		mutex_exit(&spa_namespace_lock);
		if (spa != NULL)
			return (0);
	}

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0) {
		nvlist_free(props);
		return (error);
	}

	error = spa_prop_set(spa, props);

	nvlist_free(props);
	spa_close(spa, FTAG);

	return (error);
}

static int
zfs_ioc_pool_get_props(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;
	nvlist_t *nvp = NULL;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0) {
		/*
		 * If the pool is faulted, there may be properties we can still
		 * get (such as altroot and cachefile), so attempt to get them
		 * anyway.
		 */
		mutex_enter(&spa_namespace_lock);
		if ((spa = spa_lookup(zc->zc_name)) != NULL)
			error = spa_prop_get(spa, &nvp);
		mutex_exit(&spa_namespace_lock);
	} else {
		error = spa_prop_get(spa, &nvp);
		spa_close(spa, FTAG);
	}

	if (error == 0 && zc->zc_nvlist_dst != 0)
		error = put_nvlist(zc, nvp);
	else
		error = EFAULT;

	nvlist_free(nvp);
	return (error);
}

static int
zfs_ioc_iscsi_perm_check(zfs_cmd_t *zc)
{
#ifdef sun
	nvlist_t *nvp;
	int error;
	uint32_t uid;
	uint32_t gid;
	uint32_t *groups;
	uint_t group_cnt;
	cred_t	*usercred;

	if ((error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &nvp)) != 0) {
		return (error);
	}

	if ((error = nvlist_lookup_uint32(nvp,
	    ZFS_DELEG_PERM_UID, &uid)) != 0) {
		nvlist_free(nvp);
		return (EPERM);
	}

	if ((error = nvlist_lookup_uint32(nvp,
	    ZFS_DELEG_PERM_GID, &gid)) != 0) {
		nvlist_free(nvp);
		return (EPERM);
	}

	if ((error = nvlist_lookup_uint32_array(nvp, ZFS_DELEG_PERM_GROUPS,
	    &groups, &group_cnt)) != 0) {
		nvlist_free(nvp);
		return (EPERM);
	}
	usercred = cralloc();
	if ((crsetugid(usercred, uid, gid) != 0) ||
	    (crsetgroups(usercred, group_cnt, (gid_t *)groups) != 0)) {
		nvlist_free(nvp);
		crfree(usercred);
		return (EPERM);
	}
	nvlist_free(nvp);
	error = dsl_deleg_access(zc->zc_name,
	    zfs_prop_to_name(ZFS_PROP_SHAREISCSI), usercred);
	crfree(usercred);
	return (error);
#else	/* sun */
	return (EPERM);
#endif	/* sun */
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_nvlist_src{_size}	nvlist of delegated permissions
 * zc_perm_action	allow/unallow flag
 *
 * outputs:		none
 */
static int
zfs_ioc_set_fsacl(zfs_cmd_t *zc)
{
	int error;
	nvlist_t *fsaclnv = NULL;

	if ((error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &fsaclnv)) != 0)
		return (error);

	/*
	 * Verify nvlist is constructed correctly
	 */
	if ((error = zfs_deleg_verify_nvlist(fsaclnv)) != 0) {
		nvlist_free(fsaclnv);
		return (EINVAL);
	}

	/*
	 * If we don't have PRIV_SYS_MOUNT, then validate
	 * that user is allowed to hand out each permission in
	 * the nvlist(s)
	 */

	error = secpolicy_zfs(CRED());
	if (error) {
		if (zc->zc_perm_action == B_FALSE) {
			error = dsl_deleg_can_allow(zc->zc_name,
			    fsaclnv, CRED());
		} else {
			error = dsl_deleg_can_unallow(zc->zc_name,
			    fsaclnv, CRED());
		}
	}

	if (error == 0)
		error = dsl_deleg_set(zc->zc_name, fsaclnv, zc->zc_perm_action);

	nvlist_free(fsaclnv);
	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 *
 * outputs:
 * zc_nvlist_src{_size}	nvlist of delegated permissions
 */
static int
zfs_ioc_get_fsacl(zfs_cmd_t *zc)
{
	nvlist_t *nvp;
	int error;

	if ((error = dsl_deleg_get(zc->zc_name, &nvp)) == 0) {
		error = put_nvlist(zc, nvp);
		nvlist_free(nvp);
	}

	return (error);
}

/*
 * inputs:
 * zc_name		name of volume
 *
 * outputs:		none
 */
static int
zfs_ioc_create_minor(zfs_cmd_t *zc)
{
	return (zvol_create_minor(zc->zc_name, ddi_driver_major(zfs_dip)));
}

/*
 * inputs:
 * zc_name		name of volume
 *
 * outputs:		none
 */
static int
zfs_ioc_remove_minor(zfs_cmd_t *zc)
{
	return (zvol_remove_minor(zc->zc_name));
}

/*
 * Search the vfs list for a specified resource.  Returns a pointer to it
 * or NULL if no suitable entry is found. The caller of this routine
 * is responsible for releasing the returned vfs pointer.
 */
static vfs_t *
zfs_get_vfs(const char *resource)
{
	vfs_t *vfsp;

	mtx_lock(&mountlist_mtx);
	TAILQ_FOREACH(vfsp, &mountlist, mnt_list) {
		if (strcmp(refstr_value(vfsp->vfs_resource), resource) == 0) {
			VFS_HOLD(vfsp);
			break;
		}
	}
	mtx_unlock(&mountlist_mtx);
	return (vfsp);
}

/* ARGSUSED */
static void
zfs_create_cb(objset_t *os, void *arg, cred_t *cr, dmu_tx_t *tx)
{
	zfs_creat_t *zct = arg;

	zfs_create_fs(os, cr, zct->zct_zplprops, tx);
}

#define	ZFS_PROP_UNDEFINED	((uint64_t)-1)

/*
 * inputs:
 * createprops		list of properties requested by creator
 * default_zplver	zpl version to use if unspecified in createprops
 * fuids_ok		fuids allowed in this version of the spa?
 * os			parent objset pointer (NULL if root fs)
 *
 * outputs:
 * zplprops	values for the zplprops we attach to the master node object
 * is_ci	true if requested file system will be purely case-insensitive
 *
 * Determine the settings for utf8only, normalization and
 * casesensitivity.  Specific values may have been requested by the
 * creator and/or we can inherit values from the parent dataset.  If
 * the file system is of too early a vintage, a creator can not
 * request settings for these properties, even if the requested
 * setting is the default value.  We don't actually want to create dsl
 * properties for these, so remove them from the source nvlist after
 * processing.
 */
static int
zfs_fill_zplprops_impl(objset_t *os, uint64_t zplver,
    boolean_t fuids_ok, nvlist_t *createprops, nvlist_t *zplprops,
    boolean_t *is_ci)
{
	uint64_t sense = ZFS_PROP_UNDEFINED;
	uint64_t norm = ZFS_PROP_UNDEFINED;
	uint64_t u8 = ZFS_PROP_UNDEFINED;

	ASSERT(zplprops != NULL);

	/*
	 * Pull out creator prop choices, if any.
	 */
	if (createprops) {
		(void) nvlist_lookup_uint64(createprops,
		    zfs_prop_to_name(ZFS_PROP_VERSION), &zplver);
		(void) nvlist_lookup_uint64(createprops,
		    zfs_prop_to_name(ZFS_PROP_NORMALIZE), &norm);
		(void) nvlist_remove_all(createprops,
		    zfs_prop_to_name(ZFS_PROP_NORMALIZE));
		(void) nvlist_lookup_uint64(createprops,
		    zfs_prop_to_name(ZFS_PROP_UTF8ONLY), &u8);
		(void) nvlist_remove_all(createprops,
		    zfs_prop_to_name(ZFS_PROP_UTF8ONLY));
		(void) nvlist_lookup_uint64(createprops,
		    zfs_prop_to_name(ZFS_PROP_CASE), &sense);
		(void) nvlist_remove_all(createprops,
		    zfs_prop_to_name(ZFS_PROP_CASE));
	}

	/*
	 * If the zpl version requested is whacky or the file system
	 * or pool is version is too "young" to support normalization
	 * and the creator tried to set a value for one of the props,
	 * error out.
	 */
	if ((zplver < ZPL_VERSION_INITIAL || zplver > ZPL_VERSION) ||
	    (zplver >= ZPL_VERSION_FUID && !fuids_ok) ||
	    (zplver < ZPL_VERSION_NORMALIZATION &&
	    (norm != ZFS_PROP_UNDEFINED || u8 != ZFS_PROP_UNDEFINED ||
	    sense != ZFS_PROP_UNDEFINED)))
		return (ENOTSUP);

	/*
	 * Put the version in the zplprops
	 */
	VERIFY(nvlist_add_uint64(zplprops,
	    zfs_prop_to_name(ZFS_PROP_VERSION), zplver) == 0);

	if (norm == ZFS_PROP_UNDEFINED)
		VERIFY(zfs_get_zplprop(os, ZFS_PROP_NORMALIZE, &norm) == 0);
	VERIFY(nvlist_add_uint64(zplprops,
	    zfs_prop_to_name(ZFS_PROP_NORMALIZE), norm) == 0);

	/*
	 * If we're normalizing, names must always be valid UTF-8 strings.
	 */
	if (norm)
		u8 = 1;
	if (u8 == ZFS_PROP_UNDEFINED)
		VERIFY(zfs_get_zplprop(os, ZFS_PROP_UTF8ONLY, &u8) == 0);
	VERIFY(nvlist_add_uint64(zplprops,
	    zfs_prop_to_name(ZFS_PROP_UTF8ONLY), u8) == 0);

	if (sense == ZFS_PROP_UNDEFINED)
		VERIFY(zfs_get_zplprop(os, ZFS_PROP_CASE, &sense) == 0);
	VERIFY(nvlist_add_uint64(zplprops,
	    zfs_prop_to_name(ZFS_PROP_CASE), sense) == 0);

	if (is_ci)
		*is_ci = (sense == ZFS_CASE_INSENSITIVE);

	return (0);
}

static int
zfs_fill_zplprops(const char *dataset, nvlist_t *createprops,
    nvlist_t *zplprops, boolean_t *is_ci)
{
	boolean_t fuids_ok = B_TRUE;
	uint64_t zplver = ZPL_VERSION;
	objset_t *os = NULL;
	char parentname[MAXNAMELEN];
	char *cp;
	int error;

	(void) strlcpy(parentname, dataset, sizeof (parentname));
	cp = strrchr(parentname, '/');
	ASSERT(cp != NULL);
	cp[0] = '\0';

	if (zfs_earlier_version(dataset, SPA_VERSION_USERSPACE))
		zplver = ZPL_VERSION_USERSPACE - 1;
	if (zfs_earlier_version(dataset, SPA_VERSION_FUID)) {
		zplver = ZPL_VERSION_FUID - 1;
		fuids_ok = B_FALSE;
	}

	/*
	 * Open parent object set so we can inherit zplprop values.
	 */
	if ((error = dmu_objset_open(parentname, DMU_OST_ANY,
	    DS_MODE_USER | DS_MODE_READONLY, &os)) != 0)
		return (error);

	error = zfs_fill_zplprops_impl(os, zplver, fuids_ok, createprops,
	    zplprops, is_ci);
	dmu_objset_close(os);
	return (error);
}

static int
zfs_fill_zplprops_root(uint64_t spa_vers, nvlist_t *createprops,
    nvlist_t *zplprops, boolean_t *is_ci)
{
	boolean_t fuids_ok = B_TRUE;
	uint64_t zplver = ZPL_VERSION;
	int error;

	if (spa_vers < SPA_VERSION_FUID) {
		zplver = ZPL_VERSION_FUID - 1;
		fuids_ok = B_FALSE;
	}

	error = zfs_fill_zplprops_impl(NULL, zplver, fuids_ok, createprops,
	    zplprops, is_ci);
	return (error);
}

/*
 * inputs:
 * zc_objset_type	type of objset to create (fs vs zvol)
 * zc_name		name of new objset
 * zc_value		name of snapshot to clone from (may be empty)
 * zc_nvlist_src{_size}	nvlist of properties to apply
 *
 * outputs: none
 */
static int
zfs_ioc_create(zfs_cmd_t *zc)
{
	objset_t *clone;
	int error = 0;
	zfs_creat_t zct;
	nvlist_t *nvprops = NULL;
	void (*cbfunc)(objset_t *os, void *arg, cred_t *cr, dmu_tx_t *tx);
	dmu_objset_type_t type = zc->zc_objset_type;

	switch (type) {

	case DMU_OST_ZFS:
		cbfunc = zfs_create_cb;
		break;

	case DMU_OST_ZVOL:
		cbfunc = zvol_create_cb;
		break;

	default:
		cbfunc = NULL;
		break;
	}
	if (strchr(zc->zc_name, '@') ||
	    strchr(zc->zc_name, '%'))
		return (EINVAL);

	if (zc->zc_nvlist_src != 0 &&
	    (error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &nvprops)) != 0)
		return (error);

	zct.zct_zplprops = NULL;
	zct.zct_props = nvprops;

	if (zc->zc_value[0] != '\0') {
		/*
		 * We're creating a clone of an existing snapshot.
		 */
		zc->zc_value[sizeof (zc->zc_value) - 1] = '\0';
		if (dataset_namecheck(zc->zc_value, NULL, NULL) != 0) {
			nvlist_free(nvprops);
			return (EINVAL);
		}

		error = dmu_objset_open(zc->zc_value, type,
		    DS_MODE_USER | DS_MODE_READONLY, &clone);
		if (error) {
			nvlist_free(nvprops);
			return (error);
		}

		error = dmu_objset_create(zc->zc_name, type, clone, 0,
		    NULL, NULL);
		if (error) {
			dmu_objset_close(clone);
			nvlist_free(nvprops);
			return (error);
		}
		dmu_objset_close(clone);
	} else {
		boolean_t is_insensitive = B_FALSE;

		if (cbfunc == NULL) {
			nvlist_free(nvprops);
			return (EINVAL);
		}

		if (type == DMU_OST_ZVOL) {
			uint64_t volsize, volblocksize;

			if (nvprops == NULL ||
			    nvlist_lookup_uint64(nvprops,
			    zfs_prop_to_name(ZFS_PROP_VOLSIZE),
			    &volsize) != 0) {
				nvlist_free(nvprops);
				return (EINVAL);
			}

			if ((error = nvlist_lookup_uint64(nvprops,
			    zfs_prop_to_name(ZFS_PROP_VOLBLOCKSIZE),
			    &volblocksize)) != 0 && error != ENOENT) {
				nvlist_free(nvprops);
				return (EINVAL);
			}

			if (error != 0)
				volblocksize = zfs_prop_default_numeric(
				    ZFS_PROP_VOLBLOCKSIZE);

			if ((error = zvol_check_volblocksize(
			    volblocksize)) != 0 ||
			    (error = zvol_check_volsize(volsize,
			    volblocksize)) != 0) {
				nvlist_free(nvprops);
				return (error);
			}
		} else if (type == DMU_OST_ZFS) {
			int error;

			/*
			 * We have to have normalization and
			 * case-folding flags correct when we do the
			 * file system creation, so go figure them out
			 * now.
			 */
			VERIFY(nvlist_alloc(&zct.zct_zplprops,
			    NV_UNIQUE_NAME, KM_SLEEP) == 0);
			error = zfs_fill_zplprops(zc->zc_name, nvprops,
			    zct.zct_zplprops, &is_insensitive);
			if (error != 0) {
				nvlist_free(nvprops);
				nvlist_free(zct.zct_zplprops);
				return (error);
			}
		}
		error = dmu_objset_create(zc->zc_name, type, NULL,
		    is_insensitive ? DS_FLAG_CI_DATASET : 0, cbfunc, &zct);
		nvlist_free(zct.zct_zplprops);
	}

	/*
	 * It would be nice to do this atomically.
	 */
	if (error == 0) {
		if ((error = zfs_set_prop_nvlist(zc->zc_name, nvprops)) != 0)
			(void) dmu_objset_destroy(zc->zc_name);
	}
	nvlist_free(nvprops);
	return (error);
}

/*
 * inputs:
 * zc_name	name of filesystem
 * zc_value	short name of snapshot
 * zc_cookie	recursive flag
 * zc_nvlist_src[_size] property list
 *
 * outputs:	none
 */
static int
zfs_ioc_snapshot(zfs_cmd_t *zc)
{
	nvlist_t *nvprops = NULL;
	int error;
	boolean_t recursive = zc->zc_cookie;

	if (snapshot_namecheck(zc->zc_value, NULL, NULL) != 0)
		return (EINVAL);

	if (zc->zc_nvlist_src != 0 &&
	    (error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &nvprops)) != 0)
		return (error);

	error = zfs_check_userprops(zc->zc_name, nvprops);
	if (error)
		goto out;

	if (nvprops != NULL && nvlist_next_nvpair(nvprops, NULL) != NULL &&
	    zfs_earlier_version(zc->zc_name, SPA_VERSION_SNAP_PROPS)) {
		error = ENOTSUP;
		goto out;
	}

	error = dmu_objset_snapshot(zc->zc_name, zc->zc_value,
	    nvprops, recursive);

out:
	nvlist_free(nvprops);
	return (error);
}

int
zfs_unmount_snap(char *name, void *arg)
{
	vfs_t *vfsp = NULL;

	if (arg) {
		char *snapname = arg;
		int len = strlen(name) + strlen(snapname) + 2;
		char *buf = kmem_alloc(len, KM_SLEEP);

		(void) strcpy(buf, name);
		(void) strcat(buf, "@");
		(void) strcat(buf, snapname);
		vfsp = zfs_get_vfs(buf);
		kmem_free(buf, len);
	} else if (strchr(name, '@')) {
		vfsp = zfs_get_vfs(name);
	}

	if (vfsp) {
		/*
		 * Always force the unmount for snapshots.
		 */
		int flag = MS_FORCE;
		int err;

		if ((err = vn_vfswlock(vfsp->vfs_vnodecovered)) != 0) {
			VFS_RELE(vfsp);
			return (err);
		}
		VFS_RELE(vfsp);
		mtx_lock(&Giant);	/* dounmount() */
		dounmount(vfsp, flag, curthread);
		mtx_unlock(&Giant);	/* dounmount() */
	}
	return (0);
}

/*
 * inputs:
 * zc_name	name of filesystem
 * zc_value	short name of snapshot
 *
 * outputs:	none
 */
static int
zfs_ioc_destroy_snaps(zfs_cmd_t *zc)
{
	int err;

	if (snapshot_namecheck(zc->zc_value, NULL, NULL) != 0)
		return (EINVAL);
	err = dmu_objset_find(zc->zc_name,
	    zfs_unmount_snap, zc->zc_value, DS_FIND_CHILDREN);
	if (err)
		return (err);
	return (dmu_snapshots_destroy(zc->zc_name, zc->zc_value));
}

/*
 * inputs:
 * zc_name		name of dataset to destroy
 * zc_objset_type	type of objset
 *
 * outputs:		none
 */
static int
zfs_ioc_destroy(zfs_cmd_t *zc)
{
	if (strchr(zc->zc_name, '@') && zc->zc_objset_type == DMU_OST_ZFS) {
		int err = zfs_unmount_snap(zc->zc_name, NULL);
		if (err)
			return (err);
	}

	return (dmu_objset_destroy(zc->zc_name));
}

/*
 * inputs:
 * zc_name	name of dataset to rollback (to most recent snapshot)
 *
 * outputs:	none
 */
static int
zfs_ioc_rollback(zfs_cmd_t *zc)
{
	objset_t *os;
	int error;
	zfsvfs_t *zfsvfs = NULL;

	/*
	 * Get the zfsvfs for the receiving objset. There
	 * won't be one if we're operating on a zvol, if the
	 * objset doesn't exist yet, or is not mounted.
	 */
	error = dmu_objset_open(zc->zc_name, DMU_OST_ANY, DS_MODE_USER, &os);
	if (error)
		return (error);

	if (getzfsvfs(zc->zc_name, &zfsvfs) == 0) {
		int mode;

		error = zfs_suspend_fs(zfsvfs, NULL, &mode);
		if (error == 0) {
			int resume_err;

			error = dmu_objset_rollback(os);
			resume_err = zfs_resume_fs(zfsvfs, zc->zc_name, mode);
			error = error ? error : resume_err;
		} else {
			dmu_objset_close(os);
		}
		VFS_RELE(zfsvfs->z_vfs);
	} else {
		error = dmu_objset_rollback(os);
	}
	/* Note, the dmu_objset_rollback() releases the objset for us. */

	return (error);
}

/*
 * inputs:
 * zc_name	old name of dataset
 * zc_value	new name of dataset
 * zc_cookie	recursive flag (only valid for snapshots)
 *
 * outputs:	none
 */
static int
zfs_ioc_rename(zfs_cmd_t *zc)
{
	boolean_t recursive = zc->zc_cookie & 1;

	zc->zc_value[sizeof (zc->zc_value) - 1] = '\0';
	if (dataset_namecheck(zc->zc_value, NULL, NULL) != 0 ||
	    strchr(zc->zc_value, '%'))
		return (EINVAL);

	/*
	 * Unmount snapshot unless we're doing a recursive rename,
	 * in which case the dataset code figures out which snapshots
	 * to unmount.
	 */
	if (!recursive && strchr(zc->zc_name, '@') != NULL &&
	    zc->zc_objset_type == DMU_OST_ZFS) {
		int err = zfs_unmount_snap(zc->zc_name, NULL);
		if (err)
			return (err);
	}
	return (dmu_objset_rename(zc->zc_name, zc->zc_value, recursive));
}

static void
clear_props(char *dataset, nvlist_t *props, nvlist_t *newprops)
{
	zfs_cmd_t *zc;
	nvpair_t *prop;

	if (props == NULL)
		return;
	zc = kmem_alloc(sizeof (zfs_cmd_t), KM_SLEEP);
	(void) strcpy(zc->zc_name, dataset);
	for (prop = nvlist_next_nvpair(props, NULL); prop;
	    prop = nvlist_next_nvpair(props, prop)) {
		if (newprops != NULL &&
		    nvlist_exists(newprops, nvpair_name(prop)))
			continue;
		(void) strcpy(zc->zc_value, nvpair_name(prop));
		if (zfs_secpolicy_inherit(zc, CRED()) == 0)
			(void) zfs_ioc_inherit_prop(zc);
	}
	kmem_free(zc, sizeof (zfs_cmd_t));
}

/*
 * inputs:
 * zc_name		name of containing filesystem
 * zc_nvlist_src{_size}	nvlist of properties to apply
 * zc_value		name of snapshot to create
 * zc_string		name of clone origin (if DRR_FLAG_CLONE)
 * zc_cookie		file descriptor to recv from
 * zc_begin_record	the BEGIN record of the stream (not byteswapped)
 * zc_guid		force flag
 *
 * outputs:
 * zc_cookie		number of bytes read
 */
static int
zfs_ioc_recv(zfs_cmd_t *zc)
{
	file_t *fp;
	objset_t *os;
	dmu_recv_cookie_t drc;
	zfsvfs_t *zfsvfs = NULL;
	boolean_t force = (boolean_t)zc->zc_guid;
	int error, fd;
	offset_t off;
	nvlist_t *props = NULL;
	nvlist_t *origprops = NULL;
	objset_t *origin = NULL;
	char *tosnap;
	char tofs[ZFS_MAXNAMELEN];

	if (dataset_namecheck(zc->zc_value, NULL, NULL) != 0 ||
	    strchr(zc->zc_value, '@') == NULL ||
	    strchr(zc->zc_value, '%'))
		return (EINVAL);

	(void) strcpy(tofs, zc->zc_value);
	tosnap = strchr(tofs, '@');
	*tosnap = '\0';
	tosnap++;

	if (zc->zc_nvlist_src != 0 &&
	    (error = get_nvlist(zc->zc_nvlist_src, zc->zc_nvlist_src_size,
	    &props)) != 0)
		return (error);

	fd = zc->zc_cookie;
	fp = getf(fd, 0);
	if (fp == NULL) {
		nvlist_free(props);
		return (EBADF);
	}

	if (getzfsvfs(tofs, &zfsvfs) == 0) {
		if (!mutex_tryenter(&zfsvfs->z_online_recv_lock)) {
			VFS_RELE(zfsvfs->z_vfs);
			zfsvfs = NULL;
			error = EBUSY;
			goto out;
		}
		/*
		 * If new properties are supplied, they are to completely
		 * replace the existing ones, so stash away the existing ones.
		 */
		if (props)
			(void) dsl_prop_get_all(zfsvfs->z_os, &origprops, TRUE);
	} else if (props && dmu_objset_open(tofs, DMU_OST_ANY,
	    DS_MODE_USER | DS_MODE_READONLY, &os) == 0) {
		/*
		 * Get the props even if there was no zfsvfs (zvol or
		 * unmounted zpl).
		 */
		(void) dsl_prop_get_all(os, &origprops, TRUE);

		dmu_objset_close(os);
	}

	if (zc->zc_string[0]) {
		error = dmu_objset_open(zc->zc_string, DMU_OST_ANY,
		    DS_MODE_USER | DS_MODE_READONLY, &origin);
		if (error)
			goto out;
	}

	error = dmu_recv_begin(tofs, tosnap, &zc->zc_begin_record,
	    force, origin, zfsvfs != NULL, &drc);
	if (origin)
		dmu_objset_close(origin);
	if (error)
		goto out;

	/*
	 * Reset properties.  We do this before we receive the stream
	 * so that the properties are applied to the new data.
	 */
	if (props) {
		clear_props(tofs, origprops, props);
		/*
		 * XXX - Note, this is all-or-nothing; should be best-effort.
		 */
		(void) zfs_set_prop_nvlist(tofs, props);
	}

	off = fp->f_offset;
	error = dmu_recv_stream(&drc, fp, &off);

	if (error == 0 && zfsvfs) {
		char *osname;
		int mode;

		/* online recv */
		osname = kmem_alloc(MAXNAMELEN, KM_SLEEP);
		error = zfs_suspend_fs(zfsvfs, osname, &mode);
		if (error == 0) {
			int resume_err;

			error = dmu_recv_end(&drc);
			resume_err = zfs_resume_fs(zfsvfs, osname, mode);
			error = error ? error : resume_err;
		} else {
			dmu_recv_abort_cleanup(&drc);
		}
		kmem_free(osname, MAXNAMELEN);
	} else if (error == 0) {
		error = dmu_recv_end(&drc);
	}

	zc->zc_cookie = off - fp->f_offset;
	if (off >= 0 && off <= MAXOFFSET_T)
		fp->f_offset = off;

	/*
	 * On error, restore the original props.
	 */
	if (error && props) {
		clear_props(tofs, props, NULL);
		(void) zfs_set_prop_nvlist(tofs, origprops);
	}
out:
	if (zfsvfs) {
		mutex_exit(&zfsvfs->z_online_recv_lock);
		VFS_RELE(zfsvfs->z_vfs);
	}
	nvlist_free(props);
	nvlist_free(origprops);
	releasef(fp);
	return (error);
}

/*
 * inputs:
 * zc_name	name of snapshot to send
 * zc_value	short name of incremental fromsnap (may be empty)
 * zc_cookie	file descriptor to send stream to
 * zc_obj	fromorigin flag (mutually exclusive with zc_value)
 *
 * outputs: none
 */
static int
zfs_ioc_send(zfs_cmd_t *zc)
{
	objset_t *fromsnap = NULL;
	objset_t *tosnap;
	file_t *fp;
	int error;
	offset_t off;

	error = dmu_objset_open(zc->zc_name, DMU_OST_ANY,
	    DS_MODE_USER | DS_MODE_READONLY, &tosnap);
	if (error)
		return (error);

	if (zc->zc_value[0] != '\0') {
		char *buf;
		char *cp;

		buf = kmem_alloc(MAXPATHLEN, KM_SLEEP);
		(void) strncpy(buf, zc->zc_name, MAXPATHLEN);
		cp = strchr(buf, '@');
		if (cp)
			*(cp+1) = 0;
		(void) strlcat(buf, zc->zc_value, MAXPATHLEN);
		error = dmu_objset_open(buf, DMU_OST_ANY,
		    DS_MODE_USER | DS_MODE_READONLY, &fromsnap);
		kmem_free(buf, MAXPATHLEN);
		if (error) {
			dmu_objset_close(tosnap);
			return (error);
		}
	}

	fp = getf(zc->zc_cookie, 1);
	if (fp == NULL) {
		dmu_objset_close(tosnap);
		if (fromsnap)
			dmu_objset_close(fromsnap);
		return (EBADF);
	}

	off = fp->f_offset;
	error = dmu_sendbackup(tosnap, fromsnap, zc->zc_obj, fp, &off);

	if (off >= 0 && off <= MAXOFFSET_T)
		fp->f_offset = off;
	releasef(fp);
	if (fromsnap)
		dmu_objset_close(fromsnap);
	dmu_objset_close(tosnap);
	return (error);
}

static int
zfs_ioc_inject_fault(zfs_cmd_t *zc)
{
	int id, error;

	error = zio_inject_fault(zc->zc_name, (int)zc->zc_guid, &id,
	    &zc->zc_inject_record);

	if (error == 0)
		zc->zc_guid = (uint64_t)id;

	return (error);
}

static int
zfs_ioc_clear_fault(zfs_cmd_t *zc)
{
	return (zio_clear_fault((int)zc->zc_guid));
}

static int
zfs_ioc_inject_list_next(zfs_cmd_t *zc)
{
	int id = (int)zc->zc_guid;
	int error;

	error = zio_inject_list_next(&id, zc->zc_name, sizeof (zc->zc_name),
	    &zc->zc_inject_record);

	zc->zc_guid = id;

	return (error);
}

static int
zfs_ioc_error_log(zfs_cmd_t *zc)
{
	spa_t *spa;
	int error;
	size_t count = (size_t)zc->zc_nvlist_dst_size;

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	error = spa_get_errlog(spa, (void *)(uintptr_t)zc->zc_nvlist_dst,
	    &count);
	if (error == 0)
		zc->zc_nvlist_dst_size = count;
	else
		zc->zc_nvlist_dst_size = spa_get_errlog_size(spa);

	spa_close(spa, FTAG);

	return (error);
}

static int
zfs_ioc_clear(zfs_cmd_t *zc)
{
	spa_t *spa;
	vdev_t *vd;
	int error;

	/*
	 * On zpool clear we also fix up missing slogs
	 */
	mutex_enter(&spa_namespace_lock);
	spa = spa_lookup(zc->zc_name);
	if (spa == NULL) {
		mutex_exit(&spa_namespace_lock);
		return (EIO);
	}
	if (spa->spa_log_state == SPA_LOG_MISSING) {
		/* we need to let spa_open/spa_load clear the chains */
		spa->spa_log_state = SPA_LOG_CLEAR;
	}
	mutex_exit(&spa_namespace_lock);

	if ((error = spa_open(zc->zc_name, &spa, FTAG)) != 0)
		return (error);

	spa_vdev_state_enter(spa);

	if (zc->zc_guid == 0) {
		vd = NULL;
	} else {
		vd = spa_lookup_by_guid(spa, zc->zc_guid, B_TRUE);
		if (vd == NULL) {
			(void) spa_vdev_state_exit(spa, NULL, ENODEV);
			spa_close(spa, FTAG);
			return (ENODEV);
		}
	}

	vdev_clear(spa, vd);

	(void) spa_vdev_state_exit(spa, NULL, 0);

	/*
	 * Resume any suspended I/Os.
	 */
	if (zio_resume(spa) != 0)
		error = EIO;

	spa_close(spa, FTAG);

	return (error);
}

/*
 * inputs:
 * zc_name	name of filesystem
 * zc_value	name of origin snapshot
 *
 * outputs:	none
 */
static int
zfs_ioc_promote(zfs_cmd_t *zc)
{
	char *cp;

	/*
	 * We don't need to unmount *all* the origin fs's snapshots, but
	 * it's easier.
	 */
	cp = strchr(zc->zc_value, '@');
	if (cp)
		*cp = '\0';
	(void) dmu_objset_find(zc->zc_value,
	    zfs_unmount_snap, NULL, DS_FIND_SNAPSHOTS);
	return (dsl_dataset_promote(zc->zc_name));
}

/*
 * Retrieve a single {user|group}{used|quota}@... property.
 *
 * inputs:
 * zc_name	name of filesystem
 * zc_objset_type zfs_userquota_prop_t
 * zc_value	domain name (eg. "S-1-234-567-89")
 * zc_guid	RID/UID/GID
 *
 * outputs:
 * zc_cookie	property value
 */
static int
zfs_ioc_userspace_one(zfs_cmd_t *zc)
{
	zfsvfs_t *zfsvfs;
	int error;

	if (zc->zc_objset_type >= ZFS_NUM_USERQUOTA_PROPS)
		return (EINVAL);

	error = zfsvfs_hold(zc->zc_name, B_TRUE, FTAG, &zfsvfs);
	if (error)
		return (error);

	error = zfs_userspace_one(zfsvfs,
	    zc->zc_objset_type, zc->zc_value, zc->zc_guid, &zc->zc_cookie);
	zfsvfs_rele(zfsvfs, FTAG);

	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 * zc_cookie		zap cursor
 * zc_objset_type	zfs_userquota_prop_t
 * zc_nvlist_dst[_size] buffer to fill (not really an nvlist)
 *
 * outputs:
 * zc_nvlist_dst[_size]	data buffer (array of zfs_useracct_t)
 * zc_cookie	zap cursor
 */
static int
zfs_ioc_userspace_many(zfs_cmd_t *zc)
{
	zfsvfs_t *zfsvfs;
	int error;

	error = zfsvfs_hold(zc->zc_name, B_TRUE, FTAG, &zfsvfs);
	if (error)
		return (error);

	int bufsize = zc->zc_nvlist_dst_size;
	void *buf = kmem_alloc(bufsize, KM_SLEEP);

	error = zfs_userspace_many(zfsvfs, zc->zc_objset_type, &zc->zc_cookie,
	    buf, &zc->zc_nvlist_dst_size);

	if (error == 0) {
		error = xcopyout(buf,
		    (void *)(uintptr_t)zc->zc_nvlist_dst,
		    zc->zc_nvlist_dst_size);
	}
	kmem_free(buf, bufsize);
	zfsvfs_rele(zfsvfs, FTAG);

	return (error);
}

/*
 * inputs:
 * zc_name		name of filesystem
 *
 * outputs:
 * none
 */
static int
zfs_ioc_userspace_upgrade(zfs_cmd_t *zc)
{
	objset_t *os;
	int error;
	zfsvfs_t *zfsvfs;

	if (getzfsvfs(zc->zc_name, &zfsvfs) == 0) {
		if (!dmu_objset_userused_enabled(zfsvfs->z_os->os)) {
			/*
			 * If userused is not enabled, it may be because the
			 * objset needs to be closed & reopened (to grow the
			 * objset_phys_t).  Suspend/resume the fs will do that.
			 */
			int mode;
			error = zfs_suspend_fs(zfsvfs, NULL, &mode);
			if (error == 0) {
				error = zfs_resume_fs(zfsvfs,
				    zc->zc_name, mode);
			}
		}
		if (error == 0)
			error = dmu_objset_userspace_upgrade(zfsvfs->z_os);
		VFS_RELE(zfsvfs->z_vfs);
	} else {
		error = dmu_objset_open(zc->zc_name, DMU_OST_ANY,
		    DS_MODE_USER, &os);
		if (error)
			return (error);

		error = dmu_objset_userspace_upgrade(os);
		dmu_objset_close(os);
	}

	return (error);
}

#ifdef sun
/*
 * We don't want to have a hard dependency
 * against some special symbols in sharefs
 * nfs, and smbsrv.  Determine them if needed when
 * the first file system is shared.
 * Neither sharefs, nfs or smbsrv are unloadable modules.
 */
int (*znfsexport_fs)(void *arg);
int (*zshare_fs)(enum sharefs_sys_op, share_t *, uint32_t);
int (*zsmbexport_fs)(void *arg, boolean_t add_share);

int zfs_nfsshare_inited;
int zfs_smbshare_inited;

ddi_modhandle_t nfs_mod;
ddi_modhandle_t sharefs_mod;
ddi_modhandle_t smbsrv_mod;
#endif	/* sun */
kmutex_t zfs_share_lock;

#ifdef sun
static int
zfs_init_sharefs()
{
	int error;

	ASSERT(MUTEX_HELD(&zfs_share_lock));
	/* Both NFS and SMB shares also require sharetab support. */
	if (sharefs_mod == NULL && ((sharefs_mod =
	    ddi_modopen("fs/sharefs",
	    KRTLD_MODE_FIRST, &error)) == NULL)) {
		return (ENOSYS);
	}
	if (zshare_fs == NULL && ((zshare_fs =
	    (int (*)(enum sharefs_sys_op, share_t *, uint32_t))
	    ddi_modsym(sharefs_mod, "sharefs_impl", &error)) == NULL)) {
		return (ENOSYS);
	}
	return (0);
}
#endif /* sun */

static int
zfs_ioc_share(zfs_cmd_t *zc)
{
#ifdef sun
	int error;
	int opcode;

	switch (zc->zc_share.z_sharetype) {
	case ZFS_SHARE_NFS:
	case ZFS_UNSHARE_NFS:
		if (zfs_nfsshare_inited == 0) {
			mutex_enter(&zfs_share_lock);
			if (nfs_mod == NULL && ((nfs_mod = ddi_modopen("fs/nfs",
			    KRTLD_MODE_FIRST, &error)) == NULL)) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			if (znfsexport_fs == NULL &&
			    ((znfsexport_fs = (int (*)(void *))
			    ddi_modsym(nfs_mod,
			    "nfs_export", &error)) == NULL)) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			error = zfs_init_sharefs();
			if (error) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			zfs_nfsshare_inited = 1;
			mutex_exit(&zfs_share_lock);
		}
		break;
	case ZFS_SHARE_SMB:
	case ZFS_UNSHARE_SMB:
		if (zfs_smbshare_inited == 0) {
			mutex_enter(&zfs_share_lock);
			if (smbsrv_mod == NULL && ((smbsrv_mod =
			    ddi_modopen("drv/smbsrv",
			    KRTLD_MODE_FIRST, &error)) == NULL)) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			if (zsmbexport_fs == NULL && ((zsmbexport_fs =
			    (int (*)(void *, boolean_t))ddi_modsym(smbsrv_mod,
			    "smb_server_share", &error)) == NULL)) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			error = zfs_init_sharefs();
			if (error) {
				mutex_exit(&zfs_share_lock);
				return (ENOSYS);
			}
			zfs_smbshare_inited = 1;
			mutex_exit(&zfs_share_lock);
		}
		break;
	default:
		return (EINVAL);
	}

	switch (zc->zc_share.z_sharetype) {
	case ZFS_SHARE_NFS:
	case ZFS_UNSHARE_NFS:
		if (error =
		    znfsexport_fs((void *)
		    (uintptr_t)zc->zc_share.z_exportdata))
			return (error);
		break;
	case ZFS_SHARE_SMB:
	case ZFS_UNSHARE_SMB:
		if (error = zsmbexport_fs((void *)
		    (uintptr_t)zc->zc_share.z_exportdata,
		    zc->zc_share.z_sharetype == ZFS_SHARE_SMB ?
		    B_TRUE: B_FALSE)) {
			return (error);
		}
		break;
	}

	opcode = (zc->zc_share.z_sharetype == ZFS_SHARE_NFS ||
	    zc->zc_share.z_sharetype == ZFS_SHARE_SMB) ?
	    SHAREFS_ADD : SHAREFS_REMOVE;

	/*
	 * Add or remove share from sharetab
	 */
	error = zshare_fs(opcode,
	    (void *)(uintptr_t)zc->zc_share.z_sharedata,
	    zc->zc_share.z_sharemax);

	return (error);
#else	/* sun */
	return (ENOSYS);
#endif	/* sun */
}

ace_t full_access[] = {
	{(uid_t)-1, ACE_ALL_PERMS, ACE_EVERYONE, 0}
};

#ifdef sun
/*
 * Remove all ACL files in shares dir
 */
static int
zfs_smb_acl_purge(znode_t *dzp)
{
	zap_cursor_t	zc;
	zap_attribute_t	zap;
	zfsvfs_t *zfsvfs = dzp->z_zfsvfs;
	int error;

	for (zap_cursor_init(&zc, zfsvfs->z_os, dzp->z_id);
	    (error = zap_cursor_retrieve(&zc, &zap)) == 0;
	    zap_cursor_advance(&zc)) {
		if ((error = VOP_REMOVE(ZTOV(dzp), zap.za_name, kcred,
		    NULL, 0)) != 0)
			break;
	}
	zap_cursor_fini(&zc);
	return (error);
}
#endif	/* sun */

static int
zfs_ioc_smb_acl(zfs_cmd_t *zc)
{
#ifdef sun
	vnode_t *vp;
	znode_t *dzp;
	vnode_t *resourcevp = NULL;
	znode_t *sharedir;
	zfsvfs_t *zfsvfs;
	nvlist_t *nvlist;
	char *src, *target;
	vattr_t vattr;
	vsecattr_t vsec;
	int error = 0;

	if ((error = lookupname(zc->zc_value, UIO_SYSSPACE,
	    NO_FOLLOW, NULL, &vp)) != 0)
		return (error);

	/* Now make sure mntpnt and dataset are ZFS */

	if (vp->v_vfsp->vfs_fstype != zfsfstype ||
	    (strcmp((char *)refstr_value(vp->v_vfsp->vfs_resource),
	    zc->zc_name) != 0)) {
		VN_RELE(vp);
		return (EINVAL);
	}

	dzp = VTOZ(vp);
	zfsvfs = dzp->z_zfsvfs;

	ZFS_ENTER(zfsvfs);

	/*
	 * Create share dir if its missing.
	 */
	mutex_enter(&zfsvfs->z_lock);
	if (zfsvfs->z_shares_dir == 0) {
		dmu_tx_t *tx;

		tx = dmu_tx_create(zfsvfs->z_os);
		dmu_tx_hold_zap(tx, MASTER_NODE_OBJ, TRUE,
		    ZFS_SHARES_DIR);
		dmu_tx_hold_zap(tx, DMU_NEW_OBJECT, FALSE, NULL);
		error = dmu_tx_assign(tx, TXG_WAIT);
		if (error) {
			dmu_tx_abort(tx);
		} else {
			error = zfs_create_share_dir(zfsvfs, tx);
			dmu_tx_commit(tx);
		}
		if (error) {
			mutex_exit(&zfsvfs->z_lock);
			VN_RELE(vp);
			ZFS_EXIT(zfsvfs);
			return (error);
		}
	}
	mutex_exit(&zfsvfs->z_lock);

	ASSERT(zfsvfs->z_shares_dir);
	if ((error = zfs_zget(zfsvfs, zfsvfs->z_shares_dir, &sharedir)) != 0) {
		VN_RELE(vp);
		ZFS_EXIT(zfsvfs);
		return (error);
	}

	switch (zc->zc_cookie) {
	case ZFS_SMB_ACL_ADD:
		vattr.va_mask = AT_MODE|AT_UID|AT_GID|AT_TYPE;
		vattr.va_type = VREG;
		vattr.va_mode = S_IFREG|0777;
		vattr.va_uid = 0;
		vattr.va_gid = 0;

		vsec.vsa_mask = VSA_ACE;
		vsec.vsa_aclentp = &full_access;
		vsec.vsa_aclentsz = sizeof (full_access);
		vsec.vsa_aclcnt = 1;

		error = VOP_CREATE(ZTOV(sharedir), zc->zc_string,
		    &vattr, EXCL, 0, &resourcevp, kcred, 0, NULL, &vsec);
		if (resourcevp)
			VN_RELE(resourcevp);
		break;

	case ZFS_SMB_ACL_REMOVE:
		error = VOP_REMOVE(ZTOV(sharedir), zc->zc_string, kcred,
		    NULL, 0);
		break;

	case ZFS_SMB_ACL_RENAME:
		if ((error = get_nvlist(zc->zc_nvlist_src,
		    zc->zc_nvlist_src_size, &nvlist)) != 0) {
			VN_RELE(vp);
			ZFS_EXIT(zfsvfs);
			return (error);
		}
		if (nvlist_lookup_string(nvlist, ZFS_SMB_ACL_SRC, &src) ||
		    nvlist_lookup_string(nvlist, ZFS_SMB_ACL_TARGET,
		    &target)) {
			VN_RELE(vp);
			VN_RELE(ZTOV(sharedir));
			ZFS_EXIT(zfsvfs);
			return (error);
		}
		error = VOP_RENAME(ZTOV(sharedir), src, ZTOV(sharedir), target,
		    kcred, NULL, 0);
		nvlist_free(nvlist);
		break;

	case ZFS_SMB_ACL_PURGE:
		error = zfs_smb_acl_purge(sharedir);
		break;

	default:
		error = EINVAL;
		break;
	}

	VN_RELE(vp);
	VN_RELE(ZTOV(sharedir));

	ZFS_EXIT(zfsvfs);

	return (error);
#else	/* !sun */
	return (EOPNOTSUPP);
#endif	/* !sun */
}

/*
 * pool create, destroy, and export don't log the history as part of
 * zfsdev_ioctl, but rather zfs_ioc_pool_create, and zfs_ioc_pool_export
 * do the logging of those commands.
 */
static int
zfs_ioc_jail(zfs_cmd_t *zc)
{

	return (zone_dataset_attach(curthread->td_ucred, zc->zc_name,
	    (int)zc->zc_jailid));
}

static int
zfs_ioc_unjail(zfs_cmd_t *zc)
{

	return (zone_dataset_detach(curthread->td_ucred, zc->zc_name,
	    (int)zc->zc_jailid));
}

static zfs_ioc_vec_t zfs_ioc_vec[] = {
	{ zfs_ioc_pool_create, zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_destroy,	zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_import, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_FALSE },
	{ zfs_ioc_pool_export, zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_configs,	zfs_secpolicy_none, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_stats, zfs_secpolicy_read, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_tryimport, zfs_secpolicy_config, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_scrub, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_pool_freeze, zfs_secpolicy_config, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_upgrade,	zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_pool_get_history, zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_vdev_add, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_vdev_remove, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_vdev_set_state, zfs_secpolicy_config,	POOL_NAME, B_TRUE,
	    B_FALSE },
	{ zfs_ioc_vdev_attach, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_vdev_detach, zfs_secpolicy_config, POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_vdev_setpath,	zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_TRUE },
	{ zfs_ioc_objset_stats,	zfs_secpolicy_read, DATASET_NAME, B_FALSE,
	    B_TRUE },
	{ zfs_ioc_objset_zplprops, zfs_secpolicy_read, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_dataset_list_next, zfs_secpolicy_read, DATASET_NAME, B_FALSE,
	    B_TRUE },
	{ zfs_ioc_snapshot_list_next, zfs_secpolicy_read, DATASET_NAME, B_FALSE,
	    B_TRUE },
	{ zfs_ioc_set_prop, zfs_secpolicy_none, DATASET_NAME, B_TRUE, B_TRUE },
	{ zfs_ioc_create_minor,	zfs_secpolicy_minor, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_remove_minor,	zfs_secpolicy_minor, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_create, zfs_secpolicy_create, DATASET_NAME, B_TRUE, B_TRUE },
	{ zfs_ioc_destroy, zfs_secpolicy_destroy, DATASET_NAME, B_TRUE,
	    B_TRUE},
	{ zfs_ioc_rollback, zfs_secpolicy_rollback, DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_rename, zfs_secpolicy_rename,	DATASET_NAME, B_TRUE, B_TRUE },
	{ zfs_ioc_recv, zfs_secpolicy_receive, DATASET_NAME, B_TRUE, B_TRUE },
	{ zfs_ioc_send, zfs_secpolicy_send, DATASET_NAME, B_TRUE, B_FALSE },
	{ zfs_ioc_inject_fault,	zfs_secpolicy_inject, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_clear_fault, zfs_secpolicy_inject, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_inject_list_next, zfs_secpolicy_inject, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_error_log, zfs_secpolicy_inject, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_clear, zfs_secpolicy_config, POOL_NAME, B_TRUE, B_FALSE },
	{ zfs_ioc_promote, zfs_secpolicy_promote, DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_destroy_snaps, zfs_secpolicy_destroy,	DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_snapshot, zfs_secpolicy_snapshot, DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_dsobj_to_dsname, zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_obj_to_path, zfs_secpolicy_config, NO_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_pool_set_props, zfs_secpolicy_config,	POOL_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_pool_get_props, zfs_secpolicy_read, POOL_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_set_fsacl, zfs_secpolicy_fsacl, DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_get_fsacl, zfs_secpolicy_read, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_iscsi_perm_check, zfs_secpolicy_iscsi, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_share, zfs_secpolicy_share, DATASET_NAME, B_FALSE, B_FALSE },
	{ zfs_ioc_inherit_prop, zfs_secpolicy_inherit, DATASET_NAME, B_TRUE,
	    B_TRUE },
	{ zfs_ioc_jail, zfs_secpolicy_config, DATASET_NAME, B_TRUE, B_FALSE },
	{ zfs_ioc_unjail, zfs_secpolicy_config, DATASET_NAME, B_TRUE, B_FALSE },
	{ zfs_ioc_smb_acl, zfs_secpolicy_smb_acl, DATASET_NAME, B_FALSE,
	    B_FALSE },
	{ zfs_ioc_userspace_one, zfs_secpolicy_userspace_one,
	    DATASET_NAME, B_FALSE, B_FALSE },
	{ zfs_ioc_userspace_many, zfs_secpolicy_userspace_many,
	    DATASET_NAME, B_FALSE, B_FALSE },
	{ zfs_ioc_userspace_upgrade, zfs_secpolicy_userspace_upgrade,
	    DATASET_NAME, B_FALSE, B_TRUE },
	{ zfs_ioc_vdev_setfru,	zfs_secpolicy_config, POOL_NAME, B_FALSE,
	    B_TRUE }
};

int
pool_status_check(const char *name, zfs_ioc_namecheck_t type)
{
	spa_t *spa;
	char pool[ZFS_MAXNAMELEN];
	int error;

	ASSERT(type == POOL_NAME || type == DATASET_NAME);

	error = spa_open(name, &spa, FTAG);
	if (error == 0) {
		if (spa_suspended(spa))
			error = EAGAIN;
		spa_close(spa, FTAG);
	}
	return (error);
}

static int
zfsdev_ioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flag,
    struct thread *td)
{
	zfs_cmd_t *zc = (void *)addr;
	uint_t vec;
	int error;

	vec = ZFS_IOC(cmd);

	if (vec >= sizeof (zfs_ioc_vec) / sizeof (zfs_ioc_vec[0]))
		return (EINVAL);

	error = zfs_ioc_vec[vec].zvec_secpolicy(zc, td->td_ucred);

	/*
	 * Ensure that all pool/dataset names are valid before we pass down to
	 * the lower layers.
	 */
	if (error == 0) {
		zc->zc_name[sizeof (zc->zc_name) - 1] = '\0';
		switch (zfs_ioc_vec[vec].zvec_namecheck) {
		case POOL_NAME:
			if (pool_namecheck(zc->zc_name, NULL, NULL) != 0)
				error = EINVAL;
			if (zfs_ioc_vec[vec].zvec_pool_check)
				error = pool_status_check(zc->zc_name,
				    zfs_ioc_vec[vec].zvec_namecheck);
			break;

		case DATASET_NAME:
			if (dataset_namecheck(zc->zc_name, NULL, NULL) != 0)
				error = EINVAL;
			if (zfs_ioc_vec[vec].zvec_pool_check)
				error = pool_status_check(zc->zc_name,
				    zfs_ioc_vec[vec].zvec_namecheck);
			break;

		case NO_NAME:
			break;
		}
	}

	if (error == 0)
		error = zfs_ioc_vec[vec].zvec_func(zc);

	if (error == 0) {
		if (zfs_ioc_vec[vec].zvec_his_log)
			zfs_log_history(zc);
	}

	return (error);
}

/*
 * OK, so this is a little weird.
 *
 * /dev/zfs is the control node, i.e. minor 0.
 * /dev/zvol/[r]dsk/pool/dataset are the zvols, minor > 0.
 *
 * /dev/zfs has basically nothing to do except serve up ioctls,
 * so most of the standard driver entry points are in zvol.c.
 */
static struct cdevsw zfs_cdevsw = {
	.d_version =	D_VERSION,
	.d_ioctl =	zfsdev_ioctl,
	.d_name =	ZFS_DEV_NAME
};

static void
zfsdev_init(void)
{
	zfsdev = make_dev(&zfs_cdevsw, 0x0, UID_ROOT, GID_OPERATOR, 0666,
	    ZFS_DEV_NAME);
}

static void
zfsdev_fini(void)
{
	if (zfsdev != NULL)
		destroy_dev(zfsdev);
}

static struct root_hold_token *zfs_root_token;
struct proc *zfsproc;

uint_t zfs_fsyncer_key;
extern uint_t rrw_tsd_key;

static int
zfs_modevent(module_t mod, int type, void *unused __unused)
{
	int error = 0;

	switch (type) {
	case MOD_LOAD:
		zfs_root_token = root_mount_hold("ZFS");

		mutex_init(&zfs_share_lock, NULL, MUTEX_DEFAULT, NULL);

		spa_init(FREAD | FWRITE);
		zfs_init();
		zvol_init();

		tsd_create(&zfs_fsyncer_key, NULL);
		tsd_create(&rrw_tsd_key, NULL);

		printf("ZFS storage pool version " SPA_VERSION_STRING "\n");
		root_mount_rel(zfs_root_token);

		zfsdev_init();
		break;
	case MOD_UNLOAD:
		if (spa_busy() || zfs_busy() || zvol_busy() ||
		    zio_injection_enabled) {
			error = EBUSY;
			break;
		}

		zfsdev_fini();
		zvol_fini();
		zfs_fini();
		spa_fini();

		tsd_destroy(&zfs_fsyncer_key);
		tsd_destroy(&rrw_tsd_key);

		mutex_destroy(&zfs_share_lock);
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

static moduledata_t zfs_mod = {
	"zfsctrl",
	zfs_modevent,
	0
};
DECLARE_MODULE(zfsctrl, zfs_mod, SI_SUB_VFS, SI_ORDER_ANY);
MODULE_DEPEND(zfsctrl, opensolaris, 1, 1, 1);
MODULE_DEPEND(zfsctrl, krpc, 1, 1, 1);
