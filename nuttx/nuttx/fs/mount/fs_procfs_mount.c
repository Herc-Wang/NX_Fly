/****************************************************************************
 * fs/mount/fs_procfs_mount.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>

#include "mount/mount.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)
#if !defined(CONFIG_FS_PROCFS_EXCLUDE_MOUNT) || \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_BLOCKS) || \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_USAGE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determines the size of an intermediate buffer that must be large enough
 * to handle the longest line generated by this logic.
 */

#define MOUNT_LINELEN 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mount_file_e
{
  FS_MOUNT_FILE = 0,                 /* /proc/fs/mount */
  FS_BLOCKS_FILE,                    /* /proc/fs/blocks */
  FS_USAGE_FILE,                     /* /proc/fs/usage */
};

struct mount_file_s
{
  struct procfs_file_s base;         /* Base open file structure */
  uint8_t id;                        /* See enum mount_file_e */
  char line[MOUNT_LINELEN];          /* Pre-allocated buffer for formatted lines */
};

/* The structure is used when traversing routing tables */

struct mount_info_s
{
  FAR char *line;                    /* Intermediate line buffer pointer */
  FAR char *buffer;                  /* User buffer */
  size_t    linelen;                 /* Size of the intermediate buffer */
  size_t    buflen;                  /* Size of the user buffer */
  size_t    remaining;               /* Bytes remaining in user buffer */
  size_t    totalsize;               /* Accumulated size of the copy */
  off_t     offset;                  /* Skip offset */
  bool      header;                  /* True: header has been generated */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void    mount_sprintf(FAR struct mount_info_s *info,
                 FAR const char *fmt, ...);
#ifndef CONFIG_FS_PROCFS_EXCLUDE_MOUNT
static int     mount_entry(FAR const char *mountpoint,
                 FAR struct statfs *statbuf, FAR void *arg);
#endif
#ifndef CONFIG_FS_PROCFS_EXCLUDE_BLOCKS
static int     blocks_entry(FAR const char *mountpoint,
                 FAR struct statfs *statbuf, FAR void *arg);
#endif
#ifndef CONFIG_FS_PROCFS_EXCLUDE_USAGE
static int     usage_entry(FAR const char *mountpoint,
                 FAR struct statfs *statbuf, FAR void *arg);
#endif

/* File system methods */

static int     mount_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     mount_close(FAR struct file *filep);
static ssize_t mount_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     mount_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     mount_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations mount_procfsoperations =
{
  mount_open,          /* open */
  mount_close,         /* close */
  mount_read,          /* read */
  NULL,                /* write */

  mount_dup,           /* dup */

  NULL,                /* opendir */
  NULL,                /* closedir */
  NULL,                /* readdir */
  NULL,                /* rewinddir */

  mount_stat           /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_sprintf
 *
 * Description:
 *   Generate otuput from fs/mount, fs/blocks, or fs/usage, file read.
 *
 ****************************************************************************/

static void mount_sprintf(FAR struct mount_info_s *info,
                          FAR const char *fmt, ...)
{
  size_t linesize;
  size_t copysize;
  va_list ap;

  /* Print the format and data to a line buffer */

  va_start(ap, fmt);
  linesize = vsnprintf(info->line, info->linelen, fmt, ap);
  va_end(ap);

  /* Copy the line buffer to the user buffer */

  copysize = procfs_memcpy(info->line, linesize,
                           info->buffer, info->remaining,
                           &info->offset);

  /* Update counts and pointers */

  info->totalsize += copysize;
  info->buffer    += copysize;
  info->remaining -= copysize;
}

/****************************************************************************
 * Name: mount_entry
 *
 * Description:
 *   Output one fs/mount entry
 *
 *   Format:
 *     <mountpoint> type <type>
 *
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MOUNT
static int mount_entry(FAR const char *mountpoint,
                       FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct mount_info_s *info = (FAR struct mount_info_s *)arg;
  FAR const char *fstype;

  DEBUGASSERT(mountpoint != NULL && statbuf != NULL && info != NULL);

  /* Get the file system type */

  fstype = fs_gettype(statbuf);

  /* Generate mount list one line at a time */

  mount_sprintf(info, "  %s type %s\n", mountpoint, fstype);
  return (info->totalsize >= info->buflen) ? 1 : 0;
}
#endif

/****************************************************************************
 * Name: blocks_entry
 *
 * Description:
 *   Output one fs/blocks entry
 *
 *   Format:
 *     <mountpoint> type <type>
 *
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_BLOCKS
static int blocks_entry(FAR const char *mountpoint,
                        FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct mount_info_s *info = (FAR struct mount_info_s *)arg;

  DEBUGASSERT(mountpoint != NULL && statbuf != NULL && info != NULL);

  /* Have we generated the header yet? */

  if (!info->header)
    {
      mount_sprintf(info,
                    "  Block    Number\n");
      mount_sprintf(info,
                    "  Size     Blocks       Used   Available Mounted on\n");
      info->header = true;
    }

  /* Generate blocks list one line at a time */

  mount_sprintf(info, "%6lu %10" PRIuOFF " %10" PRIuOFF
                "  %10" PRIuOFF " %s\n",
                statbuf->f_bsize, statbuf->f_blocks,
                statbuf->f_blocks - statbuf->f_bavail, statbuf->f_bavail,
                mountpoint);

  return (info->totalsize >= info->buflen) ? 1 : 0;
}
#endif

/****************************************************************************
 * Name: blocks_entry
 *
 * Description:
 *   Output one fs/usage entry
 *
 *   Format:
 *     <mountpoint> type <type>
 *
 ****************************************************************************/

#ifndef CONFIG_FS_PROCFS_EXCLUDE_USAGE
static int usage_entry(FAR const char *mountpoint,
                       FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct mount_info_s *info = (FAR struct mount_info_s *)arg;
  FAR const char *fstype;
#ifdef CONFIG_HAVE_LONG_LONG
  uint64_t size;
  uint64_t used;
  uint64_t free;
#else
  uint32_t size;
  uint32_t used;
  uint32_t free;
#endif
  int which;
  char sizelabel;
  char freelabel;
  char usedlabel;
  static const char labels[5] =
  {
    'B', 'K', 'M', 'G', 'T'
  };

  DEBUGASSERT(mountpoint != NULL && statbuf != NULL && info != NULL);

  /* Have we generated the header yet? */

  if (!info->header)
    {
      mount_sprintf(info,
        "  Filesystem    Size      Used  Available Mounted on\n");
      info->header = true;
    }

  /* Get the file system type */

  fstype = fs_gettype(statbuf);

#ifdef CONFIG_HAVE_LONG_LONG
  size = (uint64_t)statbuf->f_bsize * statbuf->f_blocks;
  free = (uint64_t)statbuf->f_bsize * statbuf->f_bavail;
  used = (uint64_t)size - free;
#else
  size = statbuf->f_bsize * statbuf->f_blocks;
  free = statbuf->f_bsize * statbuf->f_bavail;
  used = size - free;
#endif

  /* Find the label for size */

  which = 0;
  while (size >= 9999 || ((size & 0x3ff) == 0 && size != 0))
    {
      which++;
      size >>= 10;
    }

  sizelabel = labels[which];

  /* Find the label for free */

  which = 0;
  while (free >= 9999 || ((free & 0x3ff) == 0 && free != 0))
    {
      which++;
      free >>= 10;
    }

  freelabel = labels[which];

  /* Find the label for used */

  which = 0;
  while (used >= 9999 || ((used & 0x3ff) == 0 && used != 0))
    {
      which++;
      used >>= 10;
    }

  usedlabel = labels[which];

  /* Generate usage list one line at a time */

#ifdef CONFIG_HAVE_LONG_LONG
  mount_sprintf(info, "  %-10s %6llu%c %8llu%c  %8llu%c %s\n", fstype,
                size, sizelabel, used, usedlabel, free, freelabel,
                mountpoint);
#else
  mount_sprintf(info, "  %-10s %6ld%c %8ld%c  %8ld%c %s\n", fstype,
                size, sizelabel, used, usedlabel, free, freelabel,
                mountpoint);
#endif

  return (info->totalsize >= info->buflen) ? 1 : 0;
}
#endif

/****************************************************************************
 * Name: mount_open
 ****************************************************************************/

static int mount_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct mount_file_s *procfile;
  uint8_t id;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* "fs/mount", "fs/block", and "fs/usage" are the only acceptable values
   * for the relpath.
   */

#ifndef CONFIG_FS_PROCFS_EXCLUDE_MOUNT
  if (strcmp(relpath, "fs/mount") == 0)
    {
      id = FS_MOUNT_FILE;
    }
#endif
#ifndef CONFIG_FS_PROCFS_EXCLUDE_BLOCKS
  else if (strcmp(relpath, "fs/blocks") == 0)
    {
      id = FS_BLOCKS_FILE;
    }
#endif
  else if (strcmp(relpath, "fs/usage") == 0)
    {
      id = FS_USAGE_FILE;
    }
  else
    {
      ferr("ERROR: relpath is '%s'\n", relpath);
      return -ENOENT;
    }

  /* Allocate a container to hold the task and node selection */

  procfile = (FAR struct mount_file_s *)
    kmm_zalloc(sizeof(struct mount_file_s));
  if (!procfile)
    {
      ferr("ERROR: Failed to allocate file container\n");
      return -ENOMEM;
    }

  /* Save the file ID */

  procfile->id = id;

  /* Save the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)procfile;
  return OK;
}

/****************************************************************************
 * Name: mount_close
 ****************************************************************************/

static int mount_close(FAR struct file *filep)
{
  FAR struct mount_file_s *procfile;

  /* Recover our private data from the struct file instance */

  procfile = (FAR struct mount_file_s *)filep->f_priv;
  DEBUGASSERT(procfile);

  /* Release the file container structure */

  kmm_free(procfile);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: mount_read
 ****************************************************************************/

static ssize_t mount_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct mount_file_s *procfile;
  struct mount_info_s info;
  foreach_mountpoint_t handler;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  procfile = (FAR struct mount_file_s *)filep->f_priv;
  DEBUGASSERT(procfile);

  /* Provide the requested data */

  memset(&info, 0, sizeof(struct mount_info_s));
  info.line      = procfile->line;
  info.buffer    = buffer;
  info.linelen   = MOUNT_LINELEN;
  info.buflen    = buflen;
  info.remaining = buflen;
  info.offset    = filep->f_pos;
  info.header    = false;

  switch (procfile->id)
    {
#ifndef CONFIG_FS_PROCFS_EXCLUDE_MOUNT
      case FS_MOUNT_FILE:                 /* /proc/fs/mount */
        handler = mount_entry;
        break;
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_BLOCKS
      case FS_BLOCKS_FILE:                /* /proc/fs/blocks */
        handler = blocks_entry;
        break;
#endif

#ifndef CONFIG_FS_PROCFS_EXCLUDE_USAGE
      case FS_USAGE_FILE:                 /* /proc/fs/usage */
        handler = usage_entry;
        break;
#endif

      default:
        DEBUGPANIC();
        return -EINVAL;
    }

  /* Generate each entry in the routing table */

  foreach_mountpoint(handler, &info);
  ret = info.totalsize;

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mount_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int mount_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct mount_file_s *oldfile;
  FAR struct mount_file_s *newfile;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldfile = (FAR struct mount_file_s *)oldp->f_priv;
  DEBUGASSERT(oldfile);

  /* Allocate a new container to hold the task and node selection */

  newfile = (FAR struct mount_file_s *)
    kmm_malloc(sizeof(struct mount_file_s));
  if (!newfile)
    {
      ferr("ERROR: Failed to allocate file container\n");
      return -ENOMEM;
    }

  /* The copy the file information from the old container to the new */

  memcpy(newfile, oldfile, sizeof(struct mount_file_s));

  /* Save the new container in the new file structure */

  newp->f_priv = (FAR void *)newfile;
  return OK;
}

/****************************************************************************
 * Name: mount_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int mount_stat(const char *relpath, struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_FS_PROCFS_EXCLUDE_MOUNT || \
        * !CONFIG_FS_PROCFS_EXCLUDE_BLOCKS || \
        * !CONFIG_FS_PROCFS_EXCLUDE_USAGE */
#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
