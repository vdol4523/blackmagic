/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Andrzej Surowiec <emeryth@gmail.com>
 * Copyright (C) 2013 Pavol Rusnak <stick@gk2.sk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "ramdisk.h"

#define WBVAL(x) ((x) & 0xFF), (((x) >> 8) & 0xFF)
#define QBVAL(x) ((x) & 0xFF), (((x) >> 8) & 0xFF),\
		 (((x) >> 16) & 0xFF), (((x) >> 24) & 0xFF)

#define SECTOR_SIZE		512
#define BYTES_PER_SECTOR	512
#define SECTORS_PER_CLUSTER	1
#define RESERVED_SECTORS	1
#define FAT_COPIES		2
#define FAT_SECTORS		16
#define ROOT_ENTRIES		16
#define ROOT_ENTRY_LENGTH	32

#define DATA_REGION_SECTOR	(RESERVED_SECTORS + FAT_COPIES*FAT_SECTORS + \
			(ROOT_ENTRIES * ROOT_ENTRY_LENGTH) / BYTES_PER_SECTOR)


uint8_t BootSector[] = {
	0xEB, 0x3C, 0x90,					// code to jump to the bootstrap code
	'm', 'k', 'd', 'o', 's', 'f', 's', 0x00,		// OEM ID
	WBVAL(BYTES_PER_SECTOR),				// bytes per sector
	SECTORS_PER_CLUSTER,					// sectors per cluster
	WBVAL(RESERVED_SECTORS),				// # of reserved sectors (1 boot sector)
	FAT_COPIES,						// FAT copies (2)
	WBVAL(ROOT_ENTRIES),					// root entries (512)
	WBVAL(DATA_REGION_SECTOR+1024),				// total number of sectors
	0xF8,							// media descriptor (0xF8 = Fixed disk)
	WBVAL(FAT_SECTORS),					// sectors per FAT (16)
	0x20, 0x00,						// sectors per track (32)
	0x40, 0x00,						// number of heads (64)
	0x00, 0x00, 0x00, 0x00,					// hidden sectors (0)
	0x00, 0x00, 0x00, 0x00,					// large number of sectors (0)
	0x00,							// drive number (0)
	0x00,							// reserved
	0x29,							// extended boot signature
	0x69, 0x17, 0xAD, 0x53,					// volume serial number
	'R', 'A', 'M', 'D', 'I', 'S', 'K', ' ', ' ', ' ', ' ',	// volume label
	'F', 'A', 'T', '1', '2', ' ', ' ', ' '			// filesystem type
};

uint8_t FatSector[] = {
	0xF8, 0xFF, 0xFF
};

static bool erase_flag = true;
struct target_s *curr_target;

int msc_flash_read(uint32_t lba, uint8_t *copy_to)
{
	memset(copy_to, 0, SECTOR_SIZE);
	if (lba == 0) {
		memcpy(copy_to, BootSector, sizeof(BootSector));
		copy_to[SECTOR_SIZE - 2] = 0x55;
		copy_to[SECTOR_SIZE - 1] = 0xAA;
	} else if (lba == RESERVED_SECTORS  ||  lba == RESERVED_SECTORS + FAT_SECTORS) {
		memcpy(copy_to, FatSector, sizeof(FatSector));
	}
	return 0;
}

int msc_flash_write(uint32_t lba, const uint8_t *copy_from)
{
	if (lba < DATA_REGION_SECTOR) {
		erase_flag = true;
	} else {
		if (!curr_target  ||  ! t->attached) {
			curr_target = target_attach_n(1, &msc_controller);
		}
		struct target_s *t = curr_target;
		if (t  && t->attached) {
			target_halt_request(t);
			while(target_halt_poll(t, NULL) == TARGET_HALT_RUNNING)
				;
			if (erase_flag) {
				target_command(t, "erase_mass");
				erase_flag = false;
			}
			target_target_flash_write(t, t->flash->start + lba*SECTOR_SIZE, copy_from, SECTOR_SIZE)
		}
	}
	return 0;
}

int msc_flash_blocks(void)
{
	return (BootSector[0x13]) + (BootSector[0x14]<<8) + (BootSector[0x15]<<16) + (BootSector[0x16]<<24);
}

static int msc_not_impl(struct target_controller *tc)
{
	tc->errno_ = -22;
	return -1;
}

static int msc_open(struct target_controller *tc,
	        target_addr path, size_t path_len,
                enum target_open_flags flags, mode_t mode)
{
	return msc_not_impl(tc);
}

static int msc_close(struct target_controller *tc, int fd)
{
	return msc_not_impl(tc);
}

static int msc_read(struct target_controller *tc,
	         int fd, target_addr buf, unsigned int count)
{
	return msc_not_impl(tc);
}

static int msc_write(struct target_controller *tc,
	          int fd, target_addr buf, unsigned int count)
{
	return msc_not_impl(tc);
}

static long msc_lseek(struct target_controller *tc,
	           int fd, long offset, enum target_seek_flag flag)
{
	return msc_not_impl(tc);
}

static int msc_rename(struct target_controller *tc,
	           target_addr oldpath, size_t old_len,
	           target_addr newpath, size_t new_len)
{
	return msc_not_impl(tc);
}

static int msc_unlink(struct target_controller *tc,
	           target_addr path, size_t path_len)
{
	return msc_not_impl(tc);
}

static int msc_stat(struct target_controller *tc,
	         target_addr path, size_t path_len, target_addr buf)
{
	return msc_not_impl(tc);
}

static int msc_fstat(struct target_controller *tc, int fd, target_addr buf)
{
	return msc_not_impl(tc);
}

static int msc_gettimeofday(struct target_controller *tc,
		         target_addr tv, target_addr tz)
{
	return msc_not_impl(tc);
}

static int msc_isatty(struct target_controller *tc, int fd)
{
	return msc_not_impl(tc);
}

static int msc_system(struct target_controller *tc,
	           target_addr cmd, size_t cmd_len)
{
	return msc_not_impl(tc);
}

static target *cur_target;
static target *last_target;

static void msc_target_destroy_callback(struct target_controller *tc, target *t)
{
	(void)tc;
	if (cur_target == t)
		cur_target = NULL;

	if (last_target == t)
		last_target = NULL;
}

static void msc_target_printf(struct target_controller *tc,
                              const char *fmt, va_list ap)
{
	(void)tc;
	(void)fmt;
	(void)ap;
}

static struct target_controller msc_controller = {
	.destroy_callback = msc_target_destroy_callback,
	.printf = msc_target_printf,

	.open = msc_open,
	.close = msc_close,
	.read = msc_read,
	.write = msc_write,
	.lseek = msc_lseek,
	.rename = msc_rename,
	.unlink = hostio_unlink,
	.stat = msc_stat,
	.fstat = msc_fstat,
	.gettimeofday = msc_gettimeofday,
	.isatty = msc_isatty,
	.system = msc_system,
};


int msc_flash_init(void)
{
	uint32_t size = 1;

	platform_srst_set_val(true);
	int devs = -1;
	volatile struct exception e;
	TRY_CATCH (e, EXCEPTION_ALL) {
		devs = adiv5_swdp_scan();
	}

	if(devs <= 0) {
		platform_srst_set_val(false);
		gdb_out("SW-DP scan failed!\n");
		return false;
	}

	curr_target = target_attach_n(1, &msc_controller);
	
	if (curr_target->flash) {
		size = (curr_target->flash->length + SECTOR_SIZE-1)/SECTOR_SIZE + DATA_REGION_SECTOR;
		BootSector[0x13]= size & 0xff;
		BootSector[0x14]= (size>>8) & 0xff;
		BootSector[0x15]= (size>>16) & 0xff;
		BootSector[0x16]= (size>>24) & 0xff;
	}
	erase_flag = true;

	return 0;
}
