/*
 * Copyright (C) 2019 Paul Cercueil <paul@crapouillou.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <unistd.h>

#include "psxhw.h"
#include "psxmem.h"
#include "r3000a.h"

#define ARRAY_SIZE(a) (sizeof(a) ? (sizeof(a) / sizeof((a)[0])) : 0)

#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000
#endif

static const uintptr_t supported_io_bases[] = {
	0x0,
	0x10000000,
	0x80000000,
};

int lightrec_init_mmap()
{
	unsigned int i, j;
	void *map, *base;
	int err, memfd;

	memfd = shm_open("/lightrec_memfd",
			 O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
	if (memfd < 0) {
		err = -errno;
		fprintf(stderr, "Failed to create SHM: %d\n", err);
		return err;
	}

	err = ftruncate(memfd, 0x290000);
	if (err < 0) {
		err = -errno;
		fprintf(stderr, "Could not trim SHM: %d\n", err);
		goto err_close_memfd;
	}

	for (i = 0; i < ARRAY_SIZE(supported_io_bases); i++) {
		for (j = 0; j < 4; j++) {
			base = (void *)(supported_io_bases[i] + j * 0x200000);

			map = mmap(base, 0x200000, PROT_READ | PROT_WRITE,
				   MAP_SHARED | MAP_FIXED_NOREPLACE, memfd, 0);
			if (map == MAP_FAILED)
				break;
		}

		/* Impossible to map using this base */
		if (j == 0)
			continue;

		/* All mirrors mapped - we got a match! */
		if (j == 4)
			break;

		/* Only some mirrors mapped - clean the mess and try again */
		for (; j > 0; j--) {
			base = (void *)(supported_io_bases[i] + (j - 1) * 0x200000);

			munmap(base, 0x200000);
		}
	}

	if (i == ARRAY_SIZE(supported_io_bases)) {
		err = -EINVAL;
		fprintf(stderr, "Unable to mmap RAM and mirrors\n");
		goto err_close_memfd;
	}

	psxM = (s8 *)supported_io_bases[i];

	for (i = 0; i < ARRAY_SIZE(supported_io_bases); i++) {
		base = (void *)(supported_io_bases[i] + 0x1fc00000);

		map = mmap(base, 0x80000, PROT_READ | PROT_WRITE,
			   MAP_PRIVATE | MAP_FIXED_NOREPLACE, memfd, 0x200000);
		if (map != MAP_FAILED)
			break;
	}

	if (i == ARRAY_SIZE(supported_io_bases)) {
		err = -EINVAL;
		fprintf(stderr, "Unable to mmap BIOS\n");
		goto err_unmap;
	}

	psxR = (s8 *)(supported_io_bases[i] + 0x1fc00000);

	for (i = 0; i < ARRAY_SIZE(supported_io_bases); i++) {
		base = (void *)(supported_io_bases[i] + 0x1f800000);

		map = mmap(base, 0x10000, PROT_READ | PROT_WRITE,
			   MAP_PRIVATE | MAP_FIXED_NOREPLACE, memfd, 0x280000);
		if (map != MAP_FAILED)
			break;
	}

	if (i == ARRAY_SIZE(supported_io_bases)) {
		err = -EINVAL;
		fprintf(stderr, "Unable to mmap scratchpad\n");
		goto err_unmap_bios;
	}

	psxH = (s8 *)(supported_io_bases[i] + 0x1f800000);

	close(memfd);
	return 0;

err_unmap_bios:
	munmap(psxR, 0x80000);
err_unmap:
	for (j = 0; j < 4; j++)
		munmap((void *)((uintptr_t)psxM + j * 0x200000), 0x200000);
err_close_memfd:
	close(memfd);
	return err;
}

void lightrec_free_mmap()
{
	unsigned int i;

	munmap(psxH, 0x10000);
	munmap(psxR, 0x80000);
	for (i = 0; i < 4; i++)
		munmap((void *)((uintptr_t)psxM + i * 0x200000), 0x200000);
}
