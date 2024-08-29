/*
 * xmmap_user.c - This Xenomai-3.x userspace app is linked to rtdm and 
 * using mmap() can view shared memory created by the kernel module xmmap_module.c
 *
 * Copyright (c) 2021 Konstantin Smola <k@smo.la>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * compile this way: gcc -g -Wall -I.. -Wl,--no-as-needed
 * -Wl,@/usr/xenomai/lib/cobalt.wrappers -Wl,@/usr/xenomai/lib/modechk.wrappers 
 * /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld
 * -L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt -lfuse -pthread -o xmmap_user xmmap_user.c 
 */

#include <sys/mman.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "xmmap.h"   /* common memory layout definition, etc */

int main()
{
	int xmmap_fd = -1;

	shm_test_layout_t *xmmap_shmem_data_p;
	size_t _shmem_size;

	_shmem_size = sizeof(shm_test_layout_t);

	printf("Original raw size of shmem segment is: %lu \n", (long unsigned)_shmem_size);

	xmmap_fd = open("/dev/rtdm/xeno_mmap0", O_RDWR);

	if (xmmap_fd > 0) {
		printf("Opened device OK. file descriptor is: %d\n", xmmap_fd);
		printf("Attempting to mmap() the fd...\n");
		xmmap_shmem_data_p =
		    (shm_test_layout_t *) mmap(0, _shmem_size, PROT_READ | PROT_WRITE, MAP_SHARED, xmmap_fd, 0);
		if (xmmap_shmem_data_p == MAP_FAILED) {
			perror("mmap failed. Size of shared memory wrong or improper permissions");
		} else {
			printf("Shared memory address is : %p\n", xmmap_shmem_data_p);
			printf("Shared memory 0: %c\n", (char)xmmap_shmem_data_p->data[0]);
			printf("Shared memory 1: %c\n", (char)xmmap_shmem_data_p->data[1]);
			printf("Shared memory 2: %c\n", (char)xmmap_shmem_data_p->data[2]);
			printf("Shared memory 3: %c\n", (char)xmmap_shmem_data_p->data[3]);
			printf("Shared float value: %f\n", xmmap_shmem_data_p->dval);
		}
	} else {
		perror("Failed to get a file descriptor");
	}

	if (xmmap_fd > 0)
		close(xmmap_fd);

	return 0;
}
