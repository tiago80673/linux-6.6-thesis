// SPDX-License-Identifier: GPL-2.0
/*
 * THESIS V3 (SHARED_LAZY) sharing/dedup test.
 *
 * do_thesis_page_lazy() promotes each slow-tier shadow page ONCE and lets every
 * child map that single local copy read-only. This test proves the dedup that
 * the single-child coa_test cannot see: it forks NKIDS children that ALL read
 * the same private-anon region, keeps them alive simultaneously, and inspects
 * each child's physical page frame (PFN) for every offset via /proc/self/pagemap.
 *
 *   LAZY  (mode 4): all children share one physical page per offset
 *                   => distinct PFNs across the whole region ~= NPAGES,
 *                      node-0 growth ~= 1x region.
 *   NAIVE (mode 1): every child gets its own private copy
 *                   => distinct PFNs ~= NKIDS * NPAGES,
 *                      node-0 growth ~= NKIDS x region.
 *
 * Select the mode with COA_MODE (default 4). Needs root (pagemap PFNs), >= 2
 * NUMA nodes, and NUMA_BALANCING off. Children are pinned to the fast node so
 * their promoted copies land on node 0.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/prctl.h>

#include "../kselftest.h"

#define PAGE_SZ    4096
#define NPAGES     64
#define NKIDS      8
#define REGION_SZ  (NPAGES * PAGE_SZ)
#define SLOW_NODE  1
#define FAST_NODE  0

#define PR_SET_THESIS_COA  0x54484553
#define COA_NAIVE       1
#define COA_SHARED_LAZY 4

static int coa_mode(void)
{
	const char *e = getenv("COA_MODE");

	return e ? atoi(e) : COA_SHARED_LAZY;
}

static int arm_coa(int mode)
{
	return prctl(PR_SET_THESIS_COA, mode, 0, 0, 0);
}

static int first_cpu_of_node(int node)
{
	char path[128];
	int cpu = -1;
	FILE *f;

	snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/cpulist", node);
	f = fopen(path, "r");
	if (!f)
		return -1;
	if (fscanf(f, "%d", &cpu) != 1)
		cpu = -1;
	fclose(f);
	return cpu;
}

static void pin_node(int node)
{
	int cpu = first_cpu_of_node(node);
	cpu_set_t set;

	if (cpu < 0)
		ksft_exit_fail_msg("node %d has no CPU in sysfs\n", node);
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	if (sched_setaffinity(0, sizeof(set), &set) != 0)
		ksft_exit_fail_msg("sched_setaffinity(node %d): %s\n", node, strerror(errno));
}

/* Physical frame number backing a virtual address (0 if not present). Root only. */
static unsigned long pfn_of(int pmfd, void *addr)
{
	uint64_t val;
	off_t off = ((uintptr_t)addr / PAGE_SZ) * sizeof(uint64_t);

	if (pread(pmfd, &val, sizeof(val), off) != (ssize_t)sizeof(val))
		return 0;
	if (!(val & (1ULL << 63)))              /* page present bit */
		return 0;
	return val & ((1ULL << 55) - 1);        /* PFN in bits 0..54 */
}

static long node_memfree_kb(int node)
{
	char path[128], line[256];
	long kb = -1;
	FILE *f;

	snprintf(path, sizeof(path), "/sys/devices/system/node/node%d/meminfo", node);
	f = fopen(path, "r");
	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f)) {
		if (strstr(line, "MemFree:")) {
			sscanf(strstr(line, "MemFree:") + 8, "%ld", &kb);
			break;
		}
	}
	fclose(f);
	return kb;
}

static unsigned char parent_byte(int i) { return (unsigned char)(0xA5 ^ (i * 7)); }

struct sh {
	unsigned long pfn[NKIDS][NPAGES];
	int recorded;                 /* children that have filled their row */
	int bytes_ok[NKIDS];
};

int main(void)
{
	struct sh *s;
	unsigned char *r;
	int rel[2];                   /* release pipe: children block until parent closes */
	long free_before, free_after;
	int i, k, mode = coa_mode();

	ksft_print_header();
	ksft_set_plan(4);

	if (first_cpu_of_node(SLOW_NODE) < 0 || first_cpu_of_node(FAST_NODE) < 0)
		ksft_exit_skip("needs >= 2 NUMA nodes (%d and %d)\n", FAST_NODE, SLOW_NODE);

	s = mmap(NULL, sizeof(*s), PROT_READ | PROT_WRITE,
		 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (s == MAP_FAILED)
		ksft_exit_fail_msg("mmap scratch: %s\n", strerror(errno));
	if (pipe(rel) != 0)
		ksft_exit_fail_msg("pipe: %s\n", strerror(errno));

	/* Private-anon region, first-touched onto the slow node (the shadow S). */
	r = mmap(NULL, REGION_SZ, PROT_READ | PROT_WRITE,
		 MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
	if (r == MAP_FAILED)
		ksft_exit_fail_msg("mmap region: %s\n", strerror(errno));
	pin_node(SLOW_NODE);
	for (i = 0; i < NPAGES; i++)
		r[i * PAGE_SZ] = parent_byte(i);

	if (arm_coa(mode))
		ksft_exit_skip("prctl(PR_SET_THESIS_COA) unsupported (non-thesis kernel?)\n");

	free_before = node_memfree_kb(FAST_NODE);

	/* Fork NKIDS children; each reads the whole region (tripping CoA), records
	 * the PFN it now sees per offset, then blocks so all copies coexist. */
	for (k = 0; k < NKIDS; k++) {
		pid_t pid = fork();

		if (pid == 0) {
			int pmfd = open("/proc/self/pagemap", O_RDONLY);
			char c;
			int ok = 0;
			ssize_t rc;

			pin_node(FAST_NODE);
			for (i = 0; i < NPAGES; i++) {
				volatile unsigned char v = r[i * PAGE_SZ];  /* CoA trap */

				if (v == parent_byte(i))
					ok++;
				s->pfn[k][i] = pfn_of(pmfd, &r[i * PAGE_SZ]);
			}
			/* Optional write+verify: each child writes a byte per page (COW off
			 * the shared RO copy) and reads it back. Exercises the write path
			 * that BFS hits and the single-child test otherwise misses. */
			if (getenv("COA_WRITE")) {
				for (i = 0; i < NPAGES; i++) {
					r[i * PAGE_SZ + 1] = (unsigned char)(k * 13 + i);
					if ((unsigned char)r[i * PAGE_SZ + 1] !=
					    (unsigned char)(k * 13 + i))
						ok = -1;                /* COW readback wrong */
					if ((unsigned char)r[i * PAGE_SZ] != parent_byte(i))
						ok = -1;                /* clobbered read byte */
				}
			}
			s->bytes_ok[k] = ok;
			__sync_add_and_fetch(&s->recorded, 1);
			close(rel[1]);               /* child drops its write end */
			rc = read(rel[0], &c, 1);    /* blocks until parent closes rel[1] */
			(void)rc;
			_exit(0);
		}
	}
	/* Parent keeps rel[1] open so children stay blocked (all mappings live)
	 * until we have measured; closing it below releases them. */

	/* Wait until every child has faulted and recorded, so all mappings are live. */
	while (__sync_fetch_and_add(&s->recorded, 0) < NKIDS)
		sched_yield();

	free_after = node_memfree_kb(FAST_NODE);

	/* DIAG: per-child bytes_ok and the PFN each child saw at offsets 0 and 1. */
	for (k = 0; k < NKIDS; k++)
		ksft_print_msg("child %d: bytes_ok=%d pfn[0]=%#lx pfn[1]=%#lx\n",
			       k, s->bytes_ok[k], s->pfn[k][0], s->pfn[k][1]);

	/* 1. correctness: every child read the parent's bytes correctly. */
	{
		int all_ok = 1;

		for (k = 0; k < NKIDS; k++)
			if (s->bytes_ok[k] != NPAGES)
				all_ok = 0;
		ksft_test_result(all_ok, "all %d children read %d/%d correct bytes\n",
				 NKIDS, NPAGES, NPAGES);
	}

	/* 2. per-offset sharing: at each offset, do all children share one PFN? */
	{
		int shared_offsets = 0;

		for (i = 0; i < NPAGES; i++) {
			int same = 1;

			for (k = 1; k < NKIDS; k++)
				if (s->pfn[k][i] != s->pfn[0][i] || s->pfn[0][i] == 0)
					same = 0;
			if (same)
				shared_offsets++;
		}
		if (mode == COA_SHARED_LAZY)
			ksft_test_result(shared_offsets == NPAGES,
					 "LAZY: %d/%d offsets share one physical page across all children\n",
					 shared_offsets, NPAGES);
		else
			ksft_test_result(shared_offsets == 0,
					 "NAIVE: %d/%d offsets shared (expected 0 -- private copies)\n",
					 shared_offsets, NPAGES);
	}

	/* 3. total distinct physical frames across the whole region. */
	{
		unsigned long flat[NKIDS * NPAGES];
		int n = 0, distinct = 0;

		for (k = 0; k < NKIDS; k++)
			for (i = 0; i < NPAGES; i++)
				flat[n++] = s->pfn[k][i];
		for (i = 0; i < n; i++) {
			int seen = 0, j;

			for (j = 0; j < i; j++)
				if (flat[j] == flat[i]) {
					seen = 1;
					break;
				}
			if (!seen)
				distinct++;
		}
		/* lazy ~= NPAGES; naive ~= NKIDS*NPAGES. Use the midpoint as the gate. */
		if (mode == COA_SHARED_LAZY)
			ksft_test_result(distinct <= NPAGES + NPAGES / 2,
					 "LAZY: %d distinct frames for %d children x %d pages (ideal %d)\n",
					 distinct, NKIDS, NPAGES, NPAGES);
		else
			ksft_test_result(distinct >= NKIDS * NPAGES - NPAGES,
					 "NAIVE: %d distinct frames for %d children x %d pages (ideal %d)\n",
					 distinct, NKIDS, NPAGES, NKIDS * NPAGES);
	}

	/* 4. node-0 memory footprint of the promotion (informational gate). */
	{
		long used_kb = free_before - free_after;   /* +ve == memory consumed */
		long naive_kb = (long)NKIDS * NPAGES * PAGE_SZ / 1024;
		long lazy_kb = (long)NPAGES * PAGE_SZ / 1024;

		ksft_print_msg("node %d MemFree: %ld -> %ld kB (delta %ld kB); ideal lazy=%ld naive=%ld\n",
			       FAST_NODE, free_before, free_after, used_kb, lazy_kb, naive_kb);
		/*
		 * The dedup gate only holds when children READ the shared page. Under
		 * COA_WRITE every child writes every page, so each COWs a private copy
		 * (no sharing by design) and growth is ~naive -- skip the gate then.
		 */
		if (getenv("COA_WRITE"))
			ksft_test_result(1, "write workload: node-0 growth %ld kB "
					 "(full COW duplication expected)\n", used_kb);
		else if (mode == COA_SHARED_LAZY)
			ksft_test_result(used_kb < naive_kb / 2,
					 "LAZY: node-0 growth %ld kB well under naive %ld kB\n",
					 used_kb, naive_kb);
		else
			ksft_test_result(1, "NAIVE: node-0 growth %ld kB (baseline)\n", used_kb);
	}

	close(rel[1]);                  /* release children (their read() hits EOF) */
	close(rel[0]);
	for (k = 0; k < NKIDS; k++)
		wait(NULL);

	munmap(r, REGION_SZ);
	ksft_finished();
	return 0;
}
