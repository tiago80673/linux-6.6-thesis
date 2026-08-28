// SPDX-License-Identifier: GPL-2.0
/*
 * THESIS Copy-on-Access (CoA) correctness selftest.
 *
 * Verifies the custom fork path in mm/memory.c (copy_present_pte +
 * do_thesis_page). When a process opted in via prctl(PR_SET_THESIS_COA, mode)
 * (with a non-off algorithm) forks, the child's
 * PRIVATE anonymous PTEs are set PROT_NONE; the first *read* of such a page
 * traps into do_thesis_page(), which copies the page to the child's local
 * NUMA node. Shared and file-backed mappings must be left untouched.
 *
 * Two independent signals are checked:
 *
 *   EFFECT    -- where a page physically lives, queried with move_pages(2)
 *                (nodes == NULL => return the current node of each page).
 *                The unique signature of CoA is "a pure READ moves the page
 *                from the slow node to the fast node"; standard COW never
 *                copies on a read, so the page stays put.
 *
 *   MECHANISM -- the kernel exposes an exact counter of do_thesis_page()
 *                migrations at /sys/kernel/debug/thesis_coa_faults. Reading it
 *                before/after proves the handler actually ran (vs. some other
 *                code path coincidentally moving the page). Skipped if debugfs
 *                is unavailable.
 *
 * Test matrix:
 *   T1  armed   + private-anon, child READS   -> migrates to fast node (CoA)
 *   T2  unarmed + private-anon, child READS   -> stays on slow node (control)
 *   T3  armed   + SHARED-anon,  child WRITES  -> sharing intact, NOT migrated
 *
 * Requires CONFIG_NUMA with >= 2 nodes and NUMA_BALANCING off (so do_numa_page
 * cannot interfere). Runs as root (move_pages + mounting debugfs). Placement is
 * done by first-touch under CPU affinity -- deliberately NOT mbind, because an
 * MPOL_BIND policy on the region would also pin do_thesis_page's replacement
 * allocation and defeat the migration we are trying to observe.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <sys/mount.h>
#include <sys/syscall.h>

#include "../kselftest.h"

#define PAGE_SZ    4096
#define NPAGES     64
#define REGION_SZ  (NPAGES * PAGE_SZ)
#define SLOW_NODE  1            /* CXL / Optane proxy */
#define FAST_NODE  0            /* local DRAM         */

#define COUNTER_PATH "/sys/kernel/debug/thesis_coa_faults"
#define DEBUGFS_DIR  "/sys/kernel/debug"

/* CoA prctl + algorithm modes (must match the kernel headers). */
#define PR_SET_THESIS_COA  0x54484553   /* include/uapi/linux/prctl.h */
#define COA_OFF   0
#define COA_NAIVE 1

/* Select this process's CoA algorithm (per-task; inherited across fork; no root).
 * mode 0 disables. Returns 0 on success, -1 if the kernel lacks the prctl. */
static int arm_coa(int mode)
{
	return prctl(PR_SET_THESIS_COA, mode, 0, 0, 0);
}

/* Distinct byte patterns so we can tell parent fill from child overwrite. */
static inline unsigned char parent_byte(int i) { return (unsigned char)(0xA5 ^ (i * 7)); }
static inline unsigned char child_byte(int i)  { return (unsigned char)(0x5A ^ (i * 7)); }

/* Child -> parent results, lives in a SHARED mapping (never trapped by CoA). */
struct shared {
	int pages_fast;   /* region pages the child saw on FAST_NODE */
	int pages_slow;   /* region pages the child saw on SLOW_NODE */
	int bytes_ok;     /* region pages whose first byte matched parent_byte() */
};

/* --- raw syscall: query the NUMA node of a single page (no move) --- */
static int node_of(void *addr)
{
	void *pages[1] = { addr };
	int status[1] = { -1 };

	if (syscall(SYS_move_pages, 0, 1, pages, NULL, status, 0) != 0)
		return -1;
	return status[0];
}

/* First CPU of a NUMA node, from sysfs. Never assume CPU N == node N. */
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

/* Exact count of do_thesis_page() migrations; -1 if debugfs unavailable. */
static long read_counter(void)
{
	FILE *f = fopen(COUNTER_PATH, "r");
	int v = -1;

	if (!f) {
		mount("none", DEBUGFS_DIR, "debugfs", 0, NULL);
		f = fopen(COUNTER_PATH, "r");
	}
	if (!f)
		return -1;
	if (fscanf(f, "%d", &v) != 1)
		v = -1;
	fclose(f);
	return v;
}

/* mmap an anonymous region (private or shared) and first-touch it onto
 * SLOW_NODE via affinity, filling each page's first byte with parent_byte(). */
static unsigned char *make_region(int shared)
{
	int flags = MAP_ANONYMOUS | (shared ? MAP_SHARED : MAP_PRIVATE);
	unsigned char *r;
	int i;

	r = mmap(NULL, REGION_SZ, PROT_READ | PROT_WRITE, flags, -1, 0);
	if (r == MAP_FAILED)
		ksft_exit_fail_msg("mmap: %s\n", strerror(errno));

	pin_node(SLOW_NODE);                 /* first touch lands on the slow node */
	for (i = 0; i < NPAGES; i++)
		r[i * PAGE_SZ] = parent_byte(i);
	return r;
}

/* Child body for T1/T2: pin to the fast node, READ one byte per page (this is
 * what trips do_thesis_page when armed), then record each page's node + bytes. */
static void child_read_region(unsigned char *r, struct shared *sh)
{
	int i, fast = 0, slow = 0, ok = 0;

	pin_node(FAST_NODE);
	for (i = 0; i < NPAGES; i++) {
		volatile unsigned char c = r[i * PAGE_SZ];   /* read => CoA trap */
		int n;

		if (c == parent_byte(i))
			ok++;
		n = node_of(&r[i * PAGE_SZ]);
		if (n == FAST_NODE)
			fast++;
		else if (n == SLOW_NODE)
			slow++;
	}
	sh->pages_fast = fast;
	sh->pages_slow = slow;
	sh->bytes_ok = ok;
}

static int count_on_node(unsigned char *r, int node)
{
	int i, n = 0;

	for (i = 0; i < NPAGES; i++)
		if (node_of(&r[i * PAGE_SZ]) == node)
			n++;
	return n;
}

int main(void)
{
	struct shared *sh;
	unsigned char *r;
	long c0, c1;
	int i, on_slow, p_ok, p_slow, seen;
	pid_t pid;

	ksft_print_header();
	ksft_set_plan(9);

	if (first_cpu_of_node(SLOW_NODE) < 0 || first_cpu_of_node(FAST_NODE) < 0)
		ksft_exit_skip("needs >= 2 NUMA nodes (node %d and %d)\n", FAST_NODE, SLOW_NODE);

	sh = mmap(NULL, sizeof(*sh), PROT_READ | PROT_WRITE,
		  MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (sh == MAP_FAILED)
		ksft_exit_fail_msg("mmap scratch: %s\n", strerror(errno));

	/* ------------------------------------------------------------------ */
	/* T1: armed + private anon -- a child read must migrate to fast node. */
	/* ------------------------------------------------------------------ */
	r = make_region(0);
	on_slow = count_on_node(r, SLOW_NODE);
	ksft_test_result(on_slow == NPAGES,
			 "T1 setup: %d/%d private pages resident on slow node %d\n",
			 on_slow, NPAGES, SLOW_NODE);

	if (arm_coa(COA_NAIVE))                        /* opt in: naive CoA */
		ksft_exit_skip("prctl(PR_SET_THESIS_COA) unsupported (non-thesis kernel?)\n");
	c0 = read_counter();
	pid = fork();
	if (pid == 0) {
		child_read_region(r, sh);
		_exit(0);
	}
	waitpid(pid, NULL, 0);
	c1 = read_counter();

	ksft_test_result(sh->pages_fast == NPAGES,
			 "T1: child migrated %d/%d region pages to fast node %d (CoA fired)\n",
			 sh->pages_fast, NPAGES, FAST_NODE);
	ksft_test_result(sh->bytes_ok == NPAGES,
			 "T1: %d/%d migrated pages have correct bytes\n",
			 sh->bytes_ok, NPAGES);
	if (c0 < 0 || c1 < 0)
		ksft_test_result_skip("T1: debugfs counter unavailable\n");
	else
		ksft_test_result((c1 - c0) >= NPAGES,
				 "T1: do_thesis_page ran >= %d times (delta=%ld)\n",
				 NPAGES, c1 - c0);

	/* Parent's own copy must be untouched: original bytes, still slow node. */
	p_ok = 0;
	for (i = 0; i < NPAGES; i++)
		if (r[i * PAGE_SZ] == parent_byte(i))
			p_ok++;
	p_slow = count_on_node(r, SLOW_NODE);
	ksft_test_result(p_ok == NPAGES && p_slow == NPAGES,
			 "T1: parent data preserved (%d/%d bytes ok, %d/%d still on slow node)\n",
			 p_ok, NPAGES, p_slow, NPAGES);
	munmap(r, REGION_SZ);

	/* ------------------------------------------------------------------ */
	/* T2: control -- NOT armed; a child read must leave pages on slow.    */
	/* ------------------------------------------------------------------ */
	r = make_region(0);
	arm_coa(COA_OFF);                             /* opt out (disarmed) */
	c0 = read_counter();
	pid = fork();
	if (pid == 0) {
		child_read_region(r, sh);
		_exit(0);
	}
	waitpid(pid, NULL, 0);
	c1 = read_counter();

	ksft_test_result(sh->pages_slow == NPAGES,
			 "T2 control: unarmed child read left %d/%d pages on slow node (no migration)\n",
			 sh->pages_slow, NPAGES);
	if (c0 < 0 || c1 < 0)
		ksft_test_result_skip("T2 control: debugfs counter unavailable\n");
	else
		ksft_test_result((c1 - c0) == 0,
				 "T2 control: counter unchanged (delta=%ld)\n", c1 - c0);
	munmap(r, REGION_SZ);

	/* ------------------------------------------------------------------ */
	/* T3: armed + SHARED anon -- the fix. Shared pages must NOT be trapped */
	/* or copied; the child's writes stay visible to the parent.           */
	/* ------------------------------------------------------------------ */
	r = make_region(1);
	arm_coa(COA_NAIVE);                           /* opt in (armed) */
	pid = fork();
	if (pid == 0) {
		pin_node(FAST_NODE);
		for (i = 0; i < NPAGES; i++)
			r[i * PAGE_SZ] = child_byte(i);       /* write shared page */
		_exit(0);
	}
	waitpid(pid, NULL, 0);

	seen = 0;
	for (i = 0; i < NPAGES; i++)
		if (r[i * PAGE_SZ] == child_byte(i))
			seen++;
	ksft_test_result(seen == NPAGES,
			 "T3 shared: parent observed child's writes on %d/%d shared pages (sharing intact)\n",
			 seen, NPAGES);

	p_slow = count_on_node(r, SLOW_NODE);
	ksft_test_result(p_slow == NPAGES,
			 "T3 shared: %d/%d shared pages NOT migrated, still on slow node (fix verified)\n",
			 p_slow, NPAGES);
	munmap(r, REGION_SZ);

	ksft_finished();
	return 0;
}
