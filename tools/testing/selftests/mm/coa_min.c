// SPDX-License-Identifier: GPL-2.0
/* Minimal CoA reproducer. Forks COA_KIDS children SEQUENTIALLY (parent waits for
 * each). Child 0 is the copier; children 1+ must find the shared F in the family
 * table (non-copier path). Each child reads every page and checks it equals the
 * parent's byte; optionally writes (COW). Prints its node + result.
 * Usage: sudo COA_MODE=4 COA_KIDS=3 [COA_WRITE=1] ./coa_min */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <sys/syscall.h>

#define PAGE_SZ 4096
#define NPAGES  64
#define PR_SET_THESIS_COA 0x54484553

static int first_cpu_of_node(int node)
{
	char p[128]; int cpu = -1; FILE *f;
	snprintf(p, sizeof(p), "/sys/devices/system/node/node%d/cpulist", node);
	f = fopen(p, "r"); if (f) { if (fscanf(f, "%d", &cpu) != 1) cpu = -1; fclose(f); }
	return cpu;
}
static void pin(int node)
{
	int cpu = first_cpu_of_node(node);
	cpu_set_t s; CPU_ZERO(&s); CPU_SET(cpu < 0 ? 0 : cpu, &s);
	if (sched_setaffinity(0, sizeof(s), &s)) perror("setaffinity");
}
static int node_of(void *a)
{
	void *pg[1] = { a }; int st[1] = { -1 };
	if (syscall(SYS_move_pages, 0, 1, pg, NULL, st, 0)) return -2;
	return st[0];
}

int main(void)
{
	int mode = getenv("COA_MODE") ? atoi(getenv("COA_MODE")) : 4;
	int kids = getenv("COA_KIDS") ? atoi(getenv("COA_KIDS")) : 1;
	int wr   = getenv("COA_WRITE") ? 1 : 0;
	unsigned char *r = mmap(NULL, NPAGES * PAGE_SZ, PROT_READ | PROT_WRITE,
				MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
	int i, k, fail = 0;

	pin(1);					/* first-touch region on slow node */
	for (i = 0; i < NPAGES; i++)
		r[i * PAGE_SZ] = (unsigned char)(i + 1);

	if (prctl(PR_SET_THESIS_COA, mode, 0, 0, 0)) { perror("prctl"); return 2; }
	printf("mode=%d kids=%d write=%d\n", mode, kids, wr);

	int conc = getenv("COA_CONC") ? 1 : 0;	/* fork all at once (concurrent) */
	pid_t pids[64];

	for (k = 0; k < kids; k++) {
		pid_t pid = fork();
		if (pid == 0) {
			int bad = 0, cpu;
			pin(0);			/* promote to fast node */
			cpu = sched_getcpu();
			for (i = 0; i < NPAGES; i++) {
				if ((unsigned char)r[i * PAGE_SZ] != (unsigned char)(i + 1))
					bad++;
				if (wr) {
					r[i * PAGE_SZ + 1] = (unsigned char)(i ^ 0x5a);
					if ((unsigned char)r[i * PAGE_SZ] != (unsigned char)(i + 1))
						bad++;
				}
			}
			printf("  child %d: cpu=%d region_node=%d bad=%d%s\n",
			       k, cpu, node_of(&r[0]), bad, bad ? "  <-- WRONG" : "");
			fflush(stdout);
			_exit(bad ? 1 : 0);
		}
		pids[k] = pid;
		if (!conc) {				/* sequential: wait now */
			int st = 0; waitpid(pid, &st, 0);
			if (!WIFEXITED(st) || WEXITSTATUS(st)) fail++;
		}
	}
	if (conc)
		for (k = 0; k < kids; k++) {
			int st = 0; waitpid(pids[k], &st, 0);
			if (!WIFEXITED(st) || WEXITSTATUS(st)) fail++;
		}
	for (i = 0; i < NPAGES; i++)
		if (r[i * PAGE_SZ] != (unsigned char)(i + 1)) { fail++; break; }
	printf("result: %s (parent pristine=%s)\n", fail ? "FAIL" : "PASS",
	       fail ? "?" : "yes");
	return fail;
}
