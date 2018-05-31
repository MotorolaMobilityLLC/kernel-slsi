#include "debug-snapshot-log.h"

struct dbg_snapshot_log *p;

int main(int argc, char *argv[])
{
	FILE *f;
	int ch;
	long fsize;
	char *string;
	int i, j;

	f = fopen(argv[1], "rb");
	if (f == NULL) {
		fputs("file read error!", stderr);
		exit(1);
	}
	fseek(f, 0, SEEK_END);
	fsize = ftell(f);
	fseek(f, 0, SEEK_SET);  //same as rewind(f);

	string = malloc(fsize + 1);
	fread(string, fsize, 1, f);
	fclose(f);
	p = (struct dbg_snapshot_log *)string;

	printf("log = {}\n");
	for (i = 0; i < DSS_NR_CPUS; i++) {
		for (j = 0; j < DSS_LOG_MAX_NUM; j++) {
			printf("log[%.9f] = { 'type' : 'sched', 'cpu' : %d, 'comm' : '%s', 'pid' : %d}\n",
					p->task[i][j].time/1.0e9,
					i,
					p->task[i][j].task_comm,
					p->task[i][j].pid);
			if (p->task[i][j].time == 0)
				break;
		}
	}

	for (i = 0; i < DSS_LOG_MAX_NUM; i++) {
		printf("log[%.9f] = {  'type' : 'freq', 'cluster' : %d, 'freq' : %lu }\n",
				p->freq[i].time/1.0e9,
				p->freq[i].type,
				p->freq[i].target_freq);
		if (p->freq[i].time == 0)
			break;
	}

	for (i = 0; i < DSS_NR_CPUS; i++) {
		for (j = 0; j < DSS_LOG_MAX_NUM; j++) {
			printf("log[%.9f] = { 'type' : 'irq', 'cpu' : %d, 'num' : %d,"
					"'en' : %d, 'func' : '%p'}\n",
					p->irq[i][j].time/1.0e9,
					i,
					p->irq[i][j].irq,
					p->irq[i][j].en,
					p->irq[i][j].fn);
			if (p->irq[i][j].time == 0)
				break;
		}
	}

	for (i = 0; i < DSS_NR_CPUS; i++) {
		for (j = 0; j < DSS_LOG_MAX_NUM; j++) {
			printf("log[%.9f] = { 'type' : 'cpuidle', 'cpu' : %d, 'state' : %d}\n",
					p->cpuidle[i][j].time/1.0e9,
					i,
					p->cpuidle[i][j].state);
			if (p->cpuidle[i][j].time == 0)
				break;
		}
	}
	return 0;
}

