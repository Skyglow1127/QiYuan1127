/* Short DDS discovery-only observer. No application publishers or writes. */
#define _POSIX_C_SOURCE 200809L
#include <dds/dds.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void) {
    if (setenv("CYCLONEDDS_URI", "<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface name=\"enp99s0\"/></Interfaces></General></Domain></CycloneDDS>", 1)) return 1;
    dds_entity_t participant = dds_create_participant(0, NULL, NULL);
    if (participant < 0) { fprintf(stderr,"participant: %s\n",dds_strretcode(-participant)); return 1; }
    dds_entity_t reader = dds_create_reader(participant, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, NULL, NULL);
    if (reader < 0) { fprintf(stderr,"reader: %s\n",dds_strretcode(-reader)); dds_delete(participant); return 1; }
    struct timespec start, now, delay = {0,100000000};
    clock_gettime(CLOCK_MONOTONIC, &start);
    int result=0;
    do {
        void *samples[32] = {0}; dds_sample_info_t info[32];
        int count=dds_take(reader,samples,info,32,32);
        if(count<0) { fprintf(stderr,"take: %s\n",dds_strretcode(-count)); result=1; break; }
        for(int i=0;i<count;i++) if(info[i].valid_data) {
            const dds_builtintopic_endpoint_t *e=samples[i];
            printf("topic=%s\ttype=%s\n",e->topic_name,e->type_name);
        }
        if(count>0) dds_return_loan(reader,samples,count);
        fflush(stdout);
        nanosleep(&delay,NULL);
        clock_gettime(CLOCK_MONOTONIC,&now);
    } while((now.tv_sec-start.tv_sec)+(now.tv_nsec-start.tv_nsec)/1e9<12);
    dds_delete(participant);
    return result;
}
