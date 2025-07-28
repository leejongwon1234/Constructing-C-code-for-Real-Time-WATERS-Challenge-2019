#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <sched.h>

// 설정 값
#define CPU_PERFORMANCE_Frequency 3.4      // Intel i7의 clock speed (GHz)
#define GPU_PERFORMANCE_Frequency 1.3      // NVIDIA GPU의 clock speed (GHz)
#define WCET_OVERRUN_PROBABILITY 0.001 // 실행시간이 WCET를 초과할 확률

// 환경 값
// Output data
// by SFM
#define Matrix_SFM_host_size_KB 24
#define Matrix_SFM_host_size_B (Matrix_SFM_host_size_KB * 1024)
#define OUTPUT_SIZE_B_bySFM (Matrix_SFM_host_size_B)

// execution
#define PERIOD_MS 33
#define PERIOD_US (PERIOD_MS * 1000)    // microseconds 단위로 변환
#define PERIOD_NS (PERIOD_MS * 1000000) // nanoseconds 단위로 변환
// SFM은 Preprocessing, SFM_Function, SFM_postprocessing 단계로 나뉨
// CPU
// SFM_preprocessing
#define PREPROCESS_EXEC_TICKS_LB 5878560
#define PREPROCESS_EXEC_TICKS_AVG 6977531
#define PREPROCESS_EXEC_TICKS_UB 7459318
#define PREPROCESS_EXEC_TIME_LB (PREPROCESS_EXEC_TICKS_LB / CPU_PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
#define PREPROCESS_EXEC_TIME_AVG (PREPROCESS_EXEC_TICKS_AVG / CPU_PERFORMANCE_Frequency) // nanoseconds 단위로 변환
#define PREPROCESS_EXEC_TIME_UB (PREPROCESS_EXEC_TICKS_UB / CPU_PERFORMANCE_Frequency)   // nanoseconds 단위
// SFM_postprocessing
#define POSTPROCESS_EXEC_TICKS_LB 6773920
#define POSTPROCESS_EXEC_TICKS_AVG 7213436
#define POSTPROCESS_EXEC_TICKS_UB 8347392
#define POSTPROCESS_EXEC_TIME_LB (POSTPROCESS_EXEC_TICKS_LB / CPU_PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
#define POSTPROCESS_EXEC_TIME_AVG (POSTPROCESS_EXEC_TICKS_AVG / CPU_PERFORMANCE_Frequency) // nanoseconds 단위로 변환
#define POSTPROCESS_EXEC_TIME_UB (POSTPROCESS_EXEC_TICKS_UB / CPU_PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
// GPU
// SFM_Function
#define FUNCTION_EXEC_TICKS_LB 10575000
#define FUNCTION_EXEC_TICKS_AVG 10800000
#define FUNCTION_EXEC_TICKS_UB 11850000
#define FUNCTION_EXEC_TIME_LB (FUNCTION_EXEC_TICKS_LB / GPU_PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
#define FUNCTION_EXEC_TIME_AVG (FUNCTION_EXEC_TICKS_AVG / GPU_PERFORMANCE_Frequency) // nanoseconds 단위로 변환
#define FUNCTION_EXEC_TIME_UB (FUNCTION_EXEC_TICKS_UB / GPU_PERFORMANCE_Frequency)   // nanoseconds 단위로 변환

// 메세지 OFFSET 설정
#define chain_type 3 //SFM은 Chain_type 3에 해당함
#define chain_level 3 //SFM은 Chain_level 3에 해당함
#define message_size_of_chain 256 //하나의 task chain마다 할당되는 message size 크기
#define message_size_of_task 64 //하나의 task마다 할당되는 message size 크기
#define message_format_unit 16 //task가 사용하는 Message의 format 단위 16bytes

#define offset (((chain_type-1)*message_size_of_chain) + ((chain_level-2)*message_size_of_task)) //SFM의 Message가 저장되는 offset

// Planner 서버 포트 (수신단)
#define Planner_SFM_PORT 5556

// --------------------------------runnable Thread 설정 -----------------
// 실행시간 난수 생성 함수
// C에서 rand()의 결과를 [0, 1) 실수로 변환
double rand_uniform()
{
    return ((double)rand()) / ((double)RAND_MAX + 1);
}
// exec_us 값을 생성하는 분포 함수
double rand_range(double min, double avg, double max)
{
    // 확률 비율 계산
    double p_avg = (max - avg) / (max - min);
    // uniform 확률 변수 [0,1)
    double x = rand_uniform();
    if (x < p_avg)
    {
        // [min, avg] 구간 uniform
        return min + rand_uniform() * (avg - min);
    }
    else if (x < 1.0 - WCET_OVERRUN_PROBABILITY)
    {
        // [avg, max] 구간 uniform
        return avg + rand_uniform() * (max - avg);
    }
    else
    {
        // WCET overrun: max를 초과하는 희박한 경우
        double u = rand_uniform();
        return max + (avg * pow(u, 2) / 10.0); // 지수적으로 줄어드는 확률
    }
}

//전송할 메세지의 구조체
//message_size_of_task 크기는 64bytes - 16bytes 단위로 분할
//[0-15]: ID //이 중에서 1byte만 사용
//[16-31]: 주기가 깨는 시점 //16bytes
//[32-47]: Task start 시점 //16bytes //다른 task 들 때문에, recv_time으로 표기
//[48-63]: Task Data 전송 시점 //16bytes //전송을 위해 data 준비하는 시점에서부터 시작
//message_format_unit을 사용 (16bytes)
//int64_t는 8bytes size
typedef struct {
    uint8_t id;
    
    int64_t wake_sec; 
    int64_t wake_nsec;

    int64_t recv_sec; 
    int64_t recv_nsec;

    int64_t send_sec; 
    int64_t send_nsec;

} TaskHeader;

// out의 필드에서 읽어 buffer에 offset 위치부터 채움
// buffer는 출력 버퍼, offset은 해당 TaskHeader의 시작 위치
// TaskHeader는 id,wake,start,end 필드를 각 16bytes로 가짐
// TaskHeader의 각 필드는 8bytes로 분리됨 (id를 제외하고, 8bytes씩 sec, nsec를 담당)
// 따라서 총 64bytes를 작성
void write_task_header(TaskHeader *out, char *buffer, int task_offset)
{
    //ID 작성
    memcpy(buffer + task_offset, &out->id, sizeof(uint8_t)); //1byte 만큼 ID 사용
    //Wake 작성
    memcpy(buffer + task_offset + (sizeof(int64_t)*2), &out->wake_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t)*3), &out->wake_nsec, sizeof(int64_t));
    //Start 작성
    memcpy(buffer + task_offset + (sizeof(int64_t)*4), &out->recv_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t)*5), &out->recv_nsec, sizeof(int64_t));
    //End 작성
    memcpy(buffer + task_offset + (sizeof(int64_t)*6), &out->send_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t)*7), &out->send_nsec, sizeof(int64_t));
}

void *runnable_thread(void *arg)
{
    char result[OUTPUT_SIZE_B_bySFM]; // 결과를 저장할 버퍼 // 24KB
    struct timespec next, start, send_time, end; //SFM은 Edge task: start = recv_time
    int last_SFM_id = 0; // 마지막 SFM ID를 저장할 변수

    // while문을 통해 running 이전에 Planner에 Client로써 연결시도
    int Planner_sock_SFM = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in Planner_addr_SFM;
    Planner_addr_SFM.sin_family = AF_INET;
    Planner_addr_SFM.sin_port = htons(Planner_SFM_PORT);
    inet_pton(AF_INET, "127.0.0.1", &Planner_addr_SFM.sin_addr); // localhost IP 주소로 설정
    // 반복 하여 Planner에 연결 시도
    while (connect(Planner_sock_SFM, (struct sockaddr *)&Planner_addr_SFM, sizeof(Planner_addr_SFM)) < 0)
    {
        printf("[SFM] Waiting for Planner...\n");
        sleep(1); // 1초 대기 후 재시도
    }
    printf("[SFM] Connected to Planner\n");

    clock_gettime(CLOCK_MONOTONIC, &next); // 주기를 위한 시간 측정

    while(1){
        //실제: 기상 |- 실행 ㅣ- 데이터 생성 ㅣ- 전송  
        //코드 구현: 기상 |- 실행(busy loop) - 데이터 생성(id++) |- 메세지 패킷 생성 완료 - 전송 | - debuging용 출력 | - 주기 계산
        //sfm은 edge task이므로 데이터 읽기가 없음

        //1.Setup phase: 기상, 데이터 읽기 완료(sfm은 edge task라 데이터 읽기 X) 
        clock_gettime(CLOCK_MONOTONIC, &start);
        double next_ms = next.tv_sec * 1000.0 + next.tv_nsec / 1.0e6;
        printf("[SFM] wake-up at %.3f ms\n", next_ms);
        double start_ms = start.tv_sec * 1000.0 + start.tv_nsec / 1.0e6;
        printf("[SFM] Started at %.3f ms\n", start_ms);
        // ------------------ setup phase 완료 ----------

        //2.Execution phase: busy-loop, 데이터 생성
        // SFM 실행 시간 계산 (busy-loop)
        // SFM preprocessing, SFM_Function, SFM_postprocessing 단계의 실행 시간을 시뮬레이션
        // SFM preprocessing
        double pre_exec_ns = rand_range(PREPROCESS_EXEC_TIME_LB, PREPROCESS_EXEC_TIME_AVG, PREPROCESS_EXEC_TIME_UB);
        double pre_exec_us = pre_exec_ns / 1000.0; // nanoseconds to microseconds
        struct timespec exec_now;
        do {
            clock_gettime(CLOCK_MONOTONIC, &exec_now);
            double elapsed_us = (exec_now.tv_sec - start.tv_sec) * 1e6 +
                                (exec_now.tv_nsec - start.tv_nsec) / 1e3;
            if (elapsed_us >= pre_exec_us)
                break;
        } while (1);
        // SFM Function
        double func_exec_ns = rand_range(FUNCTION_EXEC_TIME_LB, FUNCTION_EXEC_TIME_AVG, FUNCTION_EXEC_TIME_UB);
        double func_exec_us = func_exec_ns / 1000.0; // nanoseconds to microseconds
        usleep((useconds_t)func_exec_us); // SFM Function 실행 시간 시뮬레이션
        // SFM postprocessing
        double post_exec_ns = rand_range(POSTPROCESS_EXEC_TIME_LB, POSTPROCESS_EXEC_TIME_AVG, POSTPROCESS_EXEC_TIME_UB);
        double post_exec_us = post_exec_ns / 1000.0; // nanoseconds to microseconds
        struct timespec post_exec_start;
        clock_gettime(CLOCK_MONOTONIC, &post_exec_start);
        do {
            clock_gettime(CLOCK_MONOTONIC, &exec_now);
            double elapsed_us = (exec_now.tv_sec - post_exec_start.tv_sec) * 1e6 +
                                (exec_now.tv_nsec - post_exec_start.tv_nsec) / 1e3;
            if (elapsed_us >= post_exec_us)
                break;
        } while (1);

        // SFM 데이터 생성
        last_SFM_id++; // SFM ID 증가
        if (last_SFM_id > 255) last_SFM_id = 1; // ID가 255를 초과하면 0으로 초기화
         // --------------Execution phase 완료--

         //3.Send phase: data packet 생성 시작
         clock_gettime(CLOCK_MONOTONIC, &send_time);
         double send_time_ms = send_time.tv_sec * 1000.0 + send_time.tv_nsec / 1.0e6;
         printf("[SFM] send at %.3f ms\n", send_time_ms);
        //message packet 생성 시작
        TaskHeader sfm;
        sfm.id = last_SFM_id; // SFM ID 설정
        sfm.wake_sec = next.tv_sec; // 주기에 의해 깬 시점
        sfm.wake_nsec = next.tv_nsec; // 주기에 의해 깬 시점
        sfm.recv_sec = start.tv_sec; //Scheduling되어 실행되는 시점
        sfm.recv_nsec = start.tv_nsec; //Scheduling되어 실행되는 시점
        sfm.send_sec = send_time.tv_sec; // execution이 끝나는 시점 (=data 전송 시점)
        sfm.send_nsec = send_time.tv_nsec; // execution이 끝나는 시점 (=data 전송 시점)

        // SFM 결과를 result 버퍼에 작성
        write_task_header(&sfm, result, offset);

        // SFM 결과 전송
        ssize_t sent_bytes = send(Planner_sock_SFM, result, OUTPUT_SIZE_B_bySFM, 0);
        if (sent_bytes < 0) {
            perror("[SFM] send() failed");
            break; // 전송 실패 시 루프 종료
        }

        //4.log print phase
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000.0 + end.tv_nsec / 1.0e6;
        printf("[SFM] finished at %.3f ms\n", end_ms); // 끝난 시점 출력
        double exec_time_ms = (send_time.tv_sec - start.tv_sec) * 1000.0 + (send_time.tv_nsec - start.tv_nsec) / 1.0e6;
        printf("[SFM] Execution time: %.3f ms\n", exec_time_ms); // 실행시간 출력
        double resp_time_ms = (end.tv_sec - next.tv_sec) * 1000.0 + (end.tv_nsec - next.tv_nsec) / 1.0e6;
        printf("[SFM] Response time: %.3f ms\n", resp_time_ms); // 반응시간 출력

        printf("[SFM] send data value: SFM = %d\n", sfm.id); //생성 data id 출력
        double post_exec_time_ms = (send_time.tv_sec - post_exec_start.tv_sec) * 1000.0 + (send_time.tv_nsec - post_exec_start.tv_nsec) / 1.0e6;
        printf("[SFM] Preprocessing time: %.3f ms\n", pre_exec_ns / 1e6); // SFM preprocessing 시간 출력
        printf("[SFM] Function time: %.3f ms\n", func_exec_ns / 1e6); // SFM Function 시간 출력
        printf("[SFM] Postprocessing: %.3f ms\n", post_exec_time_ms); // SFM postprocessing 시간 출력
        printf("[SFM] Sleeping for %d ms\n\n", PERIOD_MS); // 다음 주기까지 대기 시간 출력
        
        //5.next period cal phase
        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000)
        {
            next.tv_sec += 1;
            next.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL); 
    }
    return NULL;
}
void bind_process_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
        perror("sched_setaffinity");
        exit(EXIT_FAILURE);
    }
}

// ------------------------------
// 메인 함수
// ------------------------------
int main()
{
    bind_process_to_core(3);
    srand(time(NULL)); // 난수 초기화
    pthread_t runnable_tid;

    // runnable_thread 생성
    pthread_create(&runnable_tid, NULL, runnable_thread, NULL);

    // runnable_thread 종료 대기
    pthread_join(runnable_tid, NULL);

    return 0;
}