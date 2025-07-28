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
#define PERFORMANCE_Frequency 3.4      // Intel i7의 clock speed (GHz)
#define WCET_OVERRUN_PROBABILITY 0.001 // 실행시간이 WCET를 초과할 확률

// 환경 값
// input_data
// by planner
#define speed_object_size_KB 1
#define speed_object_size_B (speed_object_size_KB * 1024)
#define steer_object_size_KB 1
#define steer_object_size_B (steer_object_size_KB * 1024)
// DASM은 Planner Function으로부터 speed와 steer를 수신
// 실제로는 Time stamping을 위한 message format만 정의되어 있음
#define INPUT_SIZE_B_byplanner (speed_object_size_B + steer_object_size_B)

// execution
#define PERIOD_MS 5
#define PERIOD_US (PERIOD_MS * 1000)    // microseconds 단위로 변환
#define PERIOD_NS (PERIOD_MS * 1000000) // nanoseconds 단위로 변환
#define EXEC_TICKS_LB 2599990
#define EXEC_TICKS_AVG 3219990
#define EXEC_TICKS_UB 3719990
#define EXEC_TIME_LB (EXEC_TICKS_LB / PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
#define EXEC_TIME_AVG (EXEC_TICKS_AVG / PERFORMANCE_Frequency) // nanoseconds 단위로 변환
#define EXEC_TIME_UB (EXEC_TICKS_UB / PERFORMANCE_Frequency)   // nanoseconds 단위로 변환

// 메세지 OFFSET 설정
#define chain_type_1 1
#define chain_type_2 2
#define chain_type_3 3
#define chain_type_4 4
#define chain_type_5 5
#define chain_level 1
#define message_size_of_chain 256 // 하나의 task chain마다 할당되는 message size 크기
#define message_size_of_task 64   // 하나의 task마다 할당되는 message size 크기
#define message_format_unit 16    // task가 사용하는 Message의 format 단위 16bytes
// 메세지 offset
#define chain1_offset ((chain_type_1 - 1) * message_size_of_chain)
#define chain2_offset ((chain_type_2 - 1) * message_size_of_chain)
#define chain3_offset ((chain_type_3 - 1) * message_size_of_chain)
#define chain4_offset ((chain_type_4 - 1) * message_size_of_chain)
#define chain5_offset ((chain_type_5 - 1) * message_size_of_chain)

// 전역 변수
char input_buffer_byplanner[INPUT_SIZE_B_byplanner]; // 2048bytes
pthread_mutex_t buffer_lock = PTHREAD_MUTEX_INITIALIZER;

// Copy Thread: Planner로부터 데이터 수신

#define DASM_PORT 5555 // DASM 서버 포트

void *copy_thread(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char local_copy[INPUT_SIZE_B_byplanner];

    // 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("[DASM] socket error");
        exit(EXIT_FAILURE);
    }

    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DASM_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // 바인드
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("[DASM] bind error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    // 리슨
    if (listen(server_sock, 5) < 0)
    {
        perror("[DASM] listen error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("[DASM] Waiting for connection on port %d...\n", DASM_PORT);

    // 클라이언트 연결 수락
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0)
    {
        perror("[DASM] accept error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    // 해당 위치 이후부터, copy_thread 함수는 Planner로부터 데이터를 수신하여 input_buffer_byplanner에 저장하는 역할을 수행함
    // 통신은 TCP 소켓을 사용하며, 클라이언트가 연결되면 데이터를 수신하여 전역 변수에 저장함
    printf("[DASM] Connected to Planner: %s\n", inet_ntoa(client_addr.sin_addr));

    while (1)
    {
        ssize_t received = recv(client_sock, local_copy, INPUT_SIZE_B_byplanner, MSG_WAITALL);
        if (received <= 0)
        {
            perror("[DASM] recv error or connection closed\n");
            break;
        }

        pthread_mutex_lock(&buffer_lock);
        memcpy(input_buffer_byplanner, local_copy, INPUT_SIZE_B_byplanner);
        pthread_mutex_unlock(&buffer_lock);
    }
    close(client_sock);
    close(server_sock);
    return NULL;
}

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

// 메세지 구조체
// message_size_of_task 크기는 64bytes - 16bytes 단위로 분할
//[0-15]: ID //이 중에서 1byte만 사용
//[16-31]: 주기가 깨는 시점 //16bytes
//[32-47]: Task start(데이터를 모두 수신완료한 recv_time) 시점 //16bytes (recv_time)
//[48-63]: Task send 시점(= Data 전송 시점) //16bytes (send_time)
// message_format_unit을 사용 (16bytes)
// int64_t는 8bytes size
typedef struct __attribute__((packed)) 
{
    uint8_t chain_l2_id;
    int64_t chain_l2_wake_sec;
    int64_t chain_l2_wake_nsec;
    int64_t chain_l2_recv_sec;
    int64_t chain_l2_recv_nsec;
    int64_t chain_l2_send_sec;
    int64_t chain_l2_send_nsec;

    uint8_t chain_l3_id;
    int64_t chain_l3_wake_sec;
    int64_t chain_l3_wake_nsec;
    int64_t chain_l3_recv_sec;
    int64_t chain_l3_recv_nsec;
    int64_t chain_l3_send_sec;
    int64_t chain_l3_send_nsec;

    uint8_t chain_l4_id;
    int64_t chain_l4_wake_sec;
    int64_t chain_l4_wake_nsec;
    int64_t chain_l4_recv_sec;
    int64_t chain_l4_recv_nsec;
    int64_t chain_l4_send_sec;
    int64_t chain_l4_send_nsec;

    uint8_t chain_l5_id;
    int64_t chain_l5_wake_sec;
    int64_t chain_l5_wake_nsec;
    int64_t chain_l5_recv_sec;
    int64_t chain_l5_recv_nsec;
    int64_t chain_l5_send_sec;
    int64_t chain_l5_send_nsec;

} TaskHeader;

// out의 필드에 buffer에서 offset 위치부터 읽어와서 채움
// buffer는 입력 버퍼, offset은 해당 TaskHeader의 시작 위치
void parse_task_header(TaskHeader *out, const char *buffer, int offset)
{
    memcpy(&out->chain_l2_id, buffer + offset, sizeof(uint8_t));
    memcpy(&out->chain_l2_wake_sec, buffer + offset + (sizeof(int64_t) * 2), sizeof(int64_t));
    memcpy(&out->chain_l2_wake_nsec, buffer + offset + (sizeof(int64_t) * 3), sizeof(int64_t));
    memcpy(&out->chain_l2_recv_sec, buffer + offset + (sizeof(int64_t) * 4), sizeof(int64_t));
    memcpy(&out->chain_l2_recv_nsec, buffer + offset + (sizeof(int64_t) * 5), sizeof(int64_t));
    memcpy(&out->chain_l2_send_sec, buffer + offset + (sizeof(int64_t) * 6), sizeof(int64_t));
    memcpy(&out->chain_l2_send_nsec, buffer + offset + (sizeof(int64_t) * 7), sizeof(int64_t));

    memcpy(&out->chain_l3_id, buffer + offset + (sizeof(int64_t) * 8), sizeof(uint8_t));
    memcpy(&out->chain_l3_wake_sec, buffer + offset + (sizeof(int64_t) * 10), sizeof(int64_t));
    memcpy(&out->chain_l3_wake_nsec, buffer + offset + (sizeof(int64_t) * 11), sizeof(int64_t));
    memcpy(&out->chain_l3_recv_sec, buffer + offset + (sizeof(int64_t) * 12), sizeof(int64_t));
    memcpy(&out->chain_l3_recv_nsec, buffer + offset + (sizeof(int64_t) * 13), sizeof(int64_t));
    memcpy(&out->chain_l3_send_sec, buffer + offset + (sizeof(int64_t) * 14), sizeof(int64_t));
    memcpy(&out->chain_l3_send_nsec, buffer + offset + (sizeof(int64_t) * 15), sizeof(int64_t));

    memcpy(&out->chain_l4_id, buffer + offset + (sizeof(int64_t) * 16), sizeof(uint8_t));
    memcpy(&out->chain_l4_wake_sec, buffer + offset + (sizeof(int64_t) * 18), sizeof(int64_t));
    memcpy(&out->chain_l4_wake_nsec, buffer + offset + (sizeof(int64_t) * 19), sizeof(int64_t));
    memcpy(&out->chain_l4_recv_sec, buffer + offset + (sizeof(int64_t) * 20), sizeof(int64_t));
    memcpy(&out->chain_l4_recv_nsec, buffer + offset + (sizeof(int64_t) * 21), sizeof(int64_t));
    memcpy(&out->chain_l4_send_sec, buffer + offset + (sizeof(int64_t) * 22), sizeof(int64_t));
    memcpy(&out->chain_l4_send_nsec, buffer + offset + (sizeof(int64_t) * 23), sizeof(int64_t));

    memcpy(&out->chain_l5_id, buffer + offset + (sizeof(int64_t) * 24), sizeof(uint8_t));
    memcpy(&out->chain_l5_wake_sec, buffer + offset + (sizeof(int64_t) * 26), sizeof(int64_t));
    memcpy(&out->chain_l5_wake_nsec, buffer + offset + (sizeof(int64_t) * 27), sizeof(int64_t));
    memcpy(&out->chain_l5_recv_sec, buffer + offset + (sizeof(int64_t) * 28), sizeof(int64_t));
    memcpy(&out->chain_l5_recv_nsec, buffer + offset + (sizeof(int64_t) * 29), sizeof(int64_t));
    memcpy(&out->chain_l5_send_sec, buffer + offset + (sizeof(int64_t) * 30), sizeof(int64_t));
    memcpy(&out->chain_l5_send_nsec, buffer + offset + (sizeof(int64_t) * 31), sizeof(int64_t));
}

// 새로운 task의 ID가 이전과 다를 때만 출력
// print_e2e_if_new 함수는 명확히 End-to-End latency를 출력하는 함수임.
// task_name은 해당 task의 이름, task는 TaskHeader 구조체 포인터, last_id는 이전 ID를 저장하는 포인터, end는 현재 시각을 저장하는 timespec 구조체 포인터
// 이 함수는 task의 ID가 이전 ID와 다를 때만 End-to-End latency를 출력함.
// End-to-End latency는 task가 시작된 시각과 현재 시각의 차이를 계산하여 마이크로초 단위로 출력함.
// task_name은 "SFM", "Lane", "Detection", "Lidar", "CAN" 등으로 사용됨.
void print_log_if_new(const char *chain_name, TaskHeader *chain, int *last_id, struct timespec *wake, struct timespec *recv_time, struct timespec *end, int chain_level_size)
{
    // chain_level_size에 해당하는 Task의 ID 읽기 TaskHeader chain의 id를 읽어야함
    int id;
    if (chain_level_size == 3)
    {
        id = chain->chain_l3_id; // 새로받은 값들의 id
        if (id != *last_id)
        { // 변경됐네.
            *last_id = id;
            // log를 출력해야겠다.
            double chain_l3_wake_us = chain->chain_l3_wake_sec * 1000000.0 + chain->chain_l3_wake_nsec / 1.0e3;
            double chain_l3_start_us = chain->chain_l3_recv_sec * 1000000.0 + chain->chain_l3_recv_nsec / 1.0e3;
            double chain_l3_send_us = chain->chain_l3_send_sec * 1000000.0 + chain->chain_l3_send_nsec / 1.0e3;
            double chain_l2_wake_us = chain->chain_l2_wake_sec * 1000000.0 + chain->chain_l2_wake_nsec / 1.0e3;
            double chain_l2_recv_us = chain->chain_l2_recv_sec * 1000000.0 + chain->chain_l2_recv_nsec / 1.0e3;
            double chain_l2_send_us = chain->chain_l2_send_sec * 1000000.0 + chain->chain_l2_send_nsec / 1.0e3;
            double chain_l1_wake_us = wake->tv_sec * 1000000.0 + wake->tv_nsec / 1.0e3;
            double chain_l1_recv_us = recv_time->tv_sec * 1000000.0 + recv_time->tv_nsec / 1.0e3;
            double chain_l1_end_us = end->tv_sec * 1000000.0 + end->tv_nsec / 1.0e3;
            // 파일 출력
            char filename[64];
            snprintf(filename, sizeof(filename), "log_%s_tcp.txt", chain_name);
            FILE *fp = fopen(filename, "a");
            if (fp != NULL)
            {
                fprintf(fp, "ID = %d, chain_l3_wake_us = %.2f us\n", id, chain_l3_wake_us);
                fprintf(fp, "ID = %d, chain_l3_start_us = %.2f us\n", id, chain_l3_start_us);
                fprintf(fp, "ID = %d, chain_l3_send_us = %.2f us\n", id, chain_l3_send_us);
                //fprintf(fp, "ID = %d, chain_l2_wake_us = %.2f us\n", id, chain_l2_wake_us);
                fprintf(fp, "ID = %d, chain_l2_recv_us = %.2f us\n", id, chain_l2_recv_us);
                fprintf(fp, "ID = %d, chain_l2_send_us = %.2f us\n", id, chain_l2_send_us);
                //fprintf(fp, "ID = %d, chain_l1_wake_us = %.2f us\n", id, chain_l1_wake_us);
                fprintf(fp, "ID = %d, chain_l1_recv_us = %.2f us\n", id, chain_l1_recv_us);
                fprintf(fp, "ID = %d, chain_l1_end_us = %.2f us\n\n", id, chain_l1_end_us);
                fclose(fp);
            }
            else
            {
                perror("[DASM] Failed to open dasm_log.txt");
            }
        }
    }
    else if (chain_level_size == 5)
    {
        id = chain->chain_l5_id;
        if (id != *last_id)
        {
            // 차후 작성
            *last_id = id;
            printf("id 비교에서 문제 발생1");
        }
    }
    else
    {
        printf("id 비교에서 문제 발생2");
    }
}

void *runnable_thread(void *arg)
{
    char local_copy[INPUT_SIZE_B_byplanner];
    struct timespec next, start, recv_time, send_time, end; // dasm의 send_time은 output time

    int last_Lidar_grabber_id = -1;
    int last_CAN_id = -1;
    int last_SFM_id = -1;
    int last_Lane_detection_id = -1;
    int last_Detection_id = -1;

    clock_gettime(CLOCK_MONOTONIC, &next);

    while (1)
    {
        // 1. Setup phase: 기상, 데이터 읽기 완료
        // 기상
        clock_gettime(CLOCK_MONOTONIC, &start);
        double next_ms = next.tv_sec * 1000.0 + next.tv_nsec / 1.0e6;
        printf("[DASM] wake-up at %.3f ms\n", next_ms);
        double start_ms = start.tv_sec * 1000.0 + start.tv_nsec / 1.0e6;
        printf("[DASM] Started at %.3f ms\n", start_ms);

        pthread_mutex_lock(&buffer_lock);
        memcpy(local_copy, input_buffer_byplanner, INPUT_SIZE_B_byplanner);
        pthread_mutex_unlock(&buffer_lock);

        // for (int i = chain1_offset + 64; i < chain1_offset + 128; i++) {
        //     printf("%02X ", (unsigned char)local_copy[i]);
        //     if ((i - (chain1_offset + 64) + 1) % 16 == 0) printf("\n");  // 16바이트마다 줄바꿈
        // }
        // for (int i = chain3_offset + 64; i < chain3_offset + 128; i++) {
        //     printf("%02X ", (unsigned char)local_copy[i]);
        //     if ((i - (chain3_offset + 64) + 1) % 16 == 0) printf("\n");  // 16바이트마다 줄바꿈
        // }

        clock_gettime(CLOCK_MONOTONIC, &recv_time);
        double recv_time_ms = recv_time.tv_sec * 1000.0 + recv_time.tv_nsec / 1.0e6;
        printf("[DASM] received at %.3f ms\n", recv_time_ms);
        // ------------------ setup phase 완료 ----------
        // 2. Execution phase: Data 읽기 및 설정, busy-loop

        // Data 읽기 및 설정
        // TaskHeader 구조체를 사용하여 각 task의 헤더를 파싱: 읽기
        // local_copy는 input_buffer의 복사본으로, 각 task의 헤더를 읽어오기 위해 사용됨
        TaskHeader chain1_r, chain2_r, chain3_r, chain4_r, chain5_r;
        parse_task_header(&chain1_r, local_copy, chain1_offset);
        parse_task_header(&chain2_r, local_copy, chain2_offset);
        parse_task_header(&chain3_r, local_copy, chain3_offset);
        parse_task_header(&chain4_r, local_copy, chain4_offset);
        parse_task_header(&chain5_r, local_copy, chain5_offset);

        //chain1 debugging 용
        printf("%d\n", chain1_r.chain_l3_id);
        //chain2 debugging 용
        printf("%d\n", chain2_r.chain_l5_id);
        //chain3 debugging 용
        printf("%d\n", chain3_r.chain_l3_id);
        //chain4 debugging 용
        printf("%d\n", chain4_r.chain_l3_id);
        //chain5 debugging 용
        printf("%d\n", chain5_r.chain_l3_id);

        // busy-loop
        double exec_ns = rand_range(EXEC_TIME_LB, EXEC_TIME_AVG, EXEC_TIME_UB);
        double exec_us = exec_ns / 1000.0;
        struct timespec exec_now;
        do
        {
            clock_gettime(CLOCK_MONOTONIC, &exec_now);
            double elapsed_us = (exec_now.tv_sec - recv_time.tv_sec) * 1e6 +
                                (exec_now.tv_nsec - recv_time.tv_nsec) / 1e3;
            if (elapsed_us >= exec_us)
                break;
        } while (1);

        // 3. Send phase: DASM은 End task이므로 해당 phase 없음
        // 4. log print phase
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000.0 + end.tv_nsec / 1.0e6;
        printf("[DASM] Finished at %.3f ms\n", end_ms); // 끝난 시점 출력
        double exec_time_ms = (end.tv_sec - recv_time.tv_sec) * 1000.0 + (end.tv_nsec - recv_time.tv_nsec) / 1.0e6;
        printf("[DASM] Execution time: %.3f ms\n", exec_time_ms); // 실행시간 출력
        double resp_time_ms = (end.tv_sec - next.tv_sec) * 1000.0 + (end.tv_nsec - next.tv_nsec) / 1.0e6;
        printf("[DASM] Response time: %.3f ms\n", resp_time_ms); // 반응시간 출력
        printf("[DASM] Sleeping for %d ms\n\n", PERIOD_MS);      // 다음 주기까지 대기 시간 출력

        // ID 변경 시(새 Data인 경우), log 출력
        // log출력: Chain_type의 level에 따라,micorsecond 단위로, Chain에서의 시점들 전부 출력.
        print_log_if_new("Chain 1", &chain1_r, &last_Lidar_grabber_id, &next, &recv_time, &end, 5);
        print_log_if_new("Chain 2", &chain2_r, &last_CAN_id, &next, &recv_time, &end, 5);
        print_log_if_new("Chain 3", &chain3_r, &last_SFM_id, &next, &recv_time, &end, 3);
        print_log_if_new("Chain 4", &chain4_r, &last_Lane_detection_id, &next, &recv_time, &end, 3);
        print_log_if_new("Chain 5", &chain5_r, &last_Detection_id, &next, &recv_time, &end, 3);

        // 5.next period cal phase
        //  주기 계산
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
// main
// ------------------------------
int main()
{
    //process를 core에 배치
    bind_process_to_core(0); 

    pthread_t copy_tid, runnable_tid;
    srand(time(NULL));
    memset(input_buffer_byplanner, 0, INPUT_SIZE_B_byplanner);

    pthread_create(&copy_tid, NULL, copy_thread, NULL);
    pthread_create(&runnable_tid, NULL, runnable_thread, NULL);

    pthread_join(copy_tid, NULL);
    pthread_join(runnable_tid, NULL);

    return 0;
}
