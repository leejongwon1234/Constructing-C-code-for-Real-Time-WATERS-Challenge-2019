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
// by SFM
#define Matrix_SFM_host_size_KB 24
#define Matrix_SFM_host_size_B (Matrix_SFM_host_size_KB * 1024)
#define INPUT_SIZE_B_bySFM (Matrix_SFM_host_size_B)
// by Lane_detection
#define Lane_boundaries_host_size_KB 32
#define Lane_boundaries_host_size_B (Lane_boundaries_host_size_KB * 1024)
#define INPUT_SIZE_B_bylane (Lane_boundaries_host_size_B)
// by Detection
#define Bounding_box_host_size_KB 750
#define Bounding_box_host_size_B (Bounding_box_host_size_KB * 1024)
#define INPUT_SIZE_B_bydetection (Bounding_box_host_size_B)
// by ekf: Localization으로 부터 취합된, Lidar grabber와 CAN으로부터 수신한 Timestamp를 포함해야함
#define x_car_host_size_KB 1
#define x_car_host_size_B (x_car_host_size_KB * 1024)
#define y_car_host_size_KB 1
#define y_car_host_size_B (y_car_host_size_KB * 1024)
#define yaw_car_host_size_KB 1
#define yaw_car_host_size_B (yaw_car_host_size_KB * 1024)
#define vel_car_size_KB 1
#define vel_car_size_B (vel_car_size_KB * 1024)
#define yaw_rate_size_KB 1
#define yaw_rate_size_B (yaw_rate_size_KB * 1024)
#define INPUT_SIZE_B_byekf (x_car_host_size_B + y_car_host_size_B + yaw_car_host_size_B + vel_car_size_B + yaw_rate_size_B)
// by Localization: 나중에 구현, e2e의 직접연결 X
// #define Vehicle_status_host_size_KB 1
// #define Vehicle_status_host_size_B (Vehicle_status_host_size_KB * 1024)
// by Lidar_grabber: 나중에 구현, e2e의 직접연결 X
// #define Occupancy_grid_host_size_KB 500
// #define Occupancy_grid_host_size_B (Occupancy_grid_host_size_KB * 1024)
// 추가적으로 다른 Task들로부터 수신되어야하는 데이터도 존재(차후 연구 구현)
//  output data
// by Planner
#define speed_object_size_KB 1
#define speed_object_size_B (speed_object_size_KB * 1024)
#define steer_object_size_KB 1
#define steer_object_size_B (steer_object_size_KB * 1024)
#define OUTPUT_SIZE_B_byplanner (speed_object_size_B + steer_object_size_B)

// execution
#define PERIOD_MS 15
#define PERIOD_US (PERIOD_MS * 1000)    // microseconds 단위로 변환
#define PERIOD_NS (PERIOD_MS * 1000000) // nanoseconds 단위로 변환
#define EXEC_TICKS_LB 19243822
#define EXEC_TICKS_AVG 22743822
#define EXEC_TICKS_UB 26483822
#define EXEC_TIME_LB (EXEC_TICKS_LB / PERFORMANCE_Frequency)   // nanoseconds 단위로 변환
#define EXEC_TIME_AVG (EXEC_TICKS_AVG / PERFORMANCE_Frequency) // nanoseconds 단위로 변환
#define EXEC_TIME_UB (EXEC_TICKS_UB / PERFORMANCE_Frequency)   // nanoseconds 단위로 변환

// 메세지 OFFSET 설정 (Planner는 연결된 Task들의 정보를 읽고 모든 영역에 작성해야함.)
#define chain_type_1 1
#define chain_type_2 2
#define chain_type_3 3
#define chain_type_4 4
#define chain_type_5 5
#define chain_level 2
#define message_size_of_chain 256 // 하나의 task chain마다 할당되는 message size 크기
#define message_size_of_task 64   // 하나의 task마다 할당되는 message size 크기
#define message_format_unit 16    // task가 사용하는 Message의 format 단위 16bytes
// 메세지 offset
#define chain1_offset ((chain_type_1 - 1) * message_size_of_chain)
#define chain2_offset ((chain_type_2 - 1) * message_size_of_chain)
#define chain3_offset ((chain_type_3 - 1) * message_size_of_chain)
#define chain4_offset ((chain_type_4 - 1) * message_size_of_chain)
#define chain5_offset ((chain_type_5 - 1) * message_size_of_chain)

// //메세지 read offset (chain_level이 3인 task들의 data 위치) (2,3,4,5가 담겨서 256bytes)
// #define chain1_read_offset (((chain_type_1-1)*message_size_of_chain) + ((chain_level-1)*message_size_of_task)) //chain1의 read offset
// #define chain2_read_offset (((chain_type_2-1)*message_size_of_chain) + ((chain_level-1)*message_size_of_task)) //chain2의 read offset
// #define chain3_read_offset (((chain_type_3-1)*message_size_of_chain) + ((chain_level-1)*message_size_of_task)) //chain3의 read offset
// #define chain4_read_offset (((chain_type_4-1)*message_size_of_chain) + ((chain_level-1)*message_size_of_task)) //chain4의 read offset
// #define chain5_read_offset (((chain_type_5-1)*message_size_of_chain) + ((chain_level-1)*message_size_of_task)) //chain5의 read offset

// 전역 변수
char input_buffer_bySFM[INPUT_SIZE_B_bySFM];
char input_buffer_bylane[INPUT_SIZE_B_bylane];
char input_buffer_bydetection[INPUT_SIZE_B_bydetection];
char input_buffer_byekf[INPUT_SIZE_B_byekf];

pthread_mutex_t buffer_lock_bySFM = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t buffer_lock_bylane = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t buffer_lock_bydetection = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t buffer_lock_byekf = PTHREAD_MUTEX_INITIALIZER;

#define DASM_PORT 5555 // DASM 서버 포트
// Planner 서버 포트
#define Planner_SFM_PORT 5556
#define Planner_LANE_PORT 5557
#define Planner_DETECTION_PORT 5558
#define Planner_ekf_PORT 5559

// Copy Thread: SFM으로부터 데이터 수신
void *copy_thread_bySFM(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char local_copy[INPUT_SIZE_B_bySFM];
    // 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("[Planner] SFM socket error");
        exit(EXIT_FAILURE);
    }
    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Planner_SFM_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // 바인드
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("[Planner] SFM bind error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    // 리슨
    if (listen(server_sock, 5) < 0)
    {
        perror("[Planner] SFM listen error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Waiting for SFM connection on port %d...\n", Planner_SFM_PORT);
    // SFM에서 연결을 기다림
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0)
    {
        perror("[Planner] SFM accept error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Connected to SFM: %s\n", inet_ntoa(client_addr.sin_addr));
    // SFM으로부터 데이터를 수신하여 input_buffer_bySFM에 저장
    while (1)
    {
        ssize_t n = recv(client_sock, local_copy, INPUT_SIZE_B_bySFM, MSG_WAITALL);
        if (n > 0)
        {
            pthread_mutex_lock(&buffer_lock_bySFM);
            memcpy(input_buffer_bySFM, local_copy, INPUT_SIZE_B_bySFM);
            pthread_mutex_unlock(&buffer_lock_bySFM);
        }
        else
        {
            printf("[Planner] recv error or connection closed\n");
            break;
        }
    }
    close(client_sock);
    close(server_sock);
    return NULL;
}

// Copy Thread: SFM으로부터 데이터 수신
void *copy_thread_bylane(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char local_copy[INPUT_SIZE_B_bylane];
    // 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("[Planner] Lane socket error");
        exit(EXIT_FAILURE);
    }
    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Planner_LANE_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // 바인드
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("[Planner] Lane bind error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    // 리슨
    if (listen(server_sock, 5) < 0)
    {
        perror("[Planner] Lane listen error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Waiting for Lane connection on port %d...\n", Planner_LANE_PORT);
    // Lane_detection에서 연결을 기다림
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0)
    {
        perror("[Planner] Lane accept error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Connected to Lane_detection: %s\n", inet_ntoa(client_addr.sin_addr));
    // Lane_detection으로부터 데이터를 수신하여 input_buffer_bylane에 저장
    while (1)
    {
        ssize_t n = recv(client_sock, local_copy, INPUT_SIZE_B_bylane, MSG_WAITALL);
        if (n > 0)
        {
            pthread_mutex_lock(&buffer_lock_bylane);
            memcpy(input_buffer_bylane, local_copy, INPUT_SIZE_B_bylane);
            pthread_mutex_unlock(&buffer_lock_bylane);
        }
        else
        {
            printf("[Planner] recv error or connection closed\n");
            break;
        }
    }
    close(client_sock);
    close(server_sock);
    return NULL;
}

// Copy Thread: SFM으로부터 데이터 수신
void *copy_thread_bydetection(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char local_copy[INPUT_SIZE_B_bydetection];
    // 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("[Planner] Detection socket error");
        exit(EXIT_FAILURE);
    }
    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Planner_DETECTION_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // 바인드
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("[Planner] Detection bind error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    // 리슨
    if (listen(server_sock, 5) < 0)
    {
        perror("[Planner] Detection listen error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Waiting for Detection connection on port %d...\n", Planner_DETECTION_PORT);
    // Detection에서 연결을 기다림
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0)
    {
        perror("[Planner] Detection accept error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Connected to Detection: %s\n", inet_ntoa(client_addr.sin_addr));
    // Detection으로부터 데이터를 수신하여 input_buffer_bydetection에 저장
    while (1)
    {
        ssize_t n = recv(client_sock, local_copy, INPUT_SIZE_B_bydetection, MSG_WAITALL);
        if (n > 0)
        {
            pthread_mutex_lock(&buffer_lock_bydetection);
            memcpy(input_buffer_bydetection, local_copy, INPUT_SIZE_B_bydetection);
            pthread_mutex_unlock(&buffer_lock_bydetection);
        }
        else
        {
            printf("[Planner] recv error or connection closed\n");
            break;
        }
    }
    close(client_sock);
    close(server_sock);
    return NULL;
}

// Copy Thread: SFM으로부터 데이터 수신
void *copy_thread_byekf(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char local_copy[INPUT_SIZE_B_byekf];
    // 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("[Planner] ekf socket error");
        exit(EXIT_FAILURE);
    }
    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(Planner_ekf_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // 바인드
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("[Planner] ekf bind error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    // 리슨
    if (listen(server_sock, 5) < 0)
    {
        perror("[Planner] ekf listen error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Waiting for ekf connection on port %d...\n", Planner_ekf_PORT);
    // ekf에서 연결을 기다림
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
    if (client_sock < 0)
    {
        perror("[Planner] ekf accept error");
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    printf("[Planner] Connected to ekf: %s\n", inet_ntoa(client_addr.sin_addr));
    // ekf로부터 데이터를 수신하여 input_buffer_byekf에 저장
    while (1)
    {
        ssize_t n = recv(client_sock, local_copy, INPUT_SIZE_B_byekf, MSG_WAITALL);
        if (n > 0)
        {
            pthread_mutex_lock(&buffer_lock_byekf);
            memcpy(input_buffer_byekf, local_copy, INPUT_SIZE_B_byekf);
            printf("복사함");
            pthread_mutex_unlock(&buffer_lock_byekf);
        }
        else
        {
            printf("[Planner] recv error or connection closed\n");
            break;
        }
    }
    close(client_sock);
    close(server_sock);
    return NULL;
}

// Runnable Thread

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
typedef struct
{
    uint8_t id;

    int64_t wake_sec;
    int64_t wake_nsec;

    int64_t recv_sec;
    int64_t recv_nsec;

    int64_t send_sec;
    int64_t send_nsec;

} TaskHeader;

// // out의 필드에 buffer에서 offset 위치부터 읽어와서 채움
// // buffer는 입력 버퍼, offset은 해당 TaskHeader의 읽는 시작 위치
// // TaskHeader는 id,wake,start,end 필드를 각 16bytes로 가짐
// // id는 1바이트, sec와 nsec은 각각 8바이트로 구성됨
// void parse_task_header(TaskHeader *out, const char *buffer, int offset)
// {
//     memcpy(&out->id, buffer + offset, sizeof(uint8_t));

//     memcpy(&out->wake_sec, buffer + offset + (sizeof(int64_t) * 2), sizeof(int64_t));
//     memcpy(&out->wake_nsec, buffer + offset + (sizeof(int64_t) * 3), sizeof(int64_t));

//     memcpy(&out->start_sec, buffer + offset + (sizeof(int64_t) * 4), sizeof(int64_t));
//     memcpy(&out->start_nsec, buffer + offset + (sizeof(int64_t) * 5), sizeof(int64_t));

//     memcpy(&out->end_sec, buffer + offset + (sizeof(int64_t) * 6), sizeof(int64_t));
//     memcpy(&out->end_nsec, buffer + offset + (sizeof(int64_t) * 7), sizeof(int64_t));
// }

// out의 필드에서 읽어 buffer에 offset 위치부터 채움
// buffer는 출력 버퍼, offset은 해당 TaskHeader의 시작 위치
// TaskHeader는 id,wake,start,end 필드를 각 16bytes로 가짐
// TaskHeader의 각 필드는 8bytes로 분리됨 (id를 제외하고, 8bytes씩 sec, nsec를 담당)
// 따라서 총 64bytes를 작성
void write_task_header(TaskHeader *out, char *buffer, int task_offset)
{
    // ID 작성
    memcpy(buffer + task_offset, &out->id, sizeof(uint8_t)); // 1byte 만큼 ID 사용
    // Wake 작성
    memcpy(buffer + task_offset + (sizeof(int64_t) * 2), &out->wake_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t) * 3), &out->wake_nsec, sizeof(int64_t));
    // Start 작성 (or recv_time 이후부터)
    memcpy(buffer + task_offset + (sizeof(int64_t) * 4), &out->recv_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t) * 5), &out->recv_nsec, sizeof(int64_t));
    // End 작성
    memcpy(buffer + task_offset + (sizeof(int64_t) * 6), &out->send_sec, sizeof(int64_t));
    memcpy(buffer + task_offset + (sizeof(int64_t) * 7), &out->send_nsec, sizeof(int64_t));
}

void *runnable_thread(void *arg)
{
    char local_copy_bySFM[INPUT_SIZE_B_bySFM];             // 24KB만큼 입력
    char local_copy_bylane[INPUT_SIZE_B_bylane];           // 32KB만큼 입력
    char local_copy_bydetection[INPUT_SIZE_B_bydetection]; // 750KB만큼 입력
    char local_copy_byekf[INPUT_SIZE_B_byekf];             // 5KB만큼 입력
    char result[OUTPUT_SIZE_B_byplanner];                  // 2048 bytes만큼 출력
    struct timespec next, start, recv_time, send_time, end;

    // 해당 위치에 DASM에 연결 시도 로직구현 필요
    // while문을 통해 running 이전에 DASM에 Client로써 연결시도
    int dasm_sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in dasm_addr;
    dasm_addr.sin_family = AF_INET;
    dasm_addr.sin_port = htons(DASM_PORT);
    inet_pton(AF_INET, "127.0.0.1", &dasm_addr.sin_addr); // localhost IP 주소로 설정
    // 반복 연결 시도
    while (connect(dasm_sock, (struct sockaddr *)&dasm_addr, sizeof(dasm_addr)) < 0)
    {
        perror("[Planner] Waiting for DASM...");
        sleep(1);
    }
    printf("[Planner] Connected to DASM\n");

    clock_gettime(CLOCK_MONOTONIC, &next); // 주기를 위한 시간 측정

    while (1)
    {
        // 1.Setup phase: 기상, 데이터 읽기 완료
        // 기상
        clock_gettime(CLOCK_MONOTONIC, &start);
        double next_ms = next.tv_sec * 1000.0 + next.tv_nsec / 1.0e6;
        printf("[Planner] wake-up at %.3f ms\n", next_ms);
        double start_ms = start.tv_sec * 1000.0 + start.tv_nsec / 1.0e6;
        printf("[Planner] Started at %.3f ms\n", start_ms);

        // 데이터 읽기
        // 입력버퍼에서 데이터 복사: SFM, Lane_detection, Detection, ekf
        // ekf 데이터 복사 (chain1,2)
        pthread_mutex_lock(&buffer_lock_byekf);
        memcpy(local_copy_byekf, input_buffer_byekf, INPUT_SIZE_B_byekf);
        pthread_mutex_unlock(&buffer_lock_byekf);
        // SFM 데이터 복사 (chain 3)
        pthread_mutex_lock(&buffer_lock_bySFM);
        memcpy(local_copy_bySFM, input_buffer_bySFM, INPUT_SIZE_B_bySFM);
        pthread_mutex_unlock(&buffer_lock_bySFM);
        // Lane_detection 데이터 복사 (Chain 4)
        pthread_mutex_lock(&buffer_lock_bylane);
        memcpy(local_copy_bylane, input_buffer_bylane, INPUT_SIZE_B_bylane);
        pthread_mutex_unlock(&buffer_lock_bylane);
        // Detection 데이터 복사
        pthread_mutex_lock(&buffer_lock_bydetection);
        memcpy(local_copy_bydetection, input_buffer_bydetection, INPUT_SIZE_B_bydetection);
        pthread_mutex_unlock(&buffer_lock_bydetection);

        clock_gettime(CLOCK_MONOTONIC, &recv_time);
        double recv_time_ms = recv_time.tv_sec * 1000.0 + recv_time.tv_nsec / 1.0e6;
        printf("[Planner] received at %.3f ms\n", recv_time_ms);
        // ------------------ setup phase 완료 ----------
        // 2. Execution phase: Data 읽기 및 설정, busy-loop

        // Data 읽기 및 설정
        // Chain type에 해당하는 local_copy의 해당 부분을 추출해야함
        // Chain 1,2 (0:511)에 해당하는 부분 추출 및 result 저장
        memcpy(result + chain1_offset, &local_copy_byekf[chain1_offset], message_size_of_chain * 2);
        // chain 3 (512:767)
        memcpy(result + chain3_offset, &local_copy_bySFM[chain3_offset], message_size_of_chain);
        // chain 4 (768: 1023)
        memcpy(result + chain4_offset, &local_copy_bylane[chain4_offset], message_size_of_chain);
        // chain 5 (1024: 1279)
        memcpy(result + chain5_offset, &local_copy_bydetection[chain5_offset], message_size_of_chain);

        // //debugging(result)
        // for (int i = chain1_offset + 64; i < chain1_offset + 128; i++) {
        //     printf("%02X ", (unsigned char)result[i]);
        //     if ((i - (chain1_offset + 64) + 1) % 16 == 0) printf("\n");  // 16바이트마다 줄바꿈
        // }
        

        // busy-loop
        // execution time 계산
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

        // 3. send phase: data packet 생성 시작
        clock_gettime(CLOCK_MONOTONIC, &send_time);
        double send_time_ms = send_time.tv_sec * 1000.0 + send_time.tv_nsec / 1.0e6;
        printf("[Planner] send at %.3f ms\n", send_time_ms);
        //  DASM에 전송할 데이터 준비
        TaskHeader chains[5]; // chain은 5개
        for (int i = 0; i < 5; i++){
            chains[i].id = 0; // 또는 특정 값
            chains[i].wake_sec = next.tv_sec;
            chains[i].wake_nsec = next.tv_nsec;
            chains[i].recv_sec = recv_time.tv_sec;
            chains[i].recv_nsec = recv_time.tv_nsec;
            chains[i].send_sec = send_time.tv_sec;
            chains[i].send_nsec = send_time.tv_nsec;
        }
        
        // 파싱(읽은)한 TaskHeader 구조체를 사용하여 메시지 인덱스와 센싱 시각을 result에 저장
        // OFFSET에 따라 ID와 센싱 시각을 result에 저장
        // result는 OUTPUT_SIZE_B_byplanner 크기로 초기화
        write_task_header(&chains[0], result, chain1_offset); //chain1
        write_task_header(&chains[1], result, chain2_offset); //chain2
        write_task_header(&chains[2], result, chain3_offset); //chain3
        write_task_header(&chains[3], result, chain4_offset); //chain4
        write_task_header(&chains[4], result, chain5_offset); //chain5
        // result에 준비완료
        //debugging
        // for (int i = chain1_offset + 64; i < chain1_offset + 128; i++) {
        //     printf("%02X ", (unsigned char)result[i]);
        //     if ((i - (chain1_offset + 64) + 1) % 16 == 0) printf("\n");  // 16바이트마다 줄바꿈
        // }


        // 결과를 DASM에 전송
        // 여기 위치에 DASM에 전송하는 로직 구현 필요.
        ssize_t sent = send(dasm_sock, result, OUTPUT_SIZE_B_byplanner, 0);
        if (sent <= 0)
        {
            perror("[Planner] send() failed");
            break; // 또는 재연결 루프 설계
        }
        
        // 4.log print phase
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000.0 + end.tv_nsec / 1.0e6;
        printf("[Planner] finished at %.3f ms\n", end_ms);           //// 끝난 시점 출력
        double exec_time_ms = (send_time.tv_sec - recv_time.tv_sec) * 1000.0 + (send_time.tv_nsec - recv_time.tv_nsec) / 1.0e6;
        printf("[Planner] Execution time: %.3f ms\n", exec_time_ms); // 실행시간 출력
        double resp_time_ms = (end.tv_sec - next.tv_sec) * 1000.0 + (end.tv_nsec - next.tv_nsec) / 1.0e6;
        printf("[Planner] Response time: %.3f ms\n", resp_time_ms); // 반응시간 출력
        printf("[Planner] Sleeping for %d ms\n\n", PERIOD_MS);       // 다음 주기까지 대기 시간 출력

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
int main()
{
    bind_process_to_core(1);
    srand(time(NULL));
    memset(input_buffer_bySFM, 0, INPUT_SIZE_B_bySFM);
    memset(input_buffer_bylane, 0, INPUT_SIZE_B_bylane);
    memset(input_buffer_bydetection, 0, INPUT_SIZE_B_bydetection);
    memset(input_buffer_byekf, 0, INPUT_SIZE_B_byekf);

    pthread_t tid_sfm, tid_lane, tid_detection, tid_ekf;
    pthread_create(&tid_sfm, NULL, copy_thread_bySFM, NULL);
    pthread_create(&tid_lane, NULL, copy_thread_bylane, NULL);
    pthread_create(&tid_detection, NULL, copy_thread_bydetection, NULL);
    pthread_create(&tid_ekf, NULL, copy_thread_byekf, NULL);

    // runnable_thread는 Planner의 실행 로직을 담당
    pthread_t runnable_tid;
    pthread_create(&runnable_tid, NULL, runnable_thread, NULL);

    pthread_join(tid_sfm, NULL);
    pthread_join(tid_lane, NULL);
    pthread_join(tid_detection, NULL);
    pthread_join(tid_ekf, NULL);
    pthread_join(runnable_tid, NULL);

    return 0;
}
