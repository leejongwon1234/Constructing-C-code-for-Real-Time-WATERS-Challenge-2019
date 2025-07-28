#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <semaphore.h>
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
// by EKF: Localization으로 부터 취합된, Lidar grabber와 CAN으로부터 수신한 Timestamp를 포함해야함
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
#define INPUT_SIZE_B_byEKF (x_car_host_size_B + y_car_host_size_B + yaw_car_host_size_B + vel_car_size_B + yaw_rate_size_B)

//  output data
// by Planner
#define speed_object_size_KB 1
#define speed_object_size_B (speed_object_size_KB * 1024)
#define steer_object_size_KB 1
#define steer_object_size_B (steer_object_size_KB * 1024)
#define OUTPUT_SIZE_B (speed_object_size_B + steer_object_size_B)

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

// INPUT(READ) 관련 설정
#define INPUT_NUM_PROCESSES 4 // Planner가 읽도록 연결된 Process 갯수
const int INPUT_SIZE_B[INPUT_NUM_PROCESSES] = {
    INPUT_SIZE_B_byEKF, INPUT_SIZE_B_bySFM, INPUT_SIZE_B_bylane, INPUT_SIZE_B_bydetection};
const char *INPUT_SHM_NAMES[INPUT_NUM_PROCESSES] = {
    "/ekf_planner_shm", "/sfm_planner_shm", "/lane_planner_shm", "/detection_planner_shm"};
const char *INPUT_SEM_NAMES[INPUT_NUM_PROCESSES] = {
    "/ekf_planner_sem", "/sfm_planner_sem", "/lane_planner_sem", "/detection_planner_sem"};
char *INPUT_shm_ptrs[INPUT_NUM_PROCESSES]; // shared memory pointer들
sem_t *INPUT_sems[INPUT_NUM_PROCESSES];    // semaphore pointer들
int INPUT_fds[INPUT_NUM_PROCESSES];        // file descriptor들, 공유메모리 객체를 위해 존재

// OUTPUT (Write)관련 설정
#define OUTPUT_SHM_NAME "/planner_dasm_shm"
#define OUTPUT_SEM_NAME "/planner_dasm_sem"
char *OUTPUT_shm_ptr;
sem_t *OUTPUT_sem;
int OUTPUT_fd;

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
    char local_copy_bysfm[INPUT_SIZE_B_bySFM];             // 24KB만큼 입력
    char local_copy_bylane[INPUT_SIZE_B_bylane];           // 32KB만큼 입력
    char local_copy_bydetection[INPUT_SIZE_B_bydetection]; // 750KB만큼 입력
    char local_copy_byekf[INPUT_SIZE_B_byEKF];             // 5KB만큼 입력
    char result[OUTPUT_SIZE_B];                  // 2048 bytes만큼 출력
    struct timespec next, start, recv_time, send_time, end;

    clock_gettime(CLOCK_MONOTONIC, &next); // 주기를 위한 시간 측정

    while (1)
    {
        // 1.Setup phase: 기상, 데이터 읽기 완료
        // 기상
        clock_gettime(CLOCK_MONOTONIC, &start);
        double next_ms = next.tv_sec * 1000.0 + next.tv_nsec / 1.0e6;
        printf("[Planner] wake-up at %.3f ms\n", next_ms);
        double start_ms = start.tv_sec * 1000.0 + start.tv_nsec / 1.0e6;
        //printf("[Planner] Started at %.3f ms\n", start_ms);

        // 데이터 읽기 (shared memory)
        // 입력버퍼에서 데이터 복사: SFM, Lane_detection, Detection, EKF
        // EKF 데이터 복사 (chain1,2)
        sem_wait(INPUT_sems[0]);
        memcpy(local_copy_byekf, INPUT_shm_ptrs[0], INPUT_SIZE_B[0]);
        sem_post(INPUT_sems[0]);
        // SFM 데이터 복사 (chain 3)
        sem_wait(INPUT_sems[1]);
        memcpy(local_copy_bysfm, INPUT_shm_ptrs[1], INPUT_SIZE_B[1]);
        sem_post(INPUT_sems[1]);
        // Lane_detection 데이터 복사 (Chain 4)
        sem_wait(INPUT_sems[2]);
        memcpy(local_copy_bylane, INPUT_shm_ptrs[2], INPUT_SIZE_B[2]);
        sem_post(INPUT_sems[2]);
        // Detection 데이터 복사
        sem_wait(INPUT_sems[3]);
        memcpy(local_copy_bydetection, INPUT_shm_ptrs[3], INPUT_SIZE_B[3]);
        sem_post(INPUT_sems[3]);

        clock_gettime(CLOCK_MONOTONIC, &recv_time);
        double recv_time_ms = recv_time.tv_sec * 1000.0 + recv_time.tv_nsec / 1.0e6;
        //double read_time_ms = (recv_time.tv_sec - start.tv_sec) * 1000.0 + (recv_time.tv_nsec - start.tv_nsec) / 1.0e6;
        //printf("[Planner] read time: %.3f ms\n", read_time_ms); // 실행시간 출력

        //printf("[Planner] received at %.3f ms\n", recv_time_ms);
        // ------------------ setup phase 완료 ----------
        // 2. Execution phase: Data 읽기 및 설정, busy-loop

        // Data 읽기 및 설정
        // Chain type에 해당하는 local_copy의 해당 부분을 추출해야함
        // Chain 1,2 (0:511)에 해당하는 부분 추출 및 result 저장
        memcpy(result + chain1_offset, &local_copy_byekf[chain1_offset], message_size_of_chain * 2);
        // chain 3 (512:767)
        memcpy(result + chain3_offset, &local_copy_bysfm[chain3_offset], message_size_of_chain);
        // chain 4 (768: 1023)
        memcpy(result + chain4_offset, &local_copy_bylane[chain4_offset], message_size_of_chain);
        // chain 5 (1024: 1279)
        memcpy(result + chain5_offset, &local_copy_bydetection[chain5_offset], message_size_of_chain);

        // //debugging chain3(result)
        // for (int i = chain3_offset + 64; i < chain3_offset + 128; i++) {
        //     printf("%02X ", (unsigned char)result[i]);
        //     if ((i - (chain3_offset + 64) + 1) % 16 == 0) printf("\n");  // 16바이트마다 줄바꿈
        // }
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
        //printf("[Planner] send at %.3f ms\n", send_time_ms);

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

        // 결과를 DASM에 전송: memcpy
        sem_wait(OUTPUT_sem);
        memcpy(OUTPUT_shm_ptr, result, OUTPUT_SIZE_B);
        sem_post(OUTPUT_sem);

        // 4.log print phase
        clock_gettime(CLOCK_MONOTONIC, &end);
        double end_ms = end.tv_sec * 1000.0 + end.tv_nsec / 1.0e6;
        //printf("[Planner] finished at %.3f ms\n", end_ms);           //// 끝난 시점 출력
        double exec_time_ms = (send_time.tv_sec - recv_time.tv_sec) * 1000.0 + (send_time.tv_nsec - recv_time.tv_nsec) / 1.0e6;
        //printf("[Planner] Execution time: %.3f ms\n", exec_time_ms); // 실행시간 출력
        double resp_time_ms = (end.tv_sec - next.tv_sec) * 1000.0 + (end.tv_nsec - next.tv_nsec) / 1.0e6;
        //printf("[Planner] Response time: %.3f ms\n", resp_time_ms); // 반응시간 출력
        //printf("[Planner] Sleeping for %d ms\n\n", PERIOD_MS);       // 다음 주기까지 대기 시간 출력

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
    // 읽기용 shm mmap + sem_open
    for (int i = 0; i < INPUT_NUM_PROCESSES; ++i)
    {
        INPUT_fds[i] = shm_open(INPUT_SHM_NAMES[i], O_RDONLY, 0666); // shared memory 객체 생성
        if (INPUT_fds[i] == -1)
        {
            perror("input_shm_open");
            exit(EXIT_FAILURE);
        }
        INPUT_shm_ptrs[i] = mmap(NULL, INPUT_SIZE_B[i], PROT_READ, MAP_SHARED, INPUT_fds[i], 0);
        if (INPUT_shm_ptrs[i] == MAP_FAILED)
        {
            perror("input_mmap");
            exit(EXIT_FAILURE);
        }

        INPUT_sems[i] = sem_open(INPUT_SEM_NAMES[i], 0);
        if (INPUT_sems[i] == SEM_FAILED)
        {
            perror("input_sem_open");
            exit(EXIT_FAILURE);
        }
    }
    // 쓰기용 mmap + sem_open
    // 1. shared memory 객체 생성(or 열기)
    /*
    - SHM_NAME: 전역변수로 정의된 공유 메모리의 이름 (/sfm_planner_shm)
    - O_CREAT | O_RDWR: 없으면 새로 만들고, 읽기/쓰기 권한으로 엽니다.
    - 0666: Unix 퍼미션, 모든 사용자에게 읽기/쓰기 허용.
    - 반환값 shm_fd: 공유 메모리 객체의 파일 디스크립터입니다.
    */
    OUTPUT_fd = shm_open(OUTPUT_SHM_NAME, O_CREAT | O_RDWR, 0666);
    // 실패 검사
    if (OUTPUT_fd == -1)
    {
        perror("output_shm_open");
        exit(EXIT_FAILURE);
    }
    // Shared memory 객체 크기 설정 및 실패 확인
    /*
    - shm_fd: shared memory 객체의 파일 디스크립터
    - OUTPUT_SIZE_B: 전역변수로 정의된 process의 output 크기
    */
    if (ftruncate(OUTPUT_fd, OUTPUT_SIZE_B) == -1)
    {
        perror("ftruncate");
        exit(EXIT_FAILURE);
    }
    // 2. Shared memory object를 virtual memory space에 mmap을 통해 mapping
    /*
    - mmap()을 통해 공유 메모리 객체를 현재 프로세스의 가상 메모리 공간에 매핑합니다.
    - NULL: 커널이 적절한 주소를 자동 선택.
    - OUTPUT_SIZE_B: 매핑 크기 (24KB).
    - PROT_READ | PROT_WRITE: 읽기/쓰기 가능.
    - MAP_SHARED: 다른 프로세스와 메모리를 공유.
    - shm_fd: 앞서 연 공유 메모리의 파일 디스크립터.
    - 0: 오프셋 (0부터 매핑 시작).
    - 반환값 OUTPUT_shm_ptr: 공유 메모리에 접근할 수 있는 포인터입니다.
    */
    OUTPUT_shm_ptr = mmap(NULL, OUTPUT_SIZE_B, PROT_READ | PROT_WRITE, MAP_SHARED, OUTPUT_fd, 0);
    if (OUTPUT_shm_ptr == MAP_FAILED)
    {
        perror("output_mmap");
        exit(EXIT_FAILURE);
    }
    // 3. Shared memory를 위해 semaphore 생성
    OUTPUT_sem = sem_open(OUTPUT_SEM_NAME, O_CREAT, 0666, 1); // 초기값 1
    if (OUTPUT_sem == SEM_FAILED)
    {
        perror("output_sem_open");
        exit(EXIT_FAILURE);
    }

    srand(time(NULL)); // random 처리를 위해 있어야함
    // runnable_thread는 Planner의 실행 로직을 담당
    pthread_t runnable_tid;
    pthread_create(&runnable_tid, NULL, runnable_thread, OUTPUT_shm_ptr);
    pthread_join(runnable_tid, NULL);

    // 정리 (도달하지 않지만 안전하게)
    // OUTPUT
    sem_close(OUTPUT_sem);
    sem_unlink(OUTPUT_SEM_NAME);
    munmap(OUTPUT_shm_ptr, OUTPUT_SIZE_B);
    shm_unlink(OUTPUT_SHM_NAME);
    // INPUT
    for (int i = 0; i < INPUT_NUM_PROCESSES; ++i)
    {
        sem_close(INPUT_sems[i]);
        sem_unlink(INPUT_SEM_NAMES[i]);
        munmap(INPUT_shm_ptrs[i], INPUT_SIZE_B[i]);
        //shm_unlink(INPUT_SHM_NAMES[i]);
        close(INPUT_fds[i]);
    }
    return 0;
}