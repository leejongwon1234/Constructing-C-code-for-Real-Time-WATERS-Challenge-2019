import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def analyze_logs_final(file_name,period):
    """
    ID가 섞여있거나 순환되더라도 모든 로그를 정확하게 분석합니다.

    Args:
        file_path (str): 분석할 로그 파일의 경로.
    """
    file_path = os.path.join(os.path.dirname(__file__), file_name)
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"❌ 오류: '{file_name}' 파일을 찾을 수 없습니다.")
        return

    # 최종 계산 결과를 저장할 리스트
    results = []
    # ID별로 진행 중인(미완성) 데이터 블록을 저장할 딕셔너리
    incomplete_blocks = {}

    log_pattern = re.compile(r"ID = (\d+), (\w+) = ([\d.]+) us")

    for line_num, line in enumerate(lines):
        match = log_pattern.search(line)
        if not match:
            continue

        id_val, key, value = match.groups()
        id_val = int(id_val)

        # 해당 ID의 블록이 없으면 새로 생성
        if id_val not in incomplete_blocks:
            incomplete_blocks[id_val] = {}

        # 데이터 추가
        incomplete_blocks[id_val][key] = float(value)

        # 해당 ID의 블록이 7개의 데이터를 모두 모았는지 확인
        if len(incomplete_blocks[id_val]) == 7:
            block = incomplete_blocks[id_val]
            
            # 계산 수행
            e2e_latency = block['chain_l1_end_us'] - block['chain_l3_wake_us']
            execution_time = (block['chain_l3_send_us'] - block['chain_l3_start_us']) + \
                             (block['chain_l2_send_us'] - block['chain_l2_recv_us']) + \
                             (block['chain_l1_end_us'] - block['chain_l1_recv_us'])
            waiting_time = e2e_latency - execution_time
            
            results.append((e2e_latency, execution_time, waiting_time))
            
            # 처리가 완료된 블록은 딕셔너리에서 제거
            del incomplete_blocks[id_val]

    if not results:
        print("분석할 데이터를 찾지 못했습니다. 로그 파일 형식을 확인해주세요.")
        return
        
    # 처리되지 않고 남은 블록이 있는지 경고
    if incomplete_blocks:
        print(f"⚠️ 경고: {len(incomplete_blocks)}개의 ID에 대한 로그가 불완전하여 처리되지 않았습니다.")
        print(f"불완전한 ID: {list(incomplete_blocks.keys())}")


    print("results의 길이: ", len(results))
    print("걸린 시간(분):", (len(results)*period)*1.66667e-5)

    # 결과를 pandas DataFrame으로 변환
    df = pd.DataFrame(results, columns=['E2E latency', 'Execution time', 'Waiting time'])

    # df.to_csv('latency_analysis_results.csv', index=False)
    # print(f"\n✅ 총 {len(df)}개의 데이터 분석 완료. 'latency_analysis_results.csv' 파일로 저장되었습니다.")
    # print("\n--- 분석 데이터 (상위 5개) ---")
    # print(df.head())
    # print("-" * 40)
    
    # --- 통계 분석 및 시각화 (이하 코드는 이전과 동일) ---

    def plot_statistics(file_name, data_series, title, bin_width):
        mean_val, std_val, min_val, max_val = data_series.mean(), data_series.std(), data_series.min(), data_series.max()

        print(f"\n📊 {title} 통계")
        print(f"  - 평균    : {mean_val:.2f} us")
        print(f"  - 표준편차 : {std_val:.2f} us")
        print(f"  - 최소값   : {min_val:.2f} us")
        print(f"  - 최대값   : {max_val:.2f} us")
        print("-" * 40)

        plt.figure(figsize=(16, 10))
        bins = np.arange(start=int(min_val)-bin_width, 
                         stop=int(max_val)+bin_width, 
                         step=bin_width)
        plt.hist(data_series, bins=bins, color='skyblue', edgecolor='black', alpha=1)
        plt.axvline(mean_val, color='red', linestyle='dashed', linewidth=2, label=f'Mean: {mean_val:.2f}')
        plt.axvline(min_val, color='green', linestyle='dashed', linewidth=2, label=f'Min: {min_val:.2f}')
        plt.axvline(max_val, color='purple', linestyle='dashed', linewidth=2, label=f'Max: {max_val:.2f}')
        
        plt.title(f'{file_name}, {title} Histogram', fontsize=16)
        plt.xlabel('Time (us)', fontsize=12)
        plt.ylabel('Frequency', fontsize=12)
        plt.legend()
        plt.grid(True, which='both', linestyle='--', linewidth=0.5)
        file_name = f'{file_name}_{title.replace(" ", "_").lower()}_histogram.png'
        plt.tight_layout
        #plt.show()
        plt.savefig(file_name)
        plt.close()
        print(f"✅ '{file_name}' 히스토그램이 저장되었습니다.")

    print(file_name)
    plot_statistics(file_name, df['E2E latency'], 'E2E latency', bin_width=10)
    plot_statistics(file_name, df['Execution time'], 'Execution time', bin_width=10)
    plot_statistics(file_name, df['Waiting time'], 'Waiting time', bin_width=10)


if __name__ == "__main__":
    LOG_FILE_PATH3 = 'log_Chain 3_tcp.txt'
    LOG_FILE_PATH4 = 'log_Chain 4_tcp.txt'
    LOG_FILE_PATH5 = 'log_Chain 5_tcp.txt'
    analyze_logs_final(LOG_FILE_PATH3,33)
    analyze_logs_final(LOG_FILE_PATH4,66)
    analyze_logs_final(LOG_FILE_PATH5,200)
