import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

# 시리얼 통신 설정 (아두이노와 연결)
py_serial = serial.Serial(
    port='COM8',      # COM 포트 설정 (아두이노가 연결된 포트)
    baudrate=9600,    # 통신 속도 설정 (9600bps)
)

# PID 게인 설정
Kp = 2.8  # 비례 게인 (Proportional Gain)
Ki = 0.0  # 적분 게인 (Integral Gain)
Kd = 1.4  # 미분 게인 (Derivative Gain)

def PID(input):
    """
    PID 제어를 수행하는 함수
    Args:
        input (float): 현재 입력값 (센서에서 읽은 거리값)
    Returns:
        float: PID 제어 출력값
    """
    global last_error, integral_error
    error = setpoint - input  # 목표값과 현재값의 차이 (오차)
    integral_error += error * time_step  # 적분 항 계산
    derivative_error = (error - last_error) / time_step  # 미분 항 계산
    output = Kp * error + Ki * integral_error + Kd * derivative_error  # PID 제어 출력
    last_error = error  # 이전 오차 업데이트
    return output

# 서보모터 중심 각도 및 초기 거리 값 설정
base_theta = 90  # 서보모터의 중심 각도
prev_distance = 30  # 이전 초음파 거리값 (초기값 설정)

def update(frame):
    """
    애니메이션 업데이트 함수: 초음파 센서 값 읽기, PID 제어, 그래프 업데이트
    Args:
        frame (int): 현재 프레임 번호
    Returns:
        tuple: 업데이트된 빔과 공의 위치 데이터
    """
    global alpha, pos, prev_pos, joint3_pos, r, prev_distance

    # 초음파 센서 데이터 읽기 및 필터링
    if py_serial.readable():  # 시리얼 데이터 읽기 가능 여부 확인
        response = py_serial.readline()  # 시리얼로부터 한 줄 읽기
        raw_distance = float(response[:len(response) - 2].decode())  # 센서 거리값 읽기
        print(f"Raw Distance: {raw_distance:.2f}cm")

        # 거리 값 필터링 수행
        distance = filter_distance(raw_distance, prev_distance)
        prev_distance = distance  # 이전 거리값 업데이트
        print(f"Filtered Distance: {distance:.2f}cm")
    if distance > 36:
        distance = prev_distance  # 데이터 읽기 실패 시 이전 거리값 유지

    # PID 제어를 수행하여 서보모터 각도 계산
    r = distance  # 현재 거리값 업데이트
    output = PID(r)  # PID 제어 출력 계산
    output = max(70, min(147, int(base_theta + output)))  # 서보모터 각도 제한 (50~130도)
    
    # 서보모터 각도 출력값을 아두이노로 전송
    if py_serial.is_open:
        py_serial.write(f"{output}\n".encode())  # 시리얼로 각도 값 전송

    # 각도를 라디안으로 변환하여 빔과 공의 위치 계산
    theta = np.deg2rad(90 - output)  # 서보모터 각도 기준 설정
    alpha = l1 * theta / l3  # 빔의 각도 계산
    print(f"PID Output: {output}, Distance: {distance}, Setpoint: {setpoint}, prev_dist:{prev_distance}")

    # 빔의 끝점 및 공의 위치 계산
    motor_pos = [8, 5]  # 서보모터 위치
    joint3_pos = [37, 15]  # 빔 끝점 위치
    joint1_pos = [8 - l1 * np.cos(theta), 5 + l1 * np.sin(theta)]  # 빔 첫번째 관절 위치
    joint2_pos = [joint3_pos[0] - l3 * np.cos(alpha), joint3_pos[1] + l3 * np.sin(alpha)]  # 빔 두번째 관절 위치

    # 중력에 의한 공의 위치 계산
    a = m * -g * np.sin(alpha)  # 가속도 계산
    pos += a
    x = joint3_pos[0] + 1 * np.sin(alpha) - r * np.cos(alpha)  # 공의 x좌표
    if r + pos > l3 or r + pos < 0:  # 경계값 초과 시 위치 보정
        pos = prev_pos
        x = joint3_pos[0] + 1 * np.sin(alpha) - r * np.cos(alpha)
    y = joint3_pos[1] + 1 * np.cos(alpha) + r * np.sin(alpha)  # 공의 y좌표

    prev_pos = pos  # 이전 위치 업데이트

    # 그래프 업데이트
    beam.set_data([motor_pos[0], joint1_pos[0], joint2_pos[0], joint3_pos[0]],
                  [motor_pos[1], joint1_pos[1], joint2_pos[1], joint3_pos[1]])
    ball.set_center((x, y))  # 공의 위치 업데이트
    return beam, ball

def filter_distance(raw_distance, prev_distance, min_distance=3.2, max_distance=33.5, max_change=3):
    """
    초음파 센서 값을 필터링: 급격한 변화 완화 및 값 제한
    """
    # 값이 범위를 초과하면 이전 값 유지
    if raw_distance > max_distance or raw_distance < min_distance:
        return prev_distance
    
    # 급격한 변화 완화 (스무딩 처리)
    if abs(raw_distance - prev_distance) > max_change:
        return prev_distance + np.sign(raw_distance - prev_distance) * max_change

    return raw_distance


# 초기 설정값
setpoint = 16  # 목표 거리값
time_step = 0.1  # 시간 간격
last_error, integral_error = 0, 0  # PID 제어 변수 초기화

g = 9.8  # 중력 가속도 (m/s^2)
m = 0.004  # 공의 질량 (kg)
pos = 34  # 공의 초기 위치
prev_pos = 0  # 이전 공의 위치
l1, l2, l3 = 5, 10, 34  # 링크 길이 설정
alpha = 0  # 초기 각도 설정

# 그래프 초기화
motor_pos = [8, 5]
joint3_pos = [37, 15]
ball_pos = [37, 16]

plt.figure(figsize=(12, 7.5))
plt.xlim(0, 50)
plt.ylim(0, 30)

beam, = plt.plot([], [], marker='o', linestyle='-', label='Beam')  # 빔 그래프 설정
ball = plt.Circle(ball_pos, 1, color='r', label='Ball', zorder=10)  # 공 그래프 설정
plt.gca().add_patch(ball)

# 애니메이션 실행
ani = FuncAnimation(plt.gcf(), update, frames=range(200), blit=True, interval=time_step * 10)
plt.show()
