#include <Servo.h>

// 핀 정의
const int trigPin = 9;   // 초음파 센서 Trig 핀
const int echoPin = 8;  // 초음파 센서 Echo 핀
Servo servo;             // 서보 객체 생성

// 변수 초기화
long duration;  // 초음파 신호의 시간
float distance; // 거리(cm)
int servoAngle = 90; // 서보모터 초기 각도 (중립)

void setup() {
    Serial.begin(9600); // 시리얼 통신 초기화
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    servo.attach(10); // 서보모터 핀 설정 (핀 11에 연결)
    servo.write(servoAngle); // 초기 서보모터 각도 설정
}

void loop() {
    // 초음파 센서로 거리 측정
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2; // 거리 계산 (cm)
    
    // 파이썬으로 거리 데이터를 전송
    Serial.println(distance);

    // 파이썬에서 서보모터 각도 수신
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // 새 줄까지 데이터 읽기
        servoAngle = input.toInt(); // 데이터를 정수로 변환
        servoAngle = constrain(servoAngle, 0, 180); // 각도를 0~180도로 제한
        servo.write(servoAngle); // 서보모터 각도 설정
    }

    delay(50); // 안정화를 위한 딜레이
}
