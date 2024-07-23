from lane_detect import Lane
from traffic_light_detect import TrafficLight
import missions
import time
import serial

PORT = '/dev/ttyACM0'

if __name__ == "__main__":


    ser = serial.Serial(PORT, 9600)
    time.sleep(1)
    print("시리얼 포트 열기 성공")

    traffic_cam = TrafficLight(2, PORT)
    lane_cam = Lane(4, PORT)

    
    mission = int(input("미션 번호를 입력하시오 : "))
    match mission:
        case 1:
            print("1번 미션을 수행합니다.") # 차선 주행
            missions.mission1(traffic_cam,lane_cam)

        case 2:
            print("2번 미션을 수행합니다.") # 신호등 인식, 장애물 회피
            missions.mission2(traffic_cam,lane_cam)

        case 3:
            print("3번 미션을 수행합니다.") # 주차
            missions.mission3(traffic_cam,lane_cam)
