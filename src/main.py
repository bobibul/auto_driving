from lane_detect import Lane
from traffic_light_detect import TrafficLight

PORT = '/dev/ttyACM0'

if __name__ == "__main__":


    traffic_cam = TrafficLight(2, PORT)
    lane_cam = Lane(4, PORT)

    while(True):
        mission = int(input("미션 번호를 입력하시오 : "))
        match mission:
            case 1:
                print("1번 미션을 수행합니다.") # 차선 주행
                lane_cam.lane_detect()
                break

            case 2:
                print("2번 미션을 수행합니다.") # 신호등 인식, 장애물 회피
                traffic_cam.traffic_light_detect()
                lane_cam.lane_detect()
                break

            case 3:
                print("3번 미션을 수행합니다.") # 주차
                break

            case _:
                print("다시 입력해 주십시오.")
                continue


