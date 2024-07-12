from lane_detect import Lane
from traffic_light_detect import TrafficLight


if __name__ == "__main__":
    traffic_cam = TrafficLight(2)
    #lane_cam = Lane(4)

    traffic_cam.traffic_light_detect()
    #lane_cam.lane_detect()

