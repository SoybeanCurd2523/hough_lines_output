# hough_lines_output

2023년 1학기 창업연계융합설계 과목 수강 중 네비게이션 로봇 제작 프로젝트에서 만든 패키지 

실행 :
roscore
rosrun hough_lines_output output_listener

기능 :
1. hough transform 된 line들을 opencv로부터 subscribe한다.
2. lineArrayCallback이라는 함수가 실행되면 여기서 각 직선에서 2개의 점을 구하고,
calculateIntersection이라는 함수에서 수평,평행을 체크하고 기울기가 10도와 80도 사이에 있는 필터를 거쳐서 교차점을 구한 뒤,
filteredIntersection이라는 함수에서 x는 540부터 740, y는 400부터 600까지 boundary를 설정하고, 100개의 점을 누적하고
평균을 구한뒤 표준편차 1000을 설정하고 최종적으로 교차점을 구했다.
3. 이의 x 좌표를 delivery_pkg에 publish한다.

dependencies : 
roscpp, sensor_msgs, std_msgs, opencv_apps

