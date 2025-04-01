function setros() {
    source /opt/ros/humble/setup.bash
}

function setup() {
    source install/setup.bash
}

function buildall() {
    colcon build && setup
}

function build() {
    colcon build --packages-select px4_offboard && setup
}

function loadmission() {
    cp "src/mission.txt" "install/px4_offboard/share/px4_offboard/mission.txt"
}

function loadYOLO() {
    cp "src/classes.txt" "install/px4_offboard/lib/python3.10/site-packages/px4_offboard"
    cp "src/yolov4-tiny.cfg" "install/px4_offboard/lib/python3.10/site-packages/px4_offboard"
    cp "src/yolov4-tiny_last.weights" "install/px4_offboard/lib/python3.10/site-packages/px4_offboard"
}

function sim() {
    local mission_mode=${1:-f}

    ros2 launch px4_offboard offboard_velocity_control.launch.py \
        mission_mode:=$mission_mode
}

function kgz() {
    ps aux | grep gz | grep -v grep | awk '{print $2}' | xargs kill -9
}
