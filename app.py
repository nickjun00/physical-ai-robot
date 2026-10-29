
import streamlit as st

from robot_simulator import Point, RectangleObstacle, RobotSimulator

st.set_page_config(
    page_title="Physical AI Robot Simulator",
    page_icon="🤖",
    layout="wide"
)

st.title("🤖 Physical AI Robot Simulator")
st.caption("Robot Pick-and-Place Control Dashboard")

if "status" not in st.session_state:
    st.session_state.status = "IDLE"
if "robot" not in st.session_state:
    # A small workstation layout: the robot starts at the origin and must not
    # cross either the conveyor or the safety cage.
    st.session_state.robot = RobotSimulator(
        [
            RectangleObstacle("Conveyor", 2.0, -0.75, 0.5, 1.5),
            RectangleObstacle("Safety cage", 4.0, 1.0, 1.5, 0.5),
        ]
    )

col1, col2, col3 = st.columns(3)

with col1:
    if st.button("▶ Start", use_container_width=True):
        st.session_state.status = "RUNNING"

with col2:
    if st.button("⚠ Simulate Failure", use_container_width=True):
        st.session_state.status = "ERROR"

with col3:
    if st.button("↻ Reset", use_container_width=True):
        st.session_state.status = "IDLE"
        st.session_state.robot = RobotSimulator(st.session_state.robot.obstacles)

st.divider()

st.subheader("Robot Status")
st.metric("Current State", st.session_state.status)

st.subheader("Robot Workstation")
robot = st.session_state.robot
target_x, target_y = st.columns(2)
with target_x:
    x = st.number_input("Target X (m)", value=robot.position.x, step=0.25)
with target_y:
    y = st.number_input("Target Y (m)", value=robot.position.y, step=0.25)

if st.button("Move to target", use_container_width=True):
    result = robot.move_to(Point(x, y))
    if result.blocked:
        st.session_state.status = "SAFETY STOP"
        st.error(f"Movement blocked by {result.obstacle}; robot stopped at a safe clearance.")
    else:
        st.success("Target reached without an obstacle conflict.")

st.write(f"Robot position: ({robot.position.x:.2f}, {robot.position.y:.2f}) m")
readings = robot.detect_obstacles(sensor_range=3.0)
if readings:
    st.warning("Obstacle sensor: " + ", ".join(
        f"{reading.name} ({reading.distance:.2f} m)" for reading in readings
    ))
else:
    st.success("Obstacle sensor: no obstacles within 3.0 m")

st.subheader("System Logs")
st.write("Current robot state:", st.session_state.status)
