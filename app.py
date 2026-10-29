import streamlit as st

from simulator import Obstacle, Point, Simulator

st.set_page_config(page_title="Physical AI Robot Simulator", page_icon="🤖", layout="wide")
st.title("🤖 Physical AI Robot Simulator")
st.caption("Robot Pick-and-Place Control Dashboard — 2D collision simulation")

if "simulator" not in st.session_state:
    st.session_state.simulator = Simulator(obstacles=(
        Obstacle(Point(4, 4), 0.8), Obstacle(Point(7, 6), 1.0),
        Obstacle(Point(3, 7), 0.5),
    ))
sim = st.session_state.simulator

col1, col2, col3 = st.columns(3)
with col1:
    st.button("▶ Start", on_click=sim.start, use_container_width=True)
with col2:
    st.button("⚠ Simulate Failure", on_click=sim.fail, use_container_width=True)
with col3:
    st.button("↻ Reset", on_click=sim.reset, use_container_width=True)

st.divider()
st.subheader("Robot Status")
st.metric("Current State", sim.status)
st.subheader("Movement")
with st.form("movement"):
    dx = st.number_input("X movement (m)", min_value=-10.0, max_value=10.0, value=0.5, step=0.1)
    dy = st.number_input("Y movement (m)", min_value=-10.0, max_value=10.0, value=0.0, step=0.1)
    if st.form_submit_button("Move", disabled=sim.status != "RUNNING"):
        sim.move(dx, dy)
        st.rerun()

st.subheader("Robot Workstation")
st.caption("10 × 10 m workspace. Blue: robot; red: obstacles. Dashed rings include robot radius and safety margin.")
# All interpolated SVG coordinates come from the numeric model.
circles = "".join(
    f'<circle cx="{o.center.x}" cy="{o.center.y}" r="{o.radius}" fill="#dc4444"/>'
    f'<circle cx="{o.center.x}" cy="{o.center.y}" r="{o.radius + sim.robot_radius + sim.safety_margin}" '
    'fill="none" stroke="#dc4444" stroke-width="0.03" stroke-dasharray="0.12 0.1"/>'
    for o in sim.obstacles
)
st.markdown(
    '<svg viewBox="-0.2 -0.2 10.4 10.4" width="100%" style="max-width:550px" '
    'role="img" aria-label="Robot position and circular obstacles">'
    '<g transform="translate(0 10) scale(1 -1)">'
    '<rect width="10" height="10" fill="#f5f5f5" stroke="#555" stroke-width="0.05"/>'
    f'{circles}<circle cx="{sim.position.x}" cy="{sim.position.y}" '
    f'r="{sim.robot_radius}" fill="#287acc"/></g></svg>',
    unsafe_allow_html=True,
)
st.write(f"Position: ({sim.position.x:.2f}, {sim.position.y:.2f}) m")
st.write(f"Robot radius: {sim.robot_radius:.2f} m · Safety margin: {sim.safety_margin:.2f} m")
st.write(f"Nearest wall clearance: {sim.wall_clearance:.2f} m")
readings = sim.detections()
if readings:
    st.warning("Detected obstacles: " + ", ".join(
        f"#{index + 1}: {clearance:.2f} m clearance" for index, clearance in readings))
else:
    st.info(f"No obstacles within {sim.sensor_range:.1f} m sensing range.")
st.caption("Known geometry is checked along the entire path, including beyond sensing range. "
           "Unsafe commands leave the robot in place and latch a stop until Reset.")
st.subheader("Latest System Event")
st.write(sim.message)
