import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys

# Arm parameters (lengths of links)
L1 = 5
L2 = 3

def inverse_kinematics(x, y):
    """Compute joint angles given end-effector position (x, y)"""
    d = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    if np.abs(d) > 1:
        return None, None  # No solution (out of reach)

    theta2 = np.arctan2(np.sqrt(1 - d**2), d)  # Elbow up solution
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    return np.degrees(theta1), np.degrees(theta2)

def plot_robot(theta1, theta2):
    """Plot the 2-DOF robot arm with angle arcs"""
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    x1 = L1 * np.cos(theta1_rad)
    y1 = L1 * np.sin(theta1_rad)
    
    x2 = x1 + L2 * np.cos(theta1_rad + theta2_rad)
    y2 = y1 + L2 * np.sin(theta1_rad + theta2_rad)
    
    fig, ax = plt.subplots(figsize=(5,5))
    ax.plot([0, x1], [0, y1], 'ro-', lw=4, label="Link 1")
    ax.plot([x1, x2], [y1, y2], 'bo-', lw=4, label="Link 2")
    
    # Draw angle arcs
    arc1 = patches.Arc((0, 0), 2, 2, angle=0, theta1=0, theta2=theta1, color='red')
    arc2 = patches.Arc((x1, y1), 1.5, 1.5, angle=np.degrees(theta1_rad), theta1=0, theta2=theta2, color='blue')
    ax.add_patch(arc1)
    ax.add_patch(arc2)
    
    ax.set_xlim(- (L1 + L2), (L1 + L2))
    ax.set_ylim(- (L1 + L2), (L1 + L2))
    ax.set_aspect('equal')
    ax.grid()
    ax.legend()
    
    return fig, theta1, theta2

# Streamlit UI
st.title("2-DOF Arm Controller")

st.subheader("🔧 Inverse Kinematics Control")
x = st.number_input("End-Effector X (cm)", value=5.0, step=0.1)
y = st.number_input("End-Effector Y (cm)", value=5.0, step=0.1)

# Add a "Send" button to flush output to stdout only when clicked
if st.button("Send"):
    message = f"UI_DATA:x:{x},y:{y}\n"
    sys.stdout.write(message)
    sys.stdout.flush()
    st.success("📤 Data sent to stdout!")

theta1, theta2 = inverse_kinematics(x, y)

if theta1 is None or theta2 is None:
    st.error("🚨 Position out of reach!")
else:
    fig_robot, theta1, theta2 = plot_robot(theta1, theta2)
    st.pyplot(fig_robot)
    st.write(f"**Computed Angles:** θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°")


