"""
ui.py: Streamlit UI for ROS2 Data Input

Provides a simple Streamlit interface to send x and y values to a ROS2 node.

    - Text input fields for x and y values.
    - "Send" button to transmit data.
    - Data is sent to the ROS2 node via standard output, prefixed with "UI_DATA:".

Data Format:
    - "UI_DATA:x:<x_value>,y:<y_value>\n"

Usage:
    - Run via Streamlit: `streamlit run ui.py`.
    - Input x and y, click "Send".
"""

import streamlit as st
import sys

st.title("ROS2 UI Controller")

x_input = st.text_input("Enter x:")
y_input = st.text_input("Enter y:")

if st.button("Send"):
    if x_input and y_input:
        message = f"UI_DATA:x:{x_input},y:{y_input}\n"  # Add a prefix to mark UI data
        sys.stdout.write(message)
        sys.stdout.flush()
    else:
        st.warning("Please enter both x and y.")