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