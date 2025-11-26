#!/bin/bash
rostopic pub /llm_motion/instruction std_msgs/String "$1"
