FROM universalrobots/ursim_e-series

ARG URCAPS_DIR=/docker/ursim/urcaps
ARG GRIPPER_URCAPS_NAME=Robotiq_Grippers-1.8.13.22852

# Install the URCap
# It is important to remap to .jar extension so Polyscope could read it
COPY --chmod=755 ${URCAPS_DIR}/${GRIPPER_URCAPS_NAME}.urcap /urcaps/${GRIPPER_URCAPS_NAME}.jar
