# PX4-Autopilot development container
#
# The source tree itself is NOT copied into the image — it is bind-mounted
# live by docker-compose.yml so edits on the host are reflected instantly.
# This image only layers the small bits of config needed on top of the
# official PX4 development environment.
FROM px4io/px4-dev:v1.17.0

# The source is bind-mounted from an arbitrary host UID/GID, so allow git to
# operate on it regardless of ownership (applies to every user in the image).
RUN git config --system --add safe.directory '*'

# Where docker-compose.yml bind-mounts the PX4-Autopilot source tree.
WORKDIR /workspace/PX4-Autopilot
