FROM osrf/ros:rolling-desktop
SHELL ["/bin/bash", "-c"]

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN apt update
COPY . /app/src/rmw_zenoh
WORKDIR /app
RUN rosdep install --from-paths src --ignore-src --rosdistro rolling -y
RUN source /opt/ros/rolling/setup.bash && source "$HOME/.cargo/env" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/app/src/rmw_zenoh/entrypoint.sh"]
