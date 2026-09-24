# Build context: repository root (needs libraries/mavlink and ground_station/bridge)
FROM ubuntu:24.04 AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config libmosquitto-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY libraries/mavlink libraries/mavlink
COPY ground_station/bridge ground_station/bridge
RUN cmake -S ground_station/bridge -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libmosquitto1 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/build/mavlink_mqtt_bridge /src/build/fake_vehicle /usr/local/bin/
ENTRYPOINT ["mavlink_mqtt_bridge"]
