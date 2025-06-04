FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
  pkg-config \
  build-essential \
  cmake \
  git \
  protobuf-compiler \ 
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/chalmers-revere/opendlv-standard-message-set.git /opt/opendlv-msgset \
    && cd /opt/opendlv-msgset/tools/odvd \
    && mkdir -p build && cd build \
    && cmake .. && make -j$(nproc) && make install

WORKDIR ["/opt/template-opencv"]

COPY microservices/steering-angle-microservice/src/opendlv-standard-message-set-v0.9.6.odvd .
RUN opendlv-standard-message-set-generator opendlv-standard-message-set-v0.9.6.odvd