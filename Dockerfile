FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
  pkg-config \
  build-essential \
  opendlv-standard-message-set-generator \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/chalmers-revere/opendlv-standard-message-set.git /opt/opendlv-msgset \
    && cd /opt/opendlv-msgset/tools/odvd \
    && mkdir -p build && cd build \
    && cmake .. && make -j$(nproc) && make install

WORKDIR ["/opt/template-opencv"]
