FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
  pkg-config \
  build-essential \
  opendlv-standard-message-set-generator \
  && rm -rf /var/lib/apt/lists/*

WORKDIR ["/opt/template-opencv"]
