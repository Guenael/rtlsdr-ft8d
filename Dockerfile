# -------------------
# The build container
# -------------------
FROM debian:bullseye-slim AS build

RUN apt-get update && \
  apt-get -y --no-install-recommends install \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    git \
    libcurl4-openssl-dev \
    libfftw3-dev \
    libusb-1.0-0-dev \
    pkg-config \
    help2man && \
  rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/steve-m/librtlsdr.git /root/librtlsdr && \
  cd /root/librtlsdr && \
  mkdir -p /root/librtlsdr/build && \
  cd /root/librtlsdr/build && \
  cmake -Wno-dev ../ && \
  make && \
  make install && \
  rm -rf /root/librtlsdr

COPY . /root/rtlsdr_ft8d

RUN cd /root/rtlsdr_ft8d && \
  git submodule update --init --recursive && \
  make && \
  make install

# -------------------------
# The application container
# -------------------------
FROM debian:bullseye-slim

RUN apt-get update && \
  apt-get -y --no-install-recommends install \
   	libcurl4 \
    libfftw3-single3 \
    usbutils && \
  rm -rf /var/lib/apt/lists/*

COPY --from=build /usr/local/lib/librtlsdr.so.0 /usr/local/lib/librtlsdr.so.0
COPY --from=build /usr/local/bin/rtlsdr_ft8d /usr/local/bin/rtlsdr_ft8d
RUN ldconfig

ENTRYPOINT ["/usr/local/bin/rtlsdr_ft8d"]
