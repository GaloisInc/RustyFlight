FROM fedora:latest
ARG USERNAME=rustyflight
ARG PASSWORD=$USERNAME

ENV ARM_SDK_URL_BASE https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major

RUN useradd -m -G wheel $USERNAME && (echo "$USERNAME:$PASSWORD" | chpasswd)
RUN dnf install -y            \
       bzip2                  \
       clang                  \
       clang-devel            \
       clang-libs             \
       cmake                  \
       curl                   \
       findutils              \
       git                    \
       libblocksruntime-devel \
       llvm-devel             \
       llvm-static            \
       make                   \
       openssl-devel          \
       pkgconf-pkg-config     \
       python                 \
       ripgrep                \
       sudo                   \
       tmux                   \
       vim                    \
       zsh

RUN dnf -y install python-pip

WORKDIR /tmp
RUN curl -Lo arm-utils.tar.bz2 ${ARM_SDK_URL_BASE}-linux.tar.bz2
RUN mkdir /opt/arm
RUN tar -C /opt/arm --strip-components 1 -xjf arm-utils.tar.bz2
RUN pip install scan-build

ENV RUST_NIGHTLY_TOOLCHAIN=nightly-2019-12-05

USER $USERNAME
WORKDIR /home/$USERNAME
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH ${PATH}:/opt/arm/bin:/home/$USERNAME/.cargo/bin
RUN rustup update
RUN rustup install ${RUST_NIGHTLY_TOOLCHAIN}
RUN rustup component add rustfmt
RUN rustup target install thumbv7m-none-eabi --toolchain stable
RUN rustup target install thumbv7m-none-eabi --toolchain ${RUST_NIGHTLY_TOOLCHAIN}
RUN cargo +${RUST_NIGHTLY_TOOLCHAIN} install c2rust
RUN rustup default ${RUST_NIGHTLY_TOOLCHAIN}
