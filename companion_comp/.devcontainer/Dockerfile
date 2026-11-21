FROM ubuntu:24.04

# Забороняємо інтерактивні запити під час встановлення
ENV DEBIAN_FRONTEND=noninteractive

# Встановлюємо лише перевірені базові інструменти
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gdb \
    clang-format \
    sudo\
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Створюємо користувача developer (UID 1000 — стандартний для WSL/Linux)
# та дозволяємо йому використовувати sudo без введення пароля
RUN usermod -l mult ubuntu && \
    groupmod -n mult ubuntu && \
    usermod -d /home/mult -m mult && \
    echo "mult ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Створюємо папку проекту, змінюємо її власника на нашого користувача
RUN mkdir -p /MavLink_Autopilot && chown -R mult:mult /MavLink_Autopilot

# Перемикаємося на роботу від імені створеного користувача (non-root)
USER mult

# Задаємо нову дефолтну робочу директорію всередині контейнера
WORKDIR /MavLink_Autopilot

