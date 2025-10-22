# Комплекс для картографирования рельефа с БПЛА на ROS2

Программно-алгоритмический комплекс для автоматического картографирования рельефа с использованием беспилотных летательных аппаратов и фреймворка ROS 2.

## Содержание
- [Описание](#описание)
- [Архитектура системы](#архитектура-системы)
- [Установка и настройка](#установка-и-настройка)
- [Запуск системы](#запуск-системы)
- [Структура проекта](#структура-проекта)
- [Результаты](#результаты)

## Описание

Проект реализует систему автоматического картографирования местности с использованием:
- **БПЛА X500** с монохромной камерой
- **PX4 Autopilot** для управления полетом
- **ROS 2 Jazzy** для обработки данных
- **Gazebo Harmonic** для симуляции
- **QGroundControl** для планирования миссий

## Архитектура системы

PX4 Autopilot ←→ MicroXRCE-DDS-Agent ←→ ROS 2 ←→ Gazebo Bridge ←→ RViz2
↑
QGroundControl

## Установка и настройка

### Предварительные требования
- Ubuntu 22.04+ или WSL2
- 8GB+ RAM
- 20GB+ свободного места

### 1. Установка компонентов

```bash
# Клонирование репозитория
git clone https://github.com/your-username/terrain-mapping-ros2.git
cd terrain-mapping-ros2

# Запуск автоматической установки
chmod +x scripts/setup.sh
./scripts/setup.sh
```
2. Ручная установка (альтернатива)
См. полную инструкцию в INSTALLATION.md

### Запуск системы
1. Автоматический запуск (рекомендуется)

```bash
chmod +x scripts/start_sim_tmux.sh
./scripts/start_sim_tmux.sh
```

2. Ручной запуск компонентов
Запуск симуляции PX4:
```bash
cd PX4-Autopilot/
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_mono_cam_down
```

Запуск DDS агента:
```bash
MicroXRCEAgent udp4 -p 8888
```

Запуск ROS2-Gazebo моста:
```bash
ros2 run ros_gz_bridge parameter_bridge /world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

Запуск визуализации:
```bash
rviz2
```

### Структура проекта

```bash
terrain-mapping-ros2/
├── scripts/
│   ├── setup.sh              # Скрипт установки
│   ├── start_sim_tmux.sh     # Запуск симуляции
│   └── image_processing/     # Обработка изображений
├── src/
│   ├── map_recorder/         # Пакет записи карт
│   │   ├── image_saver.py    # Сохранение изображений
│   │   └── stitcher.py       # Сшивание карт
│   └── config/               # Конфигурационные файлы
├── docs/
│   ├── INSTALLATION.md       # Инструкция по установке
│   └── TROUBLESHOOTING.md    # Решение проблем
└── results/                  # Примеры результатов
```
Результаты
Система позволяет:

 - Автоматически выполнять полетные миссии

 - Записывать изображения местности с камеры БПЛА

 - Сшивать отдельные снимки в единую карту

 - Визуализировать данные в реальном времени в RViz2

