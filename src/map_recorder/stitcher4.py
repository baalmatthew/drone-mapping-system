#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os

# === ПАРАМЕТРЫ НАСТРОЙКИ ===
image_dir = os.path.expanduser('~/images')

cols = 4        # количество кадров в одном ряду
rows = 4        # количество рядов
scale = 1.0     # масштаб (1.0 — оригинал)
skip_step = 5   # 🔸 пропуск кадров (1 — все, 2 — через один и т.д.)
rotate_backwards_rows = True  # 🔸 переворачивать ряды, где дрон летел в обратном направлении
rotate_each_image = True      # 🔸 поворачивать каждый кадр на 90° по часовой стрелке

# === ЗАГРУЗКА ИЗОБРАЖЕНИЙ ===
images = sorted(glob.glob(os.path.join(image_dir, 'frame_*.jpg')))
if not images:
    print("❌ Нет изображений в папке ~/images")
    exit()

# Пропуск кадров
if skip_step > 1:
    images = images[::skip_step]

print(f"[INFO] Используется {len(images)} изображений (с учётом пропуска)")

# Пробуем загрузить одно изображение, чтобы узнать его размер
sample = cv2.imread(images[0])
if sample is None:
    print("❌ Ошибка чтения изображений")
    exit()

orig_h, orig_w, _ = sample.shape
h, w = int(orig_h * scale), int(orig_w * scale)

# === СОЗДАЁМ ПУСТОЕ ПОЛОТНО ПОД КАРТУ ===
# Поскольку поворот меняет размеры, используем максимальный размер каждого блока
block_h, block_w = (w, h) if rotate_each_image else (h, w)
canvas_height = rows * block_h
canvas_width = cols * block_w
canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

# === ОСНОВНОЙ ЦИКЛ РАЗМЕЩЕНИЯ ===
img_id = 0
for row in range(rows):
    # Определяем направление обхода
    if row % 2 == 0:
        x_range = range(cols)
        reverse = False
    else:
        x_range = reversed(range(cols))
        reverse = rotate_backwards_rows

    for col in x_range:
        if img_id >= len(images):
            break

        img = cv2.imread(images[img_id])
        if img is None:
            print(f"⚠️ Пропущен файл {images[img_id]} (ошибка чтения)")
            img_id += 1
            continue

        # Масштабирование
        if scale != 1.0:
            img = cv2.resize(img, (w, h))

        # Поворот каждого кадра на 90° по часовой стрелке
        if rotate_each_image:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        # Размеры после поворота
        img_h, img_w = img.shape[:2]

        # Разворот ряда
        if reverse:
            img = cv2.rotate(img, cv2.ROTATE_180)

        # Координаты вставки
        y0, y1 = row * block_h, row * block_h + img_h
        x0, x1 = col * block_w, col * block_w + img_w

        # Проверка границ (на случай, если размеры не совпали)
        y1 = min(y1, canvas.shape[0])
        x1 = min(x1, canvas.shape[1])
        img_h = y1 - y0
        img_w = x1 - x0

        canvas[y0:y1, x0:x1] = img[:img_h, :img_w]

        img_id += 1

# === СОХРАНЕНИЕ РЕЗУЛЬТАТА ===
output_path = os.path.join(image_dir, 'map_stitched.jpg')
cv2.imwrite(output_path, canvas)

print("✅ Сшивка завершена успешно.")
print(f"📸 Использовано {img_id} изображений.")
print(f"🗺️  Итоговая карта сохранена: {output_path}")
print(f"📏 Размер итоговой карты: {canvas.shape[1]}×{canvas.shape[0]} пикселей.")
