# Промінь-1 — γ-дозиметр на СІ-22Г / Promin-1 — SI-22G gamma dosimeter

**🇺🇦 Українська** | [🇬🇧 English](README.en.md)

Портативний побутовий гамма-дозиметр на базі лічильної трубки Гейгера–Мюллера
**СІ-22Г** та мікроконтролера **ESP32-C3 Super Mini**.

> ⚠️ **Hobby-проєкт.** Це не сертифікований і не еталонно відкалібрований
> прилад. Не покладайтеся на нього у питаннях здоров'я чи безпеки.
> Зроблено *only for fun*.

## 🎬 Демонстрація роботи

<video src="https://github.com/dark-node/Dosimeter-Radiometer.-Promin-1.-DIY-Project.-Based-on-the-ESP32-C3-mini-and-SI-22G/raw/main/media/demo-test.mp4" controls width="640" poster="media/photos-final/front-view.jpg">
  Ваш переглядач не підтримує вбудоване відео.
</video>

📹 Якщо відео не відтворюється тут — [відкрити / завантажити відео тесту](media/demo-test.mp4)

---

## Можливості

- Реєстрація імпульсів трубки через апаратне переривання (GPIO ISR).
- Розрахунок **CPM** (вікна 60 с і 10 с) та потужності дози **мкЗв/год**.
- OLED-дисплей SSD1306 128×32, три режими: `мкЗв/год`, `CPM`, `cnt`.
- Звуковий клік (п'єзо) і світлодіодний спалах на кожен імпульс.
- Індикація заряду акумулятора (резистивний дільник, опит раз на 2 хв).
- Заставка при старті (2 с).

## Технічні параметри

| Параметр | Значення |
|---|---|
| Трубка | СІ-22Г (гамма) |
| Висока напруга | ~405 В (готовий DC-DC модуль) |
| Анодний баласт | 10 МОм |
| Зняття імпульсу | з катода, формувач на BC547B → GPIO2 |
| Калібрування | 800 CPM на 1 мкЗв/год |
| Фон | ≈0.10–0.12 мкЗв/год |
| MCU | ESP32-C3 Super Mini |
| Дисплей | OLED SSD1306 128×32, I2C 400 кГц |
| Живлення | Li-ion 18650 |

Повний опис, історія розробки та BOM — у [`docs/Description.txt`](docs/Description.txt).

## Розводка GPIO (ESP32-C3)

| GPIO | Призначення |
|---|---|
| 2 | Вхід імпульсів СІ-22Г (ISR) |
| 3 | ADC заряду АКБ |
| 4 / 5 | OLED SDA / SCL |
| 6 | Світлодіод (через 330 Ω) |
| 7 / 10 | П'єзо (push-pull) |
| 8 / 9 / 20 | Кнопки: CPM / мкЗв·год / лічильник |
| 3V3 | Живлення OLED (НЕ 5 В) |

Схема детектора:

```
HV+ ──[10M]── анод СІ-22Г ── катод ── DET
DET ──[10k]── GND
DET ──[470nF]──[50k]── база BC547B ──[4.7M]── GND
емітер ── GND
колектор ──[15k]── 3V3,  відгалуження → GPIO2
```

> ⚡ Не використовувати GPIO0, GPIO1, GPIO21 (boot/UART).
> Спільна земля: ESP, HV-модуль, катод трубки, емітер транзистора.

## Структура репозиторію

```
.
├── firmware/             # Прошивка ESP-IDF (C/C++)
│   └── main/main.cpp
├── hardware/
│   ├── schematics/       # LTspice: схема з'єднань + симуляції
│   └── 3d-models/        # STL/Blender корпусу + renders/
├── media/
│   ├── photos-final/     # Фото готового пристрою
│   ├── photos-build/     # Фото процесу збірки
│   └── demo-test.mp4     # Відео тесту роботи
├── docs/
│   ├── Description.txt    # Опис, історія, BOM, специфікація
│   └── splash.png         # Вихідне зображення заставки OLED
├── LICENSE               # MIT
├── README.md             # Українська (цей файл)
└── README.en.md          # English
```

## Складання та прошивка

Потрібен [ESP-IDF](https://docs.espressif.com/projects/esp-idf/). У теці `firmware/`:

```bash
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

## Схема (LTspice)

Файл [`hardware/schematics/promin1-connections.asc`](hardware/schematics/promin1-connections.asc)
— схема з'єднань для збірки (довідник, не для симуляції).

![Schematic](hardware/schematics/promin1-connections.png)

## Галерея

| | | |
|---|---|---|
| ![](media/photos-final/front-view.jpg) | ![](media/photos-final/usvh-view.jpg) | ![](media/photos-final/cpm-view.jpg) |
| ![](media/photos-final/top-view.jpg) | ![](media/photos-final/left-view.jpg) | ![](media/photos-final/right-view.jpg) |

3D-моделі корпусу: [`hardware/3d-models/`](hardware/3d-models/)

## Ліцензія

[MIT](LICENSE). Hobby-пристрій, use at your own risk.
