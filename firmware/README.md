# Promin-1 — прошивка (ESP-IDF)

Прошивка γ-дозиметра «Промінь-1» для **ESP32-C3 Super Mini**.

Основний код: [`main/main.cpp`](main/main.cpp)
Заставка OLED: [`main/splash_bitmap.h`](main/splash_bitmap.h)

## Складання

```bash
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

## Ключові константи (main.cpp)

| Константа | Значення | Призначення |
|---|---|---|
| `GEIGER_CPM_PER_USVH` | 800 | Калібрування CPM → мкЗв/год |
| `GEIGER_CPM_WINDOW_SEC` | 60 | Вікно усереднення дози |
| `GEIGER_CPM_INST_SEC` | 10 | Вікно швидкого CPM |
| `GEIGER_DEADTIME_US` | 2500 | Антидребезг трубки |
| `BAT_CHECK_MS` | 120000 | Період опитування АКБ |
| `SPLASH_MS` | 2000 | Тривалість заставки |
| `SPLASH_INVERT_COLORS` | 1 | Інверсія кольорів заставки |

## Розводка GPIO

Див. коментар на початку `main/main.cpp` і `README.md` у корені репозиторію.

> Заставку OLED можна перегенерувати з PNG скриптом `tools/png2splash.py`
> (якщо він присутній): `py -3 tools/png2splash.py docs/splash.png -o firmware/main/splash_bitmap.h`
