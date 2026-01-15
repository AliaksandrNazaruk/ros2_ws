# Руководство по загрузке карты из Symovo API

## Важно: offsetX и offsetY

При загрузке карты из Symovo API **всегда** нужно учитывать `offsetX` и `offsetY`, которые определяют origin карты в ROS2.

## Автоматическая загрузка (рекомендуется)

Launch файл `symovo_nav2.launch.py` автоматически загружает карту из Symovo API при запуске, если не указан параметр `map_file`:

```bash
ros2 launch symovo_nav2 symovo_nav2.launch.py
```

Карта будет автоматически загружена через `load_symovo_map.py`, который правильно учитывает `offsetX` и `offsetY`.

## Ручная загрузка карты

Если нужно загрузить карту вручную:

```bash
python3 scripts/load_symovo_map.py <endpoint> <amr_id> <output_dir>
```

Пример:
```bash
python3 scripts/load_symovo_map.py https://192.168.1.100 15 maps/symovo_map
```

## Использование существующей карты

Если карта уже загружена, можно указать путь к ней:

```bash
ros2 launch symovo_nav2 symovo_nav2.launch.py map_file:=maps/symovo_map/map.yaml
```

## Проверка правильности origin

После загрузки карты можно проверить, что origin правильно учитывает offsetX и offsetY:

```bash
python3 scripts/verify_map_origin.py <map_yaml_path> [symovo_endpoint]
```

Пример:
```bash
python3 scripts/verify_map_origin.py maps/symovo_map/map.yaml https://192.168.1.100
```

## Как это работает

1. `load_symovo_map.py` вызывает `SymovoMapLoader` из `aehub_navigation`
2. `SymovoMapLoader` получает метаданные карты из Symovo API, включая `offsetX` и `offsetY`
3. При создании YAML файла, `offsetX` и `offsetY` используются как `origin` карты
4. `map_server` загружает карту с правильным origin, что позволяет планировщику правильно интерпретировать позицию робота

## Проблемы и решения

### Проблема: "Start Coordinates was outside bounds"

**Причина:** Origin карты не учитывает `offsetX` и `offsetY` из Symovo API.

**Решение:** Перезагрузить карту через `load_symovo_map.py`:
```bash
python3 scripts/load_symovo_map.py https://192.168.1.100 15 maps/symovo_map
```

Затем перезапустить `map_server` с новой картой.

### Проблема: Карта не загружается автоматически

**Причина:** Symovo API недоступен или параметры неверны.

**Решение:** 
1. Проверить доступность Symovo API
2. Убедиться, что `symovo_endpoint` и `amr_id` правильные
3. Загрузить карту вручную и указать путь через `map_file`

