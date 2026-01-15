# Защита от дубликатов узлов ROS2

## Проблема

Дублирование узлов ROS2 может вызывать:
- Конфликты топиков (несколько издателей на один топик)
- Конфликты сервисов
- Проблемы с lifecycle управлением
- Непредсказуемое поведение системы
- Лишнее использование ресурсов

## Реализованная защита

### 1. Проверка в коде узла (`navigation_integrated_node`)

Узел `navigation_integrated_node` проверяет наличие дубликатов при запуске:

```python
# В navigation_integrated_node.py
existing_nodes = self.get_node_names()
duplicate_count = sum(1 for name in existing_nodes if name == 'navigation_integrated_node')

if duplicate_count > 1:
    raise RuntimeError('Multiple instances detected!')
```

**Поведение:** Если обнаружен дубликат, узел завершается с ошибкой, предотвращая запуск второго экземпляра.

### 2. Скрипт очистки дубликатов

**Файл:** `scripts/cleanup_duplicate_nodes.sh`

Автоматически находит и останавливает дублирующиеся процессы:

```bash
# Запуск очистки
bash scripts/cleanup_duplicate_nodes.sh
```

**Что делает:**
- Находит все экземпляры `base_controller_node`
- Находит все экземпляры `lifecycle_manager_navigation`
- Оставляет только первый экземпляр каждого узла
- Останавливает остальные

### 3. Скрипт проверки существования узла

**Файл:** `scripts/check_node_exists.py`

Проверяет, существует ли узел в ROS2 графе:

```bash
# Проверка существования узла
python3 scripts/check_node_exists.py base_controller
```

**Использование:**
- В launch файлах (через условия)
- В скриптах перед запуском
- Для диагностики

### 4. Скрипт диагностики дубликатов

**Файл:** `scripts/check_duplicate_nodes.py`

Показывает все дублирующиеся узлы:

```bash
# Проверка дубликатов
python3 scripts/check_duplicate_nodes.py
```

## Рекомендации по использованию

### Перед запуском launch файла

1. **Проверить дубликаты:**
   ```bash
   python3 scripts/check_duplicate_nodes.py
   ```

2. **Очистить дубликаты (если есть):**
   ```bash
   bash scripts/cleanup_duplicate_nodes.sh
   ```

3. **Запустить launch файл:**
   ```bash
   ros2 launch symovo_nav2.launch.py
   ```

### В launch файлах

Для предотвращения дубликатов в launch файлах рекомендуется:

1. **Использовать composable nodes** (узлы в контейнере не дублируются):
   ```python
   ComposableNodeContainer(
       name='nav2_container',
       composable_node_descriptions=[...]
   )
   ```

2. **Проверять перед запуском** (через условия):
   ```python
   Node(
       ...,
       condition=UnlessCondition(check_node_exists)
   )
   ```

3. **Использовать respawn только при необходимости:**
   ```python
   Node(
       ...,
       respawn=False  # По умолчанию False
   )
   ```

### Автоматическая защита

Узел `navigation_integrated_node` автоматически:
- ✅ Проверяет дубликаты при запуске
- ✅ Завершается с ошибкой, если обнаружен дубликат
- ✅ Логирует предупреждение в консоль

## Типичные причины дубликатов

1. **Множественные launch файлы** - запуск одного и того же launch файла несколько раз
2. **Respawn=true** - автоматический перезапуск создает дубликаты при сбоях
3. **Ручной запуск узлов** - запуск узла вручную, когда он уже запущен через launch
4. **Systemd сервисы** - несколько сервисов запускают один узел
5. **Скрипты автозапуска** - несколько скриптов запускают один узел

## Решение проблем

### Если обнаружены дубликаты:

1. **Остановить все launch процессы:**
   ```bash
   pkill -f "ros2 launch"
   ```

2. **Очистить дубликаты:**
   ```bash
   bash scripts/cleanup_duplicate_nodes.sh
   ```

3. **Проверить результат:**
   ```bash
   python3 scripts/check_duplicate_nodes.py
   ```

4. **Перезапустить систему:**
   ```bash
   ros2 launch symovo_nav2.launch.py
   ```

### Если узел не запускается из-за "duplicate detected":

Это нормальное поведение защиты! Это означает, что:
- Узел уже запущен
- Защита предотвратила создание дубликата

**Решение:** Проверьте, не запущен ли узел уже:
```bash
ros2 node list | grep navigation_integrated_node
```

## Best Practices

1. ✅ **Всегда используйте один launch файл** для запуска всей системы
2. ✅ **Проверяйте дубликаты перед запуском** через `check_duplicate_nodes.py`
3. ✅ **Не запускайте узлы вручную**, если они уже в launch файле
4. ✅ **Используйте composable nodes** для Nav2 компонентов
5. ✅ **Отключите respawn** для критичных узлов (или используйте осторожно)
6. ✅ **Мониторьте систему** на наличие дубликатов

## Мониторинг

Регулярно проверяйте систему:

```bash
# Ежедневная проверка
python3 scripts/check_duplicate_nodes.py

# При проблемах с навигацией
bash scripts/cleanup_duplicate_nodes.sh
```

## См. также

- `scripts/check_duplicate_nodes.py` - диагностика дубликатов
- `scripts/cleanup_duplicate_nodes.sh` - очистка дубликатов
- `scripts/check_node_exists.py` - проверка существования узла
- `src/aehub_navigation/src/aehub_navigation/navigation_integrated_node.py` - защита в коде
