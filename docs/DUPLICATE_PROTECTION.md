# Защита от дубликатов нод

## Реализованная защита

Обе рукописные ноды (`navigation_integrated_node` и `base_controller_node`) теперь имеют встроенную защиту от дубликатов при запуске.

### Механизм защиты

Используется **файловая блокировка (file locking)** через `fcntl.flock()` (Python) и `fcntl()` (C++).

#### Принцип работы:

1. При запуске нода пытается получить **эксклюзивную блокировку** на файл:
   - `navigation_integrated_node`: `/tmp/navigation_integrated_node.lock`
   - `base_controller_node`: `/tmp/base_controller.lock`

2. Если блокировка получена:
   - Нода записывает свой PID в файл
   - Нода продолжает работу
   - Файл остается открытым до завершения ноды

3. Если блокировка НЕ получена (файл уже заблокирован):
   - Нода выводит **CRITICAL ERROR**
   - Нода завершается с кодом ошибки (exit code 1)
   - Второй экземпляр не запускается

4. При завершении ноды:
   - Блокировка автоматически освобождается
   - Файл закрывается
   - Следующий экземпляр может запуститься

### Преимущества

✅ **Надежность**: Файловая блокировка работает на уровне ОС, гарантируя эксклюзивность  
✅ **Раннее обнаружение**: Проверка происходит ДО создания ROS2 ноды  
✅ **Автоматическое освобождение**: Блокировка освобождается при завершении процесса  
✅ **Простота**: Не требует дополнительных сервисов или конфигурации  

## Использование

### Проверка защиты

```bash
# Если нода уже запущена, попытка запустить второй экземпляр должна выдать ошибку:

# Для navigation_integrated_node:
python3 -m aehub_navigation.navigation_integrated_node --ros-args ...
# Ожидаемый вывод:
# CRITICAL ERROR: navigation_integrated_node is already running!
# Another instance is holding the lock file.
# ...

# Для base_controller_node:
ros2 run base_controller base_controller_node --ros-args ...
# Ожидаемый вывод:
# CRITICAL ERROR: base_controller is already running!
# Another instance is holding the lock file.
# ...
```

### Ручная очистка блокировок

Если нода завершилась некорректно (например, kill -9), блокировка может остаться:

```bash
# Проверить, какие процессы держат блокировки:
lsof /tmp/navigation_integrated_node.lock
lsof /tmp/base_controller.lock

# Если процесс не существует, удалить файл:
rm /tmp/navigation_integrated_node.lock
rm /tmp/base_controller.lock
```

## Технические детали

### Python (navigation_integrated_node)

```python
import fcntl

lock_file = open('/tmp/navigation_integrated_node.lock', 'w')
try:
    fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Lock acquired - continue
    lock_file.write(str(os.getpid()) + '\n')
    lock_file.flush()
except BlockingIOError:
    # Lock held by another process - exit
    sys.exit(1)
```

### C++ (base_controller_node)

```cpp
#include <fcntl.h>

int lock_fd = open("/tmp/base_controller.lock", O_CREAT | O_WRONLY | O_TRUNC, 0644);
struct flock lock;
lock.l_type = F_WRLCK;
lock.l_whence = SEEK_SET;
lock.l_start = 0;
lock.l_len = 0;

if (fcntl(lock_fd, F_SETLK, &lock) < 0) {
    // Lock held by another process - exit
    std::exit(1);
}
```

## Дополнительная проверка

Помимо файловой блокировки, `navigation_integrated_node` также проверяет ROS2 граф после создания ноды (как дополнительная проверка).

## См. также

- `scripts/test_duplicate_protection.sh` - скрипт для тестирования защиты
- `docs/DUPLICATE_NODES_PREVENTION.md` - общая документация по предотвращению дубликатов
