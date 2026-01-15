#!/bin/bash
# Скрипт для исправления проблем Nav2

set -e

echo "=== Исправление проблем Nav2 ==="
echo ""

# 1. Остановить дублирующиеся узлы
echo "1. Остановка дублирующихся узлов..."
pkill -9 -f "symovo_nav2" || true
pkill -9 -f "ros2 launch" || true
sleep 2

# 2. Очистить lock файлы
echo "2. Очистка lock файлов..."
rm -f /tmp/base_controller.lock
rm -f /tmp/navigation_integrated_node.lock
rm -f /tmp/symovo_nav2_stack.lock
echo "✅ Lock файлы очищены"

# 3. Проверить, что все процессы остановлены
echo "3. Проверка процессов..."
if pgrep -f "symovo_nav2" > /dev/null; then
    echo "⚠️  Предупреждение: некоторые процессы всё ещё запущены"
    ps aux | grep -E "(symovo_nav2|ros2 launch)" | grep -v grep
else
    echo "✅ Все процессы остановлены"
fi

echo ""
echo "=== Готово ==="
echo "Теперь можно запустить launch файл заново:"
echo "ros2 launch aehub_navigation symovo_nav2.launch.py ..."
