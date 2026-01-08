#!/usr/bin/env python3
"""
Проверка импорта PositionRegistry перед запуском сервера
"""

import sys
import os

# Добавляем путь к исходникам ПЕРЕД установленным пакетом
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # /home/boris/ros2_ws/scripts
WORKSPACE_ROOT = os.path.dirname(BASE_DIR)  # /home/boris/ros2_ws
AEHUB_NAV_PATH = os.path.join(WORKSPACE_ROOT, 'src', 'aehub_navigation', 'src')
if os.path.exists(AEHUB_NAV_PATH):
    sys.path.insert(0, AEHUB_NAV_PATH)

# Принудительная перезагрузка модуля
if 'aehub_navigation.position_registry' in sys.modules:
    del sys.modules['aehub_navigation.position_registry']
if 'aehub_navigation' in sys.modules:
    del sys.modules['aehub_navigation']

print(f"BASE_DIR: {BASE_DIR}")
print(f"AEHUB_NAV_PATH: {AEHUB_NAV_PATH}")
print(f"sys.path[0:3]: {sys.path[0:3]}")

try:
    from aehub_navigation.position_registry import PositionRegistry
    import aehub_navigation.position_registry as pr_module
    
    print(f"\n✅ Successfully imported PositionRegistry")
    print(f"   From: {pr_module.__file__}")
    
    registry = PositionRegistry()
    
    # Проверяем методы
    required_methods = ['addPosition', 'removePosition', 'saveToYAML', 'loadFromYAML', 'getPosition', 'hasPosition']
    missing_methods = [m for m in required_methods if not hasattr(registry, m)]
    
    if missing_methods:
        print(f"\n❌ Missing methods: {missing_methods}")
        print(f"   Available methods: {[m for m in dir(registry) if not m.startswith('_')]}")
        sys.exit(1)
    else:
        print(f"\n✅ All required methods present:")
        for method in required_methods:
            print(f"   - {method}")
        
        # Тестируем addPosition
        print(f"\n🧪 Testing addPosition...")
        result = registry.addPosition("test_position", 1.0, 2.0, 0.0, "Test")
        if result:
            print(f"   ✅ addPosition works!")
        else:
            print(f"   ❌ addPosition failed!")
            sys.exit(1)
        
        print(f"\n✅ All checks passed! Ready to start server.")
        sys.exit(0)
        
except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

