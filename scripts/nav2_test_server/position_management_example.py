#!/usr/bin/env python3
"""
Пример использования динамического управления позициями

Демонстрирует как добавлять, удалять и сохранять позиции в PositionRegistry.
"""

import sys
import os

# Добавляем путь к пакету
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'aehub_navigation', 'src'))

from aehub_navigation.position_registry import PositionRegistry

def main():
    # Создаем экземпляр PositionRegistry
    registry = PositionRegistry()
    
    # Загружаем существующие позиции
    positions_file = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'positions.yaml')
    if not registry.loadFromYAML(positions_file):
        print(f"Ошибка: не удалось загрузить позиции из {positions_file}")
        return
    
    print(f"Загружено позиций: {registry.getPositionCount()}")
    print(f"ID позиций: {registry.getAllPositionIds()}")
    print()
    
    # Добавляем новую позицию
    print("Добавляем новую позицию 'position_F'...")
    registry.addPosition(
        position_id="position_F",
        x=10.0,
        y=20.0,
        theta=1.57,
        description="Новая позиция F"
    )
    print(f"Теперь позиций: {registry.getPositionCount()}")
    print()
    
    # Обновляем существующую позицию
    print("Обновляем позицию 'position_A'...")
    registry.addPosition(
        position_id="position_A",
        x=15.0,
        y=25.0,
        theta=0.0,
        description="Обновленная позиция A"
    )
    print()
    
    # Удаляем позицию
    print("Удаляем позицию 'position_B'...")
    if registry.removePosition("position_B"):
        print("Позиция удалена успешно")
    else:
        print("Позиция не найдена")
    print()
    
    print(f"Итоговое количество позиций: {registry.getPositionCount()}")
    print(f"ID позиций: {registry.getAllPositionIds()}")
    print()
    
    # Сохраняем изменения в файл
    backup_file = positions_file + ".backup"
    print(f"Сохраняем изменения в {backup_file}...")
    if registry.saveToYAML(backup_file):
        print("Позиции успешно сохранены!")
    else:
        print("Ошибка при сохранении позиций")
    
    print("\nПример использования:")
    print("  # Добавить позицию:")
    print("  registry.addPosition('position_X', x=1.0, y=2.0, theta=0.0, description='Описание')")
    print()
    print("  # Удалить позицию:")
    print("  registry.removePosition('position_X')")
    print()
    print("  # Сохранить в файл:")
    print("  registry.saveToYAML('config/positions.yaml')")

if __name__ == "__main__":
    main()

