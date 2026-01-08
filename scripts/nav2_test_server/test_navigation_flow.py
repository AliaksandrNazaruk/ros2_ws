#!/usr/bin/env python3
"""
Test Navigation Flow (без движения робота)

Тестирует весь процесс навигации до момента отправки команд на моторы базы.
Логирует все шаги для проверки корректности работы системы.

ВАЖНО: База робота должна быть заблокирована физически перед запуском теста!

Использование:
    python3 test_navigation_flow.py --target position_B
    python3 test_navigation_flow.py --target Shampoo --timeout 15
"""

import asyncio
import sys
import json
import time
import argparse
from datetime import datetime
from typing import Optional, Dict, Any
import logging

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# Добавляем путь к модулям
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Добавляем родительскую директорию для относительных импортов
parent_dir = os.path.dirname(script_dir)
sys.path.insert(0, parent_dir)

# Импорты модулей
try:
    # Используем абсолютные импорты через sys.path
    import importlib.util
    
    # Загружаем модули напрямую
    spec = importlib.util.spec_from_file_location("robot_position_client", 
                                                   os.path.join(script_dir, "robot_position_client.py"))
    robot_position_client = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot_position_client)
    RobotPositionClient = robot_position_client.RobotPositionClient
    
    spec = importlib.util.spec_from_file_location("config", 
                                                   os.path.join(script_dir, "config.py"))
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)
    Config = config_module.Config
    
    spec = importlib.util.spec_from_file_location("process_manager", 
                                                   os.path.join(script_dir, "process_manager.py"))
    process_manager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(process_manager)
    ProcessManager = process_manager.ProcessManager
    
    # Для mqtt_client нужно загрузить config сначала
    # Создаем fake модуль для относительного импорта
    import types
    fake_nav2_test_server = types.ModuleType('nav2_test_server')
    fake_config_module = types.ModuleType('nav2_test_server.config')
    fake_config_module.Config = Config
    fake_nav2_test_server.config = fake_config_module
    sys.modules['nav2_test_server'] = fake_nav2_test_server
    sys.modules['nav2_test_server.config'] = fake_config_module
    
    spec = importlib.util.spec_from_file_location("nav2_test_server.mqtt_client", 
                                                   os.path.join(script_dir, "mqtt_client.py"))
    mqtt_client_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mqtt_client_module)
    MQTTTestClient = mqtt_client_module.MQTTTestClient
    
except Exception as e:
    logger.error(f"Failed to import modules: {e}")
    logger.error("Make sure you're running from nav2_test_server directory")
    import traceback
    traceback.print_exc()
    sys.exit(1)


class NavigationFlowTester:
    """Тестер потока навигации с подробным логированием"""
    
    def __init__(self, robot_service_url: str = "http://localhost:8110"):
        self.robot_service_url = robot_service_url
        self.position_client: Optional[RobotPositionClient] = None
        self.mqtt_client: Optional[MQTTTestClient] = None
        
        # Загружаем конфигурацию с явным указанием пути к .env
        script_dir = os.path.dirname(os.path.abspath(__file__))
        env_file = os.path.join(script_dir, '.env')
        
        # Создаем Config с явным указанием env_file
        if os.path.exists(env_file):
            # Используем переменные окружения для загрузки .env
            import os as os_module
            # Временно устанавливаем рабочую директорию для загрузки .env
            old_cwd = os_module.getcwd()
            try:
                os_module.chdir(script_dir)
                self.config = Config()
            finally:
                os_module.chdir(old_cwd)
        else:
            self.config = Config()
        
        self.test_results = []
        
    def log_step(self, step_num: int, step_name: str, status: str, details: str = ""):
        """Логирование шага теста"""
        timestamp = datetime.now().isoformat()
        result = {
            "step": step_num,
            "name": step_name,
            "status": status,  # "ok", "error", "warning", "skip"
            "timestamp": timestamp,
            "details": details
        }
        self.test_results.append(result)
        
        status_icon = {
            "ok": "✅",
            "error": "❌",
            "warning": "⚠️",
            "skip": "⏭️"
        }.get(status, "❓")
        
        logger.info(f"{status_icon} Step {step_num}: {step_name}")
        if details:
            logger.info(f"   {details}")
    
    async def test_step_1_check_robot_service(self) -> bool:
        """Шаг 1: Проверка доступности robot_service API"""
        self.log_step(1, "Проверка robot_service API", "ok", f"URL: {self.robot_service_url}")
        
        try:
            self.position_client = RobotPositionClient(base_url=self.robot_service_url)
            
            # Попытка получить список позиций
            positions = await self.position_client.list_positions()
            
            self.log_step(
                1, 
                "Проверка robot_service API", 
                "ok", 
                f"Получено позиций: {len(positions)}"
            )
            
            if positions:
                logger.info(f"   Примеры позиций: {[p.get('position_id') or p.get('name', 'N/A') for p in positions[:3]]}")
            
            return True
            
        except Exception as e:
            self.log_step(
                1, 
                "Проверка robot_service API", 
                "error", 
                f"Ошибка: {str(e)}"
            )
            return False
    
    async def test_step_2_get_target_position(self, target_id: str) -> Optional[Dict[str, Any]]:
        """Шаг 2: Получение координат целевой позиции"""
        self.log_step(2, "Получение координат целевой позиции", "ok", f"target_id: {target_id}")
        
        try:
            position = await self.position_client.get_position(target_id)
            
            if not position:
                self.log_step(
                    2, 
                    "Получение координат целевой позиции", 
                    "error", 
                    f"Позиция '{target_id}' не найдена"
                )
                return None
            
            x = position.get('x', 0.0)
            y = position.get('y', 0.0)
            theta = position.get('theta', 0.0)
            description = position.get('description', '')
            
            self.log_step(
                2, 
                "Получение координат целевой позиции", 
                "ok", 
                f"Координаты: x={x:.2f}m, y={y:.2f}m, theta={theta:.2f}rad ({theta*180/3.14159:.1f}°)"
            )
            
            if description:
                logger.info(f"   Описание: {description}")
            
            return position
            
        except Exception as e:
            self.log_step(
                2, 
                "Получение координат целевой позиции", 
                "error", 
                f"Ошибка: {str(e)}"
            )
            return None
    
    def test_step_3_check_mqtt_connection(self) -> bool:
        """Шаг 3: Проверка подключения к MQTT брокеру"""
        self.log_step(3, "Проверка подключения к MQTT", "ok")
        
        try:
            # Логируем настройки Config Service для отладки
            logger.info(f"   Config Service URL: {self.config.config_service_url}")
            logger.info(f"   Config Service API Key: {'set' if self.config.config_service_api_key else 'not set'}")
            logger.info(f"   MQTT Broker (direct): {self.config.mqtt_broker}")
            
            self.mqtt_client = MQTTTestClient(self.config)
            
            # Попытка подключения
            logger.info("   Попытка подключения к MQTT...")
            result = self.mqtt_client.connect()
            
            # Небольшая задержка для установления соединения
            time.sleep(2)  # Увеличено до 2 секунд для надежности
            
            if self.mqtt_client.is_connected:
                broker = self.config.mqtt_broker or "не указан"
                port = self.config.mqtt_port
                robot_id = self.config.robot_id
                
                self.log_step(
                    3, 
                    "Проверка подключения к MQTT", 
                    "ok", 
                    f"Подключено: {broker}:{port}, robot_id={robot_id}"
                )
                
                # Логируем топики
                command_topic = f"aroc/robot/{robot_id}/commands/navigateTo"
                status_topic = f"aroc/robot/{robot_id}/status/navigation"
                
                logger.info(f"   Топик команд: {command_topic}")
                logger.info(f"   Топик статусов: {status_topic}")
                
                return True
            else:
                self.log_step(
                    3, 
                    "Проверка подключения к MQTT", 
                    "error", 
                    "Не удалось подключиться к MQTT брокеру"
                )
                return False
                
        except Exception as e:
            self.log_step(
                3, 
                "Проверка подключения к MQTT", 
                "error", 
                f"Ошибка подключения: {str(e)}"
            )
            return False
    
    def test_step_4_check_nav2_status(self) -> bool:
        """Шаг 4: Проверка статуса Nav2 процессов"""
        self.log_step(4, "Проверка статуса Nav2", "ok")
        
        try:
            manager = ProcessManager()
            status = manager.get_status("nav2")
            
            status_value = status.get('status', 'unknown')
            is_running = status.get('running', False)
            pid = status.get('pid')
            
            if is_running:
                self.log_step(
                    4, 
                    "Проверка статуса Nav2", 
                    "ok", 
                    f"Nav2 запущен (PID: {pid}, статус: {status_value})"
                )
                
                # Показываем последние логи
                logs = status.get('logs', [])
                if logs:
                    logger.info(f"   Последние логи ({len(logs)} строк):")
                    for log_line in logs[-3:]:
                        logger.info(f"     {log_line}")
                
                return True
            else:
                self.log_step(
                    4, 
                    "Проверка статуса Nav2", 
                    "warning", 
                    f"Nav2 не запущен (статус: {status_value})"
                )
                logger.warning("   Nav2 должен быть запущен для полного теста")
                return False
                
        except Exception as e:
            self.log_step(
                4, 
                "Проверка статуса Nav2", 
                "error", 
                f"Ошибка проверки: {str(e)}"
            )
            return False
    
    def test_step_4b_check_navigation_node(self) -> bool:
        """Шаг 4b: Проверка наличия navigation_integrated_node"""
        self.log_step(4, "Проверка navigation_integrated_node", "ok")
        
        try:
            import subprocess
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                nodes = result.stdout
                if 'navigation_integrated_node' in nodes:
                    self.log_step(
                        4, 
                        "Проверка navigation_integrated_node", 
                        "ok", 
                        "navigation_integrated_node запущен"
                    )
                    return True
                else:
                    self.log_step(
                        4, 
                        "Проверка navigation_integrated_node", 
                        "warning", 
                        "navigation_integrated_node не запущен"
                    )
                    logger.warning("   navigation_integrated_node должен быть запущен для обработки MQTT команд")
                    logger.warning("   Запустите вручную:")
                    logger.warning("     cd /home/boris/ros2_ws")
                    logger.warning("     source /opt/ros/jazzy/setup.bash")
                    logger.warning("     source install/setup.bash")
                    logger.warning("     python3 -m aehub_navigation.navigation_integrated_node \\")
                    logger.warning("       --ros-args \\")
                    logger.warning("       -p robot_id:=robot_001 \\")
                    logger.warning("       -p config_service_url:=http://localhost:7900 \\")
                    logger.warning("       -p config_service_api_key:=tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4")
                    logger.warning("   Или через launch:")
                    logger.warning("     ros2 launch aehub_navigation aehub_navigation.launch.py \\")
                    logger.warning("       robot_id:=robot_001 \\")
                    logger.warning("       config_service_url:=http://localhost:7900 \\")
                    logger.warning("       config_service_api_key:=tR-UZ2j2KutE6OYlEGbsx0h5qe071L-gC5kd1hHKfw4")
                    return False
            else:
                self.log_step(
                    4, 
                    "Проверка navigation_integrated_node", 
                    "warning", 
                    f"Не удалось проверить узлы ROS2: {result.stderr}"
                )
                return False
                
        except Exception as e:
            self.log_step(
                4, 
                "Проверка navigation_integrated_node", 
                "warning", 
                f"Ошибка проверки: {str(e)}"
            )
            return False
    
    def test_step_5_send_mqtt_command(self, target_id: str) -> Optional[str]:
        """Шаг 5: Отправка команды навигации через MQTT"""
        self.log_step(5, "Отправка команды навигации через MQTT", "ok", f"target_id: {target_id}")
        
        try:
            if not self.mqtt_client or not self.mqtt_client.is_connected:
                self.log_step(
                    5, 
                    "Отправка команды навигации через MQTT", 
                    "error", 
                    "MQTT клиент не подключен"
                )
                return None
            
            command_id = self.mqtt_client.send_navigate_command(
                target_id=target_id,
                priority="normal"
            )
            
            if command_id:
                self.log_step(
                    5, 
                    "Отправка команды навигации через MQTT", 
                    "ok", 
                    f"Команда отправлена, command_id: {command_id}"
                )
                
                # Логируем детали команды
                command_topic = f"aroc/robot/{self.config.robot_id}/commands/navigateTo"
                logger.info(f"   Топик: {command_topic}")
                logger.info(f"   Приоритет: normal")
                logger.info(f"   Timestamp: {datetime.now().isoformat()}")
                
                return command_id
            else:
                self.log_step(
                    5, 
                    "Отправка команды навигации через MQTT", 
                    "error", 
                    "Не удалось отправить команду"
                )
                return None
                
        except Exception as e:
            self.log_step(
                5, 
                "Отправка команды навигации через MQTT", 
                "error", 
                f"Ошибка: {str(e)}"
            )
            return None
    
    def test_step_6_wait_for_status(self, command_id: str, timeout: int = 10) -> bool:
        """Шаг 6: Ожидание статуса навигации"""
        self.log_step(6, "Ожидание статуса навигации", "ok", f"timeout: {timeout}s")
        
        try:
            if not self.mqtt_client:
                self.log_step(
                    6, 
                    "Ожидание статуса навигации", 
                    "error", 
                    "MQTT клиент не инициализирован"
                )
                return False
            
            start_time = time.time()
            status_received = False
            
            logger.info(f"   Ожидание статуса для command_id: {command_id}")
            logger.info(f"   Подписка на топик: aroc/robot/{self.config.robot_id}/status/navigation")
            
            while time.time() - start_time < timeout:
                # Проверяем последний статус
                last_status = self.mqtt_client.get_last_status()
                
                if last_status and last_status.get('command_id') == command_id:
                    status = last_status.get('status', 'unknown')
                    error_code = last_status.get('error_code')
                    error_message = last_status.get('error_message')
                    
                    self.log_step(
                        6, 
                        "Ожидание статуса навигации", 
                        "ok", 
                        f"Статус получен: {status}"
                    )
                    
                    logger.info(f"   Статус: {status}")
                    if error_code:
                        logger.warning(f"   Код ошибки: {error_code}")
                    if error_message:
                        logger.warning(f"   Сообщение: {error_message}")
                    
                    status_received = True
                    break
                
                time.sleep(0.5)
            
            if not status_received:
                self.log_step(
                    6, 
                    "Ожидание статуса навигации", 
                    "warning", 
                    f"Статус не получен в течение {timeout} секунд"
                )
                logger.warning("   Это может быть нормально, если navigation node не запущен")
                return False
            
            return True
            
        except Exception as e:
            self.log_step(
                6, 
                "Ожидание статуса навигации", 
                "error", 
                f"Ошибка: {str(e)}"
            )
            return False
    
    def test_step_7_check_ros2_topics(self) -> bool:
        """Шаг 7: Проверка ROS2 топиков (если доступно)"""
        self.log_step(7, "Проверка ROS2 топиков", "ok")
        
        try:
            import requests
            
            # Проверяем через API nav2_test_server
            response = requests.get("http://localhost:8000/api/monitor/topics", timeout=2)
            
            if response.status_code == 200:
                topics = response.json()
                
                # Ищем важные топики
                important_topics = [
                    '/cmd_vel',
                    '/odom',
                    '/navigate_to_pose/_action/status',
                    '/navigate_to_pose/_action/feedback'
                ]
                
                found_topics = []
                for topic_name in important_topics:
                    if topic_name in topics:
                        found_topics.append(topic_name)
                        topic_data = topics[topic_name]
                        logger.info(f"   ✅ {topic_name}: {topic_data.get('message_count', 0)} сообщений")
                
                if found_topics:
                    self.log_step(
                        7, 
                        "Проверка ROS2 топиков", 
                        "ok", 
                        f"Найдено важных топиков: {len(found_topics)}/{len(important_topics)}"
                    )
                    return True
                else:
                    self.log_step(
                        7, 
                        "Проверка ROS2 топиков", 
                        "warning", 
                        "Важные топики не найдены (возможно, Nav2 не запущен)"
                    )
                    return False
            else:
                self.log_step(
                    7, 
                    "Проверка ROS2 топиков", 
                    "warning", 
                    f"nav2_test_server недоступен (код: {response.status_code})"
                )
                return False
                
        except Exception as e:
            self.log_step(
                7, 
                "Проверка ROS2 топиков", 
                "warning", 
                f"Не удалось проверить топики: {str(e)}"
            )
            return False
    
    def test_step_8_verify_no_motor_commands(self) -> bool:
        """Шаг 8: Проверка, что команды не отправляются на моторы"""
        self.log_step(8, "Проверка блокировки команд на моторы", "ok")
        
        try:
            import requests
            
            # Проверяем топик /cmd_vel через API
            response = requests.get("http://localhost:8000/api/monitor/topics/cmd_vel", timeout=2)
            
            if response.status_code == 200:
                topic_data = response.json()
                last_message_time = topic_data.get('last_message_time')
                
                if last_message_time:
                    self.log_step(
                        8, 
                        "Проверка блокировки команд на моторы", 
                        "warning", 
                        f"Обнаружены сообщения в /cmd_vel (последнее: {last_message_time})"
                    )
                    logger.warning("   ⚠️  ВНИМАНИЕ: Команды отправляются на моторы!")
                    logger.warning("   Убедитесь, что база заблокирована физически")
                    return False
                else:
                    self.log_step(
                        8, 
                        "Проверка блокировки команд на моторы", 
                        "ok", 
                        "Команды в /cmd_vel не обнаружены (база заблокирована)"
                    )
                    return True
            else:
                self.log_step(
                    8, 
                    "Проверка блокировки команд на моторы", 
                    "warning", 
                    "Не удалось проверить /cmd_vel (API недоступен)"
                )
                return False
                
        except Exception as e:
            self.log_step(
                8, 
                "Проверка блокировки команд на моторы", 
                "warning", 
                f"Ошибка проверки: {str(e)}"
            )
            return False
    
    def print_summary(self):
        """Вывод итогового отчета"""
        print("\n" + "=" * 70)
        print("📊 ИТОГОВЫЙ ОТЧЕТ ТЕСТИРОВАНИЯ")
        print("=" * 70)
        
        total_steps = len(self.test_results)
        ok_steps = sum(1 for r in self.test_results if r['status'] == 'ok')
        error_steps = sum(1 for r in self.test_results if r['status'] == 'error')
        warning_steps = sum(1 for r in self.test_results if r['status'] == 'warning')
        
        print(f"\nВсего шагов: {total_steps}")
        print(f"✅ Успешно: {ok_steps}")
        print(f"⚠️  Предупреждения: {warning_steps}")
        print(f"❌ Ошибки: {error_steps}")
        
        print("\nДетали шагов:")
        for result in self.test_results:
            status_icon = {
                "ok": "✅",
                "error": "❌",
                "warning": "⚠️",
                "skip": "⏭️"
            }.get(result['status'], "❓")
            
            print(f"  {status_icon} Шаг {result['step']}: {result['name']}")
            if result['details']:
                print(f"     {result['details']}")
        
        print("\n" + "=" * 70)
        
        if error_steps == 0:
            print("✅ Все критические шаги пройдены успешно!")
            if warning_steps > 0:
                print("⚠️  Есть предупреждения, но они не критичны")
        else:
            print("❌ Обнаружены ошибки. Проверьте детали выше.")
        
        print("=" * 70 + "\n")
    
    async def cleanup(self):
        """Очистка ресурсов"""
        if self.position_client:
            await self.position_client.close()
        if self.mqtt_client:
            self.mqtt_client.disconnect()


async def main():
    """Главная функция"""
    parser = argparse.ArgumentParser(description='Тест потока навигации (без движения робота)')
    parser.add_argument(
        '--target',
        type=str,
        default='position_B',
        help='ID целевой позиции (по умолчанию: position_B)'
    )
    parser.add_argument(
        '--robot-service-url',
        type=str,
        default='http://localhost:8110',
        help='URL robot_service API (по умолчанию: http://localhost:8110)'
    )
    parser.add_argument(
        '--timeout',
        type=int,
        default=10,
        help='Таймаут ожидания статуса в секундах (по умолчанию: 10)'
    )
    parser.add_argument(
        '--skip-mqtt',
        action='store_true',
        help='Пропустить тесты MQTT (если MQTT не настроен)'
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("🧪 ТЕСТИРОВАНИЕ ПОТОКА НАВИГАЦИИ (БЕЗ ДВИЖЕНИЯ РОБОТА)")
    print("=" * 70)
    print(f"Целевая позиция: {args.target}")
    print(f"Robot Service URL: {args.robot_service_url}")
    print(f"Таймаут ожидания статуса: {args.timeout}s")
    print("=" * 70 + "\n")
    
    tester = NavigationFlowTester(robot_service_url=args.robot_service_url)
    
    try:
        # Шаг 1: Проверка robot_service
        if not await tester.test_step_1_check_robot_service():
            logger.error("Критическая ошибка: robot_service недоступен")
            tester.print_summary()
            return 1
        
        # Шаг 2: Получение координат целевой позиции
        target_position = await tester.test_step_2_get_target_position(args.target)
        if not target_position:
            logger.error(f"Критическая ошибка: позиция '{args.target}' не найдена")
            tester.print_summary()
            return 1
        
        # Шаг 3: Проверка MQTT подключения
        mqtt_connected = tester.test_step_3_check_mqtt_connection()
        if not mqtt_connected:
            if args.skip_mqtt:
                logger.warning("MQTT не подключен, но пропуск MQTT тестов разрешен (--skip-mqtt)")
            else:
                logger.error("Критическая ошибка: не удалось подключиться к MQTT")
                logger.error("   Используйте --skip-mqtt для пропуска MQTT тестов")
                logger.error("   Или настройте MQTT в .env файле:")
                logger.error("     MQTT_BROKER=...")
                logger.error("     MQTT_PORT=...")
                logger.error("     MQTT_USERNAME=...")
                logger.error("     MQTT_PASSWORD=...")
                logger.error("     Или используйте CONFIG_SERVICE_URL и CONFIG_SERVICE_API_KEY")
                tester.print_summary()
                return 1
        
        # Шаг 4: Проверка статуса Nav2
        tester.test_step_4_check_nav2_status()
        
        # Шаг 4b: Проверка navigation_integrated_node
        tester.test_step_4b_check_navigation_node()
        
        # Шаг 5: Отправка команды навигации (только если MQTT подключен)
        if mqtt_connected:
            command_id = tester.test_step_5_send_mqtt_command(args.target)
            if not command_id:
                logger.error("Критическая ошибка: не удалось отправить команду")
                tester.print_summary()
                return 1
            
            # Шаг 6: Ожидание статуса
            tester.test_step_6_wait_for_status(command_id, timeout=args.timeout)
        else:
            tester.log_step(5, "Отправка команды навигации через MQTT", "skip", "MQTT не подключен")
            tester.log_step(6, "Ожидание статуса навигации", "skip", "MQTT не подключен")
        
        # Шаг 7: Проверка ROS2 топиков
        tester.test_step_7_check_ros2_topics()
        
        # Шаг 8: Проверка блокировки команд на моторы
        tester.test_step_8_verify_no_motor_commands()
        
        # Итоговый отчет
        tester.print_summary()
        
        return 0
        
    except KeyboardInterrupt:
        logger.info("\n\nТестирование прервано пользователем")
        return 1
    except Exception as e:
        logger.error(f"\n\nКритическая ошибка: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        await tester.cleanup()


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)

