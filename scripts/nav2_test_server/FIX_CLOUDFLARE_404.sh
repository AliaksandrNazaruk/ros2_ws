#!/bin/bash
# Скрипт для исправления конфигурации Cloudflare Tunnel
# Перемещает правило ros2.techvisioncloud.pl на первое место

CONFIG_FILE="/etc/cloudflared/config.yml"
BACKUP_FILE="/etc/cloudflared/config.yml.backup.$(date +%Y%m%d_%H%M%S)"

echo "=== Cloudflare Tunnel Configuration Fix ==="
echo ""

# Проверка прав
if [ "$EUID" -ne 0 ]; then 
    echo "❌ This script must be run as root (use sudo)"
    exit 1
fi

# Создание резервной копии
echo "📋 Creating backup: $BACKUP_FILE"
cp "$CONFIG_FILE" "$BACKUP_FILE" || {
    echo "❌ Failed to create backup"
    exit 1
}

# Использование Python для обработки YAML
python3 << PYTHON_SCRIPT
import yaml
import sys
import os
from datetime import datetime

config_file = "/etc/cloudflared/config.yml"
backup_file = f"{config_file}.backup.{datetime.now().strftime('%Y%m%d_%H%M%S')}"

try:
    # Чтение конфигурации
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    ingress = config.get('ingress', [])
    
    # Поиск правила ros2
    ros2_rule = None
    other_rules = []
    catch_all_rule = None
    
    for rule in ingress:
        hostname = rule.get('hostname', '')
        service = rule.get('service', '')
        
        if hostname == 'ros2.techvisioncloud.pl':
            ros2_rule = rule
        elif not hostname and service == 'http_status:404':
            catch_all_rule = rule
        else:
            other_rules.append(rule)
    
    if not ros2_rule:
        print("❌ ros2.techvisioncloud.pl rule not found in configuration")
        sys.exit(1)
    
    # Пересборка списка ingress: ros2 первым, затем остальные, catch-all последним
    new_ingress = [ros2_rule] + other_rules
    if catch_all_rule:
        new_ingress.append(catch_all_rule)
    
    config['ingress'] = new_ingress
    
    # Запись обновленной конфигурации
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print("✅ Configuration updated successfully")
    print(f"   - ros2.techvisioncloud.pl moved to first position")
    print(f"   - {len(other_rules)} other rules preserved")
    if catch_all_rule:
        print(f"   - Catch-all rule remains last")
    
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)

PYTHON_SCRIPT

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Configuration fixed!"
    echo ""
    echo "📋 Next steps:"
    echo "  1. Validate configuration:"
    echo "     sudo cloudflared tunnel --config /etc/cloudflared/config.yml ingress validate"
    echo ""
    echo "  2. Restart cloudflared:"
    echo "     sudo systemctl restart cloudflared"
    echo ""
    echo "  3. Check status:"
    echo "     sudo systemctl status cloudflared"
    echo ""
    echo "  4. Test:"
    echo "     curl -I https://ros2.techvisioncloud.pl/dashboard"
    echo ""
    echo "📦 Backup saved to: $BACKUP_FILE"
else
    echo ""
    echo "❌ Failed to update configuration"
    echo "   Restore from backup: sudo cp $BACKUP_FILE $CONFIG_FILE"
    exit 1
fi

