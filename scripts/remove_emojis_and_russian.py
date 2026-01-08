#!/usr/bin/env python3
"""
Script to remove emojis and replace Russian text with English in code files.
"""

import re
import os
from pathlib import Path

# Emoji patterns to remove
EMOJI_PATTERN = re.compile(r'[]')

# Russian to English translations for common phrases
RUSSIAN_TO_ENGLISH = {
    # Common phrases
    'Error': 'Error',
    'error': 'error',
    'ERROR': 'ERROR',
    'Success': 'Success',
    'successfully': 'successfully',
    'SUCCESS': 'SUCCESS',
    'Warning': 'Warning',
    'warning': 'warning',
    'WARNING': 'WARNING',
    'Info': 'Info',
    'info': 'info',
    'INFO': 'INFO',
    'Debug': 'Debug',
    'debug': 'debug',
    'DEBUG': 'DEBUG',
    
    # Navigation specific
    'Command': 'Command',
    'command': 'command',
    'COMMAND': 'COMMAND',
    'Status': 'Status',
    'status': 'status',
    'STATUS': 'STATUS',
    'Goal': 'Goal',
    'goal': 'goal',
    'GOAL': 'GOAL',
    'Navigation': 'Navigation',
    'navigation': 'navigation',
    'NAVIGATION': 'NAVIGATION',
    
    # Actions
    'Sending': 'Sending',
    'sending': 'sending',
    'SENDING': 'SENDING',
    'Receiving': 'Receiving',
    'receiving': 'receiving',
    'RECEIVING': 'RECEIVING',
    'Waiting': 'Waiting',
    'waiting': 'waiting',
    'WAITING': 'WAITING',
    
    # Common words
    'not': 'not',
    'Not': 'Not',
    'NOT': 'NOT',
    'yes': 'yes',
    'Yes': 'Yes',
    'YES': 'YES',
    'or': 'or',
    'Or': 'Or',
    'OR': 'OR',
    'and': 'and',
    'AND': 'And',
    'AND': 'AND',
}

def remove_emojis(text: str) -> str:
    """Remove emojis from text"""
    return EMOJI_PATTERN.sub('', text)

def replace_russian_text(text: str) -> str:
    """Replace Russian text with English"""
    result = text
    for russian, english in RUSSIAN_TO_ENGLISH.items():
        result = result.replace(russian, english)
    return result

def process_file(file_path: Path) -> bool:
    """Process a single file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        original_content = content
        
        # Remove emojis
        content = remove_emojis(content)
        
        # Replace Russian text (only in strings/comments, not in code)
        # This is a simple approach - for more complex cases, use AST parsing
        content = replace_russian_text(content)
        
        if content != original_content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            return True
        return False
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
        return False

def main():
    """Main function"""
    workspace = Path('/home/boris/ros2_ws')
    
    # Files to process
    files_to_process = []
    
    # Source files
    src_dir = workspace / 'src' / 'aehub_navigation' / 'src' / 'aehub_navigation'
    if src_dir.exists():
        files_to_process.extend(src_dir.glob('*.py'))
    
    # Scripts
    scripts_dir = workspace / 'scripts'
    if scripts_dir.exists():
        files_to_process.extend(scripts_dir.glob('*.py'))
    
    print(f"Processing {len(files_to_process)} files...")
    
    modified_count = 0
    for file_path in files_to_process:
        if process_file(file_path):
            print(f"Modified: {file_path}")
            modified_count += 1
    
    print(f"\nDone! Modified {modified_count} files.")

if __name__ == '__main__':
    main()

