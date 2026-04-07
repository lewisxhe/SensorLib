#!/usr/bin/env python3
import os
import json
import configparser
from pathlib import Path

def get_project_root():
    script_dir = Path(__file__).parent.resolve()
    return script_dir.parent

def find_examples(root_dir):
    examples_dir = root_dir / "examples"
    if not examples_dir.exists():
        return []
    ino_files = list(examples_dir.rglob("*.ino"))
    return [p.relative_to(root_dir) for p in ino_files]

def load_config(script_dir):
    config = {}
    json_path = script_dir / "config.json"
    ini_path = script_dir / "config.ini"
    if json_path.exists():
        with open(json_path, "r", encoding="utf-8") as f:
            config = json.load(f)
    elif ini_path.exists():
        parser = configparser.ConfigParser()
        parser.read(ini_path, encoding="utf-8")
        if parser.has_section("DEFAULT"):
            config = dict(parser["DEFAULT"])
        elif parser.has_section("arduino"):
            config = dict(parser["arduino"])
        else:
            sections = parser.sections()
            if sections:
                config = dict(parser[sections[0]])
    else:
        print("WARNING: NO tools/config.json OR tools/config.ini FOUND, USING DEFAULT VALUES")
    defaults = {
        "fqbn": "esp32:esp32:esp32p4:DebugLevel=debug",
        "port": "COM3",
        "build_root": ".pio/build",
        "libraries": ".",
        "arduino_cli": "arduino-cli.exe"
    }
    for key, value in defaults.items():
        if key not in config:
            config[key] = value
    return config

def generate_tasks(example_paths, config, project_root):
    tasks = []
    workspace_var = "${workspaceFolder}"
    
    for ino_rel in example_paths:
        example_name = ino_rel.stem
        build_dir = f"{workspace_var}/{config['build_root']}/{example_name}"
        
        compile_label = f"Compile: {example_name}"
        compile_cmd = (
            f"{config['arduino_cli']} compile --fqbn {config['fqbn']} "
            f"--verbose -j 0 "
            f"--build-path \"{build_dir}\" "
            f"--library \"{workspace_var}/{config['library']}\" "
            f"\"{workspace_var}/{ino_rel.as_posix()}\""
        )
        tasks.append({
            "label": compile_label,
            "type": "shell",
            "command": compile_cmd,
            "group": "build",
            "problemMatcher": []
        })
        
        upload_label = f"Compile & Upload: {example_name}"
        upload_cmd = (
            f"{config['arduino_cli']} compile --fqbn {config['fqbn']} "
            f"--verbose -j 0 "
            f"--build-path \"{build_dir}\" "
            f"--library \"{workspace_var}/{config['library']}\" "
            f"--port {config['port']} --upload "
            f"\"{workspace_var}/{ino_rel.as_posix()}\""
        )
        tasks.append({
            "label": upload_label,
            "type": "shell",
            "command": upload_cmd,
            "group": {
                "kind": "build",
                "isDefault": False
            },
            "problemMatcher": []
        })
    
    return {"version": "2.0.0", "tasks": tasks}

def main():
    project_root = get_project_root()
    script_dir = Path(__file__).parent.resolve()
    print(f"PROJECT ROOT : {project_root}")
    print(f"SCRIPT DIR : {script_dir}")

    examples = find_examples(project_root)
    if not examples:
        print("No examples found")
        return

    print(f"FOUND {len(examples)} EXAMPLES:")
    for ex in examples:
        print(f"  - {ex}")
    
    config = load_config(script_dir)
    print(f"USING CONFIG : fqbn={config['fqbn']}, port={config['port']}")

    tasks_json = generate_tasks(examples, config, project_root)
    
    vscode_dir = project_root / ".vscode"
    vscode_dir.mkdir(exist_ok=True)
    tasks_path = vscode_dir / "tasks.json"
    with open(tasks_path, "w", encoding="utf-8") as f:
        json.dump(tasks_json, f, indent=2, ensure_ascii=False)

    print(f"GENERATED {tasks_path}")
    print("NOW YOU CAN PRESS Ctrl+Shift+P IN VS CODE AND SELECT 'Tasks: Run Task' TO EXECUTE THE BUILD OR UPLOAD.")

if __name__ == "__main__":
    main()
    