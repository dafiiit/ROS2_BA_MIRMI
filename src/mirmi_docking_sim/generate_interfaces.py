import os
import re
from pathlib import Path

# Konfiguration
OUTPUT_FILE = "INTERFACE_DOCUMENTATION_FULL.md"
ROOT_DIR = os.getcwd()

# Erweiterte Regex Patterns
PATTERNS = {
    # Python Nodes
    'node_class': re.compile(r'class\s+(\w+)\(Node\):'),
    'publisher': re.compile(r'create_publisher\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'subscription': re.compile(r'create_subscription\s*\(\s*(\w+),\s*[\'"]([^\'"]+)[\'"]'),
    'parameter': re.compile(r'declare_parameter\s*\(\s*[\'"]([^\'"]+)[\'"]\s*,\s*(.+)'),
    
    # Launch Files
    'launch_arg': re.compile(r'DeclareLaunchArgument\s*\(\s*[\'"]([^\'"]+)[\'"]'),
    # Sucht nach Node(package='...', executable='...') Blöcken (vereinfacht)
    'node_call_start': re.compile(r'Node\s*\('),
    
    # SDF (Gazebo)
    'sdf_sensor': re.compile(r'<sensor\s+name=[\'"]([^\'"]+)[\'"]\s+type=[\'"]([^\'"]+)[\'"]>'),
    'sdf_topic': re.compile(r'<topic>(.*?)</topic>'),
    'sdf_plugin': re.compile(r'<plugin\s+filename=[\'"]([^\'"]+)[\'"]\s+name=[\'"]([^\'"]+)[\'"]>'),
    
    # Bridge
    'bridge_topic': re.compile(r'ros_topic_name:\s*["\']?([^"\']+)["\']?'),
    'bridge_direction': re.compile(r'direction:\s*(\w+)')
}

def scan_files(root_path):
    data = {
        'py_nodes': [],
        'launch_nodes': [],
        'launch_args': [],
        'sdf_models': [],
        'bridge_config': {'topics': []},
        'all_topics': set()
    }

    for path in Path(root_path).rglob('*'):
        # Ignoriere build/install/git/cache
        if any(x in path.parts for x in ['build', 'install', '.git', '__pycache__', 'log']):
            continue
        
        if path.is_dir(): continue

        try:
            with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
        except:
            continue

        # 1. Python Source Code
        if path.suffix == '.py' and 'launch' not in path.parts:
            analyze_python_node(path, content, data)
        
        # 2. Launch Files
        if path.suffix == '.py' and 'launch' in path.parts:
            analyze_launch_file(path, content, data)

        # 3. SDF Models (Simulation Hardware)
        if path.suffix == '.sdf':
            analyze_sdf_file(path, content, data)

        # 4. Bridge Config
        if path.name == 'bridge.yaml':
            analyze_bridge_config(path, content, data)

    return data

def analyze_python_node(path, content, data):
    """Scannt eigene Python Nodes."""
    class_match = PATTERNS['node_class'].search(content)
    if not class_match: return

    node_info = {
        'file': path.name,
        'class': class_match.group(1),
        'pubs': [], 'subs': [], 'params': []
    }

    for match in PATTERNS['publisher'].findall(content):
        node_info['pubs'].append({'topic': match[1], 'type': match[0]})
        data['all_topics'].add(match[1])

    for match in PATTERNS['subscription'].findall(content):
        node_info['subs'].append({'topic': match[1], 'type': match[0]})
        data['all_topics'].add(match[1])

    for match in PATTERNS['parameter'].findall(content):
        val = match[1].split(')')[0].strip()
        node_info['params'].append({'name': match[0], 'default': val})

    data['py_nodes'].append(node_info)

def analyze_launch_file(path, content, data):
    """Scannt Launch Files nach Argumenten UND gestarteten Nodes (z.B. apriltag)."""
    # 1. Argumente
    args = PATTERNS['launch_arg'].findall(content)
    if args:
        data['launch_args'].append({'file': path.name, 'args': args})

    # 2. Nodes (komplexer, wir parsen "Node(...)" Blöcke manuell)
    # Wir suchen nach dem Start von Node( und lesen dann Zeilen bis zur schließenden Klammer
    # Dies ist ein heuristischer Parser.
    
    lines = content.split('\n')
    current_node = None
    brace_count = 0
    
    for line in lines:
        stripped = line.strip()
        
        # Start eines Node-Aufrufs
        if 'Node(' in stripped and 'import' not in stripped:
            current_node = {
                'package': '?', 'executable': '?', 'name': '?', 
                'remappings': [], 'params': [], 'file': path.name
            }
            brace_count = stripped.count('(') - stripped.count(')')
        
        elif current_node:
            brace_count += stripped.count('(') - stripped.count(')')
            
            # Infos extrahieren
            if "package=" in stripped:
                m = re.search(r"package=['\"]([^'\"]+)['\"]", stripped)
                if m: current_node['package'] = m.group(1)
            
            if "executable=" in stripped:
                m = re.search(r"executable=['\"]([^'\"]+)['\"]", stripped)
                if m: current_node['executable'] = m.group(1)
                
            if "name=" in stripped:
                m = re.search(r"name=['\"]([^'\"]+)['\"]", stripped)
                if m: current_node['name'] = m.group(1)

            # Remappings finden: ('old', 'new')
            if "remappings" in stripped or (current_node and len(current_node['remappings']) > 0 and "]" not in stripped):
                # Suche nach Tuples ('x', 'y')
                remaps = re.findall(r"\(\s*['\"]([^'\"]+)['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", stripped)
                for r in remaps:
                    current_node['remappings'].append(f"{r[0]} -> {r[1]}")
                    data['all_topics'].add(r[1])

            # Parameter (einfach: family, size etc.)
            if ":" in stripped and "{" not in stripped and "}" not in stripped:
                 # Versuche key: value zu finden (sehr grob)
                 parts = stripped.split(':')
                 if len(parts) == 2:
                     key = parts[0].strip("'\" ")
                     val = parts[1].strip(",'\" ")
                     # Filter python keywords
                     if key not in ['package', 'executable', 'name', 'output', 'emulate_tty']:
                        current_node['params'].append(f"{key}: {val}")

            if brace_count <= 0:
                # Ende des Node Blocks
                if current_node['package'] != '?':
                    data['launch_nodes'].append(current_node)
                current_node = None

def analyze_sdf_file(path, content, data):
    """Scannt SDF für Sensoren und Plugins."""
    # Sensoren finden
    sensors = []
    # Wir splitten den Content in Sensor-Blöcke (grob)
    sensor_blocks = content.split('<sensor')
    
    for block in sensor_blocks[1:]:
        # Name und Type aus dem Header holen (wurde beim split entfernt, müssen wir rekonstruieren oder regexen)
        # Wir nehmen hier eine einfachere Logik: Regex auf den rohen Content ist sicherer
        pass

    # Bessere Methode: Iteriere über Matches
    for match in PATTERNS['sdf_sensor'].finditer(content):
        name = match.group(1)
        type_ = match.group(2)
        
        # Suche Topic im Kontext nach dem Match
        start_pos = match.end()
        end_pos = content.find('</sensor>', start_pos)
        block_content = content[start_pos:end_pos]
        
        topic_match = PATTERNS['sdf_topic'].search(block_content)
        topic = topic_match.group(1) if topic_match else "default"
        
        sensors.append({'name': name, 'type': type_, 'topic': topic})
        if topic != "default":
            data['all_topics'].add(topic)

    if sensors:
        data['sdf_models'].append({'file': path.name, 'sensors': sensors})

def analyze_bridge_config(path, content, data):
    entries = content.split('- ros_topic_name:')
    for entry in entries[1:]:
        topic_match = re.search(r'^\s*["\']?([^"\']+)["\']?', entry)
        gz_topic_match = re.search(r'gz_topic_name:\s*["\']?([^"\']+)["\']?', entry)
        dir_match = re.search(r'direction:\s*(\w+)', entry)
        
        if topic_match:
            data['bridge_config']['topics'].append({
                'ros': topic_match.group(1).strip(),
                'gz': gz_topic_match.group(1).strip() if gz_topic_match else "?",
                'dir': dir_match.group(1).strip() if dir_match else "BIDIRECTIONAL"
            })

def generate_markdown(data):
    lines = []
    lines.append("# Interface Dokumentation (Full)")
    lines.append("> Beinhaltet Nodes, Launch-Konfigurationen, Hardware (SDF) und Bridge.\n")

    # 1. Externe / Standard Nodes (Launch)
    if data['launch_nodes']:
        lines.append("## 1. Gestartete Nodes (Launch)")
        lines.append("Hier sind Nodes definiert, die in Launchfiles gestartet werden (oft externe Pakete wie `apriltag_ros`).")
        lines.append("Besonders wichtig sind die **Remappings**, die zeigen, welche Topics tatsächlich verbunden sind.\n")
        
        for node in data['launch_nodes']:
            lines.append(f"### `{node['name']}` (Pkg: `{node['package']}`)")
            lines.append(f"- **Executable:** `{node['executable']}`")
            lines.append(f"- **Definiert in:** `{node['file']}`")
            
            if node['remappings']:
                lines.append("- **Remappings (Internal -> Global):**")
                for r in node['remappings']:
                    lines.append(f"  - `{r}`")
            
            if node['params']:
                lines.append("- **Parameter:**")
                for p in node['params']:
                    lines.append(f"  - `{p}`")
            lines.append("")
        lines.append("---\n")

    # 2. Simulation Hardware (SDF)
    if data['sdf_models']:
        lines.append("## 2. Simulation Interfaces (SDF/Hardware)")
        lines.append("Definiert die Sensoren und Topics, die Gazebo **intern** bereitstellt (Quelle der Daten).\n")
        
        for model in data['sdf_models']:
            lines.append(f"### Model: `{model['file']}`")
            lines.append("| Sensor Name | Typ | Gazebo Topic (Raw) |")
            lines.append("|---|---|---|")
            for s in model['sensors']:
                lines.append(f"| `{s['name']}` | `{s['type']}` | `{s['topic']}` |")
            lines.append("")
        lines.append("---\n")

    # 3. Python Nodes
    lines.append("## 3. Eigene Python Nodes")
    for node in data['py_nodes']:
        lines.append(f"### Node: `{node['class']}`")
        if node['subs']:
            lines.append("**Subs:** " + ", ".join([f"`{s['topic']}`" for s in node['subs']]))
        if node['pubs']:
            lines.append("\n**Pubs:** " + ", ".join([f"`{p['topic']}`" for p in node['pubs']]))
        lines.append("\n")
    lines.append("---\n")

    # 4. Bridge
    lines.append("## 4. Gazebo Bridge Mapping")
    lines.append("| ROS Topic | Gazebo Topic | Richtung |")
    lines.append("|---|---|---|")
    for b in data['bridge_config']['topics']:
        d = "->" if "GZ_TO" in b['dir'] else "<-" if "ROS_TO" in b['dir'] else "<->"
        lines.append(f"| `{b['ros']}` | `{b['gz']}` | {d} |")

    return "\n".join(lines)

def main():
    print(f"Scanne Workspace: {ROOT_DIR} ...")
    data = scan_files(ROOT_DIR)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(generate_markdown(data))
    print(f"Fertig: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
