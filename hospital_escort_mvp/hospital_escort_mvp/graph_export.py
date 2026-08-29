"""把医院拓扑导出为 JSON、Mermaid、DOT 和独立 SVG。"""

import argparse
import json
from pathlib import Path
from typing import Dict, Tuple
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory

from hospital_escort_mvp.hospital_graph import HospitalGraph


def _floor_key(floor: str) -> int:
    if floor.startswith('B'):
        return -int(floor[1:])
    if floor.startswith('F'):
        return int(floor[1:])
    return 0


def export_graph(graph: HospitalGraph, output_dir: str) -> Dict[str, str]:
    target = Path(output_dir)
    target.mkdir(parents=True, exist_ok=True)
    result = {}

    json_path = target / 'hospital_graph.json'
    json_path.write_text(
        json.dumps(graph.as_dict(), ensure_ascii=False, indent=2), encoding='utf-8'
    )
    result['json'] = str(json_path)

    mermaid = ['graph BT']
    for node in graph.nodes.values():
        mermaid.append(
            f'  {node.node_id}["{node.floor} · {node.name}"]'
        )
    for edge in graph.edges:
        connector = '<-->' if edge.bidirectional else '-->'
        label = f'{edge.edge_type} · {edge.travel_time_sec:g}s'
        mermaid.append(
            f'  {edge.from_node} {connector}|"{label}"| {edge.to_node}'
        )
    mermaid_path = target / 'hospital_graph.mmd'
    mermaid_path.write_text('\n'.join(mermaid) + '\n', encoding='utf-8')
    result['mermaid'] = str(mermaid_path)

    dot = ['graph hospital {', '  rankdir=BT;', '  node [shape=box];']
    for node in graph.nodes.values():
        label = f'{node.floor} · {node.name}\\n{node.map_name}'
        dot.append(f'  "{node.node_id}" [label="{label}"];')
    for edge in graph.edges:
        label = f'{edge.edge_type} {edge.travel_time_sec:g}s'
        dot.append(
            f'  "{edge.from_node}" -- "{edge.to_node}" [label="{label}"];'
        )
    dot.append('}')
    dot_path = target / 'hospital_graph.dot'
    dot_path.write_text('\n'.join(dot) + '\n', encoding='utf-8')
    result['dot'] = str(dot_path)

    floors = sorted(
        {node.floor for node in graph.nodes.values()}, key=_floor_key, reverse=True
    )
    positions: Dict[str, Tuple[float, float]] = {}
    width, row_height, top = 1320, 190, 175
    for floor_index, floor in enumerate(floors):
        center_y = top + floor_index * row_height + 80
        for node in graph.nodes.values():
            if node.floor != floor:
                continue
            positions[node.node_id] = (
                210 + (node.x + 1.8) / 3.6 * 820,
                center_y - node.y * 58,
            )

    height = top + len(floors) * row_height + 55
    itinerary = [
        ('cardiology_3f', '起点 3F心内科'),
        ('blood_draw_1f', '1 抽血'),
        ('ecg_2f', '2 心电图'),
        ('echo_waiting_2f', '等待30分钟'),
        ('cardiac_ultrasound_2f', '3 心脏彩超'),
        ('cardiology_3f', '返回 3F心内科'),
    ]
    itinerary = [item for item in itinerary if item[0] in graph.nodes]
    itinerary_nodes = {item[0] for item in itinerary}
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7f8fb"/>',
        '<style>text{font-family:Arial,"Noto Sans SC",sans-serif}'
        '.floor{fill:#fff;stroke:#e1e5ec;stroke-width:1}'
        '.walk{stroke:#aab3c2;stroke-width:2;fill:none}'
        '.lift{stroke:#d97706;stroke-width:4;fill:none}'
        '.poi{fill:#eef3f8;stroke:#8290a3}'
        '.route{fill:#e8f5ee;stroke:#18864b}'
        '.elevator{fill:#fff3df;stroke:#d97706}'
        '.corridor{fill:#f4f5f7;stroke:#aab3c2}'
        '.node{stroke-width:1.7}.label{font-size:13px;fill:#172033}'
        '.muted{font-size:11px;fill:#667085}'
        '.route-line{stroke:#18864b;stroke-width:3;fill:none}'
        '.route-dot{fill:#18864b}.route-text{font-size:12px;fill:#17633c}'
        '</style>',
        '<text x="55" y="38" font-size="23" font-weight="bold" '
        'fill="#172033">门诊楼多楼层导航网络</text>',
        '<text x="55" y="62" class="muted">完整医院节点作为底图，绿色标出本轮心内科检查闭环</text>',
    ]
    route_left, route_right, route_y = 85, width - 85, 112
    route_gap = (route_right - route_left) / max(1, len(itinerary) - 1)
    svg.append(
        f'<line class="route-line" x1="{route_left}" y1="{route_y}" '
        f'x2="{route_right}" y2="{route_y}"/>'
    )
    for index, (_, label) in enumerate(itinerary):
        x = route_left + index * route_gap
        svg.append(f'<circle class="route-dot" cx="{x}" cy="{route_y}" r="6"/>')
        svg.append(
            f'<text class="route-text" x="{x}" y="{route_y + 25}" '
            f'text-anchor="middle">{escape(label)}</text>'
        )
    for floor_index, floor in enumerate(floors):
        y = top + floor_index * row_height
        svg.append(
            f'<rect class="floor" x="55" y="{y}" width="{width - 110}" '
            'height="165" rx="10"/>'
        )
        map_name = next(
            node.map_name for node in graph.nodes.values() if node.floor == floor
        )
        svg.append(
            f'<text x="75" y="{y + 29}" font-size="18" font-weight="bold" '
            f'fill="#172033">{escape(floor)}</text>'
        )
        svg.append(
            f'<text class="muted" x="75" y="{y + 49}">{escape(map_name)}</text>'
        )
    for edge in graph.edges:
        x1, y1 = positions[edge.from_node]
        x2, y2 = positions[edge.to_node]
        if edge.edge_type == 'walk':
            svg.append(
                f'<line class="walk" x1="{x1}" y1="{y1}" '
                f'x2="{x2}" y2="{y2}"><title>'
                f'{escape(edge.edge_id)} · {edge.travel_time_sec:g}秒</title></line>'
            )
    elevator_positions = sorted(
        (
            positions[node.node_id]
            for node in graph.nodes.values()
            if node.node_type == 'elevator'
        ),
        key=lambda point: point[1],
    )
    if elevator_positions:
        shaft_x = elevator_positions[0][0]
        svg.append(
            f'<line class="lift" x1="{shaft_x}" '
            f'y1="{elevator_positions[0][1]}" x2="{shaft_x}" '
            f'y2="{elevator_positions[-1][1]}"/>'
        )
        svg.append(
            f'<text class="muted" x="{shaft_x + 75}" '
            f'y="{top + 16}">A电梯：跨层边按时间计权，切换 map_name</text>'
        )
    for node in graph.nodes.values():
        x, y = positions[node.node_id]
        if node.node_type == 'elevator':
            css_class = 'elevator'
        elif node.node_type == 'corridor':
            css_class = 'corridor'
        elif node.node_id in itinerary_nodes:
            css_class = 'route'
        else:
            css_class = 'poi'
        svg.append(
            f'<rect class="node {css_class}" x="{x - 61}" y="{y - 19}" '
            'width="122" height="38" rx="7"/>'
        )
        svg.append(
            f'<text class="label" x="{x}" y="{y + 4}" text-anchor="middle">'
            f'{escape(node.name)}</text>'
        )
    svg.append(
        f'<text class="muted" x="70" y="{height - 22}">绿色：本轮检查节点　'
        '灰色：医院扩展节点与同层 GoalNav　橙色：电梯换层</text>'
    )
    svg.append('</svg>')
    svg_path = target / 'hospital_graph.svg'
    svg_path.write_text('\n'.join(svg) + '\n', encoding='utf-8')
    result['svg'] = str(svg_path)
    return result


def main(args=None) -> None:
    default_graph = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config'
        / 'hospital_graph.yaml'
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph', default=default_graph)
    parser.add_argument('--output-dir', default='/vbot_ws/reports/hospital_graph')
    options = parser.parse_args(args)
    graph = HospitalGraph.from_yaml(options.graph)
    paths = export_graph(graph, options.output_dir)
    print(json.dumps(paths, ensure_ascii=False, indent=2))
