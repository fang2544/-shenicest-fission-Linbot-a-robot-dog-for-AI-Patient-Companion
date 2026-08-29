"""按颜色找盒子，再映射到物品名称。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

ITEMS_PATH = Path(__file__).resolve().parent / "items.json"

COLOR_ALIAS = {
    "红": "红",
    "红色": "红",
    "red": "红",
    "橙": "橙",
    "橙色": "橙",
    "orange": "橙",
    "黄": "黄",
    "黄色": "黄",
    "yellow": "黄",
    "绿": "绿",
    "绿色": "绿",
    "green": "绿",
    "青": "蓝",
    "青色": "蓝",
    "cyan": "蓝",
    "蓝": "蓝",
    "蓝色": "蓝",
    "blue": "蓝",
    "紫": "紫",
    "紫色": "紫",
    "purple": "紫",
    "粉": "粉",
    "粉色": "粉",
    "粉红": "粉",
    "pink": "粉",
    "棕": "棕",
    "棕色": "棕",
    "褐色": "棕",
    "brown": "棕",
    "白": "白",
    "白色": "白",
    "银白": "白",
    "银色": "白",
    "银": "白",
    "white": "白",
    "silver": "白",
    "黑": "黑",
    "黑色": "黑",
    "black": "黑",
    "灰": "灰",
    "灰色": "灰",
    "gray": "灰",
    "grey": "灰",
}

# OpenCV HSV：H 0–180，S/V 0–255
HSV_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "红": [((0, 110, 70), (7, 255, 255)), ((170, 110, 70), (180, 255, 255))],
    "橙": [((9, 90, 70), (20, 255, 255))],
    "黄": [((21, 80, 80), (34, 255, 255))],
    "绿": [((45, 80, 60), (75, 255, 255))],
    "青": [((86, 70, 60), (99, 255, 255))],
    "蓝": [((76, 60, 60), (128, 255, 255))],
    "紫": [((129, 50, 60), (155, 255, 255))],
    "粉": [((148, 50, 90), (168, 220, 255))],
    "棕": [((8, 80, 40), (20, 255, 140))],
}

DRAW_BGR = {
    "红": (40, 40, 230),
    "橙": (0, 140, 255),
    "黄": (0, 220, 255),
    "绿": (40, 200, 80),
    "青": (200, 200, 40),
    "蓝": (230, 140, 40),
    "紫": (200, 80, 180),
    "粉": (180, 80, 255),
    "棕": (40, 80, 140),
    "白": (230, 230, 230),
    "黑": (40, 40, 40),
    "灰": (140, 140, 140),
}


@dataclass
class ColorBox:
    color: str
    name: str
    x1: int
    y1: int
    x2: int
    y2: int
    area: int

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2



def normalize_color(raw: str) -> str:
    key = raw.strip().lower()
    if key in COLOR_ALIAS:
        return COLOR_ALIAS[key]
    key = raw.strip()
    return COLOR_ALIAS.get(key, key)


def load_items(path: Path = ITEMS_PATH) -> tuple[dict[str, str], bool, set[str]]:
    if not path.exists():
        return {}, True, set()
    data = json.loads(path.read_text(encoding="utf-8"))
    mapping: dict[str, str] = {}
    confirm: set[str] = set()
    for row in data.get("items", []):
        color = normalize_color(str(row.get("color", "")))
        name = str(row.get("name", "")).strip()
        if color and name:
            mapping[color] = name
            if row.get("confirm"):
                confirm.add(name)
    speak = bool(data.get("speak_on_appear", True))
    return mapping, speak, confirm


def _mask_color(hsv: np.ndarray, color: str) -> np.ndarray:
    if color in HSV_RANGES:
        acc = None
        for lo, hi in HSV_RANGES[color]:
            part = cv2.inRange(hsv, np.array(lo), np.array(hi))
            acc = part if acc is None else cv2.bitwise_or(acc, part)
        return acc if acc is not None else np.zeros(hsv.shape[:2], np.uint8)
    v, s = hsv[:, :, 2], hsv[:, :, 1]
    if color == "白":
        return cv2.inRange(hsv, (0, 0, 160), (180, 60, 255))
    if color == "黑":
        return cv2.inRange(hsv, (0, 0, 0), (180, 255, 55))
    if color == "灰":
        return cv2.inRange(hsv, (0, 0, 60), (180, 50, 175))
    return np.zeros(hsv.shape[:2], np.uint8)


def default_near_box(h: int, w: int) -> tuple[int, int, int, int]:
    """没画篮子时，只看画面下半中间：近处盒子大、背景墙和远处衣服在上面。"""
    return (
        int(w * 0.08),
        int(h * 0.40),
        int(w * 0.92),
        int(h * 0.98),
    )


def _iou(a: ColorBox, b: ColorBox) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    if inter <= 0:
        return 0.0
    union = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter
    return inter / union if union else 0.0


def _mean_hue(hsv: np.ndarray, box: ColorBox) -> float:
    roi = hsv[box.y1 : box.y2, box.x1 : box.x2]
    if roi.size == 0:
        return 0.0
    h, s, v = roi[:, :, 0], roi[:, :, 1], roi[:, :, 2]
    m = (s > 40) & (v > 70)
    if not np.any(m):
        return float(np.mean(h))
    return float(np.mean(h[m]))


def _hue_dist(h: float, target: float) -> float:
    d = abs(h - target)
    return min(d, 180.0 - d)


def _resolve_red_pink(hits: list[ColorBox], hsv: np.ndarray) -> list[ColorBox]:
    red = next((b for b in hits if b.color == "红"), None)
    pink = next((b for b in hits if b.color == "粉"), None)
    if red is None or pink is None or _iou(red, pink) < 0.25:
        return hits
    hue = _mean_hue(hsv, red if red.area >= pink.area else pink)
    keep = "粉" if _hue_dist(hue, 158) <= _hue_dist(hue, 0) else "红"
    return [b for b in hits if b.color != ("红" if keep == "粉" else "粉")]


def find_color_boxes(
    bgr: np.ndarray,
    color_to_name: dict[str, str],
    min_frac: float = 0.035,
) -> list[ColorBox]:
    if bgr.size == 0:
        return []
    h, w = bgr.shape[:2]
    min_area = max(2200, int(h * w * min_frac))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (7, 7), 0)
    wanted = list(color_to_name) if color_to_name else list(HSV_RANGES) + ["白", "黑"]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    hits: list[ColorBox] = []
    for color in wanted:
        mask = _mask_color(hsv, color)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for cnt in contours:
            area = int(cv2.contourArea(cnt))
            if area < min_area or area <= best_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw < 24 or bh < 24:
                continue
            ar = bw / float(bh)
            if ar < 0.35 or ar > 3.2:
                continue
            hull = cv2.contourArea(cv2.convexHull(cnt))
            if hull <= 0 or area / hull < 0.55:
                continue
            best_area = area
            best = (x, y, x + bw, y + bh, area)
        if best is None:
            continue
        x1, y1, x2, y2, area = best
        name = color_to_name.get(color) or f"{color}色盒子"
        hits.append(ColorBox(color, name, x1, y1, x2, y2, area))
    hits = _resolve_red_pink(hits, hsv)
    hits.sort(key=lambda b: b.area, reverse=True)
    return hits


def dilate_rect(
    box: tuple[int, int, int, int],
    pad: int,
    w: int,
    h: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box
    return (
        max(0, x1 - pad),
        max(0, y1 - pad),
        min(w, x2 + pad),
        min(h, y2 + pad),
    )


def point_in_rect(cx: float, cy: float, box: tuple[int, int, int, int]) -> bool:
    x1, y1, x2, y2 = box
    return x1 <= cx <= x2 and y1 <= cy <= y2


@dataclass
class _ColorState:
    name: str
    seen: int = 0
    confirmed: bool = False
    gone_since: float | None = None


class GrabTracker:
    """颜色还在画面里（换位置也算）不报。整幅里连续消失 gone_s 秒才报拿出。"""

    def __init__(self, confirm: int = 4, gone_s: float = 1.0, cooldown: float = 3.0) -> None:
        self.confirm = confirm
        self.gone_s = gone_s
        self.cooldown = cooldown
        self.hits: dict[str, _ColorState] = {}
        self.last_said: dict[str, float] = {}

    def reset(self) -> None:
        self.hits.clear()

    def update(self, boxes: list[ColorBox], now: float) -> list[str]:
        spoken: list[str] = []
        present = {b.color: b for b in boxes}
        for color, box in present.items():
            st = self.hits.get(color)
            if st is None:
                self.hits[color] = _ColorState(name=box.name, seen=1)
                continue
            st.name = box.name
            st.gone_since = None
            st.seen += 1
            if st.seen >= self.confirm:
                st.confirmed = True
        for color in list(self.hits):
            st = self.hits[color]
            if color in present:
                continue
            st.seen = 0
            if st.gone_since is None:
                st.gone_since = now
            gone = now - st.gone_since
            if not st.confirmed:
                if gone > 5.0:
                    del self.hits[color]
                continue
            if gone < self.gone_s:
                continue
            if now - self.last_said.get(color, 0) >= self.cooldown:
                spoken.append(st.name)
                self.last_said[color] = now
            del self.hits[color]
        return spoken


class NameAnnouncer:
    def __init__(self, stable: int = 6, cooldown: float = 8.0) -> None:
        self.hits: dict[str, int] = {}
        self.last_said: dict[str, float] = {}
        self.stable = stable
        self.cooldown = cooldown

    def update(self, names: list[str], now: float) -> list[str]:
        present = set(names)
        ready: list[str] = []
        for name in present:
            self.hits[name] = self.hits.get(name, 0) + 1
            if self.hits[name] < self.stable:
                continue
            if now - self.last_said.get(name, 0) < self.cooldown:
                continue
            ready.append(name)
            self.last_said[name] = now
        for name in list(self.hits):
            if name not in present:
                self.hits[name] = 0
        return ready
