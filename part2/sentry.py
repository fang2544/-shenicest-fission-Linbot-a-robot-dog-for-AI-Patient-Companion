#!/usr/bin/env python3
"""影石 Camera SDK 预览 + 篮内物品 + 人离开提醒。"""

from __future__ import annotations

import argparse
import csv
import os
import re
import select
import subprocess
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

from color_boxes import DRAW_BGR, GrabTracker, find_color_boxes, load_items

# YOLO-World 用英文提示词；画面上显示中文。
PROMPT_CN: list[tuple[str, str]] = [
    ("person", "人"),
    ("cup", "杯子"),
    ("mug", "杯子"),
    ("bottle", "瓶子"),
    ("cell phone", "手机"),
    ("keys", "钥匙"),
    ("wallet", "钱包"),
    ("book", "书"),
    ("laptop", "电脑"),
    ("backpack", "书包"),
    ("handbag", "包"),
    ("apple", "苹果"),
    ("banana", "香蕉"),
    ("remote", "遥控器"),
    ("mouse", "鼠标"),
    ("keyboard", "键盘"),
    ("glasses", "眼镜"),
    ("headphones", "耳机"),
    ("watch", "手表"),
    ("pen", "笔"),
]

CN = {en: zh for en, zh in PROMPT_CN}
CN.update(
    {
        "wine glass": "杯子",
        "bowl": "碗",
        "sandwich": "食物",
        "orange": "橙子",
        "scissors": "剪刀",
        "umbrella": "伞",
        "clock": "钟",
        "vase": "花瓶",
        "teddy bear": "玩偶",
        "toothbrush": "牙刷",
        "suitcase": "箱子",
    }
)
BASKET_OK = {k for k in CN if k != "person"}

_FONT_CACHE: dict[int, ImageFont.ImageFont] = {}
TAKES_CSV = Path(__file__).resolve().parent / "takes.csv"
RESTOCK_AT = 50


def append_take(name: str) -> None:
    new_file = not TAKES_CSV.exists()
    with TAKES_CSV.open("a", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["time", "name"])
        w.writerow([datetime.now().isoformat(timespec="seconds"), name])


@dataclass
class Roi:
    x1: int = 0
    y1: int = 0
    x2: int = 0
    y2: int = 0
    dragging: bool = False

    def ready(self) -> bool:
        return abs(self.x2 - self.x1) > 20 and abs(self.y2 - self.y1) > 20

    def box(self) -> tuple[int, int, int, int]:
        return (
            min(self.x1, self.x2),
            min(self.y1, self.y2),
            max(self.x1, self.x2),
            max(self.y1, self.y2),
        )


@dataclass
class State:
    seen_person: bool = False
    person_gone_since: float | None = None
    last_alert: float = 0.0
    empty_gray: np.ndarray | None = None
    names: list[str] = field(default_factory=list)
    dets: list[tuple[str, float]] = field(default_factory=list)
    occupied: bool = False
    person: bool = False
    label: str = "IDLE"


def occupancy(roi_bgr: np.ndarray, empty: np.ndarray | None, thr: float) -> bool:
    if empty is None or roi_bgr.size == 0:
        return False
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    empty = cv2.resize(empty, (gray.shape[1], gray.shape[0]))
    diff = cv2.absdiff(gray, empty)
    _, mask = cv2.threshold(diff, 28, 255, cv2.THRESH_BINARY)
    return float(np.mean(mask > 0)) >= thr


def crop(frame: np.ndarray, roi: Roi) -> np.ndarray:
    x1, y1, x2, y2 = roi.box()
    h, w = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    return frame[y1:y2, x1:x2]


def in_roi(cx: float, cy: float, roi: Roi) -> bool:
    x1, y1, x2, y2 = roi.box()
    return x1 <= cx <= x2 and y1 <= cy <= y2


def speak(text: str) -> None:
    try:
        subprocess.Popen(
            ["say", text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        sys.stderr.write(f"[speak] {text}\n")


def uniq_names(names: list[str]) -> list[str]:
    out = []
    for n in names:
        if n not in out:
            out.append(n)
    return out


def phrase(names: list[str]) -> str:
    items = uniq_names(names)
    if not items:
        return "请带走篮子里的物品"
    return f"请带走篮子里的{'和'.join(items[:3])}"


def identify_phrase(names: list[str], confirm: set[str] | None = None) -> str:
    items = uniq_names(names)
    if not items:
        return "没有拿出物品"
    text = f"拿出{'和'.join(items[:4])}"
    if confirm and any(n in confirm for n in items):
        text += "，请医护确认"
    return text


def cn_font(size: int) -> ImageFont.ImageFont:
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    for path in (
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/STHeiti Light.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        "/System/Library/Fonts/Supplemental/Songti.ttc",
    ):
        if os.path.exists(path):
            font = ImageFont.truetype(path, size, index=0)
            _FONT_CACHE[size] = font
            return font
    font = ImageFont.load_default()
    _FONT_CACHE[size] = font
    return font


def overlay_cn(
    frame: np.ndarray,
    labels: list[tuple[int, int, str, tuple[int, int, int]]],
    panel: list[str],
) -> np.ndarray:
    im = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")
    draw = ImageDraw.Draw(im, "RGBA")
    font = cn_font(22)
    small = cn_font(18)
    for x, y, text, bgr in labels:
        rgb = (bgr[2], bgr[1], bgr[0], 255)
        box = draw.textbbox((x, y), text, font=font)
        draw.rectangle(
            [box[0] - 4, box[1] - 3, box[2] + 4, box[3] + 3],
            fill=(0, 0, 0, 170),
        )
        draw.text((x, y), text, font=font, fill=rgb)
    if panel:
        x0, y0 = 12, 44
        widths = [draw.textbbox((0, 0), line, font=small)[2] for line in panel]
        w = max(widths) + 24
        h = 10 + len(panel) * 26
        draw.rectangle([x0, y0, x0 + w, y0 + h], fill=(0, 0, 0, 170))
        for i, line in enumerate(panel):
            draw.text((x0 + 10, y0 + 6 + i * 26), line, font=small, fill=(255, 255, 255, 255))
    return cv2.cvtColor(np.array(im.convert("RGB")), cv2.COLOR_RGB2BGR)


def load_detector():
    try:
        from ultralytics import YOLOWorld

        model = YOLOWorld("yolov8n-worldv2.pt")
        model.set_classes([en for en, _ in PROMPT_CN])
        print("物品识别：YOLO-World（杯子/手机/钥匙/钱包等）")
        return model, "world"
    except Exception as exc:
        print(f"YOLO-World 不可用，改用 COCO：{exc}")
        return YOLO("yolov8n.pt"), "coco"


def list_avfoundation_video() -> list[tuple[int, str]]:
    try:
        proc = subprocess.run(
            ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
            text=True,
            capture_output=True,
            timeout=8,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    output = f"{proc.stderr}\n{proc.stdout}"
    video_block = output.split("AVFoundation audio devices", 1)[0]
    found: list[tuple[int, str]] = []
    for line in video_block.splitlines():
        m = re.search(r"\] \[(\d+)\] (.+)$", line)
        if m:
            found.append((int(m.group(1)), m.group(2).strip()))
    return found


def find_insta360_index(prefer: str = "auto") -> tuple[int, str] | None:
    devices = list_avfoundation_video()

    def pick(*needles: str) -> tuple[int, str] | None:
        for idx, name in devices:
            low = name.lower()
            if any(n.lower() in low for n in needles):
                return idx, name
        return None

    if prefer in ("link2", "link2c", "auto"):
        hit = pick("insta360 link")
        if hit:
            return hit
    if prefer in ("x4", "auto"):
        hit = pick("x4") or pick("x5")
        if hit:
            return hit
    return pick("virtual camera") or pick("insta360")


_PANO_NOTED = False


def crop_if_pano(frame: np.ndarray) -> np.ndarray:
    """X4 Webcam 是 2:1 全景，只取中间一块当篮子。16:9（Link）不裁。"""
    global _PANO_NOTED
    h, w = frame.shape[:2]
    if w / h < 1.85:
        return frame
    cw, ch = int(w * 0.40), int(h * 0.50)
    x1 = (w - cw) // 2
    y1 = (h - ch) // 2
    if not _PANO_NOTED:
        _PANO_NOTED = True
        print(f"X4 全景 {w}x{h}，识别区域取中间 {cw}x{ch}（镜头对着篮子）")
    return frame[y1 : y1 + ch, x1 : x1 + cw]


JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"

CAMERA_HELP = """
macOS 没允许这个程序用摄像头。

X4：开机，USB 插 Mac，相机屏幕选「Webcam / USB 摄像头」。
Link 2C：关掉影石软件，拨开挡片。

系统设置 → 隐私与安全性 → 摄像头 → 打开「终端」。
"""


def resolve_source(spec: str) -> str:
    key = spec.strip().lower()
    if key not in ("auto", "link2", "link2c", "x4", "insta360"):
        return spec
    prefer = "x4" if key == "x4" else ("link2" if key in ("link2", "link2c") else "auto")
    hit = find_insta360_index(prefer)
    if hit is None:
        raise SystemExit(
            "没找到影石摄像头。X4 请选 Webcam 模式；Link 请插 USB。\n"
            '  ffmpeg -f avfoundation -list_devices true -i ""'
        )
    idx, name = hit
    print(f"用 AVFoundation 设备 [{idx}] {name}")
    return str(idx)


def open_camera_settings() -> None:
    if sys.platform != "darwin":
        return
    subprocess.Popen(
        ["open", "x-apple.systempreferences:com.apple.preference.security?Privacy_Camera"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def camera_denied_exit(detail: str = "") -> None:
    open_camera_settings()
    extra = f"\nffmpeg: {detail.strip()}\n" if detail.strip() else ""
    raise SystemExit(CAMERA_HELP + extra)


class FfmpegAvfSource:
    """macOS 上用 ffmpeg 拉 AVFoundation，避开 OpenCV 要不到相机权限的问题。"""

    LINK_VARIANTS = (
        ["-pixel_format", "uyvy422", "-framerate", "30", "-video_size", "1280x720"],
        ["-framerate", "30", "-video_size", "1280x720"],
        ["-framerate", "30"],
    )
    X4_VARIANTS = (
        ["-framerate", "30", "-video_size", "2880x1440"],
        ["-framerate", "30", "-video_size", "1920x960"],
        ["-framerate", "30"],
        ["-pixel_format", "uyvy422", "-framerate", "30", "-video_size", "1280x720"],
    )

    def __init__(self, index: int) -> None:
        self.proc: subprocess.Popen | None = None
        self._buf = b""
        self._err = ""
        last_err = ""
        name = ""
        for i, n in list_avfoundation_video():
            if i == index:
                name = n
                break
        variants = self.LINK_VARIANTS if "link" in name.lower() else self.X4_VARIANTS
        print("正在打开摄像头，若弹出「终端」相机权限，请点允许…")
        for extra in variants:
            self.release()
            self._buf = b""
            self.proc = self._spawn(index, extra)
            threading.Thread(target=self._drain_err, daemon=True).start()
            ok, frame = self._read_until(12.0)
            if ok:
                self._first = frame
                print("ffmpeg 已出画")
                return
            time.sleep(0.25)
            last_err = self._err or "无画面"
            self.release()
        camera_denied_exit(last_err)

    def _spawn(self, index: int, extra: list[str]) -> subprocess.Popen:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "avfoundation",
            *extra,
            "-i",
            f"{index}:none",
            "-an",
            "-vf",
            "fps=15,scale=1280:-2",
            "-q:v",
            "5",
            "-f",
            "mjpeg",
            "pipe:1",
        ]
        return subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

    def _drain_err(self) -> None:
        if self.proc is None or self.proc.stderr is None:
            return
        try:
            self._err = self.proc.stderr.read().decode("utf-8", "replace")
        except OSError:
            return

    def _pop_jpeg(self):
        start = self._buf.find(JPEG_SOI)
        if start < 0:
            self._buf = self._buf[-1:]
            return None
        last = None
        search = start
        while True:
            s = self._buf.find(JPEG_SOI, search)
            if s < 0:
                break
            e = self._buf.find(JPEG_EOI, s + 2)
            if e < 0:
                break
            last = (s, e + 2)
            search = e + 2
        if last is None:
            self._buf = self._buf[start:]
            return None
        jpeg = self._buf[last[0] : last[1]]
        self._buf = self._buf[last[1] :]
        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        return img

    def _read_until(self, timeout: float):
        if self.proc is None or self.proc.stdout is None:
            return False, None
        fd = self.proc.stdout.fileno()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                return False, None
            ready, _, _ = select.select([fd], [], [], 0.15)
            if not ready:
                continue
            chunk = os.read(fd, 65536)
            if not chunk:
                return False, None
            self._buf += chunk
            img = self._pop_jpeg()
            if img is not None:
                return True, img
        return False, None

    def read(self):
        first = getattr(self, "_first", None)
        if first is not None:
            self._first = None
            return True, first
        return self._read_until(2.0)

    def release(self) -> None:
        proc = self.proc
        self.proc = None
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=1)


class FrameSource:
    """auto = 优先 Link 2/2C；数字在 Mac 上走 ffmpeg。"""

    def __init__(self, spec: str) -> None:
        spec = resolve_source(spec)
        self.kind = "cam"
        self.path = spec
        self.cap = None
        self._ffmpeg = None
        self._h264 = None
        if spec.startswith("h264:"):
            self.kind = "h264"
            self.path = spec[5:]
            try:
                import av
            except ImportError as e:
                raise SystemExit("读 SDK 码流需要: pip install av") from e
            self._h264 = av.open(self.path, format="h264")
            self._gen = self._h264.decode(video=0)
        elif spec.startswith("mjpeg:"):
            self.cap = cv2.VideoCapture(spec[6:])
            if not self.cap.isOpened():
                raise SystemExit(f"打不开 MJPEG {spec[6:]}")
        elif spec.startswith("http://") or spec.startswith("https://"):
            self.cap = cv2.VideoCapture(spec)
            if not self.cap.isOpened():
                raise SystemExit(f"打不开 {spec}")
        elif spec.lower().endswith((".jpg", ".jpeg", ".png")):
            self.kind = "jpeg"
        elif sys.platform == "darwin" and spec.isdigit():
            self.kind = "ffmpeg"
            self._ffmpeg = FfmpegAvfSource(int(spec))
        else:
            self.cap = cv2.VideoCapture(int(spec))
            if not self.cap.isOpened():
                camera_denied_exit(f"打不开摄像头 {spec}")

    def read(self):
        if self.kind == "ffmpeg":
            return self._ffmpeg.read()
        if self.kind == "cam":
            return self.cap.read()
        if self.kind == "jpeg":
            img = cv2.imread(self.path)
            return img is not None, img
        try:
            frame = next(self._gen)
            return True, frame.to_ndarray(format="bgr24")
        except (StopIteration, Exception):
            return False, None

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
        if self._ffmpeg is not None:
            self._ffmpeg.release()
        if self._h264 is not None:
            self._h264.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="篮子物品识别 + 离场提醒")
    p.add_argument(
        "--source",
        default="auto",
        help="auto=优先 Link 2/2C；link2；x4；或摄像头序号",
    )
    p.add_argument("--gone-s", type=float, default=1.0, help="颜色离开整幅画面多久才算拿出")
    p.add_argument("--cooldown-s", type=float, default=10.0)
    p.add_argument("--occ", type=float, default=0.08, help="空篮像素差比例")
    p.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度（仅 --yolo）")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--yolo", action="store_true", help="额外用 YOLO 认人/通用物品")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    src = FrameSource(args.source)

    print("整幅画面当作篮子。换位置不报；离开画面约 1 秒才算拿出。")
    color_map, _speak_flag, confirm_names = load_items()
    if color_map:
        print("清单：", "、".join(f"{c}→{n}" for c, n in color_map.items()))
    else:
        print("清单还是空的：会报「红色盒子」这种颜色名。")
    grabber = GrabTracker(gone_s=args.gone_s)
    model = None
    if args.yolo:
        print("加载 YOLO…")
        model, _kind = load_detector()
    st = State()
    win = "basket-sentry"
    last_grab = ""
    counts: Counter[str] = Counter()
    ui = {"reset": (0, 0, 0, 0)}

    def hit(rect: tuple[int, int, int, int], x: int, y: int) -> bool:
        x1, y1, x2, y2 = rect
        return x1 <= x <= x2 and y1 <= y <= y2

    def on_mouse(event, x, y, _flags, _data) -> None:
        nonlocal last_grab
        if event == cv2.EVENT_LBUTTONDOWN and hit(ui["reset"], x, y):
            counts.clear()
            last_grab = ""
            print("已补货，次数清零")

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print("任一药品拿出满 50 次后出现「补货」。右上角点击可清零。")

    while True:
        ok, frame = src.read()
        if not ok or frame is None:
            print("丢帧，检查摄像头是否被占用")
            time.sleep(0.05)
            continue
        frame = crop_if_pano(frame)

        st.person = False
        st.names = []
        st.dets = []
        labels: list[tuple[int, int, str, tuple[int, int, int]]] = []
        h, w = frame.shape[:2]
        hits: list = []
        for box in find_color_boxes(frame, color_map, min_frac=0.015):
            hits.append(box)
            color_bgr = DRAW_BGR.get(box.color, (40, 220, 220))
            cv2.rectangle(frame, (box.x1, box.y1), (box.x2, box.y2), color_bgr, 2)
            labels.append((box.x1, max(8, box.y1 - 28), box.name, color_bgr))
        st.names = [b.name for b in hits]
        st.occupied = False

        now = time.time()
        grabbed = grabber.update(hits, now)
        if grabbed:
            last_grab = "、".join(uniq_names(grabbed))
            for name in uniq_names(grabbed):
                counts[name] += 1
                append_take(name)
            speak(identify_phrase(grabbed, confirm_names))

        if model is not None:
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            for r in results:
                if r.boxes is None:
                    continue
                names_map = r.names if getattr(r, "names", None) else model.names
                for b in r.boxes:
                    cls_id = int(b.cls[0])
                    name = names_map[cls_id]
                    conf = float(b.conf[0]) if b.conf is not None else 0.0
                    x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                    if name == "person":
                        st.person = True
                        color = (80, 160, 255)
                        labels.append((x1, max(8, y1 - 28), f"人 {conf:.0%}", color))
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        has_stuff = st.occupied or bool(st.names)
        if st.person:
            st.seen_person = True
            st.person_gone_since = None
            st.label = "ATTENDED"
        elif st.seen_person:
            if st.person_gone_since is None:
                st.person_gone_since = now
            gone = now - st.person_gone_since
            if has_stuff and gone >= args.leave_s:
                st.label = "LEFT_ITEMS"
                if now - st.last_alert >= args.cooldown_s:
                    speak(phrase(st.names))
                    st.last_alert = now
            elif not has_stuff:
                st.label = "EMPTY"
            else:
                st.label = "LEFT_WAIT"
        else:
            st.label = "IDLE"

        need_restock = any(n >= RESTOCK_AT for n in counts.values())
        ui["reset"] = (0, 0, 0, 0)
        if need_restock:
            btn_w, btn_h = 132, 46
            rx1, ry1 = w - btn_w - 12, 10
            rx2, ry2 = rx1 + btn_w, ry1 + btn_h
            ui["reset"] = (rx1, ry1, rx2, ry2)
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (60, 60, 200), -1)
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (255, 255, 255), 1)
            labels.append((rx1 + 36, ry1 + 10, "补货", (255, 255, 255)))

        items = uniq_names(st.names)
        tally = "  ".join(f"{n}×{c}" for n, c in counts.most_common()) or "无"
        panel = [
            "整幅画面=篮子  离屏1秒=拿出",
            "画面  " + ("、".join(items) if items else "无"),
            "刚拿  " + (last_grab or "—"),
            f"合计  {sum(counts.values())}",
            "次数  " + tally,
        ]
        if color_map:
            panel.append("清单  " + "、".join(f"{c}{n}" for c, n in color_map.items()))
        shown = overlay_cn(frame, labels, panel)
        cv2.putText(
            shown,
            "补货  |  c 补货  |  r 当前  |  l 清单  |  q"
            if need_restock
            else "r 当前  |  l 清单  |  q",
            (12, shown.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )
        cv2.imshow(win, shown)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("t"):
            speak(phrase(st.names))
        if key == ord("r"):
            speak(identify_phrase(st.names) if st.names else "篮子里没有物品")
        if key == ord("l"):
            color_map, _speak_flag, confirm_names = load_items()
            print("已加载清单：", color_map or "（空）")
        if key == ord("c") and need_restock:
            counts.clear()
            last_grab = ""
            print("已补货，次数清零")

    src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
