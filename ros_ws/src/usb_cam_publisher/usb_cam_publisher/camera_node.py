#!/usr/bin/env python3
# usb_cam_publisher/camera_node.py
import time
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header, Float32


class USBCamBridge(Node):
    def __init__(self) -> None:
        super().__init__("usb_cam_bridge")

        # --- Параметры ноды ---
        self.declare_parameter("device", "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("frame_id", "camera")

        # Производительность/сеть:
        self.declare_parameter("target_fps", 30.0)   
        self.declare_parameter("scale", 0.5)           # 0.5 => 640x360 из 1280x720
        self.declare_parameter("mono", False)          # True => mono8

        # Публикации:
        self.declare_parameter("publish_raw", True)    # базовый топик для совместимости
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("raw_every_n", 3)       # RAW раз в N кадров (уменьшает нагрузку)
        self.declare_parameter("jpeg_quality", 80)     # 1..100

        # Watchdog:
        self.declare_parameter("reopen_on_stall_sec", 1.5)

        p = lambda n: self.get_parameter(n).get_parameter_value()
        self.device  = p("device").string_value
        self.width   = int(p("width").integer_value)
        self.height  = int(p("height").integer_value)
        self.req_fps = int(p("fps").integer_value)
        self.fourcc  = p("fourcc").string_value or ""
        self.frame_id = p("frame_id").string_value

        self.target_fps = float(p("target_fps").double_value or p("target_fps").integer_value)
        self.scale      = float(p("scale").double_value or p("scale").integer_value)
        self.mono       = bool(p("mono").bool_value)

        self.publish_raw        = bool(p("publish_raw").bool_value)
        self.publish_compressed = bool(p("publish_compressed").bool_value)
        self.raw_every_n        = max(1, int(p("raw_every_n").integer_value))
        self.jpeg_quality       = int(p("jpeg_quality").integer_value)
        self.reopen_on_stall    = float(p("reopen_on_stall_sec").double_value or p("reopen_on_stall_sec").integer_value)

        # --- QoS: RAW «строго», COMPRESSED «лёгкий» ---
        qos_raw = QoSProfile(   # чтобы RViz и ноды «видели» без твиков
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_cmp = QoSProfile(   # чтобы видео не блокировалось по сети
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Паблишеры
        self.pub_fps = self.create_publisher(Float32, "camera/fps", 10)
        self.pub_img_raw = self.create_publisher(Image, "camera/image_raw", qos_raw) if self.publish_raw else None
        self.pub_img_cmp = self.create_publisher(CompressedImage, "camera/image_raw/compressed", qos_cmp) if self.publish_compressed else None

        # Очередь «последний кадр»
        self._queue: deque[np.ndarray] = deque(maxlen=1)

        # Состояние
        self._fps_frames = 0
        self._fps_t0 = time.time()
        self._frame_idx = 0
        self._last_new_frame_t = 0.0

        self._stop = False
        self._cap = None
        self._cap_lock = threading.Lock()

        # Потоки: захват и публикация
        self._t_cap = threading.Thread(target=self._capture_loop, daemon=True)
        self._t_pub = threading.Thread(target=self._publish_loop, daemon=True)
        self._t_cap.start()
        self._t_pub.start()

        self.get_logger().info(
            f"USB camera {self.device}  {self.width}x{self.height}@{self.req_fps}  fourcc={self.fourcc or 'default'}  "
            f"→ RAW:{'ON' if self.publish_raw else 'off'}  COMPRESSED:{'ON' if self.publish_compressed else 'off'}"
        )

    # --- V4L2 open ---
    def _open_camera(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS,          self.req_fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # меньше лаг
        except Exception:
            pass
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        f = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Opened {self.device}: {w}x{h}@{f:.1f}")
        return cap

    # --- Захват ---
    def _capture_loop(self):
        backoff = 0.5
        while not self._stop and rclpy.ok():
            with self._cap_lock:
                if self._cap is None:
                    self._cap = self._open_camera()
                    if self._cap is None:
                        self.get_logger().warn(f"Cannot open camera {self.device}; retry in {backoff:.1f}s")
                        time.sleep(backoff); backoff = min(backoff * 1.5, 5.0)
                        continue
                    backoff = 0.5
                ok, frame = self._cap.read()

            if not ok:
                self.get_logger().warn("Frame grab failed; reopening camera…")
                with self._cap_lock:
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                time.sleep(0.2)
                continue

            if self.scale and self.scale != 1.0:
                nw = max(1, int(frame.shape[1] * self.scale))
                nh = max(1, int(frame.shape[0] * self.scale))
                frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)

            self._queue.append(frame)
            self._last_new_frame_t = time.time()

    # --- Публикация ---
    def _publish_loop(self):
        period = 1.0 / max(self.target_fps, 1.0)
        enc_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]

        while not self._stop and rclpy.ok():
            t0 = time.time()

            # watchdog: если новых кадров нет — переоткрыть
            if self.reopen_on_stall and (time.time() - self._last_new_frame_t > self.reopen_on_stall):
                self.get_logger().warn("No new frames, reopening camera (stall watchdog)…")
                with self._cap_lock:
                    try:
                        if self._cap is not None:
                            self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                time.sleep(0.1)

            frame = self._queue[-1] if self._queue else None
            if frame is not None:
                stamp = self.get_clock().now().to_msg()

                # Готовим изображения
                if self.mono:
                    work_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # mono8
                    work_bgr = None
                else:
                    work_bgr = frame                                     # bgr8
                    work_mono = None

                # RAW (раз в N кадров)
                if self.pub_img_raw is not None and (self._frame_idx % self.raw_every_n == 0):
                    msg = Image()
                    msg.header = Header(stamp=stamp, frame_id=self.frame_id)
                    if self.mono:
                        h, w = work_mono.shape
                        msg.height, msg.width = h, w
                        msg.encoding = "mono8"
                        msg.step = msg.width * 1
                        msg.data = work_mono.tobytes()
                    else:
                        h, w = work_bgr.shape[:2]
                        msg.height, msg.width = h, w
                        msg.encoding = "bgr8"          # цвет публикуем как bgr8
                        msg.step = msg.width * 3
                        msg.data = work_bgr.tobytes()
                    self.pub_img_raw.publish(msg)

                # COMPRESSED (каждый кадр)
                if self.pub_img_cmp is not None:
                    if self.mono:
                        ok_jpg, buff = cv2.imencode(".jpg", work_mono, enc_params)
                    else:
                        ok_jpg, buff = cv2.imencode(".jpg", work_bgr, enc_params)
                    if ok_jpg:
                        cmsg = CompressedImage()
                        cmsg.header = Header(stamp=stamp, frame_id=self.frame_id)
                        cmsg.format = "jpeg"
                        cmsg.data = buff.tobytes()
                        self.pub_img_cmp.publish(cmsg)

                self._frame_idx += 1
                self._tick_fps()

            # дросселируемся до target_fps
            dt = time.time() - t0
            if period - dt > 0:
                time.sleep(period - dt)

    # --- FPS в лог и топик ---
    def _tick_fps(self):
        self._fps_frames += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            fps = self._fps_frames / (now - self._fps_t0)
            self.get_logger().info(f"Camera FPS: {fps:.1f}")
            self.pub_fps.publish(Float32(data=float(fps)))
            self._fps_frames = 0
            self._fps_t0 = now

    # --- корректное завершение ---
    def destroy_node(self):
        self._stop = True
        try:
            if hasattr(self, "_t_cap"): self._t_cap.join(timeout=1.0)
            if hasattr(self, "_t_pub"): self._t_pub.join(timeout=1.0)
        except Exception:
            pass
        with self._cap_lock:
            try:
                if self._cap is not None:
                    self._cap.release()
            except Exception:
                pass
            self._cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
