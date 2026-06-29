#!/usr/bin/env python3
"""
SO-100 VLM 交互控制 — 用户输入指令，结合摄像头画面和机械臂当前状态，由 Ollama 视觉模型决策并执行。
Requires: pip install ollama opencv-python requests

需要先启动 dfarm_server.py：
  python dfarm_server.py [port] [--cal dfarm_calibration.json]

Usage:
  python 8_dfarm_vlm_watch.py [--camera 0] [--model qwen2.5vl:7b] [--api http://127.0.0.1:8001]
"""

import argparse
import base64
import json
import re
import threading
import time

import cv2
import ollama
import requests

# ── Prompt ────────────────────────────────────────────────────────────────────

SYSTEM_PROMPT = """\
你是一个控制 SO-100 六轴机械臂的 AI 助手。摄像头安装在末端夹爪旁，朝向正下方俯视工作台。
你每次只输出一个动作，执行后会再次收到新的画面和状态，直到任务完成。

【关节说明】
  shoulder_pan  (-180~180°)：底座左右旋转，正值向右
  shoulder_lift (-120~30°) ：大臂俯仰，负值抬起
  elbow_flex    (-10~180°) ：肘部弯曲
  wrist_flex    (0~180°)   ：腕部俯仰
  wrist_roll    (-180~180°)：腕部旋转
  gripper       (0~100)    ：夹爪开合，0=闭合，50=打开

【预设姿态】
  home     - 收起待机
  overhead - 俯视工作台（摄像头朝下）

【可用动作（action 字段）】
  "none"                          不执行任何动作
  "pose:<name>"                   移动到预设姿态，如 "pose:home"
  "joints:[p,l,e,wf,wr,g]"       设置6个关节角度，如 "joints:[0,-45,9,93,0,50]"
  "pan:<度数>"                    仅旋转底座
  "gripper:<0-100>"               仅控制夹爪

每次输出一个 JSON 对象（不要有其他文字）：
{
  "scene": "一句话描述当前画面",
  "action": "动作字符串",
  "reason": "选择该动作的简短理由",
  "done": false
}
当任务已完成时，将 "done" 设为 true，"action" 设为 "none"。
"""


# ── 动作执行 ──────────────────────────────────────────────────────────────────

def execute_action(action: str, api: str):
    if not action or action == "none":
        return
    try:
        if action.startswith("pose:"):
            name = action.split(":", 1)[1]
            requests.post(f"{api}/move/pose", json={"name": name, "duration": 2.0}, timeout=10)

        elif action.startswith("joints:"):
            angles = json.loads(action.split(":", 1)[1])
            requests.post(f"{api}/move/joints", json={"angles": angles, "duration": 2.0}, timeout=10)

        elif action.startswith("pan:"):
            deg = float(action.split(":", 1)[1])
            cur = requests.get(f"{api}/joints", timeout=5).json()
            angles = [deg, cur["shoulder_lift"], cur["elbow_flex"],
                      cur["wrist_flex"], cur["wrist_roll"], cur.get("gripper", 50)]
            requests.post(f"{api}/move/joints", json={"angles": angles, "duration": 1.5}, timeout=10)

        elif action.startswith("gripper:"):
            val = float(action.split(":", 1)[1])
            cur = requests.get(f"{api}/joints", timeout=5).json()
            angles = [cur["shoulder_pan"], cur["shoulder_lift"], cur["elbow_flex"],
                      cur["wrist_flex"], cur["wrist_roll"], val]
            requests.post(f"{api}/move/joints", json={"angles": angles, "duration": 0.8}, timeout=10)

        print(f"  [执行] {action}")
    except Exception as e:
        print(f"  [执行失败] {e}")


# ── 摄像头帧缓存 ──────────────────────────────────────────────────────────────

class FrameBuffer:
    def __init__(self):
        self._frame = None
        self._lock  = threading.Lock()

    def put(self, frame):
        with self._lock:
            self._frame = frame

    def get_jpeg(self) -> bytes | None:
        with self._lock:
            if self._frame is None:
                return None
            _, buf = cv2.imencode(".jpg", self._frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buf.tobytes()


# ── VLM 推理 ──────────────────────────────────────────────────────────────────

def query_vlm(messages: list, frame_bytes: bytes, arm_state: dict, model: str) -> dict:
    """单次推理，messages 包含完整对话历史。"""
    state_str  = json.dumps(arm_state, ensure_ascii=False)
    user_content = f"当前机械臂状态：{state_str}"
    b64 = base64.b64encode(frame_bytes).decode()

    messages = messages + [{"role": "user", "content": user_content, "images": [b64]}]
    resp = ollama.chat(model=model, messages=messages)
    text = resp["message"]["content"].strip()
    m    = re.search(r'\{.*\}', text, re.DOTALL)
    if not m:
        return {"scene": text, "action": "none", "reason": "无法解析 JSON", "done": True}
    return json.loads(m.group())


def run_agent(user_cmd: str, buf: FrameBuffer, api: str, model: str, max_steps: int = 10):
    """Agentic loop：每步执行一个动作，重新观察，直到 done=True 或超过最大步数。"""
    # 对话历史：system + 用户初始指令
    messages = [
        {"role": "system",  "content": SYSTEM_PROMPT},
        {"role": "user",    "content": f"用户任务：{user_cmd}"},
    ]

    for step in range(1, max_steps + 1):
        frame_bytes = buf.get_jpeg()
        if frame_bytes is None:
            print("  [错误] 无法获取摄像头画面"); break

        arm_state = {}
        try:
            arm_state = requests.get(f"{api}/joints", timeout=3).json()
        except Exception:
            pass

        print(f"\n  [步骤 {step}/{max_steps}] 推理中...")
        try:
            result = query_vlm(messages, frame_bytes, arm_state, model)
        except Exception as e:
            print(f"  [VLM 错误] {e}"); break

        action = result.get("action", "none")
        done   = result.get("done", False)
        print(f"  scene:  {result.get('scene', '')}")
        print(f"  action: {action}  done: {done}")
        print(f"  reason: {result.get('reason', '')}")

        # 将模型输出加入历史，让下一步知道已做了什么
        messages.append({"role": "assistant", "content": json.dumps(result, ensure_ascii=False)})

        if done or action == "none":
            print("  [任务完成]")
            break

        execute_action(action, api)

        # 等待机械臂运动完成后再观察（粗略估计，可按需调整）
        time.sleep(2.5)

    else:
        print(f"  [超过最大步数 {max_steps}，停止]")


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera",   type=int, default=0)
    ap.add_argument("--model",    default="qwen3.5:35b")
    ap.add_argument("--api",      default="http://127.0.0.1:8001")
    args = ap.parse_args()

    print(f"DFarm VLM 控制  model={args.model}  api={args.api}")
    print("摄像头窗口已开启，在终端输入指令控制机械臂。输入 q 退出。\n")

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print(f"ERROR: 无法打开相机 {args.camera}"); return

    buf  = FrameBuffer()
    stop = threading.Event()

    # 摄像头显示线程
    def cam_loop():
        while not stop.is_set():
            ok, frame = cap.read()
            if ok:
                buf.put(frame)
                cv2.imshow("DFarm VLM", frame)
            if cv2.waitKey(30) & 0xFF == 27:
                stop.set(); break
            if cv2.getWindowProperty("DFarm VLM", cv2.WND_PROP_VISIBLE) < 1:
                stop.set(); break

    threading.Thread(target=cam_loop, daemon=True).start()
    time.sleep(0.5)  # 等相机预热

    try:
        while not stop.is_set():
            try:
                cmd = input(">> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if cmd.lower() == "q":
                break
            if not cmd:
                continue

            run_agent(cmd, buf, args.api, args.model)
    finally:
        stop.set()
        cap.release()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
