#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""학습 단계 — ROS2와 무관한 단독 파이썬 스크립트.

실행하면 폴더 선택 창(tkinter)이 뜬다. data/ 안의 lane_XXX 또는 all_XXX 세션 폴더를
고르면 폴더 종류를 자동 판별해 알맞은 모델(LaneNet/AllNet)을 PyTorch로 학습하고,
결과를 trained/train_XXX/ (model.pt + meta.json)로 저장한다.

  python train.py                    # GUI로 세션 폴더 선택
  python train.py data/all_001       # 폴더를 인자로 직접 지정 (GUI 생략)

학습 설정은 아래 파라미터 블록에서 수정한다.
학습 라벨(요구사항 고정):
  조향 = cur_angle — 아두이노 내부에서 실제 출력된 조향각(가변저항 환산, /out 텔레메트리)
  속도 = cmd_pwm  — joystick이 /in으로 발행한 주행모터 PWM 지시값
  (joystick의 조향 필드는 시간모드 조향PWM이라 라벨로 쓰지 않는다)
"""

import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset, random_split

sys.path.insert(0, str(Path(__file__).resolve().parent))
from donkey.models import build_model, IMG_W, IMG_H          # noqa: E402
from donkey.lane_detect import FEATURE_NAMES                  # noqa: E402
from donkey.common import (norm_angle, norm_pwm,              # noqa: E402
                           package_root, next_numbered_dir)

# ==========================================================
# ★ 학습 파라미터 (여기서 수정) ★
# ==========================================================
EPOCHS      = 50
BATCH_SIZE  = 64
LEARNING_RATE = 1e-3
VAL_SPLIT   = 0.2          # 검증셋 비율
DEVICE      = "cuda" if torch.cuda.is_available() else "cpu"
# ==========================================================


def pick_session_dir() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser().resolve()
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()
    picked = filedialog.askdirectory(
        title="세션 폴더 선택 (data/lane_XXX 또는 data/all_XXX)",
        initialdir=str(package_root() / "data"))
    root.destroy()
    if not picked:
        print("폴더를 선택하지 않아 종료합니다.")
        sys.exit(0)
    return Path(picked)


def detect_type(session_dir: Path) -> str:
    if session_dir.name.startswith("lane_"):
        return "lane"
    if session_dir.name.startswith("all_"):
        return "all"
    # 폴더명이 규칙과 다르면 내용으로 판별
    return "all" if (session_dir / "images").is_dir() else "lane"


def load_dataset(session_dir: Path, data_type: str):
    """log.csv → (X, y) 텐서. y = [angle_n, pwm_n]."""
    with open(session_dir / "log.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"log.csv가 비어있음: {session_dir}")

    # 조향=실측 각도(cur_angle), 속도=주행PWM 지시값(cmd_pwm)
    ys = [[norm_angle(float(r["cur_angle"])), norm_pwm(float(r["cmd_pwm"]))]
          for r in rows]
    y = torch.tensor(ys, dtype=torch.float32)

    if data_type == "lane":
        X = torch.tensor([[float(r[c]) for c in FEATURE_NAMES] for r in rows],
                         dtype=torch.float32)
    else:
        imgs = []
        for r in rows:
            img = cv2.imread(str(session_dir / "images" / r["image"]))
            if img is None:
                raise FileNotFoundError(session_dir / "images" / r["image"])
            img = cv2.resize(img, (IMG_W, IMG_H))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            imgs.append(img.transpose(2, 0, 1))
        X = torch.tensor(np.stack(imgs), dtype=torch.float32)

    return TensorDataset(X, y)


def main():
    session_dir = pick_session_dir()
    data_type = detect_type(session_dir)
    print(f"세션: {session_dir}  (타입: {data_type})")

    dataset = load_dataset(session_dir, data_type)
    n_val = max(1, int(len(dataset) * VAL_SPLIT))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(dataset, [n_train, n_val])
    print(f"데이터 {len(dataset)}개 (train {n_train} / val {n_val}) | device={DEVICE}")

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE)

    model = build_model(data_type).to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    loss_fn = torch.nn.MSELoss()

    out_dir = next_numbered_dir(package_root() / "trained", "train")
    out_dir.mkdir(parents=True, exist_ok=True)
    best_val = float("inf")
    history = []

    for epoch in range(1, EPOCHS + 1):
        model.train()
        train_loss = 0.0
        for X, y in train_loader:
            X, y = X.to(DEVICE), y.to(DEVICE)
            optimizer.zero_grad()
            loss = loss_fn(model(X), y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * X.size(0)
        train_loss /= n_train

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for X, y in val_loader:
                X, y = X.to(DEVICE), y.to(DEVICE)
                val_loss += loss_fn(model(X), y).item() * X.size(0)
        val_loss /= n_val

        history.append({"epoch": epoch, "loss": round(train_loss, 6),
                        "val_loss": round(val_loss, 6)})
        print(f"[{epoch:03d}/{EPOCHS}] loss={train_loss:.5f} val_loss={val_loss:.5f}")

        if val_loss < best_val:   # val_loss 최저 시점만 저장 (과적합 대비)
            best_val = val_loss
            torch.save(model.state_dict(), out_dir / "model.pt")

    with open(out_dir / "meta.json", "w", encoding="utf-8") as f:
        json.dump({
            "type": data_type,
            "session": session_dir.name,
            "angle_label": "cur_angle",
            "epochs": EPOCHS, "batch_size": BATCH_SIZE, "lr": LEARNING_RATE,
            "img_w": IMG_W, "img_h": IMG_H,
            "best_val_loss": round(best_val, 6),
            "history": history,
        }, f, ensure_ascii=False, indent=2)

    print(f"\n학습 완료 (최저 val_loss={best_val:.5f})")
    print(f"결과 → {out_dir}  (model.pt + meta.json)")
    print(f"주행:  ros2 launch donkey run.launch.py train:={out_dir.name}")


if __name__ == "__main__":
    main()
