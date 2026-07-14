#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Variant B(허프 차선특징 + state) 학습.

사용법:
  python train_feature.py --data ~/imitation_data/session_20260710_120000 [세션2 ...] \
      --epochs 100 --out models/feature_model.pt
"""

import argparse
import os
import sys

import torch
from torch.utils.data import DataLoader, random_split

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from cnn.models import FeatureMLP            # noqa: E402
from dataset import FeatureDrivingDataset     # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", nargs="+", required=True, help="session_* 디렉터리(들)")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val-split", type=float, default=0.2)
    p.add_argument("--out", default="models/feature_model.pt")
    p.add_argument("--device", default="cpu")
    args = p.parse_args()

    dataset = FeatureDrivingDataset(args.data)
    n_val = max(1, int(len(dataset) * args.val_split))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(dataset, [n_train, n_val])
    print(f"데이터 {len(dataset)}개 (train {n_train} / val {n_val})")

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size)

    model = FeatureMLP().to(args.device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = torch.nn.MSELoss()

    best_val = float("inf")
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for x, target in train_loader:
            x, target = x.to(args.device), target.to(args.device)
            optimizer.zero_grad()
            loss = loss_fn(model(x), target)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * x.size(0)
        train_loss /= n_train

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, target in val_loader:
                x, target = x.to(args.device), target.to(args.device)
                val_loss += loss_fn(model(x), target).item() * x.size(0)
        val_loss /= n_val

        if epoch % 10 == 0 or epoch == 1:
            print(f"[{epoch:03d}/{args.epochs}] loss={train_loss:.5f} val_loss={val_loss:.5f}")

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), args.out)

    print(f"저장 완료(최저 val_loss={best_val:.5f}) → {args.out}")


if __name__ == "__main__":
    main()
