#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Смета электротележки: проверяет docs/bom.json, считает итоги и пишет docs/bom.md.

Ступени (stage): main — рекомендуемая корзина; alt — запасной путь (плата КаЧу вместо Авито, новые моторы вместо донора);
opt — опции; tool — инструмент, если его нет (в привод не входит). Группа с "budget": false (тележка) в бюджет привода не входит.

  /usr/bin/python3 tools/bom.py          пересчитать, записать docs/bom.md, напечатать итоги
  /usr/bin/python3 tools/bom.py --json   напечатать итоги в JSON (для страницы)
"""
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BOM = json.loads((ROOT / "docs" / "bom.json").read_text(encoding="utf-8"))
CAP = 40000


def money(x):
    return f"{x:,.0f}".replace(",", " ") + " ₽"


def totals(bom=BOM):
    groups = {g["id"]: g for g in bom["groups"]}
    seen = set()
    for it in bom["items"]:
        assert it["id"] not in seen, f"дубль id {it['id']}"
        seen.add(it["id"])
        assert it["group"] in groups, f"нет группы {it['group']}"
        assert it["stage"] in ("main", "alt", "opt", "tool"), it["id"]
        assert it["conf"] in ("карточка", "оценка"), it["id"]
        it["sum"] = it["qty"] * it["price"]
    in_budget = lambda it: groups[it["group"]].get("budget", True)
    main = [it for it in bom["items"] if it["stage"] == "main"]
    drive = sum(it["sum"] for it in main if in_budget(it))
    cart = sum(it["sum"] for it in main if not in_budget(it))
    est = sum(it["sum"] for it in main if in_budget(it) and it["conf"] == "оценка")
    by = {k: sum(it["sum"] for it in bom["items"] if it["id"] == k) for k in ("donor", "board", "board_kachu", "motors_new")}
    # запасной путь: родная плата донора не завелась — докупаем плату в КаЧу
    alt_board = drive + by["board_kachu"]
    # без донора (не нашёлся или ось коротка): новые мотор-колёса вместо б/у гироскутера
    alt_motors = drive - by["donor"] + by["motors_new"]
    groups_sum = {g["id"]: sum(it["sum"] for it in main if it["group"] == g["id"]) for g in bom["groups"]}
    return {"drive": drive, "cart": cart, "all": drive + cart, "estimated": est, "alt_board": alt_board,
            "alt_motors": alt_motors, "cap": CAP, "reserve": CAP - drive, "groups": groups_sum,
            "opt": sum(it["sum"] for it in bom["items"] if it["stage"] == "opt"),
            "tools": sum(it["sum"] for it in bom["items"] if it["stage"] == "tool")}


def write_md(t):
    groups = {g["id"]: g for g in BOM["groups"]}
    out = [f"# Смета электротележки\n", f"Цены на {BOM['date']}. {BOM['note']}\n",
           f"- **Привод (рекомендуемая корзина): {money(t['drive'])}** из потолка {money(CAP)}, запас {money(t['reserve'])}; "
           f"из них по оценке, без карточки, — {money(t['estimated'])}.",
           f"- Тележка: {money(t['cart'])}. Всё вместе: **{money(t['all'])}**.",
           f"- Запасные пути: родная плата не завелась, плата из КаЧу — {money(t['alt_board'])}; "
           f"без донора, новые мотор-колёса — {money(t['alt_motors'])}.",
           f"- Инструмент, если своего нет (в привод не входит): {money(t['tools'])}.",
           f"- Опции (не в корзине): {money(t['opt'])}.\n"]
    for stage, title in (("main", "Рекомендуемая корзина"), ("alt", "Запасной путь"), ("opt", "Опции"), ("tool", "Инструмент, если нет")):
        rows = [it for it in BOM["items"] if it["stage"] == stage]
        if not rows:
            continue
        out.append(f"## {title}\n")
        out.append("| Группа | Что | Зачем / как выбрать | Кол-во | Цена | Сумма | Где | Цена |")
        out.append("|---|---|---|---:|---:|---:|---|---|")
        for it in rows:
            where = f"[{it['shop']}]({it['url']})" if it["url"] else it["shop"]
            out.append(f"| {groups[it['group']]['name']} | {it['name']} | {it['spec']} | {it['qty']} | {money(it['price'])} | "
                       f"{money(it['sum'])} | {where} | {it['conf']} |")
        out.append("")
    (ROOT / "docs" / "bom.md").write_text("\n".join(out), encoding="utf-8")


if __name__ == "__main__":
    t = totals()
    if "--json" in sys.argv:
        print(json.dumps(t, ensure_ascii=False))
    else:
        write_md(t)
        for k, v in t.items():
            print(k, v)
