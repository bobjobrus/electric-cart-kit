#!/bin/bash
# Сборка прошивки тележки (VARIANT_CART) из открытого проекта EFeru hoverboard-firmware-hack-FOC.
#
# Что делает:
#   1) скачивает исходники EFeru в firmware/.build/foc на закреплённый коммит (одна и та же версия всегда);
#   2) накладывает наш патч firmware/patch/0001-VARIANT_CART.patch;
#   3) ставит PlatformIO в отдельное окружение firmware/.build/venv (системный Python Apple, без Homebrew-Python);
#   4) собирает прошивки и кладёт их в firmware/bin/:
#        cart.bin        — рабочая;
#        cart-diag.bin   — самый первый пуск: синус + напряжение, ток 4 А — проверка Холлов и фаз;
#        cart-bench.bin  — для первого пуска на стенде (ток 6 А, не быстрее 2 км/ч);
#        cart-inv-l/r/lr.bin — рабочая, если левое / правое / оба колеса встали зеркально.
#      Одна и та же прошивка подходит и для STM32F103RCT6, и для GD32F103RCT6.
#
# Запуск:  ./firmware/build.sh            (первый раз ~1–2 мин и ~1,5 ГБ на диске в firmware/.build, дальше ~10 с)
# Настройки тележки (скорости, диаметр колеса, момент) — в патче, раздел VARIANT_CART в Inc/config.h.
# Чтобы поменять настройку: правим firmware/.build/foc/Inc/config.h, собираем `./firmware/build.sh --keep`,
# а когда всё устроит — сохраняем изменения обратно в патч: `./firmware/build.sh --save-patch`.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/.build"
SRC="$BUILD/foc"
VENV="$BUILD/venv"
PATCH="$HERE/patch/0001-VARIANT_CART.patch"
OUT="$HERE/bin"

UPSTREAM="https://github.com/EFeru/hoverboard-firmware-hack-FOC.git"
COMMIT="4f141cbc97b297e194fd58e56444bd0322f902ac"   # main от 04.07.2026 (Merge PR #600)
PIO_VERSION="6.2.0"

MODE="${1:-}"
mkdir -p "$BUILD" "$OUT"
export PLATFORMIO_CORE_DIR="$BUILD/pio-core"          # всё хозяйство PlatformIO — внутри .build, систему не трогаем

# --- 1. Исходники ---
if [ ! -d "$SRC/.git" ]; then
  echo ">> Скачиваю исходники EFeru..."
  git clone -q "$UPSTREAM" "$SRC"
fi

if [ "$MODE" = "--save-patch" ]; then
  # сохранить ручные правки в патч (база — закреплённый коммит)
  git -C "$SRC" diff "$COMMIT" -- Inc Src platformio.ini > "$PATCH.tmp"
  if [ ! -s "$PATCH.tmp" ]; then echo "!! Правок нет — патч не трогаю"; rm -f "$PATCH.tmp"; exit 1; fi
  mv "$PATCH.tmp" "$PATCH"
  echo ">> Патч обновлён: $PATCH"
  exit 0
fi

if [ "$MODE" != "--keep" ]; then
  echo ">> Возвращаю исходники к коммиту ${COMMIT:0:7} и накладываю патч..."
  git -C "$SRC" fetch -q origin 2>/dev/null || true
  git -C "$SRC" checkout -q -f "$COMMIT"
  git -C "$SRC" reset -q --hard "$COMMIT"
  git -C "$SRC" apply --whitespace=nowarn "$PATCH"
fi

# --- 2. PlatformIO ---
if [ ! -x "$VENV/bin/pio" ]; then
  echo ">> Ставлю PlatformIO $PIO_VERSION в $VENV ..."
  /usr/bin/python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q --upgrade pip
  # urllib3<2: системный Python Apple собран с LibreSSL, новый urllib3 с ним не работает
  "$VENV/bin/pip" install -q "urllib3<2" "platformio==$PIO_VERSION"
fi

# --- 3. Сборка ---
echo ">> Собираю VARIANT_CART, _DIAG, _BENCH и зеркальные ..."
"$VENV/bin/pio" run -d "$SRC" -e VARIANT_CART -e VARIANT_CART_DIAG -e VARIANT_CART_BENCH -e VARIANT_CART_INV_L -e VARIANT_CART_INV_R -e VARIANT_CART_INV_LR -e VARIANT_CART_DIAG_TEST

for env in VARIANT_CART VARIANT_CART_DIAG VARIANT_CART_BENCH VARIANT_CART_INV_L VARIANT_CART_INV_R VARIANT_CART_INV_LR VARIANT_CART_DIAG_TEST; do
  if [ ! -f "$SRC/.pio/build/$env/firmware.bin" ]; then
    echo "!! Сборка $env не получилась. Подробности: $VENV/bin/pio run -d $SRC -e $env"
    exit 1
  fi
done

cp "$SRC/.pio/build/VARIANT_CART/firmware.bin"       "$OUT/cart.bin"
cp "$SRC/.pio/build/VARIANT_CART_DIAG/firmware.bin"  "$OUT/cart-diag.bin"
cp "$SRC/.pio/build/VARIANT_CART_BENCH/firmware.bin" "$OUT/cart-bench.bin"
cp "$SRC/.pio/build/VARIANT_CART_INV_L/firmware.bin"  "$OUT/cart-inv-l.bin"     # левое колесо наоборот
cp "$SRC/.pio/build/VARIANT_CART_INV_R/firmware.bin"  "$OUT/cart-inv-r.bin"     # правое наоборот
cp "$SRC/.pio/build/VARIANT_CART_INV_LR/firmware.bin" "$OUT/cart-inv-lr.bin"    # оба наоборот
cp "$SRC/.pio/build/VARIANT_CART_DIAG_TEST/firmware.bin" "$OUT/cart-test30.bin" # только вывешенные колёса, запуск ключом SWD
( cd "$OUT" && shasum -a 256 cart.bin cart-diag.bin cart-bench.bin cart-inv-l.bin cart-inv-r.bin cart-inv-lr.bin cart-test30.bin > SHA256SUMS.txt )

echo
echo ">> Готово:"
ls -l "$OUT"/cart.bin "$OUT"/cart-diag.bin "$OUT"/cart-bench.bin
echo "   Перед записью сверьте аппаратную ревизию по docs/assembly.md."
