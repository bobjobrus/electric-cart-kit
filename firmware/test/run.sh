#!/bin/bash
# Прогон проверки логики тележки на Mac (без платы). Берёт cartControl() и настройки CART_* прямо из
# пропатченных исходников firmware/.build/foc (их готовит ./firmware/build.sh), собирает и запускает тест.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../.build/foc"
[ -f "$SRC/Src/util.c" ] || { echo "Сначала ./firmware/build.sh"; exit 1; }
TMP="$(mktemp -d)"
# 1) функция cartControl() целиком (от «#ifdef VARIANT_CART» с заголовком «Электротележка» до её #endif)
awk '/^#ifdef VARIANT_CART$/ && !f {getline nx; if (nx ~ /Электротележка/) {f=1; print; print nx; next} } f {print} f && /^#endif$/ {exit}' \
  "$SRC/Src/util.c" > "$TMP/cart_snippet.c"
# 2) настройки тележки из Inc/config.h (раздел VARIANT_CART до сборки для стенда): CART_*, N_MOT_MAX, окна защиты АЦП
awk '/VARIANT_CART SETTINGS/ {f=1} f && /#ifdef CART_BENCH/ {exit} f && /^[[:space:]]*#define (CART_|N_MOT_MAX|PRI_INPUT|ADC_PROTECT_THRESH)/ && !/#define CART_SIN_DRIVE$/ {sub(/^[[:space:]]+/, ""); print}' \
  "$SRC/Inc/config.h" > "$TMP/cart_config.h"
# Калибровка батареи — из той же конфигурации, что используется при сборке.
grep -E '^#define BAT_CALIB_(REAL_VOLTAGE|ADC)[[:space:]]' "$SRC/Inc/config.h" >> "$TMP/cart_config.h"
# 2б) настройки диагностической сборки cart-diag (CART_DIAG_*) — они внутри #ifdef CART_DIAG
grep -E '^[[:space:]]*#define CART_DIAG_' "$SRC/Inc/config.h" | sed -E 's/^[[:space:]]+//' >> "$TMP/cart_config.h"
echo "== cart.bin (SIN/VLT) =="
cc -w -o "$TMP/cart_test_sin" -DCART_SIN_DRIVE -DCART_SNIPPET="\"$TMP/cart_snippet.c\"" -DCART_CONFIG="\"$TMP/cart_config.h\"" "$HERE/cart_logic_test.c" -lm
"$TMP/cart_test_sin"
echo
echo "== прежний контроллер FOC/TRQ =="
cc -w -o "$TMP/cart_test" -DCART_SNIPPET="\"$TMP/cart_snippet.c\"" -DCART_CONFIG="\"$TMP/cart_config.h\"" "$HERE/cart_logic_test.c" -lm
"$TMP/cart_test"
# 2в) то же с CART_COAST 0 (плавная остановка и удержание — запасной режим из config.h)
echo
echo "== CART_COAST 0 (удержание) =="
sed -E 's/^#define CART_COAST( +)1/#define CART_COAST\10/' "$TMP/cart_config.h" > "$TMP/cart_config0.h"
grep -qE '^#define CART_COAST +0' "$TMP/cart_config0.h" || { echo "не нашёл CART_COAST 1 в настройках"; exit 1; }
cc -w -o "$TMP/cart_test0" -DCART_SNIPPET="\"$TMP/cart_snippet.c\"" -DCART_CONFIG="\"$TMP/cart_config0.h\"" "$HERE/cart_logic_test.c" -lm
"$TMP/cart_test0" | grep -E '^(FAIL|ALL OK|SOME FAILED)'
# 3) то же для cart-diag.bin (синус + напряжение с колёсика, без регулятора)
echo
echo "== cart-diag (CART_DIAG) =="
cc -w -o "$TMP/cart_test_diag" -DCART_DIAG -DCART_SNIPPET="\"$TMP/cart_snippet.c\"" -DCART_CONFIG="\"$TMP/cart_config.h\"" "$HERE/cart_logic_test.c" -lm
"$TMP/cart_test_diag"

echo "== CART_DIAG_TEST (30 секунд) =="
cc -w -o "$TMP/cart_test_timed" -DCART_DIAG -DCART_DIAG_TEST -DCART_SNIPPET="\"$TMP/cart_snippet.c\"" -DCART_CONFIG="\"$TMP/cart_config.h\"" "$HERE/cart_logic_test.c" -lm
"$TMP/cart_test_timed"
