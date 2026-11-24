#!/bin/bash
# Прошивка платы гироскутера с Mac через ST-Link V2 (три линии: GND, SWDIO, SWCLK; питание платы отдельное).
#
#   ./firmware/flash.sh check      — проверить связь с платой и защиту чтения (ничего не меняет)
#   ./firmware/flash.sh probe      — прочитать ножки чипа: совпадает ли управление транзисторами с прошивкой (ничего не меняет)
#   ./firmware/flash.sh hall       — 20 с крутить колёса рукой: откликаются ли датчики на нужных ножках (ничего не меняет)
#   ./firmware/flash.sh unlock     — снять защиту чтения. СТИРАЕТ ЗАВОДСКУЮ ПРОШИВКУ НАВСЕГДА (вернуть нельзя)
#   ./firmware/flash.sh unlock-hard— то же «тяжёлым» способом из вики EFeru (если обычный unlock не помог, чаще у GD32)
#   ./firmware/flash.sh diag       — самый первый пуск: синус + напряжение, 4 А (bin/cart-diag.bin) — проверка Холлов и фаз
#   ./firmware/flash.sh bench      — записать прошивку для стенда (bin/cart-bench.bin: 6 А, ≤2 км/ч)
#   ./firmware/flash.sh            — записать рабочую прошивку (bin/cart.bin)
#   ./firmware/flash.sh inv-l|inv-r|inv-lr — рабочая, если левое / правое / оба колеса крутятся не в ту сторону
#
# Перед каждой командой: аккумулятор подключён к плате, кнопку питания НАЖАТЬ И ДЕРЖАТЬ до конца команды
# (контроллер при прошивке отпускает «защёлку» питания, и без кнопки плата выключится посреди записи).
# Подробная инструкция: docs/assembly.md.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-flash}"

if ! command -v openocd >/dev/null 2>&1; then
  echo ">> Нет программы openocd. Ставлю через Homebrew (один раз)..."
  brew install open-ocd || { echo "!! Не удалось поставить open-ocd. Выполните: brew install open-ocd"; exit 1; }
fi

# CPUTAPID 0 = «принимать любой идентификатор ядра»: у STM32F103 он 0x1ba01477, у GD32F103 бывает 0x2ba01477,
# без этого openocd на GD32 падает с «UNEXPECTED idcode». Скорость 950 кГц — надёжно для длинных проводков.
OCD=(openocd -f interface/stlink.cfg -c "set CPUTAPID 0" -f target/stm32f1x.cfg -c "adapter speed 950")

hold_button() {
  echo
  echo "   Аккумулятор подключён? Кнопка питания НАЖАТА и удерживается?"
  read -r -p "   Держу кнопку — нажмите Enter (Ctrl+C — отмена) " _
}

check() {
  echo ">> Проверяю связь с платой..."
  local log
  log="$("${OCD[@]}" -c init -c "reset halt" -c "stm32f1x options_read 0" -c "flash probe 0" -c shutdown 2>&1)"
  echo "$log" | grep -E "Error|idcode|Cortex-M3|read protection|flash size|device id|STM32|GD32" | sed 's/^/   /'
  if echo "$log" | grep -q "open failed"; then
    echo "!! ST-Link не найден: проверьте, что он вставлен в USB (если через хаб — попробуйте прямо в Mac)."; return 1
  fi
  if echo "$log" | grep -qiE "init mode failed|unable to connect|Error connecting DP"; then
    echo "!! ST-Link есть, а плата не отвечает: проверьте 3 провода (GND, SWDIO, SWCLK), не перепутаны ли SWDIO/SWCLK,"
    echo "   держите ли кнопку питания и подключён ли аккумулятор."; return 1
  fi
  if echo "$log" | grep -q "read protection: on"; then
    echo ">> Плата ЗАЩИЩЕНА от чтения (так с завода). Нужно один раз: ./firmware/flash.sh unlock"; return 2
  fi
  if echo "$log" | grep -q "read protection: off"; then
    echo ">> Защиты нет — можно прошивать: ./firmware/flash.sh"; return 0
  fi
  echo "!! Ответ непонятный — пришлите этот вывод Claude:"; echo "$log" | tail -25; return 1
}

pin() { echo $(( ($1 >> $2) & 1 )); }   # значение, номер ножки → 0 или 1

# Плата, на которой EFeru никто не проверял (YK99): читаем уровни ножек при остановленном чипе. Сразу после сброса все
# ножки — входы без подтяжки, поэтому видно, куда их тянут микросхемы-драйверы транзисторов. Прошивка ждёт
# верхние ключи «включён = 1» (в покое вход тянут к 0) и нижние «включён = 0» (в покое вход тянут к 1): setup.c, OCNPolarity LOW.
probe() {
  local log a b c a2 b2 c2 ls=0 hs=0 n
  log="$("${OCD[@]}" -c init -c "reset halt" -c "mww 0x40021018 0x1D" -c "sleep 50" \
    -c "mdw 0x40010808" -c "mdw 0x40010C08" -c "mdw 0x40011008" -c "sleep 300" \
    -c "mdw 0x40010808" -c "mdw 0x40010C08" -c "mdw 0x40011008" -c shutdown 2>&1)"
  word() { echo "$log" | grep -i "^$1:" | sed -n "$2p" | awk '{print $2}'; }
  a="$(word 0x40010808 1)"; b="$(word 0x40010c08 1)"; c="$(word 0x40011008 1)"
  a2="$(word 0x40010808 2)"; b2="$(word 0x40010c08 2)"; c2="$(word 0x40011008 2)"
  if [ -z "$a" ] || [ -z "$b" ] || [ -z "$c" ] || [ -z "$c2" ]; then
    echo "!! Плата не ответила. Вывод для Claude:"; echo "$log" | tail -15; return 1
  fi
  a=$((16#$a)); b=$((16#$b)); c=$((16#$c)); a2=$((16#$a2)); b2=$((16#$b2)); c2=$((16#$c2))
  local L="PA7:$(pin $a 7) PB0:$(pin $b 0) PB1:$(pin $b 1) PB13:$(pin $b 13) PB14:$(pin $b 14) PB15:$(pin $b 15)"
  local H="PC6:$(pin $c 6) PC7:$(pin $c 7) PC8:$(pin $c 8) PA8:$(pin $a 8) PA9:$(pin $a 9) PA10:$(pin $a 10)"
  echo "   нижние ключи  (ждём везде 1): $L"
  echo "   верхние ключи (ждём везде 0): $H"
  echo "   кнопка питания при зажатой кнопке: PA1=$(pin $a 1) (вариант платы 0), PB9=$(pin $b 9) (вариант платы 1)"
  for n in "$(pin $a 7)" "$(pin $b 0)" "$(pin $b 1)" "$(pin $b 13)" "$(pin $b 14)" "$(pin $b 15)"; do ls=$((ls+n)); done
  for n in "$(pin $c 6)" "$(pin $c 7)" "$(pin $c 8)" "$(pin $a 8)" "$(pin $a 9)" "$(pin $a 10)"; do hs=$((hs+n)); done
  local mask_a=$(( (1<<7)|(1<<8)|(1<<9)|(1<<10) )) mask_b=$(( (1<<0)|(1<<1)|(1<<13)|(1<<14)|(1<<15) )) mask_c=$(( (1<<6)|(1<<7)|(1<<8) ))
  if [ $(( (a^a2)&mask_a )) -ne 0 ] || [ $(( (b^b2)&mask_b )) -ne 0 ] || [ $(( (c^c2)&mask_c )) -ne 0 ]; then
    echo ">> НЕЯСНО: уровни меняются сами, ножки ни к чему не подтянуты. Прошивку не включать — сначала разобрать с Claude."; return 3
  fi
  if [ "$ls" -eq 6 ] && [ "$hs" -eq 0 ]; then
    echo ">> СОВПАДАЕТ: управление транзисторами такое, как ждёт прошивка. Первое включение всё равно через лампы."; return 0
  fi
  echo ">> НЕ СОВПАДАЕТ ($ls из 6 нижних = 1, $hs из 6 верхних = 1). Прошивку на этой плате НЕ включать — сначала разобрать с Claude."; return 3
}

# Чип остановлен, транзисторы не управляются. Считаем переключения на ножках датчиков, пока колёса крутят рукой.
hall() {
  local log vals i=0 v
  local b5=0 b6=0 b7=0 c10=0 c11=0 c12=0 pb="" pc=""
  echo ">> 20 секунд: покрутите рукой левое колесо, потом правое (кнопка питания всё это время зажата)..."
  log="$("${OCD[@]}" -c init -c "reset halt" -c "mww 0x40021018 0x1D" -c "sleep 50" \
    -c "set s {}; for {set i 0} {\$i < 400} {incr i} { lappend s [stm32f1x.cpu read_memory 0x40010C08 32 1] [stm32f1x.cpu read_memory 0x40011008 32 1]; sleep 40 }; echo \"HALLS \$s\"" -c shutdown 2>&1)"
  vals="$(echo "$log" | sed -n 's/^HALLS //p')"
  if [ -z "$vals" ]; then echo "!! Плата не ответила. Вывод для Claude:"; echo "$log" | tail -15; return 1; fi
  for v in $vals; do
    if [ $((i%2)) -eq 0 ]; then
      if [ -n "$pb" ]; then
        [ "$(pin $v 5)" != "$(pin $pb 5)" ] && b5=$((b5+1))
        [ "$(pin $v 6)" != "$(pin $pb 6)" ] && b6=$((b6+1))
        [ "$(pin $v 7)" != "$(pin $pb 7)" ] && b7=$((b7+1))
      fi; pb=$v
    else
      if [ -n "$pc" ]; then
        [ "$(pin $v 10)" != "$(pin $pc 10)" ] && c10=$((c10+1))
        [ "$(pin $v 11)" != "$(pin $pc 11)" ] && c11=$((c11+1))
        [ "$(pin $v 12)" != "$(pin $pc 12)" ] && c12=$((c12+1))
      fi; pc=$v
    fi
    i=$((i+1))
  done
  echo "   левый мотор  (PB5 PB6 PB7),    переключений: $b5 $b6 $b7"
  echo "   правый мотор (PC10 PC11 PC12), переключений: $c10 $c11 $c12"
  if [ "$b5" -gt 0 ] && [ "$b6" -gt 0 ] && [ "$b7" -gt 0 ] && [ "$c10" -gt 0 ] && [ "$c11" -gt 0 ] && [ "$c12" -gt 0 ]; then
    echo ">> Все 6 датчиков откликаются на тех ножках, где их ждёт прошивка."; return 0
  fi
  echo ">> Не все датчики откликнулись: либо колесо не крутили, либо разводка другая — разобрать с Claude."; return 3
}

confirm_erase() {
  echo
  echo "   ВНИМАНИЕ: снятие защиты СТИРАЕТ заводскую прошивку гироскутера БЕЗВОЗВРАТНО."
  echo "   Скачать её заранее нельзя (она защищена), вернуть плату «как было» потом не получится."
  read -r -p "   Чтобы продолжить, напечатайте СТЕРЕТЬ и нажмите Enter: " ans
  [ "$ans" = "СТЕРЕТЬ" ] || { echo "   Отменено."; exit 1; }
}

case "$MODE" in
  check)
    hold_button
    check; exit $?
    ;;
  probe)
    hold_button
    probe; exit $?
    ;;
  hall)
    hold_button
    hall; exit $?
    ;;
  unlock)
    confirm_erase
    hold_button
    "${OCD[@]}" -c init -c "reset halt" -c "stm32f1x unlock 0" -c "reset halt" -c "stm32f1x options_read 0" -c shutdown 2>&1 \
      | grep -E "Error|unlocked|read protection|reset" | sed 's/^/   /'
    echo ">> Отпустите кнопку на 5 секунд (плата выключится), затем снова зажмите и запустите: ./firmware/flash.sh check"
    ;;
  unlock-hard)
    # Последовательность из вики EFeru (How-to-Unlock-MCU-Flash, Method 2, Step 5): вручную стирает байты опций
    confirm_erase
    hold_button
    "${OCD[@]}" -c init -c "reset halt" \
      -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" \
      -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" \
      -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" \
      -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" \
      -c "mww 0x40022010 0x2220" -c "sleep 100" \
      -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" \
      -c targets -c "halt" -c "stm32f1x unlock 0" -c shutdown 2>&1 | sed 's/^/   /' | tail -20
    echo ">> Отпустите кнопку на 5 секунд, затем: ./firmware/flash.sh check"
    ;;
  flash|diag|bench|inv-l|inv-r|inv-lr)
    BIN="$HERE/bin/cart.bin"; [ "$MODE" = "bench" ] && BIN="$HERE/bin/cart-bench.bin"
    [ "$MODE" = "diag" ] && BIN="$HERE/bin/cart-diag.bin"
    case "$MODE" in inv-l|inv-r|inv-lr) BIN="$HERE/bin/cart-$MODE.bin" ;; esac   # колесо встало зеркально
    [ -f "$BIN" ] || { echo "!! Нет файла $BIN — сначала ./firmware/build.sh"; exit 1; }
    if [ -f "$HERE/bin/SHA256SUMS.txt" ]; then
      ( cd "$HERE/bin" && grep " $(basename "$BIN")\$" SHA256SUMS.txt | shasum -a 256 -c --status ) \
        || { echo "!! Файл $BIN повреждён (контрольная сумма не сошлась) — пересоберите ./firmware/build.sh"; exit 1; }
    fi
    echo ">> Пишу $(basename "$BIN") ($(stat -f %z "$BIN") байт)"
    hold_button
    check; rc=$?
    [ $rc -eq 0 ] || exit $rc
    out="$("${OCD[@]}" -c "program {$BIN} 0x08000000 verify reset exit" 2>&1)"
    echo "$out" | grep -E "Error|wrote|Programming|Verif|\*\*" | sed 's/^/   /'
    if echo "$out" | grep -qiE "Verified OK|verified [0-9]+ bytes"; then
      echo ">> Готово. Плата перезапущена с новой прошивкой: должна прозвучать мелодия включения и 2 коротких гудка"
      echo "   (моторы разрешены). Теперь можно отпустить кнопку питания. Дальше — проверка по чек-листу в docs/research/firmware.md."
    else
      echo "!! Запись не подтверждена. Полный вывод:"; echo "$out" | tail -30; exit 1
    fi
    ;;
  *)
    echo "Команды: check | probe | hall | unlock | unlock-hard | diag | bench | inv-l | inv-r | inv-lr | (без слова) — рабочая прошивка"; exit 1
    ;;
esac
