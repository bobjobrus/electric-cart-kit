// Проверка логики cartControl() (VARIANT_CART) на Mac, без платы: грубая модель тележки (момент → сила → скорость)
// и расчёт схемы ручки (узловой метод). Это НЕ модель мотора: проверяются ручка, антидребезг, «взвод», переходы,
// накат/удержание, разгон, регулятор скорости, правила v2 и то, что никакая одиночная неисправность ручки не даёт «едем».
// С 05.11 по умолчанию CART_COAST 1 (отпустил кнопку — тяги нет, катится сама) и CART_REVERSE 0 (кнопки «назад» нет).
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define VARIANT_CART
#define DELAY_IN_MAIN_LOOP 5      // как в Inc/config.h
#define ADC_PROTECT_TIMEOUT 100   // как в Inc/config.h
#define I_MOT_MAX 15              // как в Inc/config.h
#include CART_CONFIG              // все #define CART_*, N_MOT_MAX, PRI_INPUT*, ADC_PROTECT_THRESH из раздела VARIANT_CART — делает run.sh

#define ABS(a) (((a) < 0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
// поля PRI_INPUTx: TYPE, MIN, MID, MAX, DEADBAND
#define PI_MIN_(t, mi, md, ma, db) (mi)
#define PI_MAX_(t, mi, md, ma, db) (ma)
#define PI_MIN(x) PI_MIN_(x)
#define PI_MAX(x) PI_MAX_(x)

#define BAT_LVL1 1300             // условные единицы АЦП батареи: ниже — «садится»
int16_t batVoltage = 1500;
volatile int16_t batVoltageFast = 1500;   // быстрый фильтр напряжения из Src/bldc.c (номинальная область питания)

typedef struct { int16_t raw, cmd; uint8_t typ, typDef; int16_t min, mid, max, dband; } InputStruct;
typedef struct { int16_t n_max, i_max, n_cruiseMotTgt; uint8_t b_cruiseCtrlEna; } P;
typedef struct { int16_t n_mot, iq; uint8_t z_errCode; } ExtY;

uint8_t inIdx = 0;
InputStruct input1[1], input2[1];
uint8_t timeoutFlgADC = 0;
static uint16_t timeoutCntADC = 0;
uint8_t enable = 0;
#ifdef CART_DIAG_TEST
uint8_t timeoutFlgGen = 0;
static uint32_t testClock;
uint32_t HAL_GetTick(void) { return testClock; }
#endif
int16_t speedAvg = 0;
P rtP_Left = { N_MOT_MAX*16, 12000, 0, 0 }, rtP_Right = { N_MOT_MAX*16, 12000, 0, 0 };
ExtY rtY_Left, rtY_Right;

#include CART_SNIPPET   // вырезка cartControl() из Src/util.c — делает run.sh

// ---- окно защиты АЦП, как handleTimeout() в Src/util.c ----
static void adcWindow(void) {
  if (IN_RANGE(input1[0].raw, PI_MIN(PRI_INPUT1) - ADC_PROTECT_THRESH, PI_MAX(PRI_INPUT1) + ADC_PROTECT_THRESH) &&
      IN_RANGE(input2[0].raw, PI_MIN(PRI_INPUT2) - ADC_PROTECT_THRESH, PI_MAX(PRI_INPUT2) + ADC_PROTECT_THRESH)) {
    timeoutFlgADC = 0; timeoutCntADC = 0;
  } else if (timeoutCntADC++ >= ADC_PROTECT_TIMEOUT) {
    timeoutFlgADC = 1; timeoutCntADC = ADC_PROTECT_TIMEOUT;
  }
}

// ---- crude plant ----
static double M = 60.0, SLOPE = 0.0;     // kg, percent (+ = uphill when driving forward)
static double vms = 0.0;                 // cart speed m/s (+ forward)
static int WALL = 0;                     // 1 = упёрлась: тележка не двигается вперёд
static const double N_PER_PERMILLE = 0.23;
#define RPM_PER_MS (60.0 / (3.14159265 * CART_WHEEL_DIAM_MM / 1000.0))
static int loopN = 0;

static void plant(int16_t cmdFwd) {
  double F = 0.0;
  if (rtP_Left.b_cruiseCtrlEna) {       // hold mode: assume EFeru speed loop tracks the cruise target perfectly
    vms = rtP_Left.n_cruiseMotTgt / RPM_PER_MS;
    return;
  }
  F = cmdFwd * N_PER_PERMILLE - M * 9.81 * SLOPE / 100.0;
  double roll = 0.04 * M * 9.81;
  if (fabs(vms) > 0.01) F -= (vms > 0 ? roll : -roll);
  else if (fabs(F) < roll) F = 0.0; else F -= (F > 0 ? roll : -roll);
  vms += F / M * 0.005;
  double nmax = rtP_Left.n_max / 16.0 / RPM_PER_MS;   // built-in per-wheel ceiling (idealised)
  if (vms > nmax) vms = nmax; if (vms < -nmax) vms = -nmax;
  if (WALL && vms > 0) vms = 0;
}

// колёсико: доля поворота 0..1000 → сырое значение АЦП (переменный резистор 10 кОм + 10 кОм у платы)
static int16_t knobRaw(int16_t k) { return (int16_t)lround(1000.0 * CART_KNOB_RAW_TOP / (2000 - k)); }
static int16_t KNOB_OVERRIDE = -1;      // ≥0 — подставить это сырое значение колёсика (обрыв и т.п.)

static void tick(int16_t knob, int16_t lever, int16_t holdTrq) {
  int16_t rpm = (int16_t)lround(vms * RPM_PER_MS);
  input1[0].raw = KNOB_OVERRIDE >= 0 ? KNOB_OVERRIDE : knobRaw(knob); input2[0].raw = lever;
  adcWindow();
  rtY_Left.n_mot = rpm; rtY_Right.n_mot = -rpm;          // motors mirrored
  rtY_Left.iq  =  (int16_t)((int32_t)holdTrq * 12000 / 1000);
  rtY_Right.iq = -(int16_t)((int32_t)holdTrq * 12000 / 1000);
  speedAvg = rpm;
  #ifdef CART_DIAG_TEST
  testClock += 5;
  #endif
  cartControl();
  if (!enable && ABS(input1[0].cmd) < 50 && ABS(input2[0].cmd) < 50) enable = 1;  // main.c enabling rule
  plant(input2[0].cmd);
  loopN++;
}
static void show(const char *tag) {
  printf("%-38s t=%6dms v=%5.2fkm/h cmd=%5d cruise=%d tgtL=%4d n_max=%5.1frpm adcFlt=%d beep=%d/%d\n", tag, loopN * 5,
         vms * 3.6, input2[0].cmd, rtP_Left.b_cruiseCtrlEna, rtP_Left.n_cruiseMotTgt, rtP_Left.n_max / 16.0, timeoutFlgADC,
         cartBeepCnt, cartBeepFreq);
}
static double peak; static void runFor(int ms, int16_t knob, int16_t lever) {
  peak = 0; for (int i = 0; i < ms / 5; i++) { tick(knob, lever, 0); if (fabs(vms) > peak) peak = fabs(vms); }
}

// ---- схема ручки: узловой метод ----
// Узлы: 1 PA3, 2 рычаг у пульта (COM), 3 минус у пульта, 4 +3,3 у пульта, 5 +3,3 у платы (после 220 Ом),
//       6 точка J (NO), 7 колёсико у пульта, 8 PA2.  Земля = 0 В, шина = 3,3 В.
#define NN 9
static double G[NN][NN], I[NN];
static void res(int a, int b, double r) {           // резистор между узлами (0 = земля, -1 = шина 3,3 В)
  double g = 1.0 / r;
  if (a > 0) G[a][a] += g; if (b > 0) G[b][b] += g;
  if (a > 0 && b > 0) { G[a][b] -= g; G[b][a] -= g; }
  if (a > 0 && b == -1) I[a] += g * 3.3; if (b > 0 && a == -1) I[b] += g * 3.3;
}
typedef struct { double wPlus, wLever, wMinus, wKnob, rNC, rNO, leak, pot; int pressed, back; } Ckt;
static void solve(Ckt c, double *pa3, double *pa2) {
  for (int i = 0; i < NN; i++) { I[i] = 0; for (int j = 0; j < NN; j++) G[i][j] = 0; }
  res(-1, 5, 220); res(5, 4, c.wPlus);
  res(1, 0, 4700); res(1, 2, c.wLever); res(0, 3, c.wMinus);
  res(2, 3, c.pressed ? 1e12 : c.rNC); res(2, 6, c.pressed ? c.rNO : 1e12);
  res(6, 4, 4700); if (c.back) res(6, 4, 2200 + 0.1);
  res(4, 7, c.pot + 0.1); res(7, 8, c.wKnob); res(8, 0, 10000);
  res(4, 2, c.leak);                                    // солёная плёнка между +3,3 и рычагом в пульте
  for (int i = 1; i < NN; i++) res(i, 0, 1e12);         // чтобы плавающий узел не делал систему вырожденной
  double A[NN][NN + 1];
  for (int i = 1; i < NN; i++) { for (int j = 1; j < NN; j++) A[i][j] = G[i][j]; A[i][NN] = I[i]; }
  for (int k = 1; k < NN; k++) {
    int p = k; for (int i = k + 1; i < NN; i++) if (fabs(A[i][k]) > fabs(A[p][k])) p = i;
    for (int j = 1; j <= NN; j++) { double t = A[k][j]; A[k][j] = A[p][j]; A[p][j] = t; }
    for (int i = 1; i < NN; i++) if (i != k) { double f = A[i][k] / A[k][k]; for (int j = k; j <= NN; j++) A[i][j] -= f * A[k][j]; }
  }
  *pa3 = A[1][NN] / A[1][1]; *pa2 = A[8][NN] / A[8][8];
}
static int16_t toRaw(double v) { return (int16_t)CLAMP(lround(v / 3.3 * 4095), 0, 4095); }
static const Ckt GOOD = { 0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 1e12, 5000, 0, 0 };

#define CHECK(cond, name) do { if (!(cond)) { printf("FAIL %s\n", name); ok = 0; } } while (0)
// «не тянет»: накат (CART_COAST 1) — момент 0 без режима скорости; иначе — удержание скоростью 0
#define NOT_DRIVING (input2[0].cmd == 0 && rtP_Left.b_cruiseCtrlEna == (CART_COAST ? 0 : 1))
// «тянет»: накат — момент не ноль; удержание — вышли из режима скорости
#define DRIVING (CART_COAST ? input2[0].cmd != 0 : rtP_Left.b_cruiseCtrlEna == 0)

#ifdef CART_SIN_DRIVE
// Задаём измеренные обороты напрямую. Модель момента выше не описывает SIN/VLT.
static int16_t sinTick(int16_t knob, int16_t lever, int16_t leftRpm, int16_t rightRpm) {
  input1[0].raw = KNOB_OVERRIDE >= 0 ? KNOB_OVERRIDE : knobRaw(knob);
  input2[0].raw = lever;
  rtY_Left.n_mot = leftRpm;
  rtY_Right.n_mot = -rightRpm;
  speedAvg = (leftRpm + rightRpm) / 2;
  adcWindow();
  cartControl();
  return input2[0].cmd;
}

static int16_t sinRun(int ms, int16_t knob, int16_t lever, int16_t rpm) {
  for (int i = 0; i < ms / DELAY_IN_MAIN_LOOP; i++) {
    sinTick(knob, lever, rpm, rpm);
  }
  return input2[0].cmd;
}

static int testSinDrive(void) {
  int ok = 1;
  const int16_t released = 0, pressed = (CART_LEVER_FWD_LO + CART_LEVER_FWD_HI) / 2;
  const int16_t minRpm = (CART_RPM256(CART_SPEED_MIN_KMH10) + 128) >> 8;
  const int16_t minCutRpm = minRpm + CART_SIN_SPEED_BAND_RPM;
  const int16_t maxRpm = CART_RPM256(CART_SPEED_MAX_KMH10) >> 8;
  const int16_t lowBatRpm = CART_RPM256(CART_SPEED_LOWBAT_KMH10) >> 8;
  const int16_t wheelLimit = MIN(minRpm * 5 / 4 + (CART_RPM256(5) >> 8), N_MOT_MAX);
  const int16_t rpmBelowCut = minRpm - 1;

  input1[0].min = PI_MIN(PRI_INPUT1);
  input1[0].max = PI_MAX(PRI_INPUT1);
  enable = 1;
  CHECK(sinRun(150, 0, pressed, 0) == 0, "S1 зажатая при включении кнопка не даёт тягу");
  sinRun(100, 0, released, 0);
  int16_t start = sinRun(100, 0, pressed, 0);
  CHECK(start > 0 && start <= CART_SIN_RAMP / 10, "S2 разгон ограничен по скорости нарастания команды");
  int16_t low = sinRun(500, 0, pressed, rpmBelowCut);
  CHECK(low > start, "S3 на минимуме есть плавная положительная тяга");
  CHECK(sinRun(800, 0, pressed, minRpm + 2) > 0, "S3a обычные колебания оборотов на минимуме не отключают тягу");
  CHECK(sinTick(0, pressed, minCutRpm, minCutRpm) == 0, "S4 при превышении задания тяга отключается");
  CHECK(sinTick(0, pressed, minCutRpm - 1, minCutRpm - 1) == 0, "S5 тяга не включается прямо у порога");
  int16_t resumed = sinTick(0, pressed, rpmBelowCut, rpmBelowCut);
  CHECK(resumed > 0 && resumed <= CART_SIN_RAMP * DELAY_IN_MAIN_LOOP / 1000, "S6 после наката тяга снова нарастает плавно");
  CHECK(sinTick(0, pressed, wheelLimit, 0) == 0, "S7 слишком быстрое внешнее колесо снимает тягу");
  CHECK(sinTick(0, pressed, rpmBelowCut, rpmBelowCut) > 0, "S8 обычный поворот разрешает тягу ниже порогов");
  CHECK(sinTick(0, released, rpmBelowCut, rpmBelowCut) == 0, "S9 отпускание сразу даёт накат");

  int16_t high = sinRun(1200, 1000, pressed, rpmBelowCut);
  CHECK(high > low && high <= CART_SIN_MAX_VLT, "S10 максимум сильнее минимума и ограничен");
  CHECK(sinTick(1000, pressed, maxRpm, maxRpm) == 0, "S11 на максимальном задании тяга отключается");
  batVoltage = BAT_LVL1 - 1;
  CHECK(sinRun(100, 1000, pressed, lowBatRpm) == 0, "S12 разряженная батарея снижает порог тяги");
  batVoltage = 1500;
  CHECK(sinRun(200, 0, pressed, rpmBelowCut) > 0, "S13 команда восстановилась после наката");

  KNOB_OVERRIDE = 0;
  CHECK(sinTick(0, pressed, rpmBelowCut, rpmBelowCut) == 0, "S14 обрыв колёсика снимает тягу сразу");
  KNOB_OVERRIDE = -1;
  CHECK(sinRun(100, 0, pressed, 0) == 0, "S15 восстановление контакта не вызывает пуск");
  sinRun(100, 0, released, 0);
  CHECK(sinRun(100, 0, pressed, 0) > 0, "S16 новое нажатие разрешено после восстановления");

  batVoltageFast = CART_OV_SOFT_ADC + 5;
  CHECK(sinTick(0, pressed, 0, 0) == 0 && cartOvActive, "S17 перенапряжение снимает тягу");
  batVoltageFast = 1500;
  CHECK(sinRun(CART_OV_HOLD_MS + 100, 0, pressed, 0) == 0 && !cartOvActive, "S18 после перенапряжения нужен новый пуск");
  sinRun(100, 0, released, 0);
  sinRun(100, 0, pressed, 0);
  rtY_Right.z_errCode = 4;
  CHECK(sinTick(0, pressed, 0, 0) == 0, "S19 ошибка любого мотора снимает тягу");
  rtY_Right.z_errCode = 0;
  CHECK(sinRun(100, 0, pressed, 0) == 0, "S20 исчезновение ошибки не вызывает пуск");
  sinRun(100, 0, released, 0);
  sinRun(100, 0, pressed, 0);
  cartBtnPressed = 1;
  CHECK(sinTick(0, pressed, 0, 0) == 0, "S21 кнопка питания снимает тягу");
  cartBtnPressed = 0;
  CHECK(sinRun(100, 0, pressed, 0) == 0, "S22 отпускание кнопки питания не вызывает пуск");

  sinRun(100, 0, released, 0);
  CHECK(sinRun(CART_STALL_MS + 50, 1000, pressed, 0) == 0 && cartBeepCnt == 1, "S23 упор обоих колёс отключает тягу и даёт сигнал");
  CHECK(sinRun(100, 1000, pressed, rpmBelowCut) == 0, "S24 после упора тяга вернётся только после отпускания");
  printf(ok ? "ALL OK\n" : "SOME FAILED\n");
  return ok ? 0 : 1;
}
#endif

int main(void) {
#ifdef CART_SIN_DRIVE
  return testSinDrive();
#endif
  int i, ok = 1;
  // ожидаемые скорости — из настроек CART_* (не числами), чтобы тест не ломался при смене пределов
  const double V_MID = (CART_SPEED_MIN_KMH10 + (CART_SPEED_MAX_KMH10 - CART_SPEED_MIN_KMH10) * 0.5) / 10.0;
  const double V_MAX = CART_SPEED_MAX_KMH10 / 10.0, V_MIN = CART_SPEED_MIN_KMH10 / 10.0, V_REV = CART_SPEED_REV_KMH10 / 10.0;
  const double V_DOWN = CART_SPEED_DOWN_KMH10 / 10.0, V_LOW = CART_SPEED_LOWBAT_KMH10 / 10.0;

#ifdef CART_DIAG
  // ---- cart-diag.bin: кнопка → напряжение с колёсика, без регулятора; защиты ручки те же ----
  {
    double a3, a2; Ckt d = GOOD; solve(d, &a3, &a2); int16_t dREL = toRaw(a3);
    d.pressed = 1; solve(d, &a3, &a2); int16_t dFWD = toRaw(a3);
    d = GOOD; d.pressed = 1; d.leak = 0.1; solve(d, &a3, &a2); int16_t dSHORT = toRaw(a3);
    #ifdef CART_DIAG_TEST
    enable = 1;
    runFor(2000, 0, dSHORT);
    CHECK(input2[0].cmd == 0 && !cartDiagTestLeftMs, "T1 без ключа не едет, даже с неверным входом");
    cartDiagTestRequest = 0x43543031;
    runFor(500, 0, dSHORT);
    CHECK(input2[0].cmd > 0 && input2[0].cmd < 120, "T2 плавный старт");
    runFor(1000, 0, dSHORT);
    CHECK(input2[0].cmd == 120, "T3 задано ровно 12% напряжения");
    runFor(28510, 0, dSHORT);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T4 автостоп через 30 с без связи с компьютером");
    runFor(1000, 0, dFWD);
    CHECK(input2[0].cmd == 0, "T5 физический вход сам тест не перезапускает");
    cartDiagTestRequest = 0x43543031; runFor(1500, 0, dSHORT);
    cartBtnPressed = 1; tick(0, dSHORT, 0);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T6 кнопка питания немедленно отменяет тест");
    cartBtnPressed = 0;
    cartDiagTestRequest = 0x43543031; runFor(1500, 0, dSHORT);
    batVoltageFast = CART_OV_SOFT_ADC + 5; tick(0, dSHORT, 0);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T7 перенапряжение отменяет тест");
    batVoltageFast = 1500; runFor(CART_OV_HOLD_MS + 100, 0, dSHORT);
    cartDiagTestRequest = 0x43543031; runFor(1500, 0, dSHORT);
    rtY_Left.z_errCode = 4; tick(0, dSHORT, 0);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T8 ошибка мотора отменяет тест");
    rtY_Left.z_errCode = 0;
    testClock = 0xfffff000u;
    cartDiagTestRequest = 0x43543031; runFor(30010, 0, dSHORT);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T9 автостоп при переполнении часов");
    cartDiagTestRequest = 0x43543031; runFor(1000, 0, dSHORT);
    cartDiagTestRequest = 1; tick(0, dSHORT, 0);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T10 удалённая отмена");
    cartDiagTestLevel = 1000;
    cartDiagTestRequest = 0x43543031; runFor(1500, 0, dSHORT);
    CHECK(input2[0].cmd == 200, "T11 тест ограничен максимумом 20%, даже при неверной команде");
    cartDiagTestLevel = 40; runFor(500, 0, dSHORT);
    CHECK(input2[0].cmd == 200, "T12 уровень не меняется во время уже запущенного теста");
    runFor(28010, 0, dSHORT);
    CHECK(!cartDiagTestLeftMs && input2[0].cmd == 0, "T13 автостоп на максимуме");
    printf(ok ? "ALL OK\n" : "SOME FAILED\n"); return ok ? 0 : 1;
    #endif
    WALL = 1;                                            // колесо в воздухе/упор — скорость тут не важна
    runFor(2000, 1000, dFWD); show("D1 включили с зажатой кнопкой");
    CHECK(input2[0].cmd == 0, "D1 зажата при включении — напряжения нет");
    runFor(300, 1000, dREL); runFor(100, 1000, dFWD); show("D2 отпустили, нажали: 0,1 с");
    CHECK(input2[0].cmd > 0 && input2[0].cmd <= 1000 * 100 / CART_DIAG_RAMP_MS + 5, "D2 напряжение растёт плавно");
    runFor(1500, 1000, dFWD); show("D3 колёсико максимум");
    CHECK(input2[0].cmd == CART_DIAG_VLT_MAX, "D3 колёсико максимум = CART_DIAG_VLT_MAX");
    runFor(1500, 0, dFWD); show("D4 колёсико минимум");
    CHECK(input2[0].cmd == CART_DIAG_VLT_MIN, "D4 колёсико минимум = CART_DIAG_VLT_MIN");
    runFor(1500, 1000, dFWD);
    runFor(CART_DIAG_VLT_MAX * CART_DIAG_RAMP_MS / 1000 + 50, 1000, dREL); show("D5 отпустили");
    CHECK(input2[0].cmd == 0, "D5 отпустили — напряжение плавно ушло в 0");
    runFor(300, 1000, dFWD); runFor(1500, 1000, dSHORT); show("D6 замыкание кнопки на +3,3");
    CHECK(input2[0].cmd == 0 && timeoutFlgADC, "D6 замыкание — сбой, напряжения нет");
    runFor(1000, 1000, dFWD); show("D7 после сбоя снова «нажата»");
    CHECK(input2[0].cmd == 0, "D7 после сбоя — только через отпускание");
    runFor(500, 1000, dREL); runFor(1500, 1000, dFWD);
    KNOB_OVERRIDE = 0; runFor(1500, 1000, dFWD); KNOB_OVERRIDE = -1; show("D8 обрыв колёсика");
    CHECK(input2[0].cmd == 0 && timeoutFlgADC, "D8 обрыв колёсика — сбой, напряжения нет");
    // D9. Перенапряжение (или плата завышает напряжение блока 42 В) — писк и на диагностической прошивке
    batVoltageFast = CART_OV_SOFT_ADC + 5; tick(1000, dREL, 0); show("D9 перенапряжение");
    CHECK(cartBeepFreq == 16 && cartBeepPat == 3, "D9 перенапряжение — свой писк");
    batVoltageFast = 1500; runFor(CART_OV_HOLD_MS + 400, 1000, dREL);
    CHECK(cartBeepFreq == 0, "D9b напряжение в норме — писк ушёл");
    printf(ok ? "ALL OK\n" : "SOME FAILED\n");
    return ok ? 0 : 1;
  }
#endif

  // ---- 0. Схема ручки: номиналы и одиночные неисправности ----
  double v3, v2; int16_t REL, FWD, RVS, SHORT;
  Ckt c = GOOD; solve(c, &v3, &v2); REL = toRaw(v3);
  printf("0 схема: отпущен %.2f В (%d)", v3, REL);
  c.pressed = 1; solve(c, &v3, &v2); FWD = toRaw(v3); printf(", сжат %.2f В (%d)", v3, FWD);
  c.back = 1; solve(c, &v3, &v2); RVS = toRaw(v3); printf(", сжат+назад %.2f В (%d)\n", v3, RVS);
  CHECK(cartLever(REL) == CART_REL && cartLever(FWD) == CART_FWD && cartLever(RVS) == (CART_REVERSE ? CART_REV : CART_FAULT),
        "0 номиналы в своих полосах (без кнопки «назад» её полоса — сбой)");
  c = GOOD; c.pot = 0;     solve(c, &v3, &v2); int16_t kTop = toRaw(v2);
  c.pot = 10000;           solve(c, &v3, &v2); int16_t kBot = toRaw(v2);
  printf("  колёсико: %.2f..%.2f В (%d..%d), CART_KNOB_RAW_TOP=%d\n", kBot * 3.3 / 4095, kTop * 3.3 / 4095, kBot, kTop, CART_KNOB_RAW_TOP);
  CHECK(abs(kTop - CART_KNOB_RAW_TOP) < 60, "0 верх колёсика = CART_KNOB_RAW_TOP");
  CHECK(kBot > PI_MIN(PRI_INPUT1) - ADC_PROTECT_THRESH && kTop < PI_MAX(PRI_INPUT1) + ADC_PROTECT_THRESH, "0 колёсико внутри окна защиты");
  c = GOOD; c.pressed = 1; c.leak = 0.1; solve(c, &v3, &v2); SHORT = toRaw(v3);   // рычаг замкнут на +3,3
  printf("  замыкание рычага на +3,3: %.2f В (%d)\n", v3, SHORT);
  CHECK(cartLever(SHORT) == CART_FAULT && SHORT > PI_MAX(PRI_INPUT2) + ADC_PROTECT_THRESH, "0 замыкание = сбой");
  {
    const double rs[] = { 10, 100, 1e3, 4.7e3, 1e4, 4.7e4, 1e5, 1e6, 1e12 };
    const double leaks[] = { 1e3, 2e3, 5e3, 1e4, 5e4, 1e5, 1e6 };
    int bad = 0, n = 0;
    for (int back = 0; back <= 1; back++) {        // «назад» залипла/вода в кнопке при отпущенном рычаге — тоже не «вперёд»
      for (int w = 0; w < 5; w++) for (int k = 0; k < 9; k++) {
        c = GOOD; c.back = back;
        double *f[] = { &c.wPlus, &c.wLever, &c.wMinus, &c.wKnob, &c.rNC };
        *f[w] = rs[k]; solve(c, &v3, &v2); uint8_t h = cartLever(toRaw(v3)); n++;
        if (h == CART_FWD || h == CART_REV) { bad++; printf("  !! отпущен, назад=%d, деталь %d = %.0f Ом → %.2f В\n", back, w, rs[k], v3); }
      }
      for (int k = 0; k < 7; k++) {
        c = GOOD; c.back = back; c.leak = leaks[k]; solve(c, &v3, &v2); uint8_t h = cartLever(toRaw(v3)); n++;
        if (h == CART_FWD || h == CART_REV) { bad++; printf("  !! отпущен, назад=%d, утечка +3,3→рычаг %.0f Ом → %.2f В\n", back, leaks[k], v3); }
      }
    }
    printf("  одиночные неисправности при отпущенном рычаге: %d случаев, «едем» — %d\n", n, bad);
    CHECK(bad == 0, "0 никакая одиночная неисправность не даёт «едем»");
    // Кнопка только с NO (без перекидного контакта): NC нет вовсе. Обрывы не должны давать «едем»; утечку — только показать,
    // в каком окне сопротивлений солёная плёнка «нажимает» кнопку (у перекидной кнопки такого окна нет — проверено выше)
    bad = 0; n = 0;
    for (int w = 0; w < 4; w++) for (int k = 0; k < 9; k++) {
      c = GOOD; c.rNC = 1e12;
      double *f[] = { &c.wPlus, &c.wLever, &c.wMinus, &c.wKnob };
      *f[w] = rs[k]; solve(c, &v3, &v2); uint8_t h = cartLever(toRaw(v3)); n++;
      if (h == CART_FWD || h == CART_REV) { bad++; printf("  !! кнопка NO, деталь %d = %.0f Ом → %.2f В\n", w, rs[k], v3); }
    }
    CHECK(bad == 0, "0 кнопка NO: обрыв и рост сопротивления не дают «едем»");
    double lo = 0, hi = 0;
    for (double r = 200; r < 2e5; r *= 1.02) {
      c = GOOD; c.rNC = 1e12; c.leak = r; solve(c, &v3, &v2);
      uint8_t h = cartLever(toRaw(v3));
      if (h == CART_FWD || h == CART_REV) { if (lo == 0) lo = r; hi = r; }
    }
    printf("  кнопка NO: утечка +3,3→сигнал «нажимает» её при %.1f…%.1f кОм (выше — сбой или «отпущена»)\n", lo / 1000, hi / 1000);
  }

  // 1. Power on, button released -> no drive (coast or hold)
  runFor(50, 500, REL); show("1 power-on, released");
  CHECK(NOT_DRIVING && enable, "1");

  // 2. Flat, 60 kg, knob 50% = V_MID
  runFor(4000, 500, FWD); show("2 flat, knob 50%, after 4 s");
  printf("   peak %.2f km/h\n", peak * 3.6);
  CHECK(fabs(vms * 3.6 - V_MID) < 0.15 && peak * 3.6 < V_MID + 0.4, "2 speed");

  // 3. Release -> coast to a stop on rolling resistance (CART_COAST 1) or stop ramp + hold (CART_COAST 0)
  runFor(30, 500, REL);
  int t0 = loopN; double dist = 0;
  while ((CART_COAST ? fabs(vms) > 0.011 : rtP_Left.n_cruiseMotTgt != 0) && loopN - t0 < 2000) {  // модель: ниже 0,01 м/с стоит
    tick(500, REL, 0); dist += fabs(vms) * 0.005;
    if (CART_COAST && input2[0].cmd != 0) break;
  }
  show("3 released -> stopped"); printf("   stop took %d ms, %.2f m\n", (loopN - t0) * 5, dist);
  CHECK(NOT_DRIVING && fabs(vms) < 0.012, "3");
  // накат: потолок колеса — только абсолютный N_MOT_MAX, иначе контроллер подтормаживал бы катящуюся тележку
  if (CART_COAST) CHECK(rtP_Left.n_max == N_MOT_MAX * 16 && rtP_Right.n_max == N_MOT_MAX * 16, "3b coast: no speed cap but N_MOT_MAX");

  // 4. Downhill -8 %, knob 50 %: speed must stay at V_MID, torque negative (braking)
  SLOPE = -8.0; for (i = 0; i < 100; i++) tick(500, REL, -150);   // on slope, released: coasts (or held)
  CHECK(NOT_DRIVING, "4 released on slope: no torque (coast) or hold");
  runFor(5000, 500, FWD); show("4 downhill -8% (brake)");
  printf("   peak %.2f km/h\n", peak * 3.6);
  CHECK(fabs(vms * 3.6 - V_MID) < 0.15 && input2[0].cmd < 0 && peak * 3.6 < V_MID + 0.4, "4 downhill");
  // 4b. Same slope, knob max: braking -> not faster than V_DOWN
  runFor(5000, 1000, FWD); show("4b downhill -8%, knob max");
  printf("   peak %.2f km/h\n", peak * 3.6);
  CHECK(fabs(vms * 3.6 - V_DOWN) < 0.15 && peak * 3.6 < V_DOWN + 0.4, "4b downhill cap");
  runFor(1000, 500, REL); SLOPE = 0;

  // 5. Uphill +10 %, 90 kg: bumpless start from holding torque, then climb
  M = 90; SLOPE = 10.0; vms = 0;
  for (i = 0; i < 200; i++) tick(500, REL, 400);                  // wheels held with +400 permille
  for (i = 0; i < 6; i++) tick(500, FWD, 400);
  show("5 uphill start");
  if (!CART_COAST) CHECK(input2[0].cmd >= 380, "5 bumpless");     // с накатом тележка успела откатиться — гасит регулятор
  else CHECK(input2[0].cmd > 0, "5 pulls forward while rolling back");
  runFor(5000, 500, FWD); show("5 uphill +10% 90 kg");
  CHECK(fabs(vms * 3.6 - V_MID) < 0.2, "5 climb");
  runFor(1500, 500, REL); SLOPE = 0; M = 60;
  runFor(1500, 500, FWD);                                          // pull on the flat: downhill cap released

  // 6. «Назад» on the go: stop and stay stopped while lever held; then release, lever, «назад» from standstill = reverse.
  //    Без кнопки «назад» (CART_REVERSE 0): такое значение на входе — сбой, тяги нет, назад не едет никогда
  runFor(3000, 500, FWD);
  runFor(50, 500, RVS); show("6 «назад» while moving fwd");
  CHECK(NOT_DRIVING, "6a stop");
  runFor(3000, 500, RVS); show("6 still held 3 s later");
  CHECK(NOT_DRIVING, "6b latched: no reverse");
  runFor(100, 500, REL); runFor(300, 500, FWD); runFor(4000, 500, RVS); show("6 release, lever, «назад»");
  if (CART_REVERSE) CHECK(fabs(vms * 3.6 + V_REV) < 0.15 && input2[0].cmd < 0, "6c reverse speed");
  else CHECK(NOT_DRIVING && vms > -0.01, "6c no reverse button: never reverses");
  timeoutFlgADC = 0; runFor(200, 500, REL);

  // 7. Lever wire broken mid-drive (0 V = «отпущен») -> stop; contact back with lever held -> drives (like re-squeeze)
  runFor(1000, 500, REL); runFor(2000, 500, FWD);
  runFor(25, 500, 0); show("7 lever wire broken (+25ms)");
  CHECK(NOT_DRIVING, "7a");
  runFor(500, 500, 0); runFor(100, 500, FWD); show("7 contact back, lever held");
  CHECK(DRIVING, "7b");
  // 7c. Lever shorted to +3,3 -> stop at once, ADC fault after 0.5 s; fixed with lever held -> no drive until release
  runFor(25, 500, SHORT); show("7c short to +3,3 (+25ms)");
  CHECK(NOT_DRIVING, "7c stop");
  runFor(600, 500, SHORT); CHECK(timeoutFlgADC == 1, "7c ADC fault");
  runFor(600, 500, FWD); show("7c short removed, lever held");
  CHECK(NOT_DRIVING, "7d");
  runFor(50, 500, REL); runFor(100, 500, FWD); show("7 released + squeezed again");
  CHECK(DRIVING, "7e");

  // 8. Lever stuck between bands >0.5 s -> ADC fault flag (2 beeps), hold
  runFor(1000, 500, REL);
  for (i = 0; i < 120; i++) { timeoutFlgADC = 0; tick(500, 1000, 0); }
  show("8 lever between bands 0.6 s");
  CHECK(timeoutFlgADC == 1 && NOT_DRIVING, "8");
  runFor(100, 500, REL);

  // 9. Motor error while lever held -> after re-enable must not drive until lever released
  runFor(100, 500, REL); runFor(1000, 500, FWD);
  enable = 0; runFor(200, 500, FWD); show("9 re-enabled, lever held");
  CHECK(NOT_DRIVING, "9");

  // 10. Knob max then min on the go; empty cart 25 kg
  M = 25; runFor(1000, 0, REL);
  runFor(5000, 1000, FWD); show("10 knob max, 25 kg"); printf("   peak %.2f km/h\n", peak * 3.6);
  CHECK(fabs(vms * 3.6 - V_MAX) < 0.2 && peak * 3.6 < V_MAX + 0.5, "10a");
  runFor(2000, 0, FWD); show("10 knob min after 2 s");
  CHECK(fabs(vms * 3.6 - V_MIN) < 0.15, "10b");
  M = 60;

  // 11. Knob wire broken (0 V) while driving -> ADC fault -> stop, 2 beeps
  runFor(100, 500, REL); runFor(2000, 500, FWD);
  KNOB_OVERRIDE = 0; runFor(700, 500, FWD); show("11 knob wire broken");
  CHECK(timeoutFlgADC == 1 && NOT_DRIVING, "11");
  KNOB_OVERRIDE = -1; runFor(100, 500, REL);

  // 12. «Назад» pressed before the lever (lever+back from released) -> no drive, 2 beeps; then lever alone -> drives
  runFor(500, 500, REL); runFor(500, 500, RVS); show("12 «назад» before lever");
  CHECK(NOT_DRIVING && (!CART_REVERSE || cartBeepCnt == 2), "12a");
  timeoutFlgADC = 0; runFor(100, 500, REL); runFor(1000, 500, FWD); show("12 lever alone");
  CHECK(DRIVING && vms > 0.2, "12b");

  // 13. Stall: pushed against a wall with lever held -> after CART_STALL_MS torque off, hold, 1 beep
  WALL = 1; runFor(CART_STALL_MS - 500, 500, FWD); show("13 wall, before stall time");
  CHECK(DRIVING, "13a still pulling");
  runFor(1500, 500, FWD); show("13 wall, after stall time");
  if (CART_COAST) {                      // накат: тяга уходит плавно за CART_GIVEUP_FADE_MS, а не обрывается
    CHECK(cartBeepCnt == 1 && input2[0].cmd > 0 && input2[0].cmd < CART_TRQ_FWD, "13b gave up: fading");
    runFor(CART_GIVEUP_FADE_MS, 500, FWD); show("13 after fade");
    CHECK(NOT_DRIVING, "13b gave up");
  } else CHECK(NOT_DRIVING && cartBeepCnt == 1, "13b gave up");
  WALL = 0; runFor(1000, 500, FWD); CHECK(NOT_DRIVING, "13c stays off while button held");
  runFor(100, 500, REL); runFor(1000, 500, FWD); show("13 release + squeeze");
  CHECK(DRIVING && vms > 0.2, "13d drives again");
  // 13e. Слишком крутой подъём (90 кг, 18 %): тележка встала на полной тяге и «сдалась». При накате тяга уходит
  //      плавно — за первые 0,5 с откат не больше 0,1 м (иначе груз сразу бросается на человека)
  if (CART_COAST) {
    M = 90; SLOPE = 18.0; runFor(100, 500, REL); vms = 0;
    runFor(CART_STALL_MS + 1000, 500, FWD);
    double x0 = 0; int giveUp = 0;
    for (i = 0; i < 2000 && !giveUp; i++) { tick(500, FWD, 0); giveUp = (cartBeepCnt == 1); }
    for (i = 0; i < 100; i++) { tick(500, FWD, 0); x0 += vms * 0.005; }
    show("13e steep hill, 0.5 s after giving up"); printf("   rolled back %.3f m\n", -x0);
    CHECK(giveUp && x0 > -0.1, "13e steep hill: gives up smoothly");
    runFor(3000, 500, REL); M = 60; SLOPE = 0; vms = 0; runFor(100, 500, REL);
  }

  // 14. Runaway: pushed above CART_RUNAWAY_KMH10 with lever held -> hold
  runFor(2000, 1000, FWD); vms = (CART_RUNAWAY_KMH10 / 10.0 + 0.5) / 3.6; tick(1000, FWD, 0); tick(1000, FWD, 0);
  show("14 runaway");
  if (CART_COAST) {                     // накат: тягу не снимает (иначе покатится ещё быстрее), пищит и тормозит к заданной
    CHECK(DRIVING && cartBeepFreq == 5, "14 runaway: still regulating, beeps");
    // тормозил дольше CART_DOWN_MS — включился потолок спуска V_DOWN; главное — не быстрее заданной и не встала
    runFor(3000, 1000, FWD); show("14b 3 s later");
    CHECK(vms * 3.6 <= V_MAX + 0.2 && vms * 3.6 > V_DOWN - 0.4, "14b braked back below the set speed");
  }
  else CHECK(rtP_Left.b_cruiseCtrlEna == 1, "14");
  runFor(2000, 500, REL);

  // 15. Low battery: knob max -> not faster than V_LOW
  batVoltage = BAT_LVL1 - 10; runFor(100, 1000, REL); runFor(5000, 1000, FWD); show("15 low battery, knob max");
  CHECK(fabs(vms * 3.6 - V_LOW) < 0.15, "15");
  batVoltage = 1500; runFor(2000, 500, REL);

  // 16. Power button pressed on the go (a parcel) -> stop and hold; released with lever held -> no drive until re-squeeze
  runFor(2000, 500, FWD); cartBtnPressed = 1; runFor(1500, 500, FWD); show("16 power button pressed");
  CHECK(NOT_DRIVING, "16a");
  cartBtnPressed = 0; runFor(500, 500, FWD); CHECK(NOT_DRIVING, "16b");
  runFor(100, 500, REL); runFor(500, 500, FWD); CHECK(DRIVING, "16c");

  // 17. Перенапряжение на спуске (батарея не принимает ток торможения): тормоз снимается сразу, потолок колёс снят, писк;
  //     между порогами — держится; ниже нижнего порога дольше CART_OV_HOLD_MS — снова тормозит
  CHECK(CART_OV_SOFT_OFF_ADC < CART_OV_SOFT_ADC && CART_OV_SOFT_ADC < CART_OV_HARD_OFF_ADC &&
        CART_OV_HARD_OFF_ADC < CART_OV_HARD_ADC && CART_OV_ADC(4320) < CART_OV_SOFT_OFF_ADC,
        "17 пороги по порядку; полная батарея с током торможения зимой (≈43,2 В) — ниже");
  runFor(1000, 500, REL); SLOPE = -8.0; runFor(5000, 500, FWD); show("17 downhill, braking");
  CHECK(input2[0].cmd < 0, "17a тормозит до перенапряжения");
  batVoltageFast = CART_OV_SOFT_ADC + 1; tick(500, FWD, 0); show("17 overvoltage: next cycle");
  CHECK(input2[0].cmd >= 0 && rtP_Left.n_max == CART_OV_NMAX * 16 && rtP_Right.n_max == CART_OV_NMAX * 16 &&
        cartBeepFreq == 16, "17b тормоз снят в тот же цикл, потолок снят, писк");
  batVoltageFast = (CART_OV_SOFT_ADC + CART_OV_SOFT_OFF_ADC) / 2; runFor(2000, 500, FWD); show("17 between thresholds 2 s");
  CHECK(input2[0].cmd >= 0 && rtP_Left.n_max == CART_OV_NMAX * 16, "17c между порогами — всё ещё без тормоза");
  batVoltageFast = 1500; runFor(CART_OV_HOLD_MS - 50, 500, FWD);
  CHECK(input2[0].cmd >= 0, "17d выдержка: тормоз ещё не вернулся");
  runFor(3000, 500, FWD); show("17 voltage normal again");
  // при CART_COAST 0 тележку без тормоза разогнало выше CART_RUNAWAY — правило «разогнало» включило удержание
  if (CART_COAST) CHECK(input2[0].cmd < 0 && rtP_Left.n_max <= N_MOT_MAX * 16, "17e тормоз и потолок вернулись");
  else CHECK(rtP_Left.b_cruiseCtrlEna == 1 && rtP_Right.b_cruiseCtrlEna == 1, "17e удержание вернулось");
  runFor(100, 500, REL); runFor(300, 500, FWD);
  // 17f. Перенапряжение, а тележку катит назад при нажатой кнопке: тянуть вперёд тоже нельзя (это торможение)
  SLOPE = 12.0; M = 90; runFor(300, 500, FWD); vms = -0.3; batVoltageFast = CART_OV_SOFT_ADC + 1; tick(500, FWD, 0);
  show("17f rolling back + overvoltage");
  CHECK(input2[0].cmd == 0 && rtP_Left.b_cruiseCtrlEna == 0 && rtP_Right.b_cruiseCtrlEna == 0, "17f катится назад — момент 0, без удержания");
  batVoltageFast = 1500; runFor(CART_OV_HOLD_MS + 200, 500, REL); SLOPE = 0; M = 60; vms = 0; runFor(500, 500, REL);
  // 17g. Перенапряжение во время плавного снятия тяги («сдалась» на крутом подъёме): тяга 0 в тот же цикл
  if (CART_COAST) {
    M = 90; SLOPE = 18.0; runFor(100, 500, REL); vms = 0;
    int giveUp = 0;
    for (i = 0; i < 2000 && !giveUp; i++) { tick(500, FWD, 0); giveUp = (cartBeepCnt == 1); }
    tick(500, FWD, 0); CHECK(giveUp && input2[0].cmd > 0, "17g «сдалась»: тяга уходит плавно");
    batVoltageFast = CART_OV_SOFT_ADC + 1; tick(500, FWD, 0); show("17g giving up + overvoltage");
    CHECK(input2[0].cmd == 0, "17g перенапряжение — плавное снятие отменено, момент 0");
    batVoltageFast = 1500; runFor(CART_OV_HOLD_MS + 400, 500, REL); M = 60; SLOPE = 0; vms = 0; runFor(500, 500, REL);
  }
  // 17h. Выдержка считается только по непрерывному «ниже нижнего порога»: один всплеск между порогами — отсчёт заново
  batVoltageFast = CART_OV_SOFT_ADC + 1; tick(500, REL, 0);
  batVoltageFast = CART_OV_SOFT_OFF_ADC - 5; runFor(CART_OV_HOLD_MS * 6 / 10, 500, REL);
  batVoltageFast = (CART_OV_SOFT_ADC + CART_OV_SOFT_OFF_ADC) / 2; tick(500, REL, 0);
  batVoltageFast = CART_OV_SOFT_OFF_ADC - 5; runFor(CART_OV_HOLD_MS * 6 / 10, 500, REL);
  CHECK(rtP_Left.n_max == CART_OV_NMAX * 16 || !CART_COAST, "17h всплеск между порогами — выдержка сначала");
  CHECK(cartBeepFreq == 16, "17h защита ещё действует (писк)");
  runFor(CART_OV_HOLD_MS * 6 / 10, 500, REL);
  CHECK(cartOvActive == 0, "17h после целой выдержки — снята");
  batVoltageFast = 1500; runFor(500, 500, REL);

  // 18. Перенапряжение при накате (CART_COAST 0 — вместо удержания): момент 0, без удержания, потолок снят; потом вернулся
  batVoltageFast = CART_OV_SOFT_ADC + 1; tick(500, REL, 0); show("18 coast + overvoltage");
  CHECK(input2[0].cmd == 0 && rtP_Left.b_cruiseCtrlEna == 0 && rtP_Right.b_cruiseCtrlEna == 0 &&
        rtP_Left.n_max == CART_OV_NMAX * 16 && cartOvActive == 1, "18 отпущена: момент 0, без удержания, потолок снят");
  batVoltageFast = 1500; runFor(CART_OV_HOLD_MS + 400, 500, REL);   // писк держится ещё 300 мс после снятия
  if (CART_COAST) CHECK(rtP_Left.n_max == N_MOT_MAX * 16, "18b потолок вернулся");
  CHECK(cartBeepFreq == 0, "18c писк ушёл");

  puts(ok ? "ALL OK" : "SOME FAILED");
  return ok ? 0 : 1;
}
