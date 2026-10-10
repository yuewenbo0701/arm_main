/*
 * 涵道风扇 (Ducted Fan) 调试程序
 * 适用板卡: Arduino Mega 2560
 * 功能: ESC 油门校准 + 串口控制风扇转速
 *
 * 硬件接线:
 *   ESC 信号线 (白/黄)  →  Arduino 9 号引脚 (PWM)
 *   ESC 地线   (黑/棕)  →  Arduino GND
 *   ESC 电源线 (红)     →  不接! (外接电池供电时断开红线, 避免与 Arduino 冲突)
 *   电池                 →  ESC 电源接口 (根据风扇规格选 3S~6S)
 *   风扇三相线           →  ESC 三相输出 (任意接, 方向反了交换任意两根)
 */

#include <Servo.h>

// ── 引脚定义 ──────────────────────────────────────────────
const int ESC_PIN       = 33;      // ESC 信号引脚

// ── 油门参数 ──────────────────────────────────────────────
const int THROTTLE_MIN  = 1000;   // 最小油门 (µs) — 停止
const int THROTTLE_MAX  = 2000;   // 最大油门 (µs) — 全速
const int THROTTLE_IDLE = 1100;   // 怠速油门 (刚好能转但不飞)

// ── 安全参数 ──────────────────────────────────────────────
const int  RAMP_DELAY   = 20;     // 油门渐变间隔 (ms), 越小变化越快
const int  RAMP_STEP    = 5;      // 每次渐变步长 (µs)

Servo esc;
int currentThrottle = THROTTLE_MIN;   // 当前实际油门
int targetThrottle  = THROTTLE_MIN;   // 目标油门

// ── 辅助函数: 安全输出油门 (带渐变) ────────────────────────
void setThrottle(int target) {
    target = constrain(target, THROTTLE_MIN, THROTTLE_MAX);

    if (target > currentThrottle) {
        // 加速: 逐步增加
        for (int t = currentThrottle; t <= target; t += RAMP_STEP) {
            esc.writeMicroseconds(t);
            delay(RAMP_DELAY);
        }
    } else if (target < currentThrottle) {
        // 减速: 逐步降低 (减速可稍快, 用一半延迟)
        for (int t = currentThrottle; t >= target; t -= RAMP_STEP) {
            esc.writeMicroseconds(t);
            delay(RAMP_DELAY / 2);
        }
    }

    esc.writeMicroseconds(target);
    currentThrottle = target;
}

// ── ESC 校准流程 ──────────────────────────────────────────
void calibrateESC() {
    Serial.println(F("\n========================================"));
    Serial.println(F("   ESC 油门校准流程"));
    Serial.println(F("========================================"));
    Serial.println(F("  1. 请断开电池!"));
    Serial.println(F("  2. 准备好后在串口输入 'ok' 并回车"));
    Serial.println(F("========================================"));

    // 等待用户断开电池
    while (true) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            cmd.toLowerCase();
            if (cmd == "ok") break;
        }
    }

    Serial.println(F("\n→ 正在发送最高油门信号 (2000µs)..."));
    esc.writeMicroseconds(THROTTLE_MAX);

    Serial.println(F("→ 现在请接上电池!"));
    Serial.println(F("→ 听到 ESC 发出 '滴滴' 两声后, 立即在串口输入 'ok'"));

    while (true) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            cmd.toLowerCase();
            if (cmd == "ok") break;
        }
    }

    Serial.println(F("→ 正在发送最低油门信号 (1000µs)..."));
    esc.writeMicroseconds(THROTTLE_MIN);
    delay(2000);

    Serial.println(F("→ ESC 应发出确认音 (通常是 123 三声 + 一声长鸣)"));
    Serial.println(F("→ 校准完成! 风扇已就绪。"));
    Serial.println(F("\n========================================"));
}

// ── 打印帮助信息 ──────────────────────────────────────────
void printHelp() {
    Serial.println(F("\n────────── 命令列表 ──────────"));
    Serial.println(F(" <数字>     直接设置油门 (1000~2000)"));
    Serial.println(F(" + / ++     油门 +20 / +100"));
    Serial.println(F(" - / --     油门 -20 / -100"));
    Serial.println(F(" idle       怠速 (刚好转起来)"));
    Serial.println(F(" stop       立即停止 (紧急!)"));
    Serial.println(F(" cal        重新进行 ESC 校准"));
    Serial.println(F(" status     查看当前状态"));
    Serial.println(F(" help       显示本帮助"));
    Serial.println(F("──────────────────────────────"));
}

// ── 打印状态 ──────────────────────────────────────────────
void printStatus() {
    Serial.print(F("当前油门: "));
    Serial.print(currentThrottle);
    Serial.print(F(" µs  |  百分比: "));
    int pct = map(currentThrottle, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
    Serial.print(pct);
    Serial.println(F(" %"));
}

// ── setup ─────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    // 短暂延时等待串口就绪
    delay(500);

    Serial.println(F("\n╔══════════════════════════════════════╗"));
    Serial.println(F("║   涵道风扇调试程序 v2.0             ║"));
    Serial.println(F("║   板卡: Arduino Mega 2560           ║"));
    Serial.println(F("╚══════════════════════════════════════╝"));

    // 初始化 ESC: 先发最低油门信号
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(THROTTLE_MIN);
    delay(200);

    // 询问是否需要校准
    Serial.println(F("\n是否需要 ESC 油门校准?"));
    Serial.println(F("  首次使用 / 更换电调          → 输入 'cal'"));
    Serial.println(F("  之前校准过, 直接使用          → 输入 'skip' 或等待 3 秒"));
    Serial.println(F("  查看帮助                      → 输入 'help'"));

    // 3 秒内等待用户选择
    unsigned long start = millis();
    bool choiceMade = false;

    while (millis() - start < 3000) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            cmd.toLowerCase();

            if (cmd == "cal") {
                calibrateESC();
                choiceMade = true;
                break;
            } else if (cmd == "skip") {
                choiceMade = true;
                break;
            } else if (cmd == "help") {
                printHelp();
                choiceMade = true;
                break;
            }
        }
    }

    if (!choiceMade) {
        Serial.println(F("→ 跳过校准, 使用上一次的油门范围。"));
    }

    // 确保电调已解锁 (部分 ESC 需先收到最低油门信号)
    Serial.println(F("→ 发送解锁信号 (最低油门)..."));
    esc.writeMicroseconds(THROTTLE_MIN);
    delay(3000);

    Serial.println(F("→ 就绪! 输入 'help' 查看命令列表。"));
    Serial.println(F("→ 输入油门值 (1000~2000) 开始测试。"));
}

// ── loop ──────────────────────────────────────────────────
void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        String cmd = input;
        cmd.toLowerCase();

        // ── 解析命令 ──────────────────────────────────────
        if (cmd.length() == 0) {
            // 空输入, 忽略
            return;
        }

        // "stop" — 紧急停止
        if (cmd == "stop") {
            Serial.println(F("!!! 紧急停止 !!!"));
            targetThrottle = THROTTLE_MIN;
            setThrottle(targetThrottle);
                    return;
        }

        // "idle" — 怠速
        if (cmd == "idle") {
            targetThrottle = THROTTLE_IDLE;
            Serial.print(F("→ 怠速: "));
            Serial.println(targetThrottle);
            setThrottle(targetThrottle);
                    return;
        }

        // "cal" — 重新校准
        if (cmd == "cal") {
            targetThrottle = THROTTLE_MIN;
            setThrottle(targetThrottle);
                    calibrateESC();
            return;
        }

        // "status" — 查看状态
        if (cmd == "status") {
            printStatus();
            return;
        }

        // "help" — 帮助
        if (cmd == "help") {
            printHelp();
            return;
        }

        // "+" / "++" — 增加油门
        if (cmd == "+") {
            targetThrottle = currentThrottle + 20;
            targetThrottle = constrain(targetThrottle, THROTTLE_MIN, THROTTLE_MAX);
            Serial.print(F("→ 油门 +20 → "));
            Serial.println(targetThrottle);
            setThrottle(targetThrottle);
            return;
        }
        if (cmd == "++") {
            targetThrottle = currentThrottle + 100;
            targetThrottle = constrain(targetThrottle, THROTTLE_MIN, THROTTLE_MAX);
            Serial.print(F("→ 油门 +100 → "));
            Serial.println(targetThrottle);
            setThrottle(targetThrottle);
            return;
        }

        // "-" / "--" — 减少油门
        if (cmd == "-") {
            targetThrottle = currentThrottle - 20;
            targetThrottle = constrain(targetThrottle, THROTTLE_MIN, THROTTLE_MAX);
            Serial.print(F("→ 油门 -20 → "));
            Serial.println(targetThrottle);
            setThrottle(targetThrottle);
            return;
        }
        if (cmd == "--") {
            targetThrottle = currentThrottle - 100;
            targetThrottle = constrain(targetThrottle, THROTTLE_MIN, THROTTLE_MAX);
            Serial.print(F("→ 油门 -100 → "));
            Serial.println(targetThrottle);
            setThrottle(targetThrottle);
            return;
        }

        // 尝试解析为数字
        int val = input.toInt();
        if (val > 0 || input == "0") {
            // 如果输入了 0~999 之间的数, 视为禁用值
            if (val > 0 && val < 100) {
                Serial.println(F("⚠ 请输入 1000~2000 之间的值, 或使用 + / - 微调"));
                return;
            }
            targetThrottle = val;
            targetThrottle = constrain(targetThrottle, THROTTLE_MIN, THROTTLE_MAX);
            Serial.print(F("→ 设置油门: "));
            Serial.print(targetThrottle);
            Serial.print(F(" µs ("));
            Serial.print(map(targetThrottle, THROTTLE_MIN, THROTTLE_MAX, 0, 100));
            Serial.println(F("%)"));
            setThrottle(targetThrottle);
            return;
        }

        // 未识别的命令
        Serial.print(F("⚠ 未知命令: "));
        Serial.println(input);
        Serial.println(F("  输入 'help' 查看命令列表"));
    }
}
