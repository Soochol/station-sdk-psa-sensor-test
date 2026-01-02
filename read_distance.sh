#!/bin/bash
# VL53L0X 실시간 거리 측정 스크립트 (로그 스택)
# 사용법: ./read_distance.sh

CLI="/opt/st/stm32cubeclt_1.20.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
ADDR="0x20000198"  # dbg_vl53l0x_dist 주소

echo "=== VL53L0X 실시간 거리 측정 ==="
echo "Ctrl+C로 종료"
echo "================================"
echo ""

COUNT=0

while true; do
    # 메모리 읽기 (4바이트)
    OUTPUT=$($CLI -c port=SWD freq=4000 mode=NORMAL -r32 $ADDR 1 -q 2>/dev/null)

    # 값 추출 (0x로 시작하는 hex 값)
    VALUE=$(echo "$OUTPUT" | grep -oE '0x[0-9A-Fa-f]+' | head -1)

    COUNT=$((COUNT + 1))
    TIMESTAMP=$(date +"%H:%M:%S.%3N")

    if [ -n "$VALUE" ]; then
        # 16진수를 10진수로 변환
        DIST=$((VALUE))
        printf "[%04d] %s | 거리: %5d mm\n" $COUNT "$TIMESTAMP" $DIST
    else
        printf "[%04d] %s | 연결 실패\n" $COUNT "$TIMESTAMP"
    fi

    sleep 0.3
done
