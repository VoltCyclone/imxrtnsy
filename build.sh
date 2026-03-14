#!/usr/bin/env bash
set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
RESET='\033[0m'

# ── Helpers ─────────────────────────────────────────────────────────────────
prompt_choice() {
    local prompt="$1"
    shift
    local options=("$@")
    local count=${#options[@]}

    printf "\n${CYAN}${prompt}${RESET}\n" >&2
    for i in "${!options[@]}"; do
        local num=$((i + 1))
        if [[ $i -eq 0 ]]; then
            printf "  ${BOLD}[%d] %s ${DIM}(default)${RESET}\n" "$num" "${options[$i]}" >&2
        else
            printf "  [%d] %s\n" "$num" "${options[$i]}" >&2
        fi
    done
    printf "${BOLD}> ${RESET}" >&2
    read -r choice
    choice=${choice:-1}

    if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > count )); then
        choice=1
    fi
    echo "$choice"
}

prompt_yn() {
    local prompt="$1"
    local default="${2:-y}"
    local hint="Y/n"
    [[ "$default" == "n" ]] && hint="y/N"

    printf "\n${CYAN}${prompt}${RESET} [${hint}] " >&2
    read -r answer
    answer=${answer:-$default}
    answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')
    [[ "$answer" == "y" ]]
}

# ── Banner ──────────────────────────────────────────────────────────────────
clear
printf "${BOLD}"
printf "  ╔══════════════════════════════════════╗\n"
printf "  ║         IMXRTNSY Build Config        ║\n"
printf "  ╚══════════════════════════════════════╝\n"
printf "${RESET}"
printf "  ${DIM}Teensy 4.1 (i.MX RT1062) firmware${RESET}\n"

# ── TFT Display ─────────────────────────────────────────────────────────────
tft_choice=$(prompt_choice "TFT Display:" \
    "ILI9341 (240x320)" \
    "ST7735  (128x160)" \
    "Disabled")

case "$tft_choice" in
    1) TFT=1; TFT_DRIVER=3 ;;
    2) TFT=1; TFT_DRIVER=1 ;;
    3) TFT=0; TFT_DRIVER=1 ;;
esac

# ── Touch ───────────────────────────────────────────────────────────────────
TOUCH=0
if [[ "$TFT_DRIVER" -eq 3 && "$TFT" -eq 1 ]]; then
    touch_choice=$(prompt_choice "Touch (FT6206):" \
        "Enabled" \
        "Disabled")
    [[ "$touch_choice" -eq 1 ]] && TOUCH=1
elif [[ "$TFT" -eq 1 ]]; then
    printf "\n${DIM}  Touch skipped (requires ILI9341)${RESET}\n"
fi

# ── Command Input ────────────────────────────────────────────────────────
# LPUART6 (pins 0/1) — command UART to host bridge.
NET=0
CMD_BAUD=4000000

cmd_choice=$(prompt_choice "Command input:" \
    "Serial (pins 0/1, LPUART6)" \
    "Ethernet (KMBox Net UDP)")

case "$cmd_choice" in
    1)
        cmd_baud_choice=$(prompt_choice "Command UART baud rate:" \
            "4000000" \
            "2000000" \
            "921600" \
            "Custom")
        case "$cmd_baud_choice" in
            1) CMD_BAUD=4000000 ;;
            2) CMD_BAUD=2000000 ;;
            3) CMD_BAUD=921600 ;;
            4)
                printf "${BOLD}  Baud rate> ${RESET}"
                read -r CMD_BAUD
                CMD_BAUD=${CMD_BAUD:-4000000}
                ;;
        esac
        ;;
    2)
        NET=1
        # NET mode requires TFT for IP/port/UUID display
        if [[ "$TFT" -eq 0 ]]; then
            printf "\n${YELLOW}  Note: Ethernet mode requires TFT — enabling ST7735${RESET}\n"
            TFT=1
            TFT_DRIVER=1
        fi
        ;;
esac

# ── Build Options ───────────────────────────────────────────────────────────
CLEAN=false
if prompt_yn "Clean before build?" "n"; then
    CLEAN=true
fi

FLASH=false
if prompt_yn "Flash after build?" "n"; then
    FLASH=true
fi

BUILD_BRIDGE=false
if prompt_yn "Build UART bridge (RP2350)?" "n"; then
    BUILD_BRIDGE=true
fi

PARALLEL=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# ── Summary ─────────────────────────────────────────────────────────────────
printf "\n${BOLD}  ┌─ Build Configuration ─────────────────┐${RESET}\n"

if [[ "$TFT" -eq 1 ]]; then
    [[ "$TFT_DRIVER" -eq 1 ]] && tft_label="ST7735 (128x160)" || tft_label="ILI9341 (240x320)"
    printf "  │  Display:    ${GREEN}%-25s${RESET} │\n" "$tft_label"
else
    printf "  │  Display:    ${DIM}%-25s${RESET} │\n" "Disabled"
fi

if [[ "$TOUCH" -eq 1 ]]; then
    printf "  │  Touch:      ${GREEN}%-25s${RESET} │\n" "FT6206"
else
    printf "  │  Touch:      ${DIM}%-25s${RESET} │\n" "Disabled"
fi

if [[ "$NET" -eq 1 ]]; then
    printf "  │  Cmd input:  ${GREEN}%-25s${RESET} │\n" "Ethernet (KMBox Net)"
else
    printf "  │  Cmd input:  ${GREEN}%-25s${RESET} │\n" "LPUART6 @ ${CMD_BAUD}"
fi

$CLEAN && cl="Yes" || cl="No"
$FLASH && fl="Yes" || fl="No"
$BUILD_BRIDGE && br="Yes" || br="No"
printf "  │  Clean:      %-25s │\n" "$cl"
printf "  │  Flash:      %-25s │\n" "$fl"
printf "  │  Bridge:     %-25s │\n" "$br"
printf "  │  Parallel:   %-25s │\n" "-j${PARALLEL}"
printf "  └──────────────────────────────────────┘\n"

if ! prompt_yn "Proceed?" "y"; then
    printf "${YELLOW}  Aborted.${RESET}\n"
    exit 0
fi

# ── Build ───────────────────────────────────────────────────────────────────
printf "\n"

MAKE_ARGS=(
    "TFT=${TFT}"
    "TFT_DRIVER=${TFT_DRIVER}"
    "TOUCH=${TOUCH}"
    "NET=${NET}"
    "CMD_BAUD=${CMD_BAUD}"
    "-j${PARALLEL}"
)

if $CLEAN; then
    printf "${YELLOW}Cleaning...${RESET}\n"
    make clean
    printf "${GREEN}Clean done.${RESET}\n\n"
fi

# Build firmware (without bridge target if not requested)
if $BUILD_BRIDGE; then
    printf "${CYAN}Building firmware + bridge...${RESET}\n"
    make "${MAKE_ARGS[@]}"
else
    printf "${CYAN}Building firmware...${RESET}\n"
    make "${MAKE_ARGS[@]}" firmware.hex
fi

printf "\n${GREEN}Build complete.${RESET}\n"

if $FLASH; then
    printf "\n${CYAN}Flashing Teensy 4.1...${RESET}\n"
    make flash
    printf "${GREEN}Flash complete.${RESET}\n"
fi

printf "\n${BOLD}Done.${RESET}\n"
