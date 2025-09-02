# ------------------
#  Configurables
# ------------------
APP_NAME         := "zephyr-app-base"
CMAKE_ARGS      := "-- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
ESP_BOARD       := "esp32s3_devkitc"
# Change this to match your ESP32-S3 serial port
ESP_PORT        := "/dev/tty.usbserial-210"

# ------------------
#  Tasks
# ------------------
init:
    sh ./scripts/setup.sh

# --- ESP32-S3 Tasks ---
build-esp32: clean
    (cd .. \
    && west build \
        -p always \
        -b "{{ESP_BOARD}}/esp32s3/procpu" \
        -d "{{APP_NAME}}/build" \
        {{APP_NAME}}/app \
        {{CMAKE_ARGS}} \
        -DEXTRA_DTC_OVERLAY_FILE={{justfile_directory()}}/app/boards/{{ESP_BOARD}}.overlay)

flash-esp32:
    west flash --build-dir build

monitor-esp32:
    python -m serial.tools.miniterm "{{ESP_PORT}}" 115200

run-esp32: build-esp32 flash-esp32 monitor-esp32

# --- sim tasks ---
build-sim: clean
    (cd .. \
    && west build \
    -p always \
    -b qemu_x86 \
    -d "{{APP_NAME}}/build" \
        {{APP_NAME}}/app \
        {{CMAKE_ARGS}} \
    )

flash-sim:
    QEMU_EXTRA_ARGS="-icount off" west build -t run
    
run-sim: build-sim flash-sim

# --- Default Tasks ---
run: run-esp32
build: build-esp32
flash: flash-esp32
monitor: monitor-esp32

clean: 
    rm -rf "build"