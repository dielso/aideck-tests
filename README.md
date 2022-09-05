# aideck-tests
 
 Code built and flashed with:
 ```
 docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P aideck-with-autotiler /bin/bash -c 'export GAPY_OPENOCD_CABLE=interface/ftdi/olimex-arm-usb-tiny-h.cfg; source /gap_sdk/configs/ai_deck.sh; cd /module/; make clean all image'
 cfloader flash BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/80/2M
 ```
