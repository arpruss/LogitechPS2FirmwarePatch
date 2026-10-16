asem/asemw pwm.as
objcopy -I ihex -O binary pwm.hex pwm.bin
asem/asemw standalone.as
objcopy -I ihex -O binary standalone.hex standalone.bin
python insert.py
