SHELL := /bin/zsh

build:
	pio run
	mv /Users/adammierzwa/Documents/PlatformIO/Projects/Lab_2/Lab_2_Task_1_central/.pio/build/nrf52840_dk/firmware.elf /Applications/Renode.app/Contents/MacOS/firmwares/Lab_2_Task_1_central.elf

	
run:
	mono /Applications/Renode.app/Contents/MacOS/bin/Renode.exe -e include @scripts/Lab_2_Part_1.resc

