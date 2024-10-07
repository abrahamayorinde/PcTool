This PcTool is based on the the PcTool here [https://github.com/Embetronicx/STM32-Bootloader/tree/ETX_Bootloader_2.0](https://github.com/Embetronicx/STM32-Bootloader/tree/ETX_Bootloader_2.0/Bootloader_Example/HostApp/PcTool)
which relies on this RS232 Library https://www.teuniz.net/RS-232/.

Run the below command to compile the application.

	gcc etx_ota_update_main.c RS232/rs232.c -IRS232 -Wall -Wextra -o2 -o etx_ota_app

Once you have build the application, then run the application like below.

		./etx_ota_app COMPORT_NAME APPLICATION_BIN_PATH
