################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Library/CTS_HAL.obj: C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Library/CTS_HAL.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/TI2/ccsv5/tools/compiler/msp430_4.1.5/bin/cl430" -vmsp --abi=eabi -g --include_path="C:/TI2/ccsv5/ccs_base/msp430/include" --include_path="C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Library" --include_path="C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Example_Projects/Source/RO_PINOSC_TA0_WDTp_One_Button_Compact" --include_path="C:/TI2/ccsv5/tools/compiler/msp430_4.1.5/include" --advice:power=all --gcc --define=__MSP430G2553__ --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=minimal --preproc_with_compile --preproc_dependency="Library/CTS_HAL.pp" --obj_directory="Library" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/CTS_Layer.obj: C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Library/CTS_Layer.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/TI2/ccsv5/tools/compiler/msp430_4.1.5/bin/cl430" -vmsp --abi=eabi -g --include_path="C:/TI2/ccsv5/ccs_base/msp430/include" --include_path="C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Library" --include_path="C:/Users/a0272323/Desktop/MSP430/cap_touch_slac489/Example_Projects/Source/RO_PINOSC_TA0_WDTp_One_Button_Compact" --include_path="C:/TI2/ccsv5/tools/compiler/msp430_4.1.5/include" --advice:power=all --gcc --define=__MSP430G2553__ --diag_warning=225 --display_error_number --diag_wrap=off --printf_support=minimal --preproc_with_compile --preproc_dependency="Library/CTS_Layer.pp" --obj_directory="Library" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


