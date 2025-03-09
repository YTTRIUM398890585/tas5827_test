#include "AA_user_main.h"
#include "main.h"
#include "SEGGER_RTT.h "

// ADD YOUR INCLUDES HERE
TAS5827 powerAmp;

extern I2C_HandleTypeDef hi2c1;

void user_setup()
{
	// Initialize RTT
	SEGGER_RTT_Init();
	SEGGER_RTT_WriteString(0, "Hello, RTT!\r\n");

	// ADD SETUP CODE HERE
	// I2C Address of 0b110_0000 has to be shifted to the left to be 0b1100_000X
	powerAmp.begin(0xC0, &hi2c1);

	// Reset everything
	powerAmp.setModuleReset();
	powerAmp.setRegisterReset();
	powerAmp.setFaultClear();

	// Set device mode
	powerAmp.setDevCtrl1(TAS5827::Fsw_t::FSW_1024KHZ, true, TAS5827::Modulation_t::MOD_BD);
	powerAmp.setDevCtrl2(true, false, false, TAS5827::Power_State_t::HI_Z);

	// GPIO
	powerAmp.setGPIOMode(TAS5827::GPIO_Mode_t::OUTPUT, TAS5827::GPIO_Mode_t::OUTPUT, TAS5827::GPIO_Mode_t::OUTPUT);
	powerAmp.setGPIOSel(TAS5827::GPIO_Sel_t::FAULTZ, TAS5827::GPIO_Sel_t::WARNZ, TAS5827::GPIO_Sel_t::PVDD_DROP);
	// powerAmp.setGPIOSel(TAS5827::GPIO_Sel_t::PVDD_DROP, TAS5827::GPIO_Sel_t::PVDD_DROP, TAS5827::GPIO_Sel_t::PVDD_DROP);

	// Set the play volume
	float dB = -3;

	uint8_t vol = 0;

	vol = static_cast<float>((24.0 - dB) * 2.0);

	powerAmp.setDigVolLeft(vol);
	powerAmp.setDigVolRight(vol);
	powerAmp.setAnalogGain(0x00);

	// Set other stuff
	powerAmp.setPvddUvCtrl(true, TAS5827::UV_Avg_t::NO_AVG, true);
	// powerAmp.setMiscCtrl2(true, true, true);

	// Set top play
	powerAmp.setDevCtrl2(true, false, false, TAS5827::Power_State_t::PLAY);
}

void user_loop()
{
	bool bclkOverRate;
	bool pllOverRate;
	bool p_pllNotLocked;
	bool bclkMissing;
	bool blckNotValid;
	bool fsNotValid;

	if (powerAmp.getClockDetStatus(&bclkOverRate, &pllOverRate, &p_pllNotLocked, &bclkMissing, &blckNotValid, &fsNotValid)) {
		bclkOverRate ? SEGGER_RTT_WriteString(0, "BCLK Over Rate\r\n") : SEGGER_RTT_WriteString(0, "BCLK Under Rate\r\n");
		pllOverRate ? SEGGER_RTT_WriteString(0, "PLL Over Rate\r\n") : SEGGER_RTT_WriteString(0, "PLL Under Rate\r\n");
		p_pllNotLocked ? SEGGER_RTT_WriteString(0, "PLL Not Locked\r\n") : SEGGER_RTT_WriteString(0, "PLL Locked\r\n");
		bclkMissing ? SEGGER_RTT_WriteString(0, "BCLK Missing\r\n") : SEGGER_RTT_WriteString(0, "BCLK Not Missing\r\n");
		blckNotValid ? SEGGER_RTT_WriteString(0, "BCLK Not Valid\r\n") : SEGGER_RTT_WriteString(0, "BCLK Valid\r\n");
		fsNotValid ? SEGGER_RTT_WriteString(0, "FS Not Valid\r\n") : SEGGER_RTT_WriteString(0, "FS Valid\r\n");
	}
	else {
		SEGGER_RTT_WriteString(0, "Error reading clock detection status\r\n");
	}

	uint8_t reg;

	if (powerAmp.getChanFault(&reg)) {
		SEGGER_RTT_printf(0, "getChanFault: %x\r\n", reg);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getChanFault\r\n");
	}

	if (powerAmp.getGlobalFault1(&reg)) {
		SEGGER_RTT_printf(0, "getGlobalFault1: %x\r\n", reg);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getGlobalFault1\r\n");
	}

	if (powerAmp.getGlobalFault2(&reg)) {
		SEGGER_RTT_printf(0, "getGlobalFault2: %x\r\n", reg);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getGlobalFault2\r\n");
	}

	if (powerAmp.getWarning(&reg)) {
		SEGGER_RTT_printf(0, "getWarning: %x\r\n", reg);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getWarning\r\n");
	}

	uint16_t bclkRatio = 0;

	TAS5827::FS_Monitoring_t fsMon;

	if (powerAmp.getFsMon(&reg, &fsMon)) {
		fsMon == TAS5827::FS_Monitoring_t::FS_8_KHZ     ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_8KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_16_KHZ  ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_16KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_32_KHZ  ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_32KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_48_KHZ  ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_48KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_96_KHZ  ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_96KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_192_KHZ ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_192KHZ\r\n")
		: fsMon == TAS5827::FS_Monitoring_t::FS_ERROR   ? SEGGER_RTT_WriteString(0, "getFsMon = FS_MON_ERROR\r\n")
														: SEGGER_RTT_WriteString(0, "FS sum ting wong\r\n");

		bclkRatio |= (reg << 8);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getWarning\r\n");
	}

	if (powerAmp.getBclkMonLsb(&reg)) {
		bclkRatio |= reg;
		SEGGER_RTT_printf(0, "BCLK Ratio: %u\r\n", bclkRatio);
	}
	else {
		SEGGER_RTT_WriteString(0, "Error getBclkMonLsb\r\n");
	}

	float voltage = 1;

	if (powerAmp.getPVDD(&voltage)) {
		int voltageInt = (int)(voltage * 1000); // Convert to mV

		SEGGER_RTT_printf(0, "PVDD: %u mV\r\n", voltageInt);

		// if (voltage == 0) {
		// 	SEGGER_RTT_WriteString(0, "its legit 0\r\n");
		// }
		// else {
		// 	SEGGER_RTT_WriteString(0, "somthign worng with string \r\n");
		// }
	}
	else {
		SEGGER_RTT_WriteString(0, "Error reading PVDD voltage\r\n");
	}

	TAS5827::Power_State_t powState;

	if (powerAmp.getPowState(&powState)) {
		powState == TAS5827::Power_State_t::DEEP_SLEEP ? SEGGER_RTT_WriteString(0, "Power state: DEEP_SLEEP\r\n")
		: powState == TAS5827::Power_State_t::SLEEP    ? SEGGER_RTT_WriteString(0, "Power state: SLEEP\r\n")
		: powState == TAS5827::Power_State_t::HI_Z     ? SEGGER_RTT_WriteString(0, "Power state: HI_Z\r\n")
													   : SEGGER_RTT_WriteString(0, "Power state: PLAY\r\n");
	}
	else {
		SEGGER_RTT_WriteString(0, "Error reading power state\r\n");
	}

	// LED LOOP
	HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
	HAL_Delay(500);

	SEGGER_RTT_printf(0, "========================== tick: %u\r\n", HAL_GetTick());
	powerAmp.setFaultClear();
}

void user_error_handler()
{
	// [OPTIONAL]: ADD ERROR HANDLER CODE HERE
}

// ADD OTHER HELPER FUNCTIONS HERE