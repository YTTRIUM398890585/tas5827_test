#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

class TAS5827
{
public:
	/* Enums */
	// Switching frequency
	enum class Fsw_t : uint8_t
	{
		FSW_384KHZ  = 0b000,
		FSW_480KHZ  = 0b010,
		FSW_576KHZ  = 0b011,
		FSW_768KHZ  = 0b100,
		FSW_1024KHZ = 0b101,
	};

	// Modulation type
	enum class Modulation_t : uint8_t
	{
		MOD_BD     = 0b00,
		MOD_1SPW   = 0b01,
		MOD_HYBRID = 0b10,
	};

	// Undervoltage sampling average
	enum class UV_Avg_t : uint8_t
	{
		NO_AVG     = 0b00,
		SAMPLES_16 = 0b01,
		SAMPLES_32 = 0b10,
		SAMPLES_64 = 0b11,
	};

	// Time of no input to auto mute a channel
	enum class Auto_Mute_Time_t : uint8_t
	{
		MUTE_TIME_11_5_MS  = 0b000,
		MUTE_TIME_53_MS    = 0b001,
		MUTE_TIME_106_5_MS = 0b010,
		MUTE_TIME_266_5_MS = 0b011,
		MUTE_TIME_0_535_S  = 0b100,
		MUTE_TIME_1_065_S  = 0b101,
		MUTE_TIME_2_665_S  = 0b110,
		MUTE_TIME_5_33_S   = 0b111,
	};

	// Data bit clock frequency
	// multiples of the sample frequency
	enum class BCLK_t : uint8_t
	{
		BCLK_AUTO  = 0b0000,
		BCLK_32FS  = 0b0011,
		BCLK_64FS  = 0b0101,
		BCLK_128FS = 0b0111,
		BCLK_256FS = 0b1001,
		BCLK_512FS = 0b1011,
	};

	// Sample frequency
	enum class FS_t : uint8_t
	{
		FS_AUTO      = 0b0000,
		FS_8_KHZ     = 0b0010,
		FS_16_KHZ    = 0b0100,
		FS_32_KHZ    = 0b0110,
		FS_44_1_KHZ  = 0b1000,
		FS_48_KHZ    = 0b1001,
		FS_88_2_KHZ  = 0b1010,
		FS_96_KHZ    = 0b1011,
		FS_176_4_KHZ = 0b1100,
		FS_192_KHZ   = 0b1101,
	};

	// Data format
	enum class Format_t : uint8_t
	{
		I2S     = 0b00,
		DSP_TDM = 0b01,
		RTJ     = 0b10,
		LTJ     = 0b11,
	};

	// Data bit depth
	enum class Bit_Depth_t : uint8_t
	{
		I2S_16_BIT = 0b00,
		I2S_20_BIT = 0b01,
		I2S_24_BIT = 0b10,
		I2S_32_BIT = 0b11,
	};

	// Sample frequency monitoring (for some reason its a subset of FS_t)
	enum class FS_Monitoring_t : uint8_t
	{
		FS_ERROR   = 0b0000,
		FS_8_KHZ   = 0b0010,
		FS_16_KHZ  = 0b0100,
		FS_32_KHZ  = 0b0110,
		FS_48_KHZ  = 0b1001,
		FS_96_KHZ  = 0b1011,
		FS_192_KHZ = 0b1101,
	};

	// Processing rate
	// NOTE: im guessing PROC_RATE is processing rate, but the datasheet does not specify what it is
	enum class Proc_Rate_t : uint8_t
	{
		PROC_RATE_INPUT  = 0b00,
		PROC_RATE_48KHZ  = 0b01,
		PROC_RATE_96KHZ  = 0b10,
		PROC_RATE_192KHZ = 0b11,
	};

	// Volume update frequency, in sample periods per update
	enum class Vol_Freq_t : uint8_t
	{
		VOL_1_FS    = 0b00,
		VOL_2_FS    = 0b01,
		VOL_4_FS    = 0b10,
		VOL_INSTANT = 0b11,
	};

	// Volume step size, in dB per update
	enum class Vol_Step_t : uint8_t
	{
		VOL_4_DB   = 0b00,
		VOL_2_DB   = 0b01,
		VOL_1_DB   = 0b10,
		VOL_0_5_DB = 0b11,
	};

	// closed loop bandwidth
	enum class Loop_BW_t : uint8_t
	{
		LOOP_BW_80KHZ  = 0b00100000,
		LOOP_BW_100kHZ = 0b00000000,
		LOOP_BW_120kHZ = 0b01000000,
		LOOP_BW_175kHZ = 0b01100000,
	};

	// GPIO input/output mode
	enum class GPIO_Mode_t : uint8_t
	{
		INPUT  = 0b0,
		OUTPUT = 0b1,
	};

	// GPIO signal selection
	enum class GPIO_Sel_t : uint8_t
	{
		OFF       = 0b0000,
		WARNZ     = 0b1000,
		FAULTZ    = 0b1011,
		PVDD_DROP = 0b1100,
		SDOUT     = 0b1101,
		RAMP_CLK  = 0b1110,
	};

	// Power state
	enum class Power_State_t : uint8_t
	{
		DEEP_SLEEP = 0b00,
		SLEEP      = 0b01,
		HI_Z       = 0b10,
		PLAY       = 0b11,
	};

	// Cycle-by-cycle current limit level
	// percentage of the OCP threshold
	enum class CBC_Sel_t : uint8_t
	{
		CBC_80_PERCENT = 0b00,
		CBC_60_PERCENT = 0b10,
		CBC_40_PERCENT = 0b01,
	};

	// Spread spectrum random range
	enum class SS_Rand_Range_t : uint8_t
	{
		// For Fsw of 384kHz
		// 3'b000: SS range +/- 0.62%
		// 3'b010: SS range +/- 1.88%
		// 3'b011: SS range +/- 4.38%
		// 3'b100: SS range +/- 9.38%
		// 3'b101: SS range +/- 19.38%
		// For Fsw of 576kHz
		// 3'b000: SS range +/- 0.95%
		// 3'b010: SS range +/- 2.86%
		// 3'b011: SS range +/- 6.67%
		// 3'b100: SS range +/- 14.29%
		// 3'b101: SS range +/- 29.52%
		// NOTE: not sure what the setting actuall means outside of the given example in the datasheet
		SS_Rand_1 = 0b000,
		SS_Rand_2 = 0b010,
		SS_Rand_3 = 0b011,
		SS_Rand_4 = 0b100,
		SS_Rand_5 = 0b101,
	};

	// Spread spectrum triangular range
	enum class SS_Tri_Range_t : uint8_t
	{
		SS_Tri_24kHz_5  = 0b0000,
		SS_Tri_24kHz_10 = 0b0001,
		SS_Tri_24kHz_20 = 0b0010,
		SS_Tri_24kHz_25 = 0b0011,
		SS_Tri_48kHz_5  = 0b0100,
		SS_Tri_48kHz_10 = 0b0101,
		SS_Tri_48kHz_20 = 0b0110,
		SS_Tri_48kHz_25 = 0b0111,
		SS_Tri_32kHz_5  = 0b1000,
		SS_Tri_32kHz_10 = 0b1001,
		SS_Tri_32kHz_20 = 0b1010,
		SS_Tri_32kHz_25 = 0b1011,
		SS_Tri_16kHz_5  = 0b1100,
		SS_Tri_16kHz_10 = 0b1101,
		SS_Tri_16kHz_20 = 0b1110,
		SS_Tri_16kHz_25 = 0b1111,
	};

	/* Fault Masks */
	// CHAN_FAULT
	const uint8_t MASK_CHAN_FAULT_CH1DC = 1 << 3;
	const uint8_t MASK_CHAN_FAULT_CH2DC = 1 << 2;
	const uint8_t MASK_CHAN_FAULT_CH1OC = 1 << 1;
	const uint8_t MASK_CHAN_FAULT_CH2OC = 1 << 0;

	// GLOBAL_FAULT1
	const uint8_t MASK_GLOBAL_FAULT1_BQ     = 1 << 6;
	const uint8_t MASK_GLOBAL_FAULT1_EEPROM = 1 << 5;
	const uint8_t MASK_GLOBAL_FAULT1_CLK    = 1 << 2;
	const uint8_t MASK_GLOBAL_FAULT1_PVDDOV = 1 << 1;
	const uint8_t MASK_GLOBAL_FAULT1_PVDDUV = 1 << 0;

	// GLOBAL_FAULT2
	const uint8_t MASK_GLOBAL_FAULT2_CH2CBC = 1 << 2;
	const uint8_t MASK_GLOBAL_FAULT2_CH1CBC = 1 << 1;
	const uint8_t MASK_GLOBAL_FAULT2_OTSD   = 1 << 0;

	// WARNING
	const uint8_t MASK_WARNING_CH1CBCW = 1 << 5;
	const uint8_t MASK_WARNING_CH2CBCW = 1 << 4;
	const uint8_t MASK_WARNING_OTW4    = 1 << 3;
	const uint8_t MASK_WARNING_OTW3    = 1 << 2;
	const uint8_t MASK_WARNING_OTW2    = 1 << 1;
	const uint8_t MASK_WARNING_OTW1    = 1 << 0;

	// PIN_CONTROL1
	const uint8_t MASK_PIN_CTRL1_OTSD     = 1 << 7;
	const uint8_t MASK_PIN_CTRL1_DVDDUV   = 1 << 6;
	const uint8_t MASK_PIN_CTRL1_DVDDOV   = 1 << 5;
	const uint8_t MASK_PIN_CTRL1_CLKERROR = 1 << 4;
	const uint8_t MASK_PIN_CTRL1_PVDDUV   = 1 << 3;
	const uint8_t MASK_PIN_CTRL1_PVDDOV   = 1 << 2;
	const uint8_t MASK_PIN_CTRL1_DC       = 1 << 1;
	const uint8_t MASK_PIN_CTRL1_OC       = 1 << 0;

	// PIN_CONTROL2
	const uint8_t EN_PIN_CTRL2_CBCFAULTLATCH = 1 << 7;
	const uint8_t EN_PIN_CTRL2_CBCWARNLATCH  = 1 << 6;
	const uint8_t EN_PIN_CTRL2_CLKFAULTLATCH = 1 << 5;
	const uint8_t EN_PIN_CTRL2_OTSDLATCH     = 1 << 4;
	const uint8_t EN_PIN_CTRL2_OTWLATCH      = 1 << 3;
	const uint8_t MASK_PIN_CTRL2_OTW         = 1 << 2;
	const uint8_t MASK_PIN_CTRL2_CBCWARN     = 1 << 1;
	const uint8_t MASK_PIN_CTRL2_CBCFAULT    = 1 << 0;

	// MISC_CONTROL3
	const uint8_t EN_CLKDET_LATCH = 1 << 7;
	const uint8_t EN_OTSD_AUTOREC = 1 << 4;

	TAS5827() = default;

	bool begin(uint8_t address, I2C_HandleTypeDef* i2cHandle);

	/* Setters */
	bool setModuleReset();
	bool setRegisterReset();
	bool setDevCtrl1(Fsw_t fsw, bool pbtl, Modulation_t mod);
	bool setDevCtrl2(bool dspEn, bool ch1Mute, bool ch2Mute, Power_State_t powState);
	bool setPvddUvCtrl(bool uvHiZEn, UV_Avg_t uvAvg, bool pvddDropDetectEn);
	bool setAutoIncPage(bool autoInc);
	bool setSigChCtrl(BCLK_t bclk, FS_t fs);
	bool setClockDetCtrl(bool detPll, bool detBclkRange, bool detFs, bool detBclkRatio, bool detBclkMissing);
	bool setI2sCtrl(bool blckInv);
	bool setSapCtrl1(uint8_t i2sShiftMsb, Format_t i2sFormat, bool lrclkPulseWidth, Bit_Depth_t i2sBitDepth);
	bool setSapCtrl2(uint8_t i2sShiftLsb);
	bool setDspPgmMode(bool ch1HiZ, bool ch2HiZ, bool romMode);
	bool setDspCtrl(Proc_Rate_t procRate, bool IramBoot, bool defCoeff);
	bool setDigVolLeft(uint8_t vol);
	bool setDigVolRight(uint8_t vol);
	// NOTE: mistake in the datasheet, the datasheet flipped up and down so no idea what is correct
	bool setDigVolCtrl2(Vol_Freq_t vnus, Vol_Step_t vnuf, Vol_Freq_t vnds, Vol_Step_t vndf);
	bool setDigVolCtrl3(Vol_Freq_t veds, Vol_Step_t vedf);
	bool setAutoMuteCtrl(bool bothMute, bool ch1Mute, bool ch2Mute);
	bool setAutoMuteTime(Auto_Mute_Time_t ch1Time, Auto_Mute_Time_t ch2Time);
	bool setLoopBW(Loop_BW_t loopBW);
	bool setAnalogGain(uint8_t gain);
	bool setGPIOMode(GPIO_Mode_t gpioMode0, GPIO_Mode_t gpioMode1, GPIO_Mode_t gpioMode2);
	bool setGPIOSel(GPIO_Sel_t gpioSel0, GPIO_Sel_t gpioSel1, GPIO_Sel_t gpioSel2);
	bool setMiscCtrl2(bool gpioInv0, bool gpioInv1, bool gpioInv2);
	bool setDisableSpreadSpectrum(void);
	bool setRandomSpreadSpectrum(SS_Rand_Range_t randRange);
	bool setTriangularSpreadSpectrum(SS_Tri_Range_t triRange);
	bool setPinCtrl1(uint8_t pinCtrl1);
	bool setPinCtrl2(uint8_t pinCtrl2);
	bool setMiscCtrl3(uint8_t miscCtrl3);
	bool setCBCCtrl(CBC_Sel_t levelSel, bool cbcEn, bool cbcWarnEn, bool cbcFaultEn);
	bool setFaultClear(void);

	/* Getters */
	bool getDevCtrl1(Fsw_t* p_fsw, bool* p_pbtl, Modulation_t* p_mod);
	bool getDevCtrl2(bool* p_dspEn, bool* p_ch1Mute, bool* p_ch2Mute, Power_State_t* p_powState);
	bool getPvddUvCtrl(bool* p_uvHiZEn, UV_Avg_t* p_uvAvg, bool* p_pvddDropDetectEn);
	bool getAutoIncPage(bool* p_autoInc);
	bool getSigChCtrl(BCLK_t* p_bclk, FS_t* p_fs);
	bool getClockDetCtrl(bool* p_detPll, bool* p_detBclkRange, bool* p_detFs, bool* p_detBclkRatio, bool* p_detBclkMissing);
	bool getI2sCtrl(bool* p_blckInv);
	bool getSapCtrl1(uint8_t* p_i2sShiftMsb, Format_t* p_i2sFormat, bool* p_lrclkPulseWidth, Bit_Depth_t* p_i2sBitDepth);
	bool getSapCtrl2(uint8_t* p_i2sShiftLsb);
	bool getFsMon(uint8_t* p_bclkMonMsb, FS_Monitoring_t* p_fsMon);
	bool getBclkMonLsb(uint8_t* p_bclkMonLsb);
	bool getClockDetStatus(
		bool* p_bclkOverRate, bool* p_pllOverRate, bool* p_pllNotLocked, bool* p_bclkMissing, bool* p_blckNotValid,
		bool* p_fsNotValid
	);
	bool getDspPgmMode(bool* p_ch1HiZ, bool* p_ch2HiZ, bool* p_romMode);
	bool getDspCtrl(Proc_Rate_t* p_procRate, bool* p_IramBoot, bool* p_defCoeff);
	bool getDigVolLeft(uint8_t* p_vol);
	bool getDigVolRight(uint8_t* p_vol);
	// NOTE: mistake in the datasheet, the datasheet flipped up and down so no idea what is correct
	bool getDigVolCtrl2(Vol_Freq_t* p_vnus, Vol_Step_t* p_vnuf, Vol_Freq_t* p_vnds, Vol_Step_t* p_vndf);
	bool getDigVolCtrl3(Vol_Freq_t* p_eds, Vol_Step_t* p_edf);
	bool getAutoMuteCtrl(bool* p_bothMute, bool* p_ch1Mute, bool* p_ch2Mute);
	bool getAutoMuteTime(Auto_Mute_Time_t* p_ch1Time, Auto_Mute_Time_t* p_ch2Time);
	bool getLoopBW(Loop_BW_t* p_loopBW);
	bool getAnalogGain(float* p_gain);
	bool getPVDD(float* p_pvdd);
	bool getGPIOMode(GPIO_Mode_t* p_gpioMode0, GPIO_Mode_t* p_gpioMode1, GPIO_Mode_t* p_gpioMode2);
	bool getGPIOSel(GPIO_Sel_t* p_gpioSel0, GPIO_Sel_t* p_gpioSel1, GPIO_Sel_t* p_gpioSel2);
	bool getMiscCtrl2(bool* p_gpioInv0, bool* p_gpioInv1, bool* p_gpioInv2);
	bool getPowState(Power_State_t* p_powState);
	bool getAutoMuteState(bool* p_ch1Mute, bool* p_ch2Mute);
	bool getSpreadSpectrumCtrl(bool* p_triangularEn, bool* p_randomEn, SS_Rand_Range_t* p_randRange, SS_Tri_Range_t* p_triRange);
	bool getChanFault(uint8_t* p_chanFault);
	bool getGlobalFault1(uint8_t* p_globalFault1);
	bool getGlobalFault2(uint8_t* p_globalFault2);
	bool getWarning(uint8_t* p_warning);
	bool getPinCtrl1(uint8_t* p_pinCtrl1);
	bool getPinCtrl2(uint8_t* p_pinCtrl2);
	bool getMiscCtrl3(uint8_t* p_miscCtrl3);
	bool getCbcCtrl(CBC_Sel_t* p_levelSel, bool* p_cbcEn, bool* p_cbcWarnEn, bool* p_cbcFaultEn);

private:
	uint8_t            address;
	I2C_HandleTypeDef* i2cHandler;

	/* Registers */
	const uint8_t REG_RESET_CTRL        = 0x01;
	const uint8_t REG_DEVICE_CTRL1      = 0x02;
	const uint8_t REG_DEVICE_CTRL2      = 0x03;
	const uint8_t REG_PVDD_UV_CONTROL   = 0x04;
	const uint8_t REG_I2C_PAGE_AUTO_INC = 0x0F;
	const uint8_t REG_SIG_CH_CTRL       = 0x28;
	const uint8_t REG_CLOCK_DET_CTRL    = 0x29;
	const uint8_t REG_SDOUT_SEL         = 0x30;
	const uint8_t REG_I2S_CTRL          = 0x31;
	const uint8_t REG_SAP_CTRL1         = 0x33;
	const uint8_t REG_SAP_CTRL2         = 0x34;
	const uint8_t REG_SAP_CTRL3         = 0x35;
	const uint8_t REG_FS_MON            = 0x37;
	const uint8_t REG_BCLK_MON          = 0x38;
	const uint8_t REG_CLKDET_STATUS     = 0x39;
	const uint8_t REG_DSP_PGM_MODE      = 0x40;
	const uint8_t REG_DSP_CTRL          = 0x46;
	const uint8_t REG_DIG_VOL_LEFT      = 0x4C;
	const uint8_t REG_DIG_VOL_RIGHT     = 0x4D;
	const uint8_t REG_DIG_VOL_CTRL2     = 0x4E;
	const uint8_t REG_DIG_VOL_CTRL3     = 0x4F;
	const uint8_t REG_AUTO_MUTE_CTRL    = 0x50;
	const uint8_t REG_AUTO_MUTE_TIME    = 0x51;
	const uint8_t REG_ANA_CTRL          = 0x53;
	const uint8_t REG_AGAIN             = 0x54;
	const uint8_t REG_ADC_RPT           = 0x5E;
	const uint8_t REG_GPIO_CTRL         = 0x60;
	const uint8_t REG_GPIO0_SEL         = 0x61;
	const uint8_t REG_GPIO1_SEL         = 0x62;
	const uint8_t REG_GPIO2_SEL         = 0x63;
	const uint8_t REG_GPIO_INPUT_SEL    = 0x64;
	const uint8_t REG_MISC_CTRL1        = 0x65;
	const uint8_t REG_MISC_CTRL2        = 0x66;
	const uint8_t REG_DIE_ID            = 0x67;
	const uint8_t REG_POWER_STATE       = 0x68;
	const uint8_t REG_AUTOMUTE_STATE    = 0x69;
	const uint8_t REG_RAMP_PHASE_CTRL   = 0x6A;
	const uint8_t REG_RAMP_SS_CTRL0     = 0x6B;
	const uint8_t REG_RAMP_SS_CTRL1     = 0x6C;
	const uint8_t REG_CHAN_FAULT        = 0x70;
	const uint8_t REG_GLOBAL_FAULT1     = 0x71;
	const uint8_t REG_GLOBAL_FAULT2     = 0x72;
	const uint8_t REG_WARNING           = 0x73;
	const uint8_t REG_PIN_CONTROL1      = 0x74;
	const uint8_t REG_PIN_CONTROL2      = 0x75;
	const uint8_t REG_MISC_CONTROL3     = 0x76;
	const uint8_t REG_CBC_CONTROL       = 0x77;
	const uint8_t REG_FAULT_CLEAR       = 0x78;

	bool writeRegister(uint8_t reg, uint8_t value);
	bool readRegister(uint8_t reg, uint8_t* p_value);
};
