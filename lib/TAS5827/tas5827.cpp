#include "tas5827.h"
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief begin with the I2C address and the I2C handle
 *
 * @param address 7-bits I2C address
 * @param DUMMY_I2C_HANDLE TODO: PLEASE HANDLE THIS
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::begin(uint8_t address, I2C_HandleTypeDef* i2cHandle)
{
	this->address    = address;
	this->i2cHandler = i2cHandle;
	return true;
}

/* ------------------------------------------------------------ */
/* Setters                                                      */
/* ------------------------------------------------------------ */

/**
 * @brief Reset interpolation filter and the DAC modules.
 * Since the DSP is also reset, the coefficient RAM content will also be cleared by the DSP.
 * This bit is auto cleared and can be set only in Hi-Z mode.
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setModuleReset()
{
	return writeRegister(REG_RESET_CTRL, (1 << 4));
}

/**
 * @brief Reset mode registers back to their initial values.
 * The RAM content is not cleared.
 * This bit is auto cleared and must be set only when the DAC is in Hi-Z mode
 * (resetting registers when the DAC is running is prohibited and not supported).
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setRegisterReset()
{
	return writeRegister(REG_RESET_CTRL, (1 << 0));
}

/**
 * @brief set the device control 1
 *
 * @param fsw switching frequency
 * @param pbtl true - parallel bridge tied load, false - bridge tied load
 * @param mod modulation type
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDevCtrl1(Fsw_t fsw, bool pbtl, Modulation_t mod)
{
	uint8_t devCtrl1 = 0;

	devCtrl1 |= (static_cast<uint8_t>(fsw) & 0x07) << 4;
	devCtrl1 |= pbtl ? (1 << 2) : 0;
	devCtrl1 |= static_cast<uint8_t>(mod) & 0x03;

	return writeRegister(REG_DEVICE_CTRL1, devCtrl1);
}

/**
 * @brief set the device control 2
 *
 * @param dspEn enables the DSP
 * When the bit is made 0, DSP will start powering up and send out data.
 * This needs to be made 0 only after all the input clocks are settled so that DMA channels do not go out of sync.
 * @param ch1Mute mute channel 1 - soft mute request
 * @param ch2Mute mute channel 2 - soft mute request
 * @param powState power state
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDevCtrl2(bool dspEn, bool ch1Mute, bool ch2Mute, Power_State_t powState)
{
	uint8_t devCtrl2 = 0;

	devCtrl2 |= dspEn ? 0 : (1 << 4);
	devCtrl2 |= ch1Mute ? (1 << 3) : 0;
	devCtrl2 |= ch2Mute ? (1 << 2) : 0;
	devCtrl2 |= static_cast<uint8_t>(powState) & 0x03;

	return writeRegister(REG_DEVICE_CTRL2, devCtrl2);
}

/**
 * @brief set the PVDD undervoltage control register
 *
 * @param uvHiZEn enable the Hi-Z mode when UVLO is detected
 * @param uvAvg UVLO averaging type
 * @param pvddDropDetectEn enable the PVDD drop detection
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setPvddUvCtrl(bool uvHiZEn, UV_Avg_t uvAvg, bool pvddDropDetectEn)
{
	uint8_t pvddUvCtrl = 0;

	pvddUvCtrl |= uvHiZEn ? (1 << 3) : 0;
	pvddUvCtrl |= (static_cast<uint8_t>(uvAvg) & 0x03) << 1;
	pvddUvCtrl |= pvddDropDetectEn ? (1 << 0) : 0;

	return writeRegister(REG_PVDD_UV_CONTROL, pvddUvCtrl);
}

/**
 * @brief set the auto increment page
 *
 * @param autoInc true - auto increment page
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setAutoIncPage(bool autoInc)
{
	return writeRegister(REG_I2C_PAGE_AUTO_INC, autoInc ? 0 : (1 << 3));
}

/**
 * @brief set the signal channel control
 *
 * @param bclk bit clock
 * @param fs sample frequency
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setSigChCtrl(BCLK_t bclk, FS_t fs)
{
	uint8_t sigChCtrl = 0;

	sigChCtrl |= (static_cast<uint8_t>(bclk) & 0x0F) << 4;
	sigChCtrl |= (static_cast<uint8_t>(fs) & 0x0F) << 0;

	return writeRegister(REG_SIG_CH_CTRL, sigChCtrl);
}

/**
 * @brief set the clock fault detection control
 *
 * @param detPll PLL clock detection enable
 * @param detBclkRange bit clock range detection enable
 * @param detFs sample frequency detection enable
 * @param detBclkRatio bit clock to sample frequency ratio detection enable
 * @param detBclkMissing bit clock missing detection enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setClockDetCtrl(bool detPll, bool detBclkRange, bool detFs, bool detBclkRatio, bool detBclkMissing)
{
	uint8_t clockDetCtrl = 0;

	clockDetCtrl |= detPll ? 0 : (1 << 6);
	clockDetCtrl |= detBclkRange ? 0 : (1 << 5);
	clockDetCtrl |= detFs ? 0 : (1 << 4);
	clockDetCtrl |= detBclkRatio ? 0 : (1 << 3);
	clockDetCtrl |= detBclkMissing ? 0 : (1 << 2);

	return writeRegister(REG_CLOCK_DET_CTRL, clockDetCtrl);
}

/**
 * @brief set the I2S control
 *
 * @param blckInv bit clock inversion, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setI2sCtrl(bool blckInv)
{
	uint8_t i2sCtrl = 0;

	i2sCtrl |= blckInv ? (1 << 5) : 0;

	return writeRegister(REG_I2S_CTRL, i2sCtrl);
}

/**
 * @brief set the serial audio port control 1
 *
 * @param i2sShiftMsb I2S data shift MSB, combine with the 8 bits in low register 34h REG_SAP_CTRL2.
 * @param i2sFormat I2S data format
 * @param lrclkPulseWidth LRCLK pulse width, if true, the LRCLK pulse is shorter than 8 x BCLK, set to '01' else '00'
 * @param i2sBitDepth I2S data bit depth
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setSapCtrl1(uint8_t i2sShiftMsb, Format_t i2sFormat, bool lrclkPulseWidth, Bit_Depth_t i2sBitDepth)
{
	uint8_t sapCtrl1 = 0;

	sapCtrl1 |= (i2sShiftMsb & (1 << 0)) << 7;
	sapCtrl1 |= (static_cast<uint8_t>(i2sFormat) & 0x03) << 4;
	sapCtrl1 |= lrclkPulseWidth ? (1 << 2) : 0;
	sapCtrl1 |= (static_cast<uint8_t>(i2sBitDepth) & 0x03) << 0;

	return writeRegister(REG_SAP_CTRL1, sapCtrl1);
}

/**
 * @brief set the serial audio port control 2
 *
 * @param i2sShiftLsb I2S data shift LSB, combine with the 8 bits in high register 33h REG_SAP_CTRL1.
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setSapCtrl2(uint8_t i2sShiftLsb)
{
	return writeRegister(REG_SAP_CTRL2, i2sShiftLsb);
}

/**
 * @brief set the DSP Program Mode
 *
 * @param ch1HiZ channel 1 Hi-Z mode, true for Hi-Z
 * @param ch2HiZ channel 2 Hi-Z mode, true for Hi-Z
 * @param romMode ROM mode, true for ROM mode, false for RAM mode
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDspPgmMode(bool ch1HiZ, bool ch2HiZ, bool romMode)
{
	uint8_t dspPgmMode = 0;

	dspPgmMode |= ch1HiZ ? (1 << 3) : 0;
	dspPgmMode |= ch2HiZ ? (1 << 2) : 0;
	dspPgmMode |= romMode ? (1 << 0) : 0;

	return writeRegister(REG_DSP_PGM_MODE, dspPgmMode);
}

/**
 * @brief set the DSP control
 *
 * @param procRate processing rate
 * @param IramBoot true - boot from IRAM, false - boot from IROM
 * @param defCoeff true - use default coefficients from ZROM, false - use custom coefficients
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDspCtrl(Proc_Rate_t procRate, bool IramBoot, bool defCoeff)
{
	uint8_t dspCtrl = 0;

	dspCtrl |= (static_cast<uint8_t>(procRate) & 0x03) << 3;
	dspCtrl |= IramBoot ? (1 << 1) : 0;
	dspCtrl |= defCoeff ? (1 << 0) : 0;

	return writeRegister(REG_DSP_CTRL, dspCtrl);
}

/**
 * @brief set the digital volume left, channel 1
 * The volume is set in -0.5 dB steps.
 * 0 = +24.0 dB
 * 1 = +23.5 dB
 * ...
 * 254 = -103 dB
 * 255 = Mute
 *
 * @param vol volume to set
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDigVolLeft(uint8_t vol)
{
	return writeRegister(REG_DIG_VOL_LEFT, vol);
}

/**
 * @brief set the digital volume right, channel 2
 * The volume is set in -0.5 dB steps.
 * 0 = +24.0 dB
 * 1 = +23.5 dB
 * ...
 * 254 = -103 dB
 * 255 = Mute
 *
 * @param vol volume to set
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDigVolRight(uint8_t vol)
{
	return writeRegister(REG_DIG_VOL_RIGHT, vol);
}

/**
 * @brief set the digital volume control 2
 *
 * @param vnus volume update speed for volume up, in dB per update
 * @param vnuf volume update frequency for volume up, in sample periods per update
 * @param vnds volume update speed for volume down, in dB per update
 * @param vndf volume update frequency for volume down, in sample periods per update
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDigVolCtrl2(Vol_Freq_t vnus, Vol_Step_t vnuf, Vol_Freq_t vnds, Vol_Step_t vndf)
{
	uint8_t digVolCtrl2 = 0;

	digVolCtrl2 |= (static_cast<uint8_t>(vnus) & 0x03) << 6;
	digVolCtrl2 |= (static_cast<uint8_t>(vnuf) & 0x03) << 4;
	digVolCtrl2 |= (static_cast<uint8_t>(vnds) & 0x03) << 2;
	digVolCtrl2 |= (static_cast<uint8_t>(vndf) & 0x03) << 0;

	return writeRegister(REG_DIG_VOL_CTRL2, digVolCtrl2);
}

/**
 * @brief set the digital volume control 3
 *
 * @param veds emergency volume update speed for volume down, in dB per update
 * @param vedf emergency volume update frequency for volume down, in sample periods per update
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDigVolCtrl3(Vol_Freq_t veds, Vol_Step_t vedf)
{
	uint8_t digVolCtrl3 = 0;

	digVolCtrl3 |= (static_cast<uint8_t>(veds) & 0x03) << 6;
	digVolCtrl3 |= (static_cast<uint8_t>(vedf) & 0x03) << 4;

	return writeRegister(REG_DIG_VOL_CTRL3, digVolCtrl3);
}

/**
 * @brief set the auto mute control
 *
 * @param bothMute true - both channels are only muted when both channels are about to be auto muted
 * @param ch1Mute true - channel 1 is set to auto muted
 * @param ch2Mute true - channel 2 is set to auto muted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setAutoMuteCtrl(bool bothMute, bool ch1Mute, bool ch2Mute)
{
	uint8_t autoMuteCtrl = 0;

	autoMuteCtrl |= bothMute ? (1 << 2) : 0;
	autoMuteCtrl |= ch2Mute ? (1 << 1) : 0;
	autoMuteCtrl |= ch1Mute ? (1 << 0) : 0;

	return writeRegister(REG_AUTO_MUTE_CTRL, autoMuteCtrl);
}

/**
 * @brief set the auto mute time
 *
 * @param ch1Time channel 1 auto mute time
 * @param ch2Time channel 2 auto mute time
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setAutoMuteTime(Auto_Mute_Time_t ch1Time, Auto_Mute_Time_t ch2Time)
{
	uint8_t autoMuteTime = 0;

	autoMuteTime |= (static_cast<uint8_t>(ch1Time) & 0x03) << 4;
	autoMuteTime |= (static_cast<uint8_t>(ch2Time) & 0x03) << 0;

	return writeRegister(REG_AUTO_MUTE_TIME, autoMuteTime);
}

/**
 * @brief set the loop bandwidth for the class D amplifier
 *
 * @param loopBW only can be 80, 100, 120 or 175 kHz
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setLoopBW(Loop_BW_t loopBW)
{
	// PHASE_CTL here is always set to 0 = out of phase, since it is not used
	// only one TAS5827 is used
	return writeRegister(REG_ANA_CTRL, static_cast<uint8_t>(loopBW));
}

/**
 * @brief set the analog gain
 *
 * @param gain 0 to 31 (0 dB to -15.5 dB)
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setAnalogGain(uint8_t gain)
{
	gain = gain & 0x1F;
	return writeRegister(REG_AGAIN, gain);
}

/**
 * @brief set the GPIOs to be input/output
 *
 * @param gpioMode0 GPIO 0 input/output
 * @param gpioMode1 GPIO 1 input/output
 * @param gpioMode2 GPIO 2 input/output
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOMode(GPIO_Mode_t gpioMode0, GPIO_Mode_t gpioMode1, GPIO_Mode_t gpioMode2)
{
	uint8_t gpio_ctrl = 0;

	gpio_ctrl |= (static_cast<uint8_t>(gpioMode2) & (1 << 0)) << 2;
	gpio_ctrl |= (static_cast<uint8_t>(gpioMode1) & (1 << 0)) << 1;
	gpio_ctrl |= (static_cast<uint8_t>(gpioMode0) & (1 << 0)) << 0;

	return writeRegister(REG_GPIO_CTRL, gpio_ctrl);
}

/**
 * @brief set the GPIOs to be OFF, WARNZ, FAULTZ, PVDD_DROP, SDOUT or RAMP_CLK
 *
 * @param gpioSel0 GPIO 0 mode
 * @param gpioSel1 GPIO 1 mode
 * @param gpioSel2 GPIO 2 mode
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setGPIOSel(GPIO_Sel_t gpioSel0, GPIO_Sel_t gpioSel1, GPIO_Sel_t gpioSel2)
{
	if (!writeRegister(REG_GPIO0_SEL, static_cast<uint8_t>(gpioSel0) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO1_SEL, static_cast<uint8_t>(gpioSel1) & 0x0F))
		return false;
	if (!writeRegister(REG_GPIO2_SEL, static_cast<uint8_t>(gpioSel2) & 0x0F))
		return false;
	return true;
}

/**
 * @brief set the GPIOs inversion setting
 *
 * @param gpioInv0 GPIO 0 inversion, true for inverted
 * @param gpioInv1 GPIO 1 inversion, true for inverted
 * @param gpioInv2 GPIO 2 inversion, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setMiscCtrl2(bool gpioInv0, bool gpioInv1, bool gpioInv2)
{
	uint8_t miscCtrl2 = 0;

	miscCtrl2 |= gpioInv2 ? (1 << 2) : 0;
	miscCtrl2 |= gpioInv1 ? (1 << 1) : 0;
	miscCtrl2 |= gpioInv0 ? (1 << 0) : 0;

	return writeRegister(REG_MISC_CTRL2, miscCtrl2);
}

/**
 * @brief disable the spread spectrum
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setDisableSpreadSpectrum(void)
{
	return writeRegister(REG_RAMP_SS_CTRL0, 0);
}

/**
 * @brief enable the random spread spectrum
 *
 * @param randRange range of the random spread spectrum
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setRandomSpreadSpectrum(SS_Rand_Range_t randRange)
{
	uint8_t spreadSpectrumCtrl0 = 0;
	spreadSpectrumCtrl0 |= 1 << 1;

	uint8_t spreadSpectrumCtrl1 = 0;
	spreadSpectrumCtrl1 |= (static_cast<uint8_t>(randRange) & 0x07) << 4;

	if (!writeRegister(REG_RAMP_SS_CTRL0, spreadSpectrumCtrl0))
		return false;
	if (!writeRegister(REG_RAMP_SS_CTRL1, spreadSpectrumCtrl1))
		return false;
	return true;
}

/**
 * @brief enable the triangular spread spectrum
 *
 * @param triRange range of the triangular spread spectrum
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setTriangularSpreadSpectrum(SS_Tri_Range_t triRange)
{
	uint8_t spreadSpectrumCtrl0 = 0;
	spreadSpectrumCtrl0 |= 1 << 0;

	uint8_t spreadSpectrumCtrl1 = 0;
	spreadSpectrumCtrl1 |= (static_cast<uint8_t>(triRange) & 0x0F);

	if (!writeRegister(REG_RAMP_SS_CTRL0, spreadSpectrumCtrl0))
		return false;
	if (!writeRegister(REG_RAMP_SS_CTRL1, spreadSpectrumCtrl1))
		return false;
	return true;
}

/**
 * @brief set the pin control 1
 *
 * @param pinCtrl1 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setPinCtrl1(uint8_t pinCtrl1)
{
	return writeRegister(REG_PIN_CONTROL1, pinCtrl1);
}

/**
 * @brief set the pin control 2
 *
 * @param pinCtrl2 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setPinCtrl2(uint8_t pinCtrl2)
{
	return writeRegister(REG_PIN_CONTROL2, pinCtrl2);
}

/**
 * @brief set the miscellaneous control 3
 *
 * @param miscCtrl3 value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setMiscCtrl3(uint8_t miscCtrl3)
{
	miscCtrl3 = miscCtrl3 & 0x90;
	return writeRegister(REG_MISC_CONTROL3, miscCtrl3);
}

/**
 * @brief set the CBC control
 *
 * @param levelSel Cycle-By-Cycle current limiting level, percentage of the Over-Current Threshold:
 * @param cbcEn CBC enable
 * @param cbcWarnEn CBC warning enable
 * @param cbcFaultEn CBC fault enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setCBCCtrl(CBC_Sel_t levelSel, bool cbcEn, bool cbcWarnEn, bool cbcFaultEn)
{
	uint8_t cbcCtrl = 0;

	cbcCtrl |= (static_cast<uint8_t>(levelSel) & 0x03) << 3;
	cbcCtrl |= cbcEn ? 1 << 2 : 0;
	cbcCtrl |= cbcWarnEn ? 1 << 1 : 0;
	cbcCtrl |= cbcFaultEn ? 1 << 0 : 0;

	return writeRegister(REG_CBC_CONTROL, cbcCtrl);
}

/**
 * @brief clear the analog fault
 *
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::setFaultClear(void)
{
	return writeRegister(REG_FAULT_CLEAR, (1 << 7));
}

/* ------------------------------------------------------------ */
/* Getters                                                      */
/* ------------------------------------------------------------ */

/**
 * @brief get the device control 1
 *
 * @param p_fsw pointer to return the switching frequency in Fsw_t
 * @param p_pbtl pointer to return the parallel bridge tied load
 * @param p_mod pointer to return the modulation type in Modulation_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDevCtrl1(Fsw_t* p_fsw, bool* p_pbtl, Modulation_t* p_mod)
{
	uint8_t reg;

	if (readRegister(REG_DEVICE_CTRL1, &reg)) {
		*p_fsw  = static_cast<Fsw_t>((reg & 0x70) >> 4);
		*p_pbtl = static_cast<bool>(reg & (1 << 2));
		*p_mod  = static_cast<Modulation_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the device control 2
 *
 * @param p_dspEn pointer to return the DSP enable, true for not reset
 * @param p_ch1Mute pointer to return the channel 1 mute state, true for mute
 * @param p_ch2Mute pointer to return the channel 2 mute state, true for mute
 * @param p_powState pointer to return the power state in Power_State_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDevCtrl2(bool* p_dspEn, bool* p_ch1Mute, bool* p_ch2Mute, Power_State_t* p_powState)
{
	uint8_t reg;

	if (readRegister(REG_DEVICE_CTRL2, &reg)) {
		*p_dspEn    = !static_cast<bool>(reg & (1 << 4));
		*p_ch1Mute  = static_cast<bool>(reg & (1 << 3));
		*p_ch2Mute  = static_cast<bool>(reg & (1 << 2));
		*p_powState = static_cast<Power_State_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the PVDD undervoltage control register
 *
 * @param p_uvHiZEn pointer to return if Hi-Z mode will be entered when UVLO is detected
 * @param p_uvAvg pointer to return the UVLO averaging type in UV_Avg_t
 * @param p_pvddDropDetectEn pointer to return if the PVDD drop detection is enabled
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPvddUvCtrl(bool* p_uvHiZEn, UV_Avg_t* p_uvAvg, bool* p_pvddDropDetectEn)
{
	uint8_t reg;

	if (readRegister(REG_PVDD_UV_CONTROL, &reg)) {
		*p_uvHiZEn          = static_cast<bool>(reg & (1 << 3));
		*p_uvAvg            = static_cast<UV_Avg_t>((reg & 0x06) >> 1);
		*p_pvddDropDetectEn = static_cast<bool>(reg & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the auto increment page
 *
 * @param p_autoInc pointer to return the auto increment page
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAutoIncPage(bool* p_autoInc)
{
	uint8_t reg;

	if (readRegister(REG_I2C_PAGE_AUTO_INC, &reg)) {
		*p_autoInc = !static_cast<bool>(reg & (1 << 3));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the signal channel control
 *
 * @param p_bclk pointer to return the bit clock in BCLK_t
 * @param p_fs pointer to return the sample frequency in FS_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getSigChCtrl(BCLK_t* p_bclk, FS_t* p_fs)
{
	uint8_t reg;

	if (readRegister(REG_SIG_CH_CTRL, &reg)) {
		*p_bclk = static_cast<BCLK_t>((reg & 0xF0) >> 4);
		*p_fs   = static_cast<FS_t>((reg & 0x0F));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the clock fault detection control
 *
 * @param p_detPll pointer to return the PLL clock detection enable
 * @param p_detBclkRange pointer to return the bit clock range detection enable
 * @param p_detFs pointer to return the sample frequency detection enable
 * @param p_detBclkRatio pointer to return the bit clock to sample frequency ratio detection enable
 * @param p_detBclkMissing pointer to return the bit clock missing detection enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getClockDetCtrl(bool* p_detPll, bool* p_detBclkRange, bool* p_detFs, bool* p_detBclkRatio, bool* p_detBclkMissing)
{
	uint8_t reg;

	if (readRegister(REG_CLOCK_DET_CTRL, &reg)) {
		*p_detPll       = static_cast<bool>(reg & (1 << 6));
		*p_detBclkRange = static_cast<bool>(reg & (1 << 5));
		*p_detFs        = static_cast<bool>(reg & (1 << 4));
		*p_detBclkRatio = static_cast<bool>(reg & (1 << 3));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the I2S control
 *
 * @param p_blckInv pointer to return the bit clock inversion, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getI2sCtrl(bool* p_blckInv)
{
	uint8_t reg;

	if (readRegister(REG_I2S_CTRL, &reg)) {
		*p_blckInv = static_cast<bool>(reg & (1 << 5));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the serial audio port control 1
 *
 * @param p_i2sShiftMsb pointer to return the I2S data shift MSB
 * @param p_i2sFormat pointer to return the I2S data format in Format_t
 * @param p_lrclkPulseWidth pointer to return the LRCLK pulse width, true if the LRCLK pulse is shorter than 8 x BCLK
 * @param p_i2sBitDepth pointer to return the I2S data bit depth in Bit_Depth_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getSapCtrl1(uint8_t* p_i2sShiftMsb, Format_t* p_i2sFormat, bool* p_lrclkPulseWidth, Bit_Depth_t* p_i2sBitDepth)
{
	uint8_t reg;

	if (readRegister(REG_SAP_CTRL1, &reg)) {
		*p_i2sShiftMsb     = static_cast<uint8_t>(reg & (1 << 7)) >> 7;
		*p_i2sFormat       = static_cast<Format_t>((reg & 0x30) >> 4);
		*p_lrclkPulseWidth = static_cast<bool>(reg & (1 << 2));
		*p_i2sBitDepth     = static_cast<Bit_Depth_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the serial audio port control 2
 *
 * @param p_i2sShiftLsb pointer to return the I2S data shift LSB
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getSapCtrl2(uint8_t* p_i2sShiftLsb)
{
	return readRegister(REG_SAP_CTRL2, p_i2sShiftLsb);
}

/**
 * @brief get the sampling frequency monitoring and MSB of bit clock frequency monitoring
 *
 * @param p_bclkMonMsb pointer to return the MSB of the bit clock frequency monitoring
 * @param p_fsMon pointer to return the sampling frequency monitoring in FS_Monitoring_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getFsMon(uint8_t* p_bclkMonMsb, FS_Monitoring_t* p_fsMon)
{
	uint8_t reg;

	if (readRegister(REG_FS_MON, &reg)) {
		*p_bclkMonMsb = static_cast<uint8_t>(reg & 0x03) >> 4;
		*p_fsMon      = static_cast<FS_Monitoring_t>(reg & 0x0F);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get LSB of bit clock frequency monitoring
 *
 * @param p_bclkMonLsb pointer to return the LSB of the bit clock frequency monitoring
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getBclkMonLsb(uint8_t* p_bclkMonLsb)
{
	return readRegister(REG_BCLK_MON, p_bclkMonLsb);
}

/**
 * @brief get the clock detection status
 *
 * @param p_bclkOverRate pointer to return, true = over rate, false = under rate
 * @param p_pllOverRate pointer to return, true = over rate, false = under rate
 * @param p_pllNotLocked pointer to return if the PLL is locked, true = not locked
 * @param p_bclkMissing pointer to return if the bit clock is missing, true = missing
 * @param p_blckValid pointer to return if the bit clock is valid, true = not valid
 * @param p_fsValid pointer to return if the sample frequency is valid, true = not valid
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getClockDetStatus(
	bool* p_bclkOverRate, bool* p_pllOverRate, bool* p_pllNotLocked, bool* p_bclkMissing, bool* p_blckNotValid, bool* p_fsNotValid
)
{
	uint8_t reg;

	if (readRegister(REG_CLKDET_STATUS, &reg)) {
		*p_bclkOverRate = static_cast<bool>(reg & (1 << 5));
		*p_pllOverRate  = static_cast<bool>(reg & (1 << 4));
		*p_pllNotLocked = static_cast<bool>(reg & (1 << 3));
		*p_bclkMissing  = static_cast<bool>(reg & (1 << 2));
		*p_blckNotValid = static_cast<bool>(reg & (1 << 1));
		*p_fsNotValid   = static_cast<bool>(reg & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the DSP Program Mode
 *
 * @param p_ch1HiZ pointer to return if channel 1 is set to be in Hi-Z mode
 * @param p_ch2HiZ pointer to return if channel 2 is set to be in Hi-Z mode
 * @param p_romMode pointer to return if the ROM mode is enabled, true = ROM mode, false = RAM mode
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDspPgmMode(bool* p_ch1HiZ, bool* p_ch2HiZ, bool* p_romMode)
{
	uint8_t reg;

	if (readRegister(REG_DSP_PGM_MODE, &reg)) {
		*p_ch1HiZ  = static_cast<bool>(reg & (1 << 3));
		*p_ch2HiZ  = static_cast<bool>(reg & (1 << 2));
		*p_romMode = static_cast<bool>(reg & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the DSP control
 *
 * @param p_procRate pointer to return the processing rate in Proc_Rate_t
 * @param p_IramBoot pointer to return if the boot is from IRAM, true = boot from IRAM, false = boot from IROM
 * @param p_defCoeff pointer to return if the default coefficients from ZROM are used, true = use default coefficients from ZROM,
 * false = use custom coefficients
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDspCtrl(Proc_Rate_t* p_procRate, bool* p_IramBoot, bool* p_defCoeff)
{
	uint8_t reg;

	if (readRegister(REG_DSP_CTRL, &reg)) {
		*p_procRate = static_cast<Proc_Rate_t>((reg & 0x18) >> 3);
		*p_IramBoot = static_cast<bool>(reg & (1 << 1));
		*p_defCoeff = static_cast<bool>(reg & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the digital volume left, channel 1
 * The volume is set in -0.5 dB steps.
 * 0 = +24.0 dB
 * 1 = +23.5 dB
 * ...
 * 254 = -103 dB
 * 255 = Mute
 *
 * @param p_vol pointer to return the volume in -0.5 dB steps
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDigVolLeft(uint8_t* p_vol)
{
	return readRegister(REG_DIG_VOL_LEFT, p_vol);
}

/**
 * @brief get the digital volume right, channel 2
 * The volume is set in -0.5 dB steps.
 * 0 = +24.0 dB
 * 1 = +23.5 dB
 * ...
 * 254 = -103 dB
 * 255 = Mute
 *
 * @param p_vol pointer to return the volume in -0.5 dB steps
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDigVolRight(uint8_t* p_vol)
{
	return readRegister(REG_DIG_VOL_RIGHT, p_vol);
}

/**
 * @brief get the digital volume control 2
 *
 * @param p_vnus pointer to return the volume update speed for volume up in Vol_Freq_t
 * @param p_vnuf pointer to return the volume update frequency for volume up in Vol_Step_t
 * @param p_vnds pointer to return the volume update speed for volume down in Vol_Freq_t
 * @param p_vndf pointer to return the volume update frequency for volume down in Vol_Step_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDigVolCtrl2(Vol_Freq_t* p_vnus, Vol_Step_t* p_vnuf, Vol_Freq_t* p_vnds, Vol_Step_t* p_vndf)
{
	uint8_t reg;

	if (readRegister(REG_DIG_VOL_CTRL2, &reg)) {
		*p_vnus = static_cast<Vol_Freq_t>((reg & 0xC0) >> 6);
		*p_vnuf = static_cast<Vol_Step_t>((reg & 0x30) >> 4);
		*p_vnds = static_cast<Vol_Freq_t>((reg & 0x0C) >> 2);
		*p_vndf = static_cast<Vol_Step_t>((reg & 0x03) >> 0);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the digital volume control 3
 *
 * @param p_veds pointer to return the emergency volume update speed for volume down in Vol_Freq_t
 * @param p_vedf pointer to return the emergency volume update frequency for volume down in Vol_Step_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getDigVolCtrl3(Vol_Freq_t* p_veds, Vol_Step_t* p_vedf)
{
	uint8_t reg;

	if (readRegister(REG_DIG_VOL_CTRL3, &reg)) {
		*p_veds = static_cast<Vol_Freq_t>((reg & 0xC0) >> 6);
		*p_vedf = static_cast<Vol_Step_t>((reg & 0x30) >> 4);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the auto mute control
 *
 * @param p_bothMute pointer to return if both channels are set to be muted only when both channels are about to be auto
 * muted
 * @param p_ch1Mute pointer to return if channel 1 is set to auto muted
 * @param p_ch2Mute pointer to return if channel 2 is set to auto muted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAutoMuteCtrl(bool* p_bothMute, bool* p_ch1Mute, bool* p_ch2Mute)
{
	uint8_t reg;

	if (readRegister(REG_AUTO_MUTE_CTRL, &reg)) {
		*p_bothMute = static_cast<bool>(reg & (1 << 2));
		*p_ch2Mute  = static_cast<bool>(reg & (1 << 1));
		*p_ch1Mute  = static_cast<bool>(reg & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the auto mute time
 *
 * @param p_ch1Time pointer to return the channel 1 auto mute time in Auto_Mute_Time_t
 * @param p_ch2Time pointer to return the channel 2 auto mute time in Auto_Mute_Time_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAutoMuteTime(Auto_Mute_Time_t* p_ch1Time, Auto_Mute_Time_t* p_ch2Time)
{
	uint8_t reg;

	if (readRegister(REG_AUTO_MUTE_TIME, &reg)) {
		*p_ch1Time = static_cast<Auto_Mute_Time_t>((reg & 0x30) >> 4);
		*p_ch2Time = static_cast<Auto_Mute_Time_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the loop bandwidth
 *
 * @param p_loopBW pointer to return the loop bandwidth in enum Loop_BW_t
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getLoopBW(Loop_BW_t* p_loopBW)
{
	uint8_t reg;

	if (readRegister(REG_ANA_CTRL, &reg)) {
		*p_loopBW = static_cast<Loop_BW_t>(reg & 0x60);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the analog gain
 *
 * @param p_gain pointer to return the analog p_ in dB
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAnalogGain(float* p_gain)
{
	uint8_t reg;

	if (readRegister(REG_AGAIN, &reg)) {
		*p_gain = static_cast<float>(reg & 0x1F) * -0.5f;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the PVDD voltage
 *
 * @param p_pvdd pointer to return the PVDD voltage
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPVDD(float* p_pvdd)
{
	uint8_t reg;

	if (readRegister(REG_ADC_RPT, &reg)) {
		// LSB is 0.12 V
		*p_pvdd = static_cast<float>(reg) * 0.12f;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the GPIOs mode
 *
 * @param p_gpioMode0 pointer to GPIO 0 mode to return
 * @param p_gpioMode1 pointer to GPIO 1 mode to return
 * @param p_gpioMode2 pointer to GPIO 2 mode to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOMode(GPIO_Mode_t* p_gpioMode0, GPIO_Mode_t* p_gpioMode1, GPIO_Mode_t* p_gpioMode2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO_CTRL, &reg)) {
		*p_gpioMode0 = static_cast<GPIO_Mode_t>(reg & (1 << 0));
		*p_gpioMode1 = static_cast<GPIO_Mode_t>((reg & (1 << 1)) >> 1);
		*p_gpioMode2 = static_cast<GPIO_Mode_t>((reg & (1 << 2)) >> 2);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the GPIOs function
 *
 * @param p_gpioSel0 pointer to GPIO 0 function to return
 * @param p_gpioSel1 pointer to GPIO 1 function to return
 * @param p_gpioSel2 pointer to GPIO 2 function to return
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGPIOSel(GPIO_Sel_t* p_gpioSel0, GPIO_Sel_t* p_gpioSel1, GPIO_Sel_t* p_gpioSel2)
{
	uint8_t reg;

	if (readRegister(REG_GPIO0_SEL, &reg)) {
		*p_gpioSel0 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO1_SEL, &reg)) {
		*p_gpioSel1 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	if (readRegister(REG_GPIO2_SEL, &reg)) {
		*p_gpioSel2 = static_cast<GPIO_Sel_t>(reg & 0x0F);
	}
	else {
		return false;
	}

	return true;
}

/**
 * @brief get the GPIOs inversion setting
 *
 * @param p_gpioInv0 pointer to GPIO 0 inversion to return, true for inverted
 * @param p_gpioInv1 pointer to GPIO 1 inversion to return, true for inverted
 * @param p_gpioInv2 pointer to GPIO 2 inversion to return, true for inverted
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getMiscCtrl2(bool* p_gpioInv0, bool* p_gpioInv1, bool* p_gpioInv2)
{
	uint8_t reg;

	if (readRegister(REG_MISC_CTRL2, &reg)) {
		*p_gpioInv0 = static_cast<bool>(reg & (1 << 0));
		*p_gpioInv1 = static_cast<bool>(reg & (1 << 1));
		*p_gpioInv2 = static_cast<bool>(reg & (1 << 2));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the power state
 *
 * @param p_powState pointer to return the power state
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPowState(Power_State_t* p_powState)
{
	uint8_t reg;

	if (readRegister(REG_POWER_STATE, &reg)) {
		*p_powState = static_cast<Power_State_t>(reg & 0x03);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the auto mute state
 *
 * @param p_ch1Mute pointer to return the channel 1 mute state
 * @param p_ch2Mute pointer to return the channel 2 mute state
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getAutoMuteState(bool* p_ch1Mute, bool* p_ch2Mute)
{
	uint8_t reg;

	if (readRegister(REG_AUTOMUTE_STATE, &reg)) {
		*p_ch1Mute = static_cast<bool>(reg & (1 << 0));
		*p_ch2Mute = static_cast<bool>(reg & (1 << 1));
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the spread spectrum control
 *
 * @param p_triangularEn pointer to return the triangular spread spectrum enable
 * @param p_randomEn pointer to return the random spread spectrum enable
 * @param p_randRange pointer to return the random spread spectrum range
 * @param p_triRange pointer to return the triangular spread spectrum range
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getSpreadSpectrumCtrl(
	bool* p_triangularEn, bool* p_randomEn, SS_Rand_Range_t* p_randRange, SS_Tri_Range_t* p_triRange
)
{
	uint8_t reg;

	if (readRegister(REG_RAMP_SS_CTRL0, &reg)) {
		*p_triangularEn = static_cast<bool>(reg & (1 << 0));
		*p_randomEn     = static_cast<bool>(reg & (1 << 1));
	}
	else {
		return false;
	}

	if (readRegister(REG_RAMP_SS_CTRL1, &reg)) {
		*p_randRange = static_cast<SS_Rand_Range_t>((reg & 0x70) >> 4);
		*p_triRange  = static_cast<SS_Tri_Range_t>(reg & 0x0F);
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the channel fault
 *
 * @param p_chanFault pointer to return the channel fault
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getChanFault(uint8_t* p_chanFault)
{
	uint8_t reg;

	if (readRegister(REG_CHAN_FAULT, &reg)) {
		*p_chanFault = reg & 0x0F;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the global fault 1
 *
 * @param p_globalFault1 pointer to return the global fault 1
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGlobalFault1(uint8_t* p_globalFault1)
{
	uint8_t reg;

	if (readRegister(REG_GLOBAL_FAULT1, &reg)) {
		*p_globalFault1 = reg & 0x67;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the global fault 2
 *
 * @param p_globalFault2 pointer to return the global fault 2
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getGlobalFault2(uint8_t* p_globalFault2)
{
	uint8_t reg;

	if (readRegister(REG_GLOBAL_FAULT2, &reg)) {
		*p_globalFault2 = reg & 0x07;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the warning
 *
 * @param p_warning pointer to return the warning
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getWarning(uint8_t* p_warning)
{
	uint8_t reg;

	if (readRegister(REG_WARNING, &reg)) {
		*p_warning = reg & 0x07;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the pin control 1
 *
 * @param p_pinCtrl1 pointer to return the pin control 1
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPinCtrl1(uint8_t* p_pinCtrl1)
{
	uint8_t reg;

	if (readRegister(REG_PIN_CONTROL1, &reg)) {
		*p_pinCtrl1 = reg;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the pin control 2
 *
 * @param p_pinCtrl2 pointer to return the pin control 2
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getPinCtrl2(uint8_t* p_pinCtrl2)
{
	uint8_t reg;

	if (readRegister(REG_PIN_CONTROL2, &reg)) {
		*p_pinCtrl2 = reg;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the miscellaneous control 3
 *
 * @param p_miscCtrl3 pointer to return the miscellaneous control 3
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getMiscCtrl3(uint8_t* p_miscCtrl3)
{
	uint8_t reg;

	if (readRegister(REG_MISC_CONTROL3, &reg)) {
		*p_miscCtrl3 = reg & 0x90;
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief get the CBC control
 *
 * @param p_levelSel pointer to return the Cycle-By-Cycle current limiting level
 * @param p_cbcEn pointer to return the CBC enable
 * @param p_cbcWarnEn pointer to return the CBC warning enable
 * @param p_cbcFaultEn pointer to return the CBC fault enable
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::getCbcCtrl(CBC_Sel_t* p_levelSel, bool* p_cbcEn, bool* p_cbcWarnEn, bool* p_cbcFaultEn)
{
	uint8_t cbcCtrl = 0;

	if (readRegister(REG_CBC_CONTROL, &cbcCtrl)) {
		*p_levelSel   = static_cast<CBC_Sel_t>((cbcCtrl & 0x18) >> 3);
		*p_cbcEn      = static_cast<bool>(cbcCtrl & (1 << 2));
		*p_cbcWarnEn  = static_cast<bool>(cbcCtrl & (1 << 1));
		*p_cbcFaultEn = static_cast<bool>(cbcCtrl & (1 << 0));
		return true;
	}
	else {
		return false;
	}
}

/* ------------------------------------------------------------ */
/* Private Helpers                                              */
/* ------------------------------------------------------------ */

/**
 * @brief write to register
 *
 * @param reg register address
 * @param value value to write
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::writeRegister(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(this->i2cHandler, this->address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&value, 1, 1000);

	/* Check the communication status */
	if (status != HAL_OK) {
		return false;
	}
	else {
		return true;
	}
}

/**
 * @brief read from register
 *
 * @param reg register address
 * @param p_value pointer to return value read
 * @return true - OK
 * @return false - Error
 */
bool TAS5827::readRegister(uint8_t reg, uint8_t* p_value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(this->i2cHandler, this->address, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, p_value, 1, 1000);

	/* Check the communication status */
	if (status != HAL_OK) {
		return false;
	}
	else {
		return true;
	}
}
