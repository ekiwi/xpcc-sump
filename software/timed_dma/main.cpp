/*
 * Connect a UART <-> USB adapter to PA2 and PA3
 *
 * Connect the 8MHz clock out (PA8) to e.g. PE15
 *
 */

#include <xpcc/architecture/platform.hpp>
#include <xpcc/debug/logger.hpp>
#include "../stm32f4_discovery.hpp"

// Create an IODeviceWrapper around the Uart Peripheral we want to use
xpcc::IODeviceWrapper< Usart2 > loggerDevice;

// Set all four logger streams to use the UART
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);

// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::DEBUG


// Allocate 64k bytes Memory
// constexpr uint16_t SAMPLE_LENGTH = 0xffff / 2;
constexpr uint16_t SAMPLE_LENGTH = 100;
uint16_t samples[SAMPLE_LENGTH];

constexpr int SAMPLE_FREQUENCY = MHz4;

// Enums
enum class
Channel : uint32_t
{
	Channel0 = 0,
	Channel1 = DMA_SxCR_CHSEL_0,
	Channel2 = DMA_SxCR_CHSEL_1,
	Channel3 = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
	Channel4 = DMA_SxCR_CHSEL_2,
	Channel5 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0,
	Channel6 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1,
	Channel7 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
};

enum class
MemoryBurstTransfer : uint32_t
{
	Single 		= 0,
	Increment4 	= DMA_SxCR_MBURST_0,
	Increment8 	= DMA_SxCR_MBURST_1,
	Increment16 = DMA_SxCR_MBURST_1 | DMA_SxCR_MBURST_0,
};

enum class
PeripheralBurstTransfer : uint32_t
{
	Single 		= 0,
	Increment4 	= DMA_SxCR_PBURST_0,
	Increment8 	= DMA_SxCR_PBURST_1,
	Increment16 = DMA_SxCR_PBURST_1 | DMA_SxCR_PBURST_0,
};

enum class
Priority : uint32_t
{
	Low 		= 0,
	Medium  	= DMA_SxCR_PL_0,
	High 		= DMA_SxCR_PL_1,
	VeryHigh 	= DMA_SxCR_PL_1 | DMA_SxCR_PL_0,
};

/// In direct mode (if the FIFO is not used)
/// MSIZE is forced by hardware to the same value as PSIZE
enum class
MemoryDataSize : uint32_t
{
	Byte 		= 0,
	HalfWord 	= DMA_SxCR_MSIZE_0,
	Word 		= DMA_SxCR_MSIZE_1,
};

enum class
PeripheralDataSize : uint32_t
{
	Byte 		= 0,
	HalfWord 	= DMA_SxCR_PSIZE_0,
	Word 		= DMA_SxCR_PSIZE_1,
};

enum class
MemoryIncrementMode : uint32_t
{
	Fixed 		= 0,
	Increment 	= DMA_SxCR_MINC, ///< incremented according to MemoryDataSize
};

enum class
PeripheralIncrementMode : uint32_t
{
	Fixed 		= 0,
	Increment 	= DMA_SxCR_PINC, ///< incremented according to PeripheralDataSize
};

enum class
DataTransferDirection : uint32_t
{
	/// Source: DMA_SxPAR; Sink: DMA_SxM0AR
	PeripheralToMemory 	= 0,
	/// Source: DMA_SxM0AR; Sink: DMA_SxPAR
	MemoryToPeripheral 	= DMA_SxCR_DIR_0,
	/// Source: DMA_SxPAR; Sink: DMA_SxM0AR
	MemoryToMemory 		= DMA_SxCR_DIR_1,
};

enum class
FlowControl : uint32_t
{
	Dma 		= 0,
	Peripheral 	= DMA_SxCR_PFCTRL, ///< the peripheral is the flow controller
};


// ----------------------------------------------------------------------------
MAIN_FUNCTION
{
	typedef ExternalOscillator<MHz8> externalOscillator;
	typedef SystemClock<Pll<ExternalOscillator<MHz8>, MHz168, MHz48> > systemClock;
	systemClock::enable();

	LedOrange::setOutput(xpcc::Gpio::HIGH);
	LedGreen::setOutput(xpcc::Gpio::LOW);
	LedRed::setOutput(xpcc::Gpio::LOW);
	LedBlue::setOutput(xpcc::Gpio::LOW);

	// Initialize Usart
	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx);
	Usart2::initialize<systemClock, 115200>(10);

	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# DMA Demo for STM32F4Discoveryboards       #" << xpcc::endl;
	XPCC_LOG_INFO << "# powered by XPCC                           #" << xpcc::endl;
	XPCC_LOG_INFO << "# https://github.com/roboterclubaachen/xpcc #" << xpcc::endl;
	XPCC_LOG_INFO << "#############################################" << xpcc::endl;
	XPCC_LOG_INFO << "# Sampling Freqeuncy: " << kHz(SAMPLE_FREQUENCY) << "kHz" << xpcc::endl;

	// Init Gpio Inputs
	GpioInputE0::setInput(Gpio::InputType::Floating);
	GpioInputE1::setInput(Gpio::InputType::Floating);
	GpioInputE2::setInput(Gpio::InputType::Floating);
	GpioInputE3::setInput(Gpio::InputType::Floating);
	GpioInputE4::setInput(Gpio::InputType::Floating);
	GpioInputE5::setInput(Gpio::InputType::Floating);
	GpioInputE6::setInput(Gpio::InputType::Floating);
	GpioInputE7::setInput(Gpio::InputType::Floating);
	GpioInputE8::setInput(Gpio::InputType::Floating);
	GpioInputE9::setInput(Gpio::InputType::Floating);
	GpioInputE10::setInput(Gpio::InputType::Floating);
	GpioInputE11::setInput(Gpio::InputType::Floating);
	GpioInputE12::setInput(Gpio::InputType::Floating);
	GpioInputE13::setInput(Gpio::InputType::Floating);
	GpioInputE14::setInput(Gpio::InputType::Floating);
	GpioInputE15::setInput(Gpio::InputType::Floating);


	for(uint32_t i = 0; i < SAMPLE_LENGTH; ++i) {
		samples[i] = 0xffff;
	}

	// Setup DMA2
	// enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	// reset timer
	RCC->AHB1RSTR |=  RCC_AHB1RSTR_DMA2RST;
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
	// Using Stream6
	DMA2_Stream6->CR &= ~DMA_SxCR_EN;
	while(DMA2_Stream6->CR & DMA_SxCR_EN); // wait for channel to be disabled
	// Configure
	DMA2_Stream6->CR = 	  static_cast<uint32_t>(Channel::Channel0)
						// use Channel0 as it is connected to Timer1 Channel1-3
						| static_cast<uint32_t>(MemoryBurstTransfer::Single)
						| static_cast<uint32_t>(PeripheralBurstTransfer::Single)
						| static_cast<uint32_t>(Priority::VeryHigh)
						| static_cast<uint32_t>(MemoryDataSize::HalfWord)
						| static_cast<uint32_t>(PeripheralDataSize::HalfWord)
						| static_cast<uint32_t>(MemoryIncrementMode::Increment)
				// since in Mem2Mem mode peripheral seems to be the source:
						| static_cast<uint32_t>(PeripheralIncrementMode::Fixed)
						| static_cast<uint32_t>(DataTransferDirection::PeripheralToMemory)
				// does this only mean that the peripheral event controlls the dma?
						| static_cast<uint32_t>(FlowControl::Dma);
	// Source (DMA_SxPAR) is GPIO Port E
	DMA2_Stream6->PAR = reinterpret_cast<uint32_t>(&(GPIOE->IDR ));
	// Sink (DMA_SxM0AR) is memory
	DMA2_Stream6->M0AR = reinterpret_cast<uint32_t>(&samples[0]);
	// transfer as many bytes as fit into allocated memory
	DMA2_Stream6->NDTR = SAMPLE_LENGTH;
	// do not use FIFO
	DMA2_Stream6->FCR = 0;

	// Start Timer1
	constexpr uint16_t OVERFLOW = (systemClock::Timer1 / SAMPLE_FREQUENCY) - 1;
	constexpr uint16_t COMPARE  = (OVERFLOW / 2) + 1;	// 50% duty cycle
	Timer1::enable();
	Timer1::setMode(Timer1::Mode::UpCounter);
	Timer1::setPrescaler(1);	// 168MHz
	Timer1::setOverflow(OVERFLOW);
	Timer1::configureOutputChannel(3, Timer1::OutputCompareMode::Pwm, COMPARE);
	GpioOutputA10::configure(Gpio::OutputType::PushPull);
	GpioOutputA10::connect(Timer1::Channel3);
	Timer1::applyAndReset();
	Timer1::start();
	Timer1::enableOutput();

	// Use Timer1 Channel3 as DMA Trigger
	TIM1->DIER |= TIM_DIER_CC3DE;	// CC 3 DMA Request enable


	// Start Timer2 as sample signal
	constexpr uint16_t T2_OVERFLOW = (systemClock::Timer2 / kHz100) + 1;
	constexpr uint16_t T2_COMPARE  = (T2_OVERFLOW / 2) + 1;	// 50% duty cycle
	Timer2::enable();
	Timer2::setMode(Timer1::Mode::UpCounter);
	Timer2::setPrescaler(1);
	Timer2::setOverflow(T2_OVERFLOW);
	Timer2::configureOutputChannel(4, Timer2::OutputCompareMode::Pwm, T2_COMPARE);
	GpioOutputB11::configure(Gpio::OutputType::PushPull);
	GpioOutputB11::connect(Timer2::Channel4);
	Timer2::applyAndReset();
	Timer2::start();


	LedBlue::set();
	// Enable DMA
	DMA2_Stream6->CR |= DMA_SxCR_EN;
	// Wait for DMA to finish
	while(DMA2_Stream6->CR & DMA_SxCR_EN);
	LedBlue::reset();

	// Output Sampled Data
	for(int i = 0; i < SAMPLE_LENGTH; ++i) {
		uint16_t sample = samples[i];
		for(int bit = 0; bit < 16; ++bit) {
			if(sample & 0x8000) {
				XPCC_LOG_INFO << "1";
			} else {
				XPCC_LOG_INFO << "0";
			}
			sample <<= 1;
		}
		XPCC_LOG_INFO << xpcc::endl;
	}

	while (1)
	{
		LedOrange::toggle();
		xpcc::delay_ms(500);
	}

	return 0;
}
