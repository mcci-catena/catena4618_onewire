/*

Module:  catena4618_onewire.ino

Function:
        Sensor program for Catena 4618 with OneWire probe.

Copyright notice:
        This file copyright (C) 2021 by

                MCCI Corporation
                3520 Krums Corners Road
                Ithaca, NY  14850

        See project LICENSE file for license information.

Author:
        Terry Moore, MCCI Corporation	September 2021

Revision history:
        See https://github.com/mcci-catena/Catena-Sketches

*/

#include <Catena.h>

#include <Arduino_LoRaWAN.h>
#include <Catena_BootloaderApi.h>
#include <Catena_CommandStream.h>
#include <Catena_Download.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Serial.h>
#include <Catena_Si1133.h>
#include <Catena_TxBuffer.h>
#include <cmath>
#include <lmic.h>
#include <mcciadk_baselib.h>
#include <type_traits>
#include <Wire.h>

#include <OneWire.h>
#include <DallasTemperature.h>


using namespace McciCatena;

#if defined(ARDUINO_MCCI_CATENA_4618)
# include <Catena-SHT3x.h>
  using cTemperatureSensor = McciCatenaSht3x::cSHT3x;
  using namespace McciCatenaSht3x;
#else
#  error Platform not supported
#endif

// because of bug #319, we need to make sure we have the right
// library version.
static_assert(
        CATENA_ARDUINO_PLATFORM_VERSION_COMPARE_GE(
                CATENA_ARDUINO_PLATFORM_VERSION,
                CATENA_ARDUINO_PLATFORM_VERSION_CALC(0, 21, 3, 1)
                ),
        "catena-arduino-platform library must be at least v0.21.3-pre1 or later"
        );

/****************************************************************************\
|
|	MANIFEST CONSTANTS & TYPEDEFS
|
\****************************************************************************/

constexpr uint8_t kUplinkPort = 6;

enum class FlagsSensorPort6 : uint8_t
        {
        FlagVbat = 1 << 0,
        FlagVcc = 1 << 1,
        FlagBoot = 1 << 2,
        FlagTH = 1 << 3,	// temperature, humidity
        FlagLight = 1 << 4,	// Si1133 "white" as a float
        FlagVbus = 1 << 5,	// Vbus input
        FlagOneWireT = 1 << 6,  // 
        };

constexpr FlagsSensorPort6 operator| (const FlagsSensorPort6 lhs, const FlagsSensorPort6 rhs)
        {
        return FlagsSensorPort6(uint8_t(lhs) | uint8_t(rhs));
        };

FlagsSensorPort6 operator|= (FlagsSensorPort6 &lhs, const FlagsSensorPort6 &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

/* adjustable timing parameters */
enum    {
        // set this to interval between transmissions, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time, but we attempt
        // to compensate for the gross effects below.
        CATCFG_T_CYCLE = 6 * 60,        // every 6 minutes
        CATCFG_T_CYCLE_TEST = 30,       // every 30 seconds
        CATCFG_T_CYCLE_INITIAL = 30,    // every 30 seconds initially
        CATCFG_INTERVAL_COUNT_INITIAL = 10,     // repeat for 5 minutes
        CATCFG_T_REBOOT = 30 * 24 * 60 * 60,    // reboot every 30 days
        };

/* additional timing parameters; ususually you don't change these. */
enum    {
        CATCFG_T_WARMUP = 1,
        CATCFG_T_SETTLE = 5,
        CATCFG_T_OVERHEAD = (CATCFG_T_WARMUP + CATCFG_T_SETTLE + 4),
        CATCFG_T_MIN = CATCFG_T_OVERHEAD,
        CATCFG_T_MAX = CATCFG_T_CYCLE < 60 * 60 ? 60 * 60 : CATCFG_T_CYCLE,     // normally one hour max.
        CATCFG_INTERVAL_COUNT = 30,
        };

enum    {
        PIN_ONE_WIRE =  A2,        // XSDA1 == A2
        PIN_ONE_WIRE_POWER = D11,
        };

constexpr uint32_t CATCFG_GetInterval(uint32_t tCycle)
        {
        return (tCycle < CATCFG_T_OVERHEAD + 1)
                ? 1
                : tCycle - CATCFG_T_OVERHEAD
                ;
        }

enum    {
        CATCFG_T_INTERVAL = CATCFG_GetInterval(CATCFG_T_CYCLE),
        };

// forward reference to the command function
static cCommandStream::CommandFn cmdUpdate;

// the individual commmands are put in this table
static const cCommandStream::cEntry sMyExtraCommmands[] =
        {
        { "fallback", cmdUpdate },
        { "update", cmdUpdate },
        // other commands go here....
        };

/* a top-level structure wraps the above and connects to the system table */
/* it optionally includes a "first word" so you can for sure avoid name clashes */
static cCommandStream::cDispatch
sMyExtraCommands_top(
        sMyExtraCommmands,              /* this is the pointer to the table */
        sizeof(sMyExtraCommmands),      /* this is the size of the table */
        "system"                        /* this is the "first word" for all the commands in this table*/
        );

// forwards
static void settleDoneCb(osjob_t *pSendJob);
static void warmupDoneCb(osjob_t *pSendJob);
static void txNotProvisionedCb(osjob_t *pSendJob);
static void sleepDoneCb(osjob_t *pSendJob);
static Arduino_LoRaWAN::SendBufferCbFn sendBufferDoneCb;
static Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;

/****************************************************************************\
|
|   handy constexpr to extract the base name of a file
|
\****************************************************************************/

// two-argument version: first arg is what to return if we don't find
// a directory separator in the second part.
static constexpr const char *filebasename(const char *s, const char *p)
    {
    return p[0] == '\0'                     ? s                            :
           (p[0] == '/' || p[0] == '\\')    ? filebasename(p + 1, p + 1)   :
                                              filebasename(s, p + 1)       ;
    }

static constexpr const char *filebasename(const char *s)
    {
    return filebasename(s, s);
    }


/****************************************************************************\
|
|	READ-ONLY DATA
|
\****************************************************************************/

static const char sVersion[] = "0.1.0";

/****************************************************************************\
|
|	VARIABLES
|
\****************************************************************************/

// the primary object
Catena gCatena;

//
// the LoRaWAN backhaul.  Note that we use the
// Catena version so it can provide hardware-specific
// information to the base class.
//
Catena::LoRaWAN gLoRaWAN;

//
// the LED
//
StatusLed gLed (Catena::PIN_STATUS_LED);

//   The temperature/humidity sensor
cTemperatureSensor gTemperatureSensor {Wire};
bool fTemperatureSensor;

//   The LUX sensor
Catena_Si1133 gSi1133;
bool fLight;

//   The temperature sensor
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensor_OneWireTemp(&oneWire);
constexpr bool fMayHaveOneWireTemp = true;

/* instantiate SPI */
SPIClass gSPI2(
                Catena::PIN_SPI2_MOSI,
                Catena::PIN_SPI2_MISO,
                Catena::PIN_SPI2_SCK
                );

/* instantiate flash */
Catena_Mx25v8035f gFlash;
bool gfFlash;

/* instantiate a serial object */
cSerial<decltype(Serial)> gSerial(Serial);

/* instantiate the bootloader API */
cBootloaderApi gBootloaderApi;

/* instantiate the downloader */
cDownload gDownload;

//  USB power
bool fUsbPower;

// have we printed the sleep info?
bool g_fPrintedSleeping = false;

//  the job that's used to synchronize us with the LMIC code
static osjob_t sensorJob;
void sensorJob_cb(osjob_t *pJob);

// the cycle time to use
unsigned gTxCycle;
// remaining before we reset to default
unsigned gTxCycleCount;

// reboot time: we force a reboot once a month.
uint32_t gRebootMs;


/*

Name:	setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
        {
        gCatena.begin();

        setup_platform();
        setup_light();
        setup_temp_rh();
        setup_onewire();
        gfFlash = setup_flash();
        setup_download();
        setup_uplink();
        }

void setup_platform(void)
        {
#ifdef USBCON
        // if running unattended, don't wait for USB connect.
        if (! (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
                {
                while (!Serial)
                        /* wait for USB attach */
                        yield();
                }
#endif

        gCatena.SafePrintf("\n");
        gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("This is %s V%s.\n", filebasename(__FILE__), sVersion);
                {
                char sRegion[16];
                gCatena.SafePrintf("Target network: %s / %s\n",
                                gLoRaWAN.GetNetworkName(),
                                gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                                );
                }
        gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
        gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");

        gCatena.SafePrintf("SYSCLK: %u MHz\n", unsigned(gCatena.GetSystemClockRate() / (1000 * 1000)));

#ifdef USBCON
        gCatena.SafePrintf("USB enabled\n");
#else
        gCatena.SafePrintf("USB disabled\n");
#endif

        gLoRaWAN.SetReceiveBufferBufferCb(receiveMessage);
        setTxCycleTime(CATCFG_T_CYCLE_INITIAL, CATCFG_INTERVAL_COUNT_INITIAL);

        Catena::UniqueID_string_t CpuIDstring;

        gCatena.SafePrintf(
                "CPU Unique ID: %s\n",
                gCatena.GetUniqueIDstring(&CpuIDstring)
                );

        gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
        gCatena.SafePrintf("\n");

        // set up the LED
        gLed.begin();
        gCatena.registerObject(&gLed);
        gLed.Set(LedPattern::FastFlash);

        // set up LoRaWAN
        gCatena.SafePrintf("LoRaWAN init: ");
        if (!gLoRaWAN.begin(&gCatena))
                {
                gCatena.SafePrintf("failed\n");
                }
        else
                {
                gCatena.SafePrintf("succeeded\n");
                }

        gCatena.registerObject(&gLoRaWAN);

        /* find the platform */
        const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

        uint32_t flags;
        const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

        if (pPlatform)
                {
                gCatena.SafePrintf("EUI64: ");
                for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
                        {
                        gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
                        }
                gCatena.SafePrintf("\n");
                flags = gCatena.GetPlatformFlags();
                gCatena.SafePrintf(
                        "Platform Flags:  %#010x\n",
                        flags
                        );
                gCatena.SafePrintf(
                        "Operating Flags:  %#010x\n",
                        gCatena.GetOperatingFlags()
                        );
                }
        else
                {
                gCatena.SafePrintf("**** no platform, check provisioning ****\n");
                flags = 0;
                }
        }

void setup_light(void)
        {
        if (gSi1133.begin())
                {
                fLight = true;

                auto const measConfig =	Catena_Si1133::ChannelConfiguration_t()
                                                .setAdcMux(Catena_Si1133::InputLed_t::LargeWhite)
                                                .setSwGainCode(7)
                                                .setHwGainCode(4)
                                                .setPostShift(1)
                                                .set24bit(true);

                gSi1133.configure(0, measConfig, 0);
                }
        else
                {
                fLight = false;
                gCatena.SafePrintf("No Si1133 found: check hardware\n");
                }
        }

void setup_temp_rh(void)
        {
        if (gTemperatureSensor.begin())
                {
                fTemperatureSensor = true;
                }
        else
                {
                fTemperatureSensor = false;
                gCatena.SafePrintf("No temperature/humidity sensor found: check hardware\n");
                }
        }

void onewire_power_on(void)
        {
        pinMode(PIN_ONE_WIRE_POWER, OUTPUT);
        digitalWrite(PIN_ONE_WIRE_POWER, 1);
        delay(50);
        }

void onewire_power_off(void)
        {
        digitalWrite(PIN_ONE_WIRE_POWER, 0);
        pinMode(PIN_ONE_WIRE_POWER, INPUT);        
        }

void setup_onewire(void)
        {
	if (fMayHaveOneWireTemp)
		{
                onewire_power_on();
		sensor_OneWireTemp.begin();
		if (sensor_OneWireTemp.getDeviceCount() == 0)
			{
			gCatena.SafePrintf("No one-wire temperature sensor detected\n");
			}
		else
			{
			gCatena.SafePrintf("One-wire temperature sensor detected\n");
			}
                onewire_power_off();
		}
        }

bool setup_flash(void)
        {
        bool fFlashFound;

        gSPI2.begin();
        if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
                {
                fFlashFound = true;
		uint8_t ManufacturerId;
		uint16_t DeviceId;

		gFlash.readId(&ManufacturerId, &DeviceId);
		gCatena.SafePrintf(
			"FLASH found, ManufacturerId=%02x, DeviceId=%04x\n",
			ManufacturerId, DeviceId
			);
                gFlash.powerDown();
                gSPI2.end();
                gCatena.SafePrintf("FLASH found, put power down\n");
                }
        else
                {
                fFlashFound = false;
                gFlash.end();
                gSPI2.end();
                gCatena.SafePrintf("No FLASH found: check hardware\n");
                }

        return fFlashFound;
        }

void setup_uplink(void)
        {
        LMIC_setClockError(1*65536/100);

        /* figure out when to reboot */
        gRebootMs = (CATCFG_T_REBOOT + os_getRndU2() - 32768) * 1000;

        /* trigger a join by sending the first packet */
        if (!(gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fManufacturingTest)))
                {
                if (! gLoRaWAN.IsProvisioned())
                        gCatena.SafePrintf("LoRaWAN not provisioned yet. Use the commands to set it up.\n");
                else
                        {
                        gLed.Set(LedPattern::Joining);

                        /* trigger a join by sending the first packet */
                        startSendingUplink();
                        }
                }
        }

void setup_download()
        {
        /* add our application-specific commands */
        gCatena.addCommands(
                /* name of app dispatch table, passed by reference */
                sMyExtraCommands_top,
                /*
                || optionally a context pointer using static_cast<void *>().
                || normally only libraries (needing to be reentrant) need
                || to use the context pointer.
                */
                nullptr
                );

        gDownload.begin(gFlash, gBootloaderApi);
        }

/* process "system" "update" / "system" "fallback" -- args are ignored */
// argv[0] is "update" or "fallback"
// argv[1..argc-1] are the (ignored) arguments
static cCommandStream::CommandStatus cmdUpdate(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        )
        {
        cCommandStream::CommandStatus result;

        pThis->printf(
                "Update firmware: echo off, timeout %d seconds\n", 
                (cDownload::kTransferTimeoutMs + 500) / 1000
                );

        if (! gfFlash)
                {
                pThis->printf(
                        "** flash not found at init time, can't update **\n"
                        );
                return cCommandStream::CommandStatus::kIoError;
                }

        gSPI2.begin();
        gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS);

        struct context_t
                {
                cCommandStream *pThis;
                bool fWorking;
                cDownload::Status_t status;
                cCommandStream::CommandStatus cmdStatus;
                cDownload::Request_t request;
                };
                
        context_t context { pThis, true };

        auto doneFn =
                [](void *pUserData, cDownload::Status_t status) -> void
                        {
                        context_t * const pCtx = (context_t *)pUserData;
        
                        cCommandStream * const pThis = pCtx->pThis;
                        cCommandStream::CommandStatus cmdStatus;

                        cmdStatus = cCommandStream::CommandStatus::kSuccess;

                        if (status != cDownload::Status_t::kSuccessful)
                                {
                                pThis->printf(
                                        "download error, status %u\n",
                                        unsigned(status)
                                        );
                                cmdStatus = cCommandStream::CommandStatus::kIoError;
                                }

                        pCtx->cmdStatus = cmdStatus;
                        pCtx->fWorking = false;
                        };

        if (gDownload.evStartSerialDownload(
                argv[0][0] == 'u' ? cDownload::DownloadRq_t::GetUpdate
                                  : cDownload::DownloadRq_t::GetFallback,
                gSerial,
                context.request,
                doneFn,
                (void *) &context)
                )
                {
                while (context.fWorking)
                        gCatena.poll();

                result = context.cmdStatus;
                }
        else
                {
                pThis->printf(
                        "download launch failure\n"
                        );
                result = cCommandStream::CommandStatus::kInternalError;
                }

        gFlash.powerDown();
        gSPI2.end();

        return result;
        }

// The Arduino loop routine -- in our case, we just drive the other loops.
// If we try to do too much, we can break the LMIC radio. So the work is
// done by outcalls scheduled from the LMIC os loop.
void fillBuffer(TxBuffer_t &b);

void loop()
        {
        gCatena.poll();

        /* for mfg test, don't tx, just fill -- this causes output to Serial */
        if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fManufacturingTest))
                {
                TxBuffer_t b;
                fillBuffer(b);
                delay(1000);
                // since the light sensor was stopped in fillbuffer, restart it.
                }
        }

void fillBuffer(TxBuffer_t &b)
        {
        if (fLight)
                gSi1133.start(true);

        b.begin();
        FlagsSensorPort6 flag;

        flag = FlagsSensorPort6(0);

        // we have no format byte for this.
        uint8_t * const pFlag = b.getp();
        b.put(0x00); /* will be set to the flags */

        // vBat is sent as 5000 * v
        float vBat = gCatena.ReadVbat();
        gCatena.SafePrintf("vBat:    %d mV\n", (int) (vBat * 1000.0f));
        b.putV(vBat);
        flag |= FlagsSensorPort6::FlagVbat;

        // vBus is sent as 5000 * v
        float vBus = gCatena.ReadVbus();
        gCatena.SafePrintf("vBus:    %d mV\n", (int) (vBus * 1000.0f));
        fUsbPower = (vBus > 3.0) ? true : false;

        uint32_t bootCount;
        if (gCatena.getBootCount(bootCount))
                {
                b.putBootCountLsb(bootCount);
                flag |= FlagsSensorPort6::FlagBoot;
                }

        if (fTemperatureSensor)
                {
                cTemperatureSensor::Measurements m;
                bool fResult = gTemperatureSensor.getTemperatureHumidity(m);

                if (fResult)
                        {
                        if (Serial.dtr())
                                {
                                Serial.print("Env:  T: ");
                                Serial.print(m.Temperature);
                                Serial.print(" C, RH: ");
                                Serial.print(m.Humidity);
                                Serial.println("%");
                                }

                        // temperature is 2 bytes from -0x80.00 to +0x7F.FF degrees C
                        // humidity is 2 bytes, where 0 == 0/0xFFFF and 0xFFFFF == 1.
                        b.putT(m.Temperature);

                        // no method for 2-byte RH, direct encode it.
                        b.put2uf((m.Humidity / 100.0f) * 65535.0f);

                        flag |= FlagsSensorPort6::FlagTH;
                        }
                else
                        {
                        gCatena.SafePrintf(
                                "Env sensor not found\n"
                                );
                        }
                }

        if (fLight)
                {
                gSi1133.start(true);
                uint32_t tBegin = millis();

                while (! gSi1133.isOneTimeReady())
                        {
                        if (millis() - tBegin > 1000)
                                break;

                        yield();
                        }

                uint32_t data[1];
                gSi1133.readMultiChannelData(data, 1);
                gSi1133.stop();

                gCatena.SafePrintf(
                        "Si1133:  %u White\n",
                        data[0]
                        );
                b.putLux(LMIC_f2uflt16(data[0] / pow(2.0, 24)));

                flag |= FlagsSensorPort6::FlagLight;
                }

        b.putV(vBus);
        flag |= FlagsSensorPort6::FlagVbus;

        float oneWireTemp = 0.0f;
        if (measureOneWireTemp(oneWireTemp))
                {
                b.putT(oneWireTemp);
                flag |= FlagsSensorPort6::FlagOneWireT;
                }

        *pFlag = uint8_t(flag);
        }

void startSendingUplink(void)
        {
        TxBuffer_t b;
        LedPattern savedLed = gLed.Set(LedPattern::Measuring);

        fillBuffer(b);
        if (savedLed != LedPattern::Joining)
                gLed.Set(LedPattern::Sending);
        else
                gLed.Set(LedPattern::Joining);

        bool fConfirmed = false;
        if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
                {
                gCatena.SafePrintf("requesting confirmed tx\n");
                fConfirmed = true;
                }

        gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, NULL, fConfirmed, kUplinkPort);
        }

static void sendBufferDoneCb(
        void *pContext,
        bool fStatus
        )
        {
        osjobcb_t pFn;

        gLed.Set(LedPattern::Settling);

        pFn = settleDoneCb;
        if (! fStatus)
                {
                if (!gLoRaWAN.IsProvisioned())
                        {
                        // we'll talk about it at the callback.
                        pFn = txNotProvisionedCb;

                        // but prevent join attempts now.
                        gLoRaWAN.Shutdown();
                        }
                else
                        gCatena.SafePrintf("send buffer failed\n");
                }

        os_setTimedCallback(
                &sensorJob,
                os_getTime()+sec2osticks(CATCFG_T_SETTLE),
                pFn
                );
        }

static void txNotProvisionedCb(
        osjob_t *pSendJob
        )
        {
        gCatena.SafePrintf("LoRaWAN not provisioned yet. Use the commands to set it up.\n");
        gLoRaWAN.Shutdown();
        gLed.Set(LedPattern::NotProvisioned);
        }

static void settleDoneCb(
        osjob_t *pSendJob
        )
        {
        const bool fDeepSleep = checkDeepSleep();

        if (uint32_t(millis()) > gRebootMs)
                {
                // time to reboot
                NVIC_SystemReset();
                }

        if (! g_fPrintedSleeping)
                doSleepAlert(fDeepSleep);

        /* count what we're up to */
        updateSleepCounters();

        if (fDeepSleep)
                doDeepSleep(pSendJob);
        else
                doLightSleep(pSendJob);
        }

bool checkDeepSleep(void)
        {
        bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        bool fDeepSleep;

        if (fDeepSleepTest)
                {
                fDeepSleep = true;
                }
#ifdef USBCON
        else if (Serial.dtr())
                {
                fDeepSleep = false;
                }
#endif
        else if (gCatena.GetOperatingFlags() &
                        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
                {
                fDeepSleep = false;
                }
        else if ((gCatena.GetOperatingFlags() &
                        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
                {
                fDeepSleep = true;
                }
        else
                {
                fDeepSleep = false;
                }

        return fDeepSleep;
        }

void doSleepAlert(const bool fDeepSleep)
        {
        g_fPrintedSleeping = true;

        if (fDeepSleep)
                {
                bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
                const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

                gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                                   " (USB will disconnect while asleep)"
#endif
                                   ": ",
                                        deepSleepDelay
                                        );

                // sleep and print
                gLed.Set(LedPattern::TwoShort);

                for (auto n = deepSleepDelay; n > 0; --n)
                        {
                        uint32_t tNow = millis();

                        while (uint32_t(millis() - tNow) < 1000)
                                {
                                gCatena.poll();
                                yield();
                                }
                        gCatena.SafePrintf(".");
                        }
                gCatena.SafePrintf("\nStarting deep sleep.\n");
                uint32_t tNow = millis();
                while (uint32_t(millis() - tNow) < 100)
                        {
                        gCatena.poll();
                        yield();
                        }
                }
        else
                gCatena.SafePrintf("using light sleep\n");
        }

void updateSleepCounters(void)
        {
        // update the sleep parameters
        if (gTxCycleCount > 1)
                {
                // values greater than one are decremented and ultimately reset to default.
                --gTxCycleCount;
                }
        else if (gTxCycleCount == 1)
                {
                // it's now one (otherwise we couldn't be here.)
                gCatena.SafePrintf("resetting tx cycle to default: %u\n", CATCFG_T_CYCLE);

                gTxCycleCount = 0;
                gTxCycle = CATCFG_T_CYCLE;
                }
        else
                {
                // it's zero. Leave it alone.
                }
        }

void doDeepSleep(osjob_t *pJob)
        {
        bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        uint32_t const sleepInterval = CATCFG_GetInterval(
                        fDeepSleepTest ? CATCFG_T_CYCLE_TEST : gTxCycle
                        );

        /* ok... now it's time for a deep sleep */
        gLed.Set(LedPattern::Off);
        deepSleepPrepare();

        /* sleep */
        gCatena.Sleep(sleepInterval);

        /* recover from sleep */
        deepSleepRecovery();

        /* and now... we're awake again. trigger another measurement */
        sleepDoneCb(pJob);
        }

void deepSleepPrepare(void)
        {
        Serial.end();
        Wire.end();
        SPI.end();
        if (gfFlash)
                gSPI2.end();
        }

void deepSleepRecovery(void)
        {
        Serial.begin();
        Wire.begin();
        SPI.begin();
        if (gfFlash)
                gSPI2.begin();
        }

void doLightSleep(osjob_t *pJob)
        {
        uint32_t interval = sec2osticks(CATCFG_GetInterval(gTxCycle));

        gLed.Set(LedPattern::Sleeping);

        if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fQuickLightSleep))
                {
                interval = 1;
                }

        gLed.Set(LedPattern::Sleeping);
        os_setTimedCallback(
                &sensorJob,
                os_getTime() + interval,
                sleepDoneCb
                );
        }

static void sleepDoneCb(
        osjob_t *pJob
        )
        {
        gLed.Set(LedPattern::WarmingUp);

        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_WARMUP),
                warmupDoneCb
                );
        }

static void warmupDoneCb(
        osjob_t *pJob
        )
        {
        startSendingUplink();
        }

static void receiveMessage(
        void *pContext,
        uint8_t port,
        const uint8_t *pMessage,
        size_t nMessage
        )
        {
        unsigned txCycle;
        unsigned txCount;

        if (port == 0)
                {
                gCatena.SafePrintf("MAC message:");
                for (unsigned i = 0; i < LMIC.dataBeg; ++i)
                        {
                        gCatena.SafePrintf(" %02x", LMIC.frame[i]);
                        }
                gCatena.SafePrintf("\n");
                return;
                }

        else if (! (port == 1 && 2 <= nMessage && nMessage <= 3))
                {
                gCatena.SafePrintf("invalid message port(%02x)/length(%x)\n",
                        port, nMessage
                        );
                return;
                }

        txCycle = (pMessage[0] << 8) | pMessage[1];

        if (txCycle < CATCFG_T_MIN || txCycle > CATCFG_T_MAX)
                {
                gCatena.SafePrintf("tx cycle time out of range: %u\n", txCycle);
                return;
                }

        // byte [2], if present, is the repeat count.
        // explicitly sending zero causes it to stick.
        txCount = CATCFG_INTERVAL_COUNT;
        if (nMessage >= 3)
                {
                txCount = pMessage[2];
                }

        setTxCycleTime(txCycle, txCount);
        }

void setTxCycleTime(
        unsigned txCycle,
        unsigned txCount
        )
        {
        if (txCount > 0)
                gCatena.SafePrintf(
                        "message cycle time %u seconds for %u messages\n",
                        txCycle, txCount
                        );
        else
                gCatena.SafePrintf(
                        "message cycle time %u seconds indefinitely\n",
                        txCycle
                        );

        gTxCycle = txCycle;
        gTxCycleCount = txCount;
        }

static bool checkOneWireSensorPresent(void)
	{
	// this is unpleasant. But the way to deal with plugging is to call
	// begin again.
	if (fMayHaveOneWireTemp)
		{
		sensor_OneWireTemp.begin();
		return sensor_OneWireTemp.getDeviceCount() != 0;
		}
	else
		return false;
	}

static bool measureOneWireTemp(float &tempDegreesC)
        {
	/*
	|| Measure and transmit the OneWire
	|| tranducer value. This is complicated because we want
	|| to support plug/unplug and the sw interface is not
	|| really hot-pluggable.
	*/
        onewire_power_on();

	bool fOneWireTemp = checkOneWireSensorPresent();

	if (fOneWireTemp)
		{
		sensor_OneWireTemp.requestTemperatures();
		float oneWireTempC = sensor_OneWireTemp.getTempCByIndex(0);
                if (Serial.dtr())
                        {
                        Serial.print("Probe temperature: "); Serial.print(oneWireTempC); Serial.println(" C");
                        }
                tempDegreesC = oneWireTempC;
                return true;
		}
	else if (fMayHaveOneWireTemp)
		{
		gCatena.SafePrintf("No probe attached\n");
                return false;
		}

        onewire_power_off();
        }
