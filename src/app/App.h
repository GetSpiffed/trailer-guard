#pragma once

#include <Arduino.h>

#include "drivers/GpsL76k.h"
#include "services/FixSnapshot.h"

// Hoofdtoestand van de applicatiegate.
enum class GateState : uint8_t {
	WaitingForBoot,
	Running
};

// Globale boot/wakeup-indicatie voor startupgedrag.
enum class BootWakeCause : uint8_t {
	ColdBoot,
	DeepSleepWake
};

// Hoog-niveau startupmodus voor het openen van de gate.
enum class StartupMode : uint8_t {
	ManualStart,
	AutoStart
};

// Fase van het LoRaWAN join-proces.
enum class JoinState : uint8_t {
	Radio,
	Restore,
	TestUplink,
	Join,
	Ok,
	Fail
};

// Resultaatstaat van de laatste uplink.
enum class UplinkResult : uint8_t {
	Idle,
	Ok,
	Fail
};

// Samengevatte runtime-state van de applicatie.
struct AppState {
	GateState gate = GateState::WaitingForBoot;
	BootWakeCause wakeCause = BootWakeCause::ColdBoot;
	StartupMode startupMode = StartupMode::ManualStart;
	uint8_t resetReason = 0;
	JoinState joinState = JoinState::Radio;
	UplinkResult uplinkResult = UplinkResult::Idle;
	int16_t lastLoraErr = 0;
	uint32_t nextActionMs = 0;
	uint32_t nextUplinkMs = 0;
	uint32_t lastUplinkOkMs = 0;
	uint8_t joinAttempts = 0;
	uint8_t radioInitAttempts = 0;
	bool radioReady = false;
	bool joined = false;
	bool sessionRestored = false;
	uint8_t testUplinkBackoff = 0;
	bool forceOtaaThisBoot = false;
	bool inputReady = false;
	bool initDone = false;
};

// Hoofdapplicatie die alle services aanstuurt.
class App {
public:
	// Initialiseer hardware en services.
	void begin();
	// Voer de periodieke applicatielus uit.
	void tick();

private:
	void fixSnapshotUpdate(const CurrentFix& fix, uint32_t nowMs);
	// Schakel WiFi/Bluetooth uit om energie te besparen.
	void disableRadios();
	// Zet join-gerelateerde state terug naar defaults.
	void resetJoinState();
	// Dwing een OTAA reset en herstart van het join-proces af.
	void forceOtaaReset();
	// Start het netwerkpad en open de gate.
	void startNetwork();
	// Detecteer boot/wakeup-informatie.
	void detectBootInfo();
	// Bepaal de startupmodus op basis van de bootinformatie.
	static StartupMode determineStartupMode(BootWakeCause cause);

	AppState state_;
	FixSnapshot fixSnapshot_;
};
