#pragma once

#include <Arduino.h>

// Hoofdtoestand van de applicatiegate.
enum class GateState : uint8_t {
	WaitingForBoot,
	Running
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
	// Schakel WiFi/Bluetooth uit om energie te besparen.
	void disableRadios();
	// Zet join-gerelateerde state terug naar defaults.
	void resetJoinState();
	// Dwing een OTAA reset en herstart van het join-proces af.
	void forceOtaaReset();

	AppState state_;
};
