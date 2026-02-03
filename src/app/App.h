#pragma once

#include <Arduino.h>

enum class GateState : uint8_t {
	WaitingForBoot,
	Running
};

enum class JoinState : uint8_t {
	Radio,
	Restore,
	TestUplink,
	Join,
	Ok,
	Fail
};

enum class UplinkResult : uint8_t {
	Idle,
	Ok,
	Fail
};

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

class App {
public:
	void begin();
	void tick();

private:
	void disableRadios();
	void resetJoinState();
	void forceOtaaReset();

	AppState state_;
};
