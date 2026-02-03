#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>

#include "app/App.h"
#include "drivers/LoraRadioSx1262.h"
#include "services/TrackerService.h"

#ifndef RADIOLIB_ERR_UNSUPPORTED
#define RADIOLIB_ERR_UNSUPPORTED RADIOLIB_ERR_UNKNOWN
#endif

#ifndef RADIOLIB_LORAWAN_NONCES_BUF_SIZE
#define RADIOLIB_LORAWAN_NONCES_BUF_SIZE 64
#endif

class LoraWanService {
public:
	LoraWanService();

	bool initRadio(AppState& state);
	bool loadSession(LoRaWANSchemeSession_t& session);
	bool saveSession(const LoRaWANSchemeSession_t& session);
	void clearSession();

	bool restoreSession(AppState& state, const LoRaWANSchemeSession_t& session);
	bool fetchSession(AppState& state, LoRaWANSchemeSession_t& session);
	int16_t sendUplink(const uint8_t* payload, size_t len, uint8_t fport);

	void resetJoinTracking();
	void updateJoinFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv);
	void updateUplinkFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv);

private:
	struct StoredSession {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		LoRaWANSchemeSession_t session;
	};

	struct StoredNonces {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		uint8_t nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
	};

	static bool isSessionInvalidError(int16_t err);
	static bool isUplinkOkError(int16_t err);
	static bool loadNoncesToNode(AppState& state, LoRaWANNode& node);
	static bool saveNoncesFromNode(AppState& state, LoRaWANNode& node);
	static void clearNonces();

	template <typename NodeT, typename SessionT>
	static auto setNodeSession(NodeT& target, const SessionT& session, int)
			-> decltype(target.setSession(&session), int16_t()) {
		return target.setSession(&session);
	}

	template <typename NodeT, typename SessionT>
	static auto setNodeSession(NodeT& target, const SessionT& session, long)
			-> decltype(target.setSession(session), int16_t()) {
		return target.setSession(session);
	}

	template <typename NodeT, typename SessionT>
	static int16_t setNodeSession(NodeT&, const SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	template <typename NodeT, typename SessionT>
	static auto getNodeSession(NodeT& target, SessionT& session, int)
			-> decltype(target.getSession(&session), int16_t()) {
		return target.getSession(&session);
	}

	template <typename NodeT, typename SessionT>
	static auto getNodeSession(NodeT& target, SessionT& session, long)
			-> decltype(target.getSession(session), int16_t()) {
		return target.getSession(session);
	}

	template <typename NodeT, typename SessionT>
	static int16_t getNodeSession(NodeT&, SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	static constexpr uint32_t SESSION_MAGIC = 0x4C57534Eu; // "LWSN"
	static constexpr uint16_t SESSION_VERSION = 1;
	static constexpr uint32_t NONCES_MAGIC = 0x4E4F4E43u;  // "NONC"
	static constexpr uint16_t NONCES_VERSION = 1;

	LoraRadioSx1262 radio_;
	LoRaWANNode node_;
	bool otaaInited_ = false;
	bool noncesLoaded_ = false;
	JoinState prevJoinState_ = JoinState::Fail;
};
