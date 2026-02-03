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

// Service die LoRaWAN join-, sessie- en uplinklogica beheert.
class LoraWanService {
public:
	// Maak de service aan en configureer de node.
	LoraWanService();

	// Initialiseer de radiohardware en update status.
	bool initRadio(AppState& state);
	// Laad een opgeslagen LoRaWAN-sessie uit NVS.
	bool loadSession(LoRaWANSchemeSession_t& session);
	// Sla een LoRaWAN-sessie op in NVS.
	bool saveSession(const LoRaWANSchemeSession_t& session);
	// Wis een opgeslagen LoRaWAN-sessie.
	void clearSession();

	// Herstel een sessie in de node en update state.
	bool restoreSession(AppState& state, const LoRaWANSchemeSession_t& session);
	// Lees de actuele sessie uit de node.
	bool fetchSession(AppState& state, LoRaWANSchemeSession_t& session);
	// Verzend een uplinkpayload via de LoRaWAN-node.
	int16_t sendUplink(const uint8_t* payload, size_t len, uint8_t fport);

	// Reset join-gerelateerde interne tracking.
	void resetJoinTracking();
	// Verwerk het joinproces en test-uplinks.
	void updateJoinFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv);
	// Verwerk periodieke uplinks na een succesvolle join.
	void updateUplinkFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv);

private:
	// Gestructureerde opslag voor een LoRaWAN-sessie.
	struct StoredSession {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		LoRaWANSchemeSession_t session;
	};

	// Gestructureerde opslag voor LoRaWAN nonces.
	struct StoredNonces {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		uint8_t nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
	};

	// Controleer of een foutcode duidt op een ongeldige sessie.
	static bool isSessionInvalidError(int16_t err);
	// Controleer of een uplinkresultaat functioneel OK is.
	static bool isUplinkOkError(int16_t err);
	// Laad opgeslagen nonces naar de nodebuffer.
	static bool loadNoncesToNode(AppState& state, LoRaWANNode& node);
	// Sla huidige nonces uit de nodebuffer op.
	static bool saveNoncesFromNode(AppState& state, LoRaWANNode& node);
	// Wis opgeslagen nonces uit NVS.
	static void clearNonces();

	template <typename NodeT, typename SessionT>
	// Zet sessie op de node via pointer-overload indien beschikbaar.
	static auto setNodeSession(NodeT& target, const SessionT& session, int)
			-> decltype(target.setSession(&session), int16_t()) {
		return target.setSession(&session);
	}

	template <typename NodeT, typename SessionT>
	// Zet sessie op de node via value-overload indien beschikbaar.
	static auto setNodeSession(NodeT& target, const SessionT& session, long)
			-> decltype(target.setSession(session), int16_t()) {
		return target.setSession(session);
	}

	template <typename NodeT, typename SessionT>
	// Fallback voor setSession wanneer niet ondersteund.
	static int16_t setNodeSession(NodeT&, const SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	template <typename NodeT, typename SessionT>
	// Lees sessie uit de node via pointer-overload indien beschikbaar.
	static auto getNodeSession(NodeT& target, SessionT& session, int)
			-> decltype(target.getSession(&session), int16_t()) {
		return target.getSession(&session);
	}

	template <typename NodeT, typename SessionT>
	// Lees sessie uit de node via value-overload indien beschikbaar.
	static auto getNodeSession(NodeT& target, SessionT& session, long)
			-> decltype(target.getSession(session), int16_t()) {
		return target.getSession(session);
	}

	template <typename NodeT, typename SessionT>
	// Fallback voor getSession wanneer niet ondersteund.
	static int16_t getNodeSession(NodeT&, SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	// Magic om sessiedata in opslag te herkennen.
	static constexpr uint32_t SESSION_MAGIC = 0x4C57534Eu; // "LWSN"
	// Versie van het sessie-opslagformaat.
	static constexpr uint16_t SESSION_VERSION = 1;
	// Magic om nonce-data in opslag te herkennen.
	static constexpr uint32_t NONCES_MAGIC = 0x4E4F4E43u;  // "NONC"
	// Versie van het nonce-opslagformaat.
	static constexpr uint16_t NONCES_VERSION = 1;

	LoraRadioSx1262 radio_;
	LoRaWANNode node_;
	bool otaaInited_ = false;
	bool noncesLoaded_ = false;
	JoinState prevJoinState_ = JoinState::Fail;
};
