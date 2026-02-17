#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <array>

#include "app/App.h"
#include "drivers/LoraRadioSx1262.h"

#ifndef RADIOLIB_ERR_UNSUPPORTED
#define RADIOLIB_ERR_UNSUPPORTED RADIOLIB_ERR_UNKNOWN
#endif

#ifndef RADIOLIB_LORAWAN_NONCES_BUF_SIZE
#define RADIOLIB_LORAWAN_NONCES_BUF_SIZE 64
#endif

// Service die LoRaWAN join-, sessie- en uplinklogica beheert.
class LoraWanService {
public:
	using SessionBuffer = std::array<uint8_t, RADIOLIB_LORAWAN_SESSION_BUF_SIZE>;

	// Maak de service aan en configureer de node.
	LoraWanService();

	// Initialiseer de radiohardware en update status.
	bool initRadio(AppState& state);
	// Laad een opgeslagen LoRaWAN-sessie uit NVS.
	bool loadSession(SessionBuffer& session);
	// Sla een LoRaWAN-sessie op in NVS.
	bool saveSession(const SessionBuffer& session);
	// Wis een opgeslagen LoRaWAN-sessie.
	void clearSession();

	// Herstel een sessie in de node en update state.
	bool restoreSession(AppState& state, const SessionBuffer& session);
	// Lees de actuele sessie uit de node.
	bool fetchSession(AppState& state, SessionBuffer& session);
	// Verzend een uplinkpayload via de LoRaWAN-node.
	int16_t sendUplink(const uint8_t* payload, size_t len, uint8_t fport);

	// Reset join-gerelateerde interne tracking.
	void resetJoinTracking();
	// Verwerk het joinproces.
	void updateJoinFlow(AppState& state);

private:
	// Gestructureerde opslag voor een LoRaWAN-sessie.
	struct StoredSession {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		uint8_t session[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
	};

	// Gestructureerde opslag voor LoRaWAN nonces.
	struct StoredNonces {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		uint8_t nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
	};

	// Laad opgeslagen nonces naar de nodebuffer.
	static bool loadNoncesToNode(AppState& state, LoRaWANNode& node);
	// Sla huidige nonces uit de nodebuffer op.
	static bool saveNoncesFromNode(AppState& state, LoRaWANNode& node);
	// Wis opgeslagen nonces uit NVS.
	static void clearNonces();
	// Zorg dat de OTAA-configuratie is geladen zodat sessieherstel werkt.
	bool ensureOtaaInited(AppState& state);

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
