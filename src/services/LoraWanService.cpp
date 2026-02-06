#include "services/LoraWanService.h"

#include <Arduino.h>

#include "config/AppConfig.h"
#include "secrets.h"

#ifndef RADIOLIB_LORAWAN_NONCES_BUF_SIZE
#define RADIOLIB_LORAWAN_NONCES_BUF_SIZE 64
#endif

LoraWanService::LoraWanService()
		: node_(&radio_.radio(), &EU868) {
}

namespace {
const char* joinStateLabel(JoinState state) {
	switch (state) {
		case JoinState::Radio: return "RADIO";
		case JoinState::Restore: return "RESTORE";
		case JoinState::TestUplink: return "TEST_UP";
		case JoinState::Join: return "JOIN";
		case JoinState::Ok: return "OK";
		case JoinState::Fail: return "FAIL";
		default: return "?";
	}
}
} // namespace

bool LoraWanService::loadSession(LoRaWANSchemeSession_t& session) {
	Preferences prefs;
	if (!prefs.begin("lorawan", true)) {
		Serial.println("[lorawan] session load: prefs begin failed");
		return false;
	}
	if (!prefs.isKey("session")) {
		Serial.println("[lorawan] session load: no session key");
		prefs.end();
		return false;
	}
	size_t len = prefs.getBytesLength("session");
	if (len != sizeof(StoredSession)) {
		Serial.printf("[lorawan] session load: size mismatch len=%u expected=%u\n",
					  static_cast<unsigned>(len),
					  static_cast<unsigned>(sizeof(StoredSession)));
		prefs.end();
		return false;
	}
	StoredSession stored{};
	size_t read = prefs.getBytes("session", &stored, sizeof(stored));
	prefs.end();
	if (read != sizeof(stored)) {
		Serial.printf("[lorawan] session load: read mismatch read=%u\n", static_cast<unsigned>(read));
		return false;
	}
	if (stored.magic != SESSION_MAGIC) {
		Serial.println("[lorawan] session load: magic mismatch");
		return false;
	}
	if (stored.version != SESSION_VERSION) {
		Serial.printf("[lorawan] session load: version mismatch %u\n", static_cast<unsigned>(stored.version));
		return false;
	}
	if (stored.size != sizeof(LoRaWANSchemeSession_t)) {
		Serial.printf("[lorawan] session load: payload size mismatch %u\n", static_cast<unsigned>(stored.size));
		return false;
	}
	session = stored.session;
	Serial.println("[lorawan] session load: ok");
	return true;
}

bool LoraWanService::saveSession(const LoRaWANSchemeSession_t& session) {
	Preferences prefs;
	if (!prefs.begin("lorawan", false)) {
		Serial.println("[lorawan] session save: prefs begin failed");
		return false;
	}
	StoredSession stored{};
	stored.magic = SESSION_MAGIC;
	stored.version = SESSION_VERSION;
	stored.size = sizeof(LoRaWANSchemeSession_t);
	stored.session = session;
	size_t written = prefs.putBytes("session", &stored, sizeof(stored));
	prefs.end();
	Serial.printf("[lorawan] session save: %s\n", written == sizeof(stored) ? "ok" : "failed");
	return written == sizeof(stored);
}

void LoraWanService::clearSession() {
	Preferences prefs;
	if (!prefs.begin("lorawan", false)) {
		Serial.println("[lorawan] session clear: prefs begin failed");
		return;
	}
	bool removed = prefs.remove("session");
	prefs.end();
	Serial.printf("[lorawan] session clear: %s\n", removed ? "ok" : "missing");
}

bool LoraWanService::initRadio(AppState& state) {
	int16_t lastErr = RADIOLIB_ERR_NONE;
	if (!radio_.begin(AppConfig::RADIO_TCXO_VOLTAGE, lastErr)) {
		state.lastLoraErr = lastErr;
		return false;
	}
	state.lastLoraErr = RADIOLIB_ERR_NONE;
	return true;
}

bool LoraWanService::restoreSession(AppState& state, const LoRaWANSchemeSession_t& session) {
	int16_t st = setNodeSession(node_, session, 0);
	if (st != RADIOLIB_ERR_NONE) {
		Serial.printf("[lorawan] session restore: failed err=%d\n", st);
		state.lastLoraErr = st;
		return false;
	}
	Serial.println("[lorawan] session restore: ok");
	state.lastLoraErr = RADIOLIB_ERR_NONE;
	return true;
}

bool LoraWanService::fetchSession(AppState& state, LoRaWANSchemeSession_t& session) {
	int16_t st = getNodeSession(node_, session, 0);
	if (st != RADIOLIB_ERR_NONE) {
		Serial.printf("[lorawan] session fetch: failed err=%d\n", st);
		state.lastLoraErr = st;
		return false;
	}
	Serial.println("[lorawan] session fetch: ok");
	state.lastLoraErr = RADIOLIB_ERR_NONE;
	return true;
}

int16_t LoraWanService::sendUplink(const uint8_t* payload, size_t len, uint8_t fport) {
	return node_.sendReceive(payload, len, fport);
}

bool LoraWanService::isSessionInvalidError(int16_t err) {
	switch (err) {
#ifdef RADIOLIB_ERR_MIC_MISMATCH
		case RADIOLIB_ERR_MIC_MISMATCH:
#endif
#ifdef RADIOLIB_ERR_INVALID_FCNT
		case RADIOLIB_ERR_INVALID_FCNT:
#endif
#ifdef RADIOLIB_ERR_DOWNLINK_MALFORMED
		case RADIOLIB_ERR_DOWNLINK_MALFORMED:
#endif
			return true;
		default:
			return false;
	}
}

bool LoraWanService::isUplinkOkError(int16_t err) {
	if (err == RADIOLIB_ERR_NONE) return true;
#ifdef RADIOLIB_ERR_RX_TIMEOUT
	if (err == RADIOLIB_ERR_RX_TIMEOUT) return true;
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
	if (err == RADIOLIB_ERR_TIMEOUT) return true;
#endif
	return false;
}

bool LoraWanService::loadNoncesToNode(AppState& state, LoRaWANNode& node) {
	Preferences prefs;
	if (!prefs.begin("lorawan", true)) {
		Serial.println("[lorawan] nonces load: prefs begin failed");
		return false;
	}
	if (!prefs.isKey("nonces")) {
		Serial.println("[lorawan] nonces load: no nonces key");
		prefs.end();
		return false;
	}

	size_t len = prefs.getBytesLength("nonces");
	if (len != sizeof(StoredNonces)) {
		Serial.printf("[lorawan] nonces load: size mismatch len=%u expected=%u\n",
					  static_cast<unsigned>(len),
					  static_cast<unsigned>(sizeof(StoredNonces)));
		prefs.end();
		return false;
	}

	StoredNonces stored{};
	size_t read = prefs.getBytes("nonces", &stored, sizeof(stored));
	prefs.end();

	if (read != sizeof(stored)) return false;
	if (stored.magic != NONCES_MAGIC) {
		Serial.println("[lorawan] nonces load: magic mismatch");
		return false;
	}
	if (stored.version != NONCES_VERSION) {
		Serial.printf("[lorawan] nonces load: version mismatch %u\n", static_cast<unsigned>(stored.version));
		return false;
	}
	if (stored.size != RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
		Serial.printf("[lorawan] nonces load: payload size mismatch %u\n", static_cast<unsigned>(stored.size));
		return false;
	}

	int16_t st = node.setBufferNonces(stored.nonces);
	if (st != RADIOLIB_ERR_NONE) {
		Serial.printf("[lorawan] nonces load: node set failed err=%d\n", st);
		state.lastLoraErr = st;
		return false;
	}

	state.lastLoraErr = RADIOLIB_ERR_NONE;
	Serial.println("[lorawan] nonces load: ok");
	return true;
}

bool LoraWanService::saveNoncesFromNode(AppState& state, LoRaWANNode& node) {
	StoredNonces stored{};
	stored.magic = NONCES_MAGIC;
	stored.version = NONCES_VERSION;
	stored.size = RADIOLIB_LORAWAN_NONCES_BUF_SIZE;

	uint8_t* p = node.getBufferNonces();
	if (!p) {
		Serial.println("[lorawan] nonces save: node buffer missing");
		state.lastLoraErr = RADIOLIB_ERR_UNKNOWN;
		return false;
	}

	memcpy(stored.nonces, p, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

	Preferences prefs;
	if (!prefs.begin("lorawan", false)) {
		Serial.println("[lorawan] nonces save: prefs begin failed");
		return false;
	}
	size_t written = prefs.putBytes("nonces", &stored, sizeof(stored));
	prefs.end();

	state.lastLoraErr = RADIOLIB_ERR_NONE;
	Serial.printf("[lorawan] nonces save: %s\n", written == sizeof(stored) ? "ok" : "failed");
	return written == sizeof(stored);
}

void LoraWanService::clearNonces() {
	Preferences prefs;
	if (!prefs.begin("lorawan", false)) {
		Serial.println("[lorawan] nonces clear: prefs begin failed");
		return;
	}
	bool removed = prefs.remove("nonces");
	prefs.end();
	Serial.printf("[lorawan] nonces clear: %s\n", removed ? "ok" : "missing");
}

void LoraWanService::resetJoinTracking() {
	otaaInited_ = false;
	noncesLoaded_ = false;
	prevJoinState_ = JoinState::Fail;
}

bool LoraWanService::ensureOtaaInited(AppState& state) {
	if (otaaInited_) return true;
	int16_t st = node_.beginOTAA(JOIN_EUI, DEV_EUI, NWK_KEY, APP_KEY);
	if (st != RADIOLIB_ERR_NONE) {
		Serial.printf("[lorawan] otaa init failed err=%d\n", st);
		state.lastLoraErr = st;
		return false;
	}
	otaaInited_ = true;
	Serial.println("[lorawan] otaa init ok");
	return true;
}

void LoraWanService::updateJoinFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv) {
	if (state.gate != GateState::Running) return;

	uint32_t now = millis();

	if (state.joinState == JoinState::Join && prevJoinState_ != JoinState::Join) {
		otaaInited_ = false;
		noncesLoaded_ = false;
		Serial.printf("[lorawan] entering %s state\n", joinStateLabel(state.joinState));
	}
	prevJoinState_ = state.joinState;

	switch (state.joinState) {
		case JoinState::Radio: {
			if (now < state.nextActionMs) return;
			state.radioReady = initRadio(state);
			if (state.radioReady) {
				Serial.println("[lorawan] radio init ok");
				state.joinState = JoinState::Restore;
				state.nextActionMs = now;
			} else {
				state.radioInitAttempts++;
				Serial.printf("[lorawan] radio init fail (%u/%u) err=%d\n",
							  static_cast<unsigned>(state.radioInitAttempts),
							  static_cast<unsigned>(AppConfig::RADIO_INIT_MAX_ATTEMPTS),
							  state.lastLoraErr);
				if (state.radioInitAttempts >= AppConfig::RADIO_INIT_MAX_ATTEMPTS) {
					state.joinState = JoinState::Fail;
				} else {
					state.joinState = JoinState::Radio;
					state.nextActionMs = now + AppConfig::JOIN_RETRY_MS;
				}
			}
			break;
		}

		case JoinState::Restore: {
			LoRaWANSchemeSession_t session;
			if (loadSession(session) && ensureOtaaInited(state) && restoreSession(state, session)) {
				Serial.println("[lorawan] session restore ok, test uplink");
				state.sessionRestored = true;
				state.testUplinkBackoff = 0;
				state.joinState = JoinState::TestUplink;
				state.nextActionMs = now + AppConfig::RESTORE_TEST_DELAY_MS;
			} else {
				clearNonces();
				Serial.println("[lorawan] no session restore, start join (OTAA)");
				state.joinState = JoinState::Join;
				state.nextActionMs = now;
			}
			break;
		}

		case JoinState::TestUplink: {
			if (now < state.nextActionMs) return;

			uint8_t payload[24];
			size_t n = tracker.buildPayload(payload, sizeof(payload), fix, battMv);

			int16_t st = sendUplink(payload, n, AppConfig::LORAWAN_FPORT);
			state.lastLoraErr = st;
			state.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;
			Serial.printf("[lorawan] test uplink result=%s err=%d\n",
						  (state.uplinkResult == UplinkResult::Ok) ? "OK" : "FAIL",
						  st);

			if (state.uplinkResult == UplinkResult::Ok) {
				state.joined = true;
				state.joinState = JoinState::Ok;
				state.testUplinkBackoff = 0;

				LoRaWANSchemeSession_t saved;
				if (fetchSession(state, saved)) {
					(void)saveSession(saved);
				}

				state.nextUplinkMs = now + tracker.nextUplinkIntervalMs(fix);
			} else if (isSessionInvalidError(st)) {
				Serial.println("[lorawan] test uplink invalid session, restarting join");
				clearSession();
				clearNonces();
				state.sessionRestored = false;

				state.joinState = JoinState::Join;
				state.nextActionMs = now;
			} else {
				if (state.testUplinkBackoff < 5) state.testUplinkBackoff++;
				uint32_t backoffMs = AppConfig::JOIN_RETRY_MS + static_cast<uint32_t>(state.testUplinkBackoff) * 10000;
				state.joinState = JoinState::TestUplink;
				state.nextActionMs = now + backoffMs;
			}
			break;
		}

		case JoinState::Join: {
			if (now < state.nextActionMs) return;

			if (state.joinAttempts >= AppConfig::JOIN_MAX_ATTEMPTS) {
				state.joinState = JoinState::Fail;
				Serial.println("[lorawan] join failed: max attempts reached");
				return;
			}

			if (!ensureOtaaInited(state)) {
				Serial.println("[lorawan] join waiting for otaa init");
				state.nextActionMs = now + AppConfig::JOIN_RETRY_MS;
				return;
			}

			if (!noncesLoaded_) {
				if (!loadNoncesToNode(state, node_)) {
					Serial.println("[lorawan] join nonces missing, continuing with fresh OTAA");
				}
			}
			noncesLoaded_ = true;

			state.joinAttempts++;
			Serial.printf("[lorawan] join attempt %u/%u\n",
						  static_cast<unsigned>(state.joinAttempts),
						  static_cast<unsigned>(AppConfig::JOIN_MAX_ATTEMPTS));

			int16_t st = node_.activateOTAA();
			(void)saveNoncesFromNode(state, node_);

			if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_LORAWAN_NEW_SESSION) {
				state.lastLoraErr = st;
				Serial.printf("[lorawan] join attempt failed err=%d\n", st);
				state.nextActionMs = now + AppConfig::JOIN_RETRY_MS;
				return;
			}

			state.lastLoraErr = RADIOLIB_ERR_NONE;
			state.joined = true;
			state.joinState = JoinState::Ok;
			Serial.println("[lorawan] join ok");
			state.testUplinkBackoff = 0;

			LoRaWANSchemeSession_t session;
			if (fetchSession(state, session)) {
				(void)saveSession(session);
			}

			state.nextUplinkMs = now + tracker.nextUplinkIntervalMs(fix);
			break;
		}

		case JoinState::Ok:
		case JoinState::Fail:
		default:
			break;
	}
}

void LoraWanService::updateUplinkFlow(AppState& state, const CurrentFix& fix, TrackerService& tracker, uint16_t battMv) {
	if (!state.joined) return;

	uint32_t now = millis();
	if (state.nextUplinkMs == 0 || now < state.nextUplinkMs) return;

	uint8_t payload[24];
	size_t n = tracker.buildPayload(payload, sizeof(payload), fix, battMv);

	int16_t st = sendUplink(payload, n, AppConfig::LORAWAN_FPORT);
	state.lastLoraErr = st;
	state.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;

	if (state.uplinkResult == UplinkResult::Ok) {
		state.lastUplinkOkMs = now;
		LoRaWANSchemeSession_t session;
		if (fetchSession(state, session)) {
			(void)saveSession(session);
		}
	} else if (isSessionInvalidError(st)) {
		Serial.println("[lorawan] uplink invalid session, restarting join");
		clearSession();
		clearNonces();
		state.joined = false;
		state.sessionRestored = false;
		state.joinState = JoinState::Join;
		state.nextActionMs = now + AppConfig::JOIN_RETRY_MS;
	}

	state.nextUplinkMs = now + tracker.nextUplinkIntervalMs(fix);
}
