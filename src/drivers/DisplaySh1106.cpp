#include "drivers/DisplaySh1106.h"

#include <RadioLib.h>

#include "config/AppConfig.h"
#include "config/BoardPins.h"

DisplaySh1106::DisplaySh1106()
		: u8g2_(U8G2_R0, BoardPins::DEV_SCL, BoardPins::DEV_SDA, U8X8_PIN_NONE) {
}

void DisplaySh1106::begin() {
	u8g2_.begin();
	u8g2_.setFont(u8g2_font_6x10_tf);
}

const char* DisplaySh1106::joinStateText(JoinState state) {
	switch (state) {
		case JoinState::Radio: return "RADIO";
		case JoinState::Restore: return "RESTORE";
		case JoinState::TestUplink: return "TESTUP";
		case JoinState::Join: return "JOIN";
		case JoinState::Ok: return "OK";
		case JoinState::Fail: return "FAIL";
		default: return "?";
	}
}

const char* DisplaySh1106::loraErrLabel(int16_t err, bool uplinkOk) {
	if (uplinkOk) return "OK";
	switch (err) {
		case RADIOLIB_ERR_NONE: return "NONE";
#ifdef RADIOLIB_ERR_JOIN_FAILED
		case RADIOLIB_ERR_JOIN_FAILED: return "JOIN_FAILED";
#endif
#ifdef RADIOLIB_ERR_NO_JOIN_ACCEPT
		case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "NO_ACCEPT";
#endif
#ifdef RADIOLIB_ERR_RX_TIMEOUT
		case RADIOLIB_ERR_RX_TIMEOUT: return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
		case RADIOLIB_ERR_TIMEOUT: return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_MIC_MISMATCH
		case RADIOLIB_ERR_MIC_MISMATCH: return "MIC_MISMATCH";
#endif
#ifdef RADIOLIB_ERR_INVALID_FCNT
		case RADIOLIB_ERR_INVALID_FCNT: return "INVALID_FCNT";
#endif
#ifdef RADIOLIB_ERR_DOWNLINK_MALFORMED
		case RADIOLIB_ERR_DOWNLINK_MALFORMED: return "DOWNLINK_BAD";
#endif
		default: return "OTHER";
	}
}

void DisplaySh1106::acceptScrollText(ScrollText& st, const char* newText, U8G2& u8g2,
										 const char* sep, uint32_t minUpdateMs) {
	if (!newText) newText = "";

	if (strncmp(st.buf, newText, sizeof(st.buf)) == 0) return;

	uint32_t now = millis();
	if (st.lastAcceptMs != 0 && (now - st.lastAcceptMs) < minUpdateMs) {
		return;
	}

	strncpy(st.buf, newText, sizeof(st.buf) - 1);
	st.buf[sizeof(st.buf) - 1] = 0;
	st.lastAcceptMs = now;

	st.textW = u8g2.getStrWidth(st.buf);
	st.sepW = u8g2.getStrWidth(sep);
	int16_t screenW = u8g2.getDisplayWidth();
	bool scrollingNow = (st.textW > screenW);

	if (!st.scrolling && scrollingNow) {
		st.x = 0;
		st.lastStepMs = now;
	}

	st.scrolling = scrollingNow;
}

void DisplaySh1106::drawMarqueeLoop(U8G2& u8g2, int y, ScrollText& st,
								 const char* sep, uint8_t speedPx, uint16_t stepMs) {
	const char* text = st.buf;
	if (!text || !text[0]) return;

	const int16_t screenW = u8g2.getDisplayWidth();
	const int16_t textW = st.textW > 0 ? st.textW : u8g2.getStrWidth(text);

	if (textW <= screenW) {
		st.x = 0;
		u8g2.drawStr(0, y, text);
		return;
	}

	const int16_t sepW = st.sepW > 0 ? st.sepW : u8g2.getStrWidth(sep);
	const int16_t periodW = textW + sepW;
	uint32_t now = millis();

	if (now - st.lastStepMs >= stepMs) {
		st.lastStepMs = now;
		st.x -= static_cast<int16_t>(speedPx);

		if (st.x <= -periodW) {
			st.x += periodW;
		}
	}

	int16_t x0 = st.x;

	u8g2.drawStr(x0, y, text);
	u8g2.drawStr(x0 + textW, y, sep);
	u8g2.drawStr(x0 + periodW, y, text);
	u8g2.drawStr(x0 + periodW + textW, y, sep);
}

void DisplaySh1106::showBootHold(uint32_t heldMs) {
	u8g2_.clearBuffer();
	u8g2_.setFont(u8g2_font_6x10_tf);

	const char* statusLine = nullptr;
	if (heldMs < AppConfig::FORCE_OTAA_MIN_MS) {
		statusLine = "RELEASE: JOIN (short)";
	} else if (heldMs <= AppConfig::FORCE_OTAA_MAX_MS) {
		statusLine = "FORCE OTAA: READY";
	} else {
		statusLine = "FORCE OTAA: TOO LONG";
	}

	acceptScrollText(holdScroll_, statusLine, u8g2_, " - ", 120);
	drawMarqueeLoop(u8g2_, 12, holdScroll_, " <<< ", 3, 40);

	char line1[32];
	snprintf(line1, sizeof(line1), "HOLD %lums", static_cast<unsigned long>(heldMs));
	u8g2_.drawStr(0, 34, line1);

	const uint16_t cap = AppConfig::FORCE_OTAA_MAX_MS;
	uint16_t w = (heldMs >= cap) ? 128 : static_cast<uint16_t>((heldMs * 128UL) / cap);
	u8g2_.drawFrame(0, 52, 128, 10);
	u8g2_.drawBox(1, 53, (w > 2 ? w - 2 : 0), 8);

	u8g2_.sendBuffer();
}

void DisplaySh1106::renderMain(const AppState& state, float battVoltageV, uint8_t battPct, const char* gpsLine) {
	u8g2_.clearBuffer();
	u8g2_.setFont(u8g2_font_6x10_tf);

	char line[32];

	if (battVoltageV > 0.0f) {
		snprintf(line, sizeof(line), "BAT %.2fV %u%%", static_cast<double>(battVoltageV),
					 static_cast<unsigned>(battPct));
	} else {
		snprintf(line, sizeof(line), "BAT --.-V %u%%", static_cast<unsigned>(battPct));
	}
	u8g2_.drawStr(0, 12, line);

	if (state.gate != GateState::Running) {
		if (state.initDone) {
			const char* readyLine = (state.startupMode == StartupMode::AutoStart)
										 ? "AUTO START"
										 : "READY: short=start long=otaa";
			acceptScrollText(readyScroll_, readyLine, u8g2_, " ", 200);
			drawMarqueeLoop(u8g2_, 42, readyScroll_, " <<< ", 3, 40);
		} else {
			u8g2_.drawStr(0, 42, "LORA: init...");
		}
	} else {
		uint32_t now = millis();
		int32_t countdown = 0;
		if (state.nextActionMs > now) {
			countdown = static_cast<int32_t>((state.nextActionMs - now) / 1000);
		}

		uint8_t attempt = (state.joinState == JoinState::Radio) ? state.radioInitAttempts : state.joinAttempts;
		uint8_t maxAttempt = (state.joinState == JoinState::Radio) ? AppConfig::RADIO_INIT_MAX_ATTEMPTS
																							: AppConfig::JOIN_MAX_ATTEMPTS;

		const char* phase = joinStateText(state.joinState);

		if (state.joinState == JoinState::Ok || state.joinState == JoinState::Fail) {
			snprintf(line, sizeof(line), "LORA %s", phase);
		} else if (countdown > 0) {
			snprintf(line, sizeof(line), "LORA %s %u/%u T-%lds",
					 phase, static_cast<unsigned>(attempt), static_cast<unsigned>(maxAttempt),
					 static_cast<long>(countdown));
		} else {
			snprintf(line, sizeof(line), "LORA %s %u/%u",
					 phase, static_cast<unsigned>(attempt), static_cast<unsigned>(maxAttempt));
		}

		if (state.forceOtaaThisBoot) {
			size_t len = strlen(line);
			if (len < sizeof(line) - 7) strcat(line, " FORCE");
		}

		u8g2_.drawStr(0, 30, line);

		if (state.initDone) {
			const char* readyLine = (state.startupMode == StartupMode::AutoStart)
										 ? "AUTO START"
										 : "READY: short=start long=otaa";
			acceptScrollText(readyScroll_, readyLine, u8g2_, " ", 200);
			drawMarqueeLoop(u8g2_, 42, readyScroll_, " <<< ", 3, 40);
		} else {
			u8g2_.drawStr(0, 42, "LORA: init...");
		}
	}

	if (gpsLine) {
		acceptScrollText(gpsScroll_, gpsLine, u8g2_, " <<< ", 200);
	}
	drawMarqueeLoop(u8g2_, 54, gpsScroll_, " <<< ", 3, 40);

	u8g2_.sendBuffer();
}

void DisplaySh1106::showModeMessage(const char* line) {
	u8g2_.clearBuffer();
	u8g2_.setFont(u8g2_font_6x10_tf);
	u8g2_.drawStr(0, 30, line);
	u8g2_.sendBuffer();
}
