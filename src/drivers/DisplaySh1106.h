#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

#include "app/App.h"

class DisplaySh1106 {
public:
	DisplaySh1106();

	void begin();
	void renderMain(const AppState& state, float battVoltageV, uint8_t battPct, const char* gpsLine);
	void showBootHold(uint32_t heldMs);
	void showModeMessage(const char* line);

private:
	struct ScrollText {
		int16_t x = 0;
		uint32_t lastStepMs = 0;
		char buf[96] = {0};
		uint32_t lastAcceptMs = 0;
		int16_t textW = 0;
		int16_t sepW = 0;
		bool scrolling = false;
	};

	static const char* joinStateText(JoinState state);
	static const char* loraErrLabel(int16_t err, bool uplinkOk);
	static void acceptScrollText(ScrollText& st, const char* newText, U8G2& u8g2,
												 const char* sep = " - ", uint32_t minUpdateMs = 1000);
	static void drawMarqueeLoop(U8G2& u8g2, int y, ScrollText& st,
								 const char* sep = " - ", uint8_t speedPx = 1,
								 uint16_t stepMs = 40);

	U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2_;
	ScrollText gpsScroll_;
	ScrollText readyScroll_;
	ScrollText holdScroll_;
};
