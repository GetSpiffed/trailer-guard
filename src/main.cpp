#include <Arduino.h>

#include "app/App.h"

static App app;

// Initialiseer de applicatie en hardware.
void setup() {
	app.begin();
}

// Draai de hoofdloop van de applicatie.
void loop() {
	app.tick();
}
