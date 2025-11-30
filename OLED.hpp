#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display configuration
static constexpr uint8_t SCREEN_WIDTH   = 128;
static constexpr uint8_t SCREEN_HEIGHT  = 64;
static constexpr uint8_t SCREEN_ADDRESS = 0x3C;
static constexpr int8_t  OLED_RESET     = -1;  // -1 if sharing Arduino reset pin

namespace mickeymouse {

class OLED {
public:
    // Constructor/destructor
    OLED() : display_(new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET)) {}
    ~OLED() { delete display_; }

    // Initialize the display; returns true on success
    bool begin() {
        if (!display_->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            return false;
        }
        display_->clearDisplay();
        display_->setTextSize(1);
        display_->setTextColor(SSD1306_WHITE);
        display_->setCursor(0, 0);
        display_->display();
        return true;
    }

    // Print a multi-line string (\n-delimited) to the display
    void printText(const String& text) {
        display_->clearDisplay();
        display_->setCursor(0, 0);

        const int lineHeight = 8;
        int y = 0;
        int start = 0;
        int next;

        // Loop through lines
        while ((next = text.indexOf('\n', start)) != -1 && y < display_->height()) {
            display_->setCursor(0, y);
            display_->print(text.substring(start, next));
            start = next + 1;
            y += lineHeight;
        }
        // Print remaining text
        if (y < display_->height()) {
            display_->setCursor(0, y);
            display_->print(text.substring(start));
        }
        display_->display();
    }

    // Clear the display
    void clear() {
        display_->clearDisplay();
        display_->display();
    }

private:
    Adafruit_SSD1306* display_;
};

} // namespace mickeymouse

