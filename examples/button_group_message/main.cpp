#include <Arduino.h>
#include <Mesh.h>

#ifdef ESP32
  #include <SPIFFS.h>
  #include <esp_sleep.h>
  #include <driver/rtc_io.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/ui/MomentaryButton.h>
#include <RTClib.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

// LoRa radio parameters (override in platformio.ini build_flags)
#ifndef LORA_FREQ
  #define LORA_FREQ      915.0
#endif
#ifndef LORA_BW
  #define LORA_BW        250
#endif
#ifndef LORA_SF
  #define LORA_SF        10
#endif
#ifndef LORA_CR
  #define LORA_CR        5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS  32
#endif

// Must be at least 1 for group channel support
#ifndef MAX_GROUP_CHANNELS
  #define MAX_GROUP_CHANNELS 1
#endif

// Group channel pre-shared key — must match all nodes in the group.
#ifndef GROUP_PSK
  #define GROUP_PSK  "LKaloQVV3gxuP+/5FMn6CQ=="
#endif

// Node display name (stored in flash after first boot)
#ifndef NODE_NAME
  #define NODE_NAME  "ButtonNode"
#endif

// Message sent when the button is pressed
#ifndef BUTTON_MESSAGE
  #define BUTTON_MESSAGE  "StopStopStop"
#endif

// How long the display stays on after a short press of the user button
#ifndef DISPLAY_ON_MILLIS
  #define DISPLAY_ON_MILLIS 10000
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

/* -------------------------------------------------------------------------------------- */

class MyMesh : public BaseChatMesh {
  FILESYSTEM* _fs;
  char _node_name[32];
  ChannelDetails* _group;

protected:
  float getAirtimeBudgetFactor() const override { return 1.0; }
  int calcRxDelay(float score, uint32_t air_time) const override { return 0; }
  bool allowPacketForward(const mesh::Packet* packet) override { return true; }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    Serial.printf("ADVERT from: %s\n", contact.name);
  }

  void onContactPathUpdated(const ContactInfo& contact) override { }

  ContactInfo* processAck(const uint8_t* data) override { return NULL; }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char* text) override {
    Serial.printf("MSG from %s: %s\n", from.name, text);
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char* text) override { }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t* sender_prefix, const char* text) override { }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char* text) override {
    Serial.printf("%s\r\n", text);
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override { return 0; }
  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override { }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }

  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    uint8_t path_hash_count = path_len & 63;
    return SEND_TIMEOUT_BASE_MILLIS +
           ((pkt_airtime_millis * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_hash_count + 1));
  }

  void onSendTimeout() override {
    Serial.println("Send timeout (no ACK).");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
    : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    strncpy(_node_name, NODE_NAME, sizeof(_node_name) - 1);
    _node_name[sizeof(_node_name) - 1] = 0;
    _group = NULL;
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;
    BaseChatMesh::begin();

    IdentityStore store(fs, "/identity");
    if (!store.load("_main", self_id, _node_name, sizeof(_node_name))) {
      // No stored identity — generate a new one from radio noise entropy
      self_id = radio_new_identity();
      strncpy(_node_name, NODE_NAME, sizeof(_node_name) - 1);
      store.save("_main", self_id, _node_name);
      Serial.println("New identity created.");
    }

    _group = addChannel("Group", GROUP_PSK);

    Serial.printf("Node: %s\n", _node_name);
    Serial.println("Press the button to send a group message.");
    Serial.println();
  }

  void sendSelfAdvert(int delay_millis = 0) {
    auto pkt = createSelfAdvert(_node_name);
    if (pkt) sendFlood(pkt, delay_millis);
  }

  void sendButtonMessage() {
    if (!_group) {
      Serial.println("ERROR: group channel not configured.");
      return;
    }
    uint32_t now = getRTCClock()->getCurrentTime();
    bool sent = sendGroupMessage(now, _group->channel, _node_name, BUTTON_MESSAGE, strlen(BUTTON_MESSAGE));
    if (sent) {
      Serial.printf("Sent: \"%s: %s\"\n", _node_name, BUTTON_MESSAGE);
    } else {
      Serial.println("ERROR: send failed (out of packet memory?).");
    }
  }

  void loop() {
    BaseChatMesh::loop();
  }
};

#ifdef DISPLAY_CLASS
static void powerOff() {
  if (!display.isOn()) {
    display.turnOn();
  }

  display.startFrame();
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.print("Powering");
  display.setCursor(0, 32);
  display.print("off...");
  display.endFrame();

  // Wait for button release before sleeping, otherwise the device
  // wakes immediately because PIN_USER_BTN is still LOW
  while (digitalRead(PIN_USER_BTN) == LOW) { delay(10); }
  delay(100);  // debounce

  display.turnOff();
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_USER_BTN, ESP_EXT1_WAKEUP_ANY_LOW);
  Serial.println("Powering off.");
  Serial.flush();
  esp_deep_sleep_start();
}
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

#ifndef BUTTON_PIN
  #define BUTTON_PIN 2
#endif

static MomentaryButton button(BUTTON_PIN, 0, true, true);

void halt() { while (1) ; }

void setup() {
  Serial.begin(115200);
  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

#ifdef ESP32
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#endif

  radio_driver.setParams(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_driver.setTxPower(LORA_TX_POWER);

  button.begin();

#ifdef DISPLAY_CLASS
  display.begin();
#endif

  // Announce this node to the mesh after a short delay
  the_mesh.sendSelfAdvert(1200);
}

void loop() {
  the_mesh.loop();
  rtc_clock.tick();

  const unsigned long now_ms = millis();
  static unsigned long last_send_ms = 0;
  if (button.isPressed() && now_ms - last_send_ms >= 1000) {
    last_send_ms = now_ms;
    the_mesh.sendButtonMessage();
  }

#ifdef DISPLAY_CLASS
  static unsigned long display_on_ms = 0;
  static unsigned long last_display_ms = 0;

  int btn_event = user_btn.check();
  if (btn_event == BUTTON_EVENT_LONG_PRESS) {
    powerOff();
  } else if (btn_event != BUTTON_EVENT_NONE) {
    // Short press (or multi-click): turn the display on, or extend the timeout if already on
    if (!display.isOn()) {
      display.turnOn();
    }
    display_on_ms = now_ms;
    last_display_ms = 0;
  }

  if (display.isOn() && now_ms - display_on_ms >= DISPLAY_ON_MILLIS) {
    display.turnOff();
  }

  if (display.isOn() && now_ms - last_display_ms >= 1000) {
    last_display_ms = now_ms;
    uint16_t mv = board.getBattMilliVolts();
    int pct = constrain((int)(mv - 3000) * 100 / 1200, 0, 100);
    char buf[16];
    snprintf(buf, sizeof(buf), "Batt: %d%%", pct);
    display.startFrame();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(buf);
    display.endFrame();
  }
#endif
}
