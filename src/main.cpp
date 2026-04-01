#include <WiFi.h>
#include <esp_now.h>

// ===== 設定 =====
#define UART_BAUD 115200
#define MAX_DATA 180
#define WINDOW_SIZE 16

#define TYPE_DATA 1
#define TYPE_ACK  2

#define RTO 50  // 再送タイムアウト(ms)

// ===== 相手MAC（書き換え）=====
// No2: 88:13:bf:0c:15:c4
// No3: ac:15:18:d7:97:e4
//uint8_t PEER_MAC[] = {0x88,0x13,0xbf,0x0c,0x15,0xc4};  // to No2
uint8_t PEER_MAC[] = {0xac,0x15,0x18,0xd7,0x97,0xe4}; //to No3


// ===== パケット =====
typedef struct {
  uint8_t type;
  uint16_t seq;
  uint16_t ack;
  uint16_t bitmap;
  uint8_t len;
  uint8_t data[MAX_DATA];
  uint16_t crc;
} Packet;

// ===== 状態 =====
uint16_t send_base = 0;
uint16_t next_seq  = 0;

uint16_t recv_base = 0;

// 受信バッファ
Packet recv_buffer[WINDOW_SIZE];
bool recv_valid[WINDOW_SIZE] = {0};

// 送信バッファ
Packet send_buffer[WINDOW_SIZE];
uint32_t send_time[WINDOW_SIZE];

// 輻輳制御
int cwnd = 4;

// ===== CRC（簡易）=====
uint16_t calc_crc(uint8_t *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// ===== ACK送信 =====
void sendAck(const uint8_t *mac) {
  Packet ack = {};
  ack.type = TYPE_ACK;
  ack.ack = recv_base;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (recv_valid[(recv_base + i) % WINDOW_SIZE]) {
      ack.bitmap |= (1 << i);
    }
  }

  esp_now_send(mac, (uint8_t*)&ack, sizeof(Packet));
}

// ===== DATA受信 =====
void handleData(Packet &pkt, const uint8_t *mac) {

  // CRCチェック
  if (calc_crc(pkt.data, pkt.len) != pkt.crc) {
    return;
  }

  if (pkt.seq >= recv_base && pkt.seq < recv_base + WINDOW_SIZE) {

    int idx = pkt.seq % WINDOW_SIZE;

    if (!recv_valid[idx]) {
      recv_buffer[idx] = pkt;
      recv_valid[idx] = true;
    }

    // 順序出力
    while (recv_valid[recv_base % WINDOW_SIZE]) {
      Packet &p = recv_buffer[recv_base % WINDOW_SIZE];
      Serial.write(p.data, p.len);
      recv_valid[recv_base % WINDOW_SIZE] = false;
      recv_base++;
    }
  }

  sendAck(mac);
}

// ===== ACK受信 =====
void handleAck(Packet &pkt) {

  // 累積ACK
  while (send_base < pkt.ack) {
    send_base++;
  }

  // ビットマップACK
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (pkt.bitmap & (1 << i)) {
      uint16_t seq = pkt.ack + i;
      if (seq >= send_base && seq < next_seq) {
        send_base = seq + 1;
      }
    }
  }

  // 輻輳制御（簡易）
  cwnd++;
  if (cwnd > WINDOW_SIZE) cwnd = WINDOW_SIZE;
}

// ===== 受信コールバック =====
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  if (len < sizeof(Packet)) return;

  Packet pkt;
  memcpy(&pkt, incomingData, sizeof(Packet));

  if (pkt.type == TYPE_DATA) {
    handleData(pkt, mac);
  } else if (pkt.type == TYPE_ACK) {
    handleAck(pkt);
  }
}

// ===== 再送 =====
void checkRetransmit() {

  for (uint16_t i = send_base; i < next_seq; i++) {

    int idx = i % WINDOW_SIZE;

    if (millis() - send_time[idx] > RTO) {

      esp_now_send(PEER_MAC,
        (uint8_t*)&send_buffer[idx],
        sizeof(Packet));

      send_time[idx] = millis();

      // 輻輳制御
      cwnd /= 2;
      if (cwnd < 1) cwnd = 1;
    }
  }
}

// ===== 送信 =====
void sendData() {

  if (Serial.available()) {

    if (next_seq < send_base + cwnd) {

      Packet pkt = {};
      pkt.type = TYPE_DATA;
      pkt.seq = next_seq;

      pkt.len = Serial.readBytes(pkt.data, MAX_DATA);

      pkt.crc = calc_crc(pkt.data, pkt.len);

      int idx = next_seq % WINDOW_SIZE;
      send_buffer[idx] = pkt;
      send_time[idx] = millis();

      esp_now_send(PEER_MAC, (uint8_t*)&pkt, sizeof(Packet));

      next_seq++;
    }
  }
}

// ===== 初期化 =====
void setup() {

  Serial.begin(UART_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, PEER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer add failed");
    return;
  }

  Serial.println("READY");
}

// ===== メイン =====
void loop() {
  sendData();
  checkRetransmit();
}