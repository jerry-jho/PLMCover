#include "esphome.h"
#include "SoftwareSerial.h"

#define MYPORT_TX 19
#define MYPORT_RX 22


class PLMCover : public PollingComponent, public Cover {
 public:
  PLMCover() : PollingComponent(1000) {}

  void setup() override {

    cmd_set[0] = 0x55;
    cmd_set[1] = 0x01;
    cmd_set[2] = 0x00; //ID
    cmd_set[3] = 0x03; //Command Write
    cmd_set[4] = 0x04; //precentage
    cmd_set[5] = 0x00; //precentage value
    cmd_set[6] = 0x00; //CRC LOW
    cmd_set[7] = 0x00; //CRC HIGH

    cmd_get[0] = 0x55;
    cmd_get[1] = 0x01;
    cmd_get[2] = 0x00; //ID
    cmd_get[3] = 0x01; //Command Read
    cmd_get[4] = 0x02; //precentage
    cmd_get[5] = 0x01; //1 byte
    cmd_get[6] = 0x00; //CRC LOW
    cmd_get[7] = 0x00; //CRC HIGH

    cmd_stop[0] = 0x55;
    cmd_stop[1] = 0x01;
    cmd_stop[2] = 0x00; //ID
    cmd_stop[3] = 0x03; //Command Write
    cmd_stop[4] = 0x03; //Stop
    cmd_stop[5] = 0x00; //CRC LOW
    cmd_stop[6] = 0x00; //CRC HIGH
    cmd_stop[7] = 0x00;

    RS485.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  }
  int hwControl(unsigned char *d, int len) {
    d[2] = _id;
    crc16(d, len-2);
    ESP_LOGI("Cover", "%02X %02X %02X %02X %02X %02X %02X %02X", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
    while(len--) {
      RS485.write((char)(*d));
      d++;
    }
    return 0;
  }
  void crc16(unsigned char *data, int len) {
    unsigned int i, j, tmp, CRC16;
    CRC16 = 0xFFFF;
    for (i = 0; i < len; i++) {
        CRC16 ^= data[i];
        for (j = 0; j < 8; j++) {
            tmp = (unsigned int)(CRC16 & 0x0001);
            CRC16 >>= 1;
            if (tmp == 1) {
                CRC16 ^= 0xA001;
            }
        }
    }
    data[i++] = (unsigned char) (CRC16 & 0x00FF);
    data[i++] = (unsigned char) ((CRC16 & 0xFF00)>>8);
  }

  CoverTraits get_traits() override {
    auto traits = CoverTraits();
    traits.set_is_assumed_state(false);
    traits.set_supports_position(true);
    traits.set_supports_tilt(false);
    traits.set_supports_stop(true);
    return traits;
  }
  void control(const CoverCall &call) override {
    ESP_LOGI("Cover", "C%d Control", _id);
    if (call.get_position().has_value()) {
      int pos = (*call.get_position())*100;
      ESP_LOGI("Cover", "C%d Position %d", _id, pos);
      cmd_set[5] = pos;
      hwControl(cmd_set, 8);
      update_status(pos);
    }
    if (call.get_stop()) {
      ESP_LOGI("Cover", "C%d Stop", _id);
      hwControl(cmd_stop, 7);
    }
  }
  void update_status(int pos) {
    this->position = pos / 100.0;
    this->publish_state();
  }
  void update() override {
    // TODO 
  }
  void setID(int id) {
    _id = id;
  }
 private:
  int _id;
  int _current_pos;
  float _target_pos;
  unsigned char cmd_set[8];
  unsigned char cmd_get[8];
  unsigned char cmd_stop[8];
  EspSoftwareSerial::UART RS485;
};