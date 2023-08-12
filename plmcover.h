#include "esphome.h"
#include "SoftwareSerial.h"
#include <mutex>

#define MYPORT_TX 19
#define MYPORT_RX 22

#define CVR_DOOYA
//#define CVR_WISER


#ifdef CVR_DOOYA
#define SET_COMMAND_LENGTH 8
#define GET_COMMAND_LENGTH 8
#define STOP_COMMAND_LENGTH 7
#define ID_INDEX 2
#define PV_INDEX 5
#endif

#ifdef CVR_WISER
#define SET_COMMAND_LENGTH 7
#define GET_COMMAND_LENGTH 7
#define STOP_COMMAND_LENGTH 7
#define ID_INDEX 0
#define PV_INDEX 4
#endif

#define REC_BUFFER_SIZE 128

static std::mutex serial_mtx;


class CyclicBuffer {
public:
  CyclicBuffer() {}
  void begin(unsigned char * b, int l) {
    _b = b;
    _l = l;
  }
  unsigned char & operator[] (int i) {
    return _b[i%_l];
  }
private:
  unsigned char * _b;
  int _l;
};

class RS485Cover : public PollingComponent {
 public:
  RS485Cover() : PollingComponent(1000) {}
  void setup() override {
    #ifdef CVR_DOOYA
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
    #endif

    #ifdef CVR_WISER
      cmd_set[0] = 0x00; //AL
      cmd_set[1] = 0x04; //AH
      cmd_set[2] = 0x02; //Command Write
      cmd_set[3] = 0x01; //precentage
      cmd_set[4] = 0x04; //precentage value
      cmd_set[5] = 0x00; //CRC LOW
      cmd_set[6] = 0x00; //CRC HIGH
      cmd_set[7] = 0x00;

      cmd_get[0] = 0x00;
      cmd_get[1] = 0x00;
      cmd_get[2] = 0x00;
      cmd_get[3] = 0x00;
      cmd_get[4] = 0x00;
      cmd_get[5] = 0x00;
      cmd_get[6] = 0x00;
      cmd_get[7] = 0x00;

      cmd_stop[0] = 0x00; //AL
      cmd_stop[1] = 0x04; //AH
      cmd_stop[2] = 0x02; //Command Write
      cmd_stop[3] = 0x02; //Stop
      cmd_stop[4] = 0x00; //
      cmd_stop[5] = 0x00; //CRC LOW
      cmd_stop[6] = 0x00; //CRC HIGH
      cmd_stop[7] = 0x00;
    #endif

    rec.begin(rec_mem, REC_BUFFER_SIZE);
    SSerial.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  }
  void setpos(int id, int pos) {
    cmd_set[PV_INDEX] = pos;
    _hw(id, cmd_set, SET_COMMAND_LENGTH);
  }
  void setstop(int id) {
    _hw(id, cmd_stop, STOP_COMMAND_LENGTH);
  }
  void update() override {

  }
 private:
  int _hw(int id, unsigned char *d, int len) {
    d[ID_INDEX] = id;
    _crc16(d, len-2);
    int wlen = len;
    do {
      std::lock_guard<std::mutex> lck(serial_mtx);
      ESP_LOGI("RS485Cover WR", "%02X %02X %02X %02X %02X %02X %02X %02X", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
      while(wlen--) {
        SSerial.write((char)(*d));
        d++;
      }
    } while (0);
    return 0;
  }
  void _crc16(unsigned char *data, int len) {
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
  unsigned char cmd_set[8];
  unsigned char cmd_get[8];
  unsigned char cmd_stop[8];
  unsigned char rec_mem[REC_BUFFER_SIZE];
  CyclicBuffer rec;
  EspSoftwareSerial::UART SSerial;  
};

class PLMCover : public PollingComponent, public Cover {
 public:
  PLMCover(int id, RS485Cover * cover) : PollingComponent(1000) {
    _id = id;
    _cover = cover;
  }
  void setup() override {
    t_counter = 0;
    _cover->setup();
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
      _cover->setpos(_id, pos);
      update_status(pos);
    }
    if (call.get_stop()) {
      ESP_LOGI("Cover", "C%d Stop", _id);
      _cover->setstop(_id);
    }
  }
  void update_status(int pos) {
    this->position = pos / 100.0;
    this->publish_state();
  }
  void update() override {
    // t_counter ++;

    // if (t_counter == 30) {
    //   t_counter = 0;
    //   hwControl(cmd_get, GET_COMMAND_LENGTH);
    // }
    // do {
    //   std::lock_guard<std::mutex> lck(serial_mtx);
    //   if (RS485.available()) {
    //     ESP_LOGI("Cover", "C%d Buffer %d", _id, RS485.available());
    //     int i = 0;
    //     while(RS485.available()) {
    //       char ch = RS485.read();
    //       rec[i] = ch;
    //       i ++;
    //     }
    //     int count = i;
    //     String ss = "";
    //     char cc[4];
    //     for (int j=0;j<count;j++) {
    //       sprintf(cc, "%02X ", rec[j]);
    //       ss += cc;
    //     }
    //     ESP_LOGI("Cover RS485 RD", ss.c_str());
    //     #ifdef CVR_DOOYA
    //       int j=0;
    //       while(j<count) {
    //         if (rec[j] = 0x55) {
    //           int cap_id = rec[j+ID_INDEX];
    //           if (cap_id == _id && rec[j+3] == 0x01) {
    //             update_status(rec[j+5]);
    //             break;
    //           }
    //         }
    //         j++;
    //       }
    //     #endif
    //   }
    // } while (0);

  }

 private:
  int _id;
  RS485Cover * _cover;
  int t_counter;
  
};