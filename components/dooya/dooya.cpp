#include "dooya.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dooya {

static const char *const TAG = "dooya.cover";

using namespace esphome::cover;

uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 0x01) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

CoverTraits Dooya::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  return traits;
}

void Dooya::control(const CoverCall &call) {
  if (call.get_stop()) {
    uint8_t data[2] = {CONTROL, STOP};
    this->send_command_(data, 2);
  } else if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos != this->position) {
      if (pos == COVER_OPEN) {
        uint8_t data[2] = {CONTROL, OPEN};
        this->send_command_(data, 2);
      } else if (pos == COVER_CLOSED) {
        uint8_t data[2] = {CONTROL, CLOSE};
        this->send_command_(data, 2);
      } else {
        uint8_t data[3] = {CONTROL, SET_POSITION, (uint8_t)(pos * 100)};
        this->send_command_(data, 3);
      }
    }
  }
}

void Dooya::send_update() {
  uint8_t data[3] = {READ, this->current_request_, 0x01};
  this->send_command_(data, 3);
}

void Dooya::on_uart_multi_byte(uint8_t byte) { // вызывается при получении байта uart
  uint32_t at = this->rx_buffer_.size();       // номер последнего полученного байта
  uint8_t *data = &this->rx_buffer_[0];               // указатель на первый байт сообщения
//  switch (at) {
//    case 0:
  if (at == 0) {
      if (byte == START_CODE)
        this->rx_buffer_.push_back(byte);   // получили заголовок = байт0
       return; 
  }
  if (at == 1) {
      if (byte == this->address_[0])
        this->rx_buffer_.push_back(byte);    // получили адрес0
      else
        this->rx_buffer_.clear();
      return;
  }
  if (at == 2) {
      if (byte == this->address_[1])
        this->rx_buffer_.push_back(byte);    // получили адрес1
      else
        this->rx_buffer_.clear();
      return;
  }      
  if (at == 3) {
      if (byte == CONTROL || byte == READ)  
        this->rx_buffer_.push_back(byte);    // получили команду управлния или чтения
      else
        this->rx_buffer_.clear();
      return;
  }
  if (at < 6) {                    // получили данные и возможно, начало crc 
      this->rx_buffer_.push_back(byte);
      return;
  }      
  if (at >= 6)  {                    // возможно, получили весь пакет
      this->rx_buffer_.push_back(byte);
//      std::string pretty_cmd = format_hex_pretty(rx_buffer_);
 //     ESP_LOGI(TAG,  "Получено побайтно: %S ", pretty_cmd.c_str() );
    
      std::vector<uint8_t> frame(this->rx_buffer_.begin(), this->rx_buffer_.end() - 2);
      uint16_t crc = crc16(&frame[0], frame.size());      // получили crc
//      ESP_LOGI(TAG,  "Получено crc & 0xFF= %X crc >> 8 = %X [-2] = %X [-1] = %X", crc & 0xFF, crc >> 8, rx_buffer_.end()[-2], rx_buffer_.end()[-1]);
      if (((crc & 0xFF) == this->rx_buffer_.end()[-2]) && ((crc >> 8) == this->rx_buffer_.end()[-1])) {  // если пришло всё сообщение
        if (this->rx_buffer_[3] == CONTROL)
          this->process_response_();
        else
          this->process_status_(); 
      this->rx_buffer_.clear();
      }
          
      return;
    } //  if (at >= 6) 

      
  
  
} //function

void Dooya::process_response_() {
  this->parent_->ready_to_tx = true;
  std::vector<uint8_t> frame(this->rx_buffer_.begin(), this->rx_buffer_.end());
  uint16_t crc = crc16(&frame[0], frame.size());
  
 // std::string pretty_cmd = format_hex_pretty(rx_buffer_);
 // ESP_LOGI(TAG,  "Получен пакет ответа: %S ", pretty_cmd.c_str() );
  

    switch (this->rx_buffer_[4]) {
      case STOP:
        this->current_operation = COVER_OPERATION_IDLE;
        break;
      case OPEN:
        this->current_operation = COVER_OPERATION_OPENING;
        break;
      case CLOSE:
        this->current_operation = COVER_OPERATION_CLOSING;
        break;
      case SET_POSITION:
        if (this->rx_buffer_[5] > (uint8_t)(this->position * 100))
          this->current_operation = COVER_OPERATION_OPENING;
        else
          this->current_operation = COVER_OPERATION_CLOSING;
        break;
      default:
        ESP_LOGE(TAG, "Invalid control operation received");
        return;
    } // switch
    this->publish_state(false);
}

void Dooya::process_status_() {
  this->parent_->ready_to_tx = true;
  std::vector<uint8_t> frame(this->rx_buffer_.begin(), this->rx_buffer_.end() - 2);
  uint16_t crc = crc16(&frame[0], frame.size());
  
//  std::string pretty_cmd = format_hex_pretty(rx_buffer_);
//  ESP_LOGI(TAG,  "Получен пакет статуса: %S ", pretty_cmd.c_str() );
  
  

    if (this->current_request_ == GET_POSITION) {
      float pos = 0.5f;
      if (this->rx_buffer_[6] != 0xFF)
        pos = clamp((float) this->rx_buffer_[6] / 100, 0.0f, 1.0f);
      if (this->position != pos) {
        this->position = pos;
        this->publish_state(false);
      }
      this->current_request_ = GET_STATUS;
    } else {
      switch (this->rx_buffer_[6]) {
        case 0:
          if (this->current_operation != COVER_OPERATION_IDLE) {
            this->current_operation = COVER_OPERATION_IDLE;
            //this->publish_state(false);
          } //if
          break;
        case 1:
          if (this->current_operation != COVER_OPERATION_OPENING) {
            this->current_operation = COVER_OPERATION_OPENING;
       //     this->publish_state(false);
          } //if
          break;
        case 2:
          if (this->current_operation != COVER_OPERATION_CLOSING) {
            this->current_operation = COVER_OPERATION_CLOSING;
        //    this->publish_state(false);
          } // if
          break;
        default:
          ESP_LOGE(TAG, "Invalid status operation received");
          return;
      } // switch
  //    this->current_request_ = GET_POSITION;
    } // else
  
}

void Dooya::send_command_(const uint8_t *data, uint8_t len) {
  std::vector<uint8_t> frame = {START_CODE, this->address_[0], this->address_[1]};
  for (size_t i = 0; i < len; i++) {
    frame.push_back(data[i]);
  }
  uint16_t crc = crc16(&frame[0], frame.size());
  frame.push_back(crc >> 0);
  frame.push_back(crc >> 8);

  this->send(frame);
}

void Dooya::dump_config() {
  ESP_LOGCONFIG(TAG, "Dooya:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", this->address_[0], this->address_[1]);
}

}  // namespace dooya
}  // namespace esphome
