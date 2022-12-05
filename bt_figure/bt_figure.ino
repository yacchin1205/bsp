// Bluetooth Speaker + LED effect on M5Atom Matrix
// ----
// A2DP based on https://github.com/pschatzmann/ESP32-A2DP/
// BLE referred to https://github.com/espressif/esp-idf/blob/7869f4e151e3ea2a308b991fbb8e9baa4cec313c/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md

#include <M5Atom.h>

// A2DP
#include <BluetoothA2DPCommon.h>
#include <BluetoothA2DP.h>
#include <BluetoothA2DPSink.h>
#include <A2DPVolumeControl.h>

// BLE
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>

// FFT
#include <KickFFT.h>

#ifdef DEBUG
#define DEBUGF(...)  Serial.printf(__VA_ARGS__)
#else  // DEBUG
#define DEBUGF(...)
#endif // DEBUG

// BLE
#define ESP_FIG_APP_ID              0x56

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify   = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t fig_data_notify_ccc[2] = {0x00, 0x00};

// BLE: Service and Characteristics UUIDs

// BLE: SERVICE_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c900
static const uint16_t SERVICE_UUID_FIG = 0xFFF1;

static uint8_t GATTS_SERVICE_UUID_FIG[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  //first uuid, 16bit, [12],[13] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xF1, 0xFF, 0x00, 0x00,
};

// BLE: Characteristics
// BLE: Title: CHARACTERISTIC_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c901
static uint8_t GATTS_CHAR_UUID_PLAYBACK_TITLE[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  0x01, 0xc9, 0xe0, 0x52, 0xa0, 0x65, 0x99, 0xb3, 0xd3, 0x49, 0x94, 0x68, 0xf3, 0xe1, 0xb1, 0x0c,
};
// BLE: Artist: CHARACTERISTIC_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c902
static uint8_t GATTS_CHAR_UUID_PLAYBACK_ARTIST[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  0x02, 0xc9, 0xe0, 0x52, 0xa0, 0x65, 0x99, 0xb3, 0xd3, 0x49, 0x94, 0x68, 0xf3, 0xe1, 0xb1, 0x0c,
};
// BLE: Album: CHARACTERISTIC_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c903
static uint8_t GATTS_CHAR_UUID_PLAYBACK_ALBUM[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  0x03, 0xc9, 0xe0, 0x52, 0xa0, 0x65, 0x99, 0xb3, 0xd3, 0x49, 0x94, 0x68, 0xf3, 0xe1, 0xb1, 0x0c,
};
// BLE: State: CHARACTERISTIC_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c910
static uint8_t GATTS_CHAR_UUID_PLAYBACK_STATE[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  0x10, 0xc9, 0xe0, 0x52, 0xa0, 0x65, 0x99, 0xb3, 0xd3, 0x49, 0x94, 0x68, 0xf3, 0xe1, 0xb1, 0x0c,
};
// BLE: Beats: CHARACTERISTIC_UUID = 0cb1e1f3-6894-49d3-b399-65a052e0c911
static uint8_t GATTS_CHAR_UUID_BEATS[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  0x11, 0xc9, 0xe0, 0x52, 0xa0, 0x65, 0x99, 0xb3, 0xd3, 0x49, 0x94, 0x68, 0xf3, 0xe1, 0xb1, 0x0c,
};

static const uint8_t default_playback_value[] = "";

// BLE: Status
#define SVC_INST_ID                 0

enum{
  FIG_IDX_SVC,
  FIG_IDX_PLAYBACK_TITLE_CHAR,
  FIG_IDX_PLAYBACK_TITLE_VAL,
  FIG_IDX_PLAYBACK_TITLE_CFG,
  FIG_IDX_PLAYBACK_ARTIST_CHAR,
  FIG_IDX_PLAYBACK_ARTIST_VAL,
  FIG_IDX_PLAYBACK_ARTIST_CFG,
  FIG_IDX_PLAYBACK_ALBUM_CHAR,
  FIG_IDX_PLAYBACK_ALBUM_VAL,
  FIG_IDX_PLAYBACK_ALBUM_CFG,
  FIG_IDX_PLAYBACK_STATE_CHAR,
  FIG_IDX_PLAYBACK_STATE_VAL,
  FIG_IDX_PLAYBACK_STATE_CFG,
  FIG_IDX_BEATS_CHAR,
  FIG_IDX_BEATS_VAL,
  FIG_IDX_BEATS_CFG,
  FIG_IDX_NB,
};

esp_gatt_if_t current_ble_gatts_if;
uint16_t current_ble_conn_id;
bool is_ble_connected;
bool enable_ble_playback_state_ntf;
bool enable_ble_playback_title_ntf;
bool enable_ble_playback_album_ntf;
bool enable_ble_playback_artist_ntf;
bool enable_ble_beats_ntf;
uint16_t fig_handle_table[FIG_IDX_NB] = {0};

#define GATTS_PLAYBACK_CHAR_VAL_LEN_MAX 500

static esp_ble_adv_data_t adv_data = {
  .set_scan_rsp        = false,
  .include_name        = true,
  .include_txpower     = true,
  .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
  .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
  .appearance          = 0x00,
  .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
  .p_manufacturer_data = NULL, //test_manufacturer,
  .service_data_len    = 0,
  .p_service_data      = NULL,
  .service_uuid_len    = sizeof(GATTS_SERVICE_UUID_FIG),
  .p_service_uuid      = (uint8_t *) GATTS_SERVICE_UUID_FIG,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
  .adv_int_min         = 0x20,
  .adv_int_max         = 0x40,
  .adv_type            = ADV_TYPE_IND,
  .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
  .channel_map         = ADV_CHNL_ALL,
  .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Declaration for Characteristics
static const esp_gatts_attr_db_t fig_gatt_db[FIG_IDX_NB] =
{
  // Service Declaration
  [FIG_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(SERVICE_UUID_FIG), (uint8_t *) &SERVICE_UUID_FIG}},
  // Characteristic Declaration - Title
  [FIG_IDX_PLAYBACK_TITLE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
  // Characteristic Value - Title
  [FIG_IDX_PLAYBACK_TITLE_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)GATTS_CHAR_UUID_PLAYBACK_TITLE, ESP_GATT_PERM_READ,
      GATTS_PLAYBACK_CHAR_VAL_LEN_MAX, sizeof(default_playback_value), (uint8_t *)default_playback_value}},
  // Client Characteristic Configuration Descriptor - Title
  [FIG_IDX_PLAYBACK_TITLE_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(fig_data_notify_ccc), (uint8_t *)fig_data_notify_ccc}},
  // Characteristic Declaration - Artist
  [FIG_IDX_PLAYBACK_ARTIST_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
  // Characteristic Value - Artist
  [FIG_IDX_PLAYBACK_ARTIST_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)GATTS_CHAR_UUID_PLAYBACK_ARTIST, ESP_GATT_PERM_READ,
      GATTS_PLAYBACK_CHAR_VAL_LEN_MAX, sizeof(default_playback_value), (uint8_t *)default_playback_value}},
  // Client Characteristic Configuration Descriptor - Artist
  [FIG_IDX_PLAYBACK_ARTIST_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(fig_data_notify_ccc), (uint8_t *)fig_data_notify_ccc}},
  // Characteristic Declaration - Album
  [FIG_IDX_PLAYBACK_ALBUM_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
  // Characteristic Value - Album
  [FIG_IDX_PLAYBACK_ALBUM_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)GATTS_CHAR_UUID_PLAYBACK_ALBUM, ESP_GATT_PERM_READ,
      GATTS_PLAYBACK_CHAR_VAL_LEN_MAX, sizeof(default_playback_value), (uint8_t *)default_playback_value}},
  // Client Characteristic Configuration Descriptor - Album
  [FIG_IDX_PLAYBACK_ALBUM_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(fig_data_notify_ccc), (uint8_t *)fig_data_notify_ccc}},
  // Characteristic Declaration - Notification
  [FIG_IDX_PLAYBACK_STATE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
  // Characteristic Value - Notification
  [FIG_IDX_PLAYBACK_STATE_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)GATTS_CHAR_UUID_PLAYBACK_STATE, ESP_GATT_PERM_READ,
      GATTS_PLAYBACK_CHAR_VAL_LEN_MAX, sizeof(default_playback_value), (uint8_t *)default_playback_value}},
  // Client Characteristic Configuration Descriptor - Notification
  [FIG_IDX_PLAYBACK_STATE_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(fig_data_notify_ccc), (uint8_t *)fig_data_notify_ccc}},
  // Characteristic Declaration - Beats
  [FIG_IDX_BEATS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},
  // Characteristic Value - Beats
  [FIG_IDX_BEATS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)GATTS_CHAR_UUID_BEATS, ESP_GATT_PERM_READ,
      GATTS_PLAYBACK_CHAR_VAL_LEN_MAX, sizeof(default_playback_value), (uint8_t *)default_playback_value}},
  // Client Characteristic Configuration Descriptor - Beats
  [FIG_IDX_BEATS_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(fig_data_notify_ccc), (uint8_t *)fig_data_notify_ccc}},
};

// A2DP: I2S
#define I2S_BCK_IO_NUM 19
#define I2S_WS_IO_NUM 23
#define I2S_DATA_OUT_NUM 22

// Effects
uint16_t current_rate = 0;

#define STREAM_ENTRY_SIZE 128
#define DATA_MOVING_SIZE_BITS 5
#define DATA_MOVING_SIZE (2 << DATA_MOVING_SIZE_BITS)
#define CHANNEL_NUMS 2

#define MAX_MAG 14000
#define MAX_MAG_RANGE 0.6
#define BAR_SIZE 5

#define LED_A 21
#define LED_B 25

// A2DP
BluetoothA2DPSink a2dp_sink;

// Function definitions

void display(float* values /* length=5 */);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define AVRC_METADATA_DATA1_PLAYBACK_TITLE    0x01
#define AVRC_METADATA_DATA1_PLAYBACK_ARTIST   0x02
#define AVRC_METADATA_DATA1_PLAYBACK_ALBUM    0x04

int resolve_playback_char_index(uint8_t data1, bool* pnotify)
{
  if ((data1 & AVRC_METADATA_DATA1_PLAYBACK_TITLE) != 0x00) {
    if (pnotify != NULL) {
      *pnotify = enable_ble_playback_title_ntf;
    }
    return FIG_IDX_PLAYBACK_TITLE_VAL;
  }
  if ((data1 & AVRC_METADATA_DATA1_PLAYBACK_ARTIST) != 0x00) {
    if (pnotify != NULL) {
      *pnotify = enable_ble_playback_artist_ntf;
    }
    return FIG_IDX_PLAYBACK_ARTIST_VAL;
  }
  if ((data1 & AVRC_METADATA_DATA1_PLAYBACK_ALBUM) != 0x00) {
    if (pnotify != NULL) {
      *pnotify = enable_ble_playback_album_ntf;
    }
    return FIG_IDX_PLAYBACK_ALBUM_VAL;
  }
  return -1;
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
  uint8_t error = 0xff;

  for(int i = 0; i < FIG_IDX_NB ; i++){
    if(handle == fig_handle_table[i]){
      return i;
    }
  }
  return error;
}

static void gatts_event_handler_(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      Serial.println("ESP_GATTS_REG_EVT and ESP_GATT_OK");
    } else {
      Serial.printf("Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
      return;
    }
  }
  gatts_profile_event_handler(event, gatts_if, param);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
  uint8_t res = 0xff;
  esp_err_t ret;

  switch (event)
  {
    case ESP_GATTS_REG_EVT:
      ret = esp_ble_gap_config_adv_data(&adv_data);
      if (ret){
        Serial.printf("config adv data failed, error code = %x", ret);
        break;
      }
      ret = esp_ble_gatts_create_attr_tab(fig_gatt_db, gatts_if, FIG_IDX_NB, SVC_INST_ID);
      if (ret){
        Serial.printf("create attr table failed, error code = %x", ret);
        break;
      }          
      break;
    case ESP_GATTS_READ_EVT:
      break;
    case ESP_GATTS_WRITE_EVT:
    {
      res = find_char_and_desr_index(p_data->write.handle);
      if (p_data->write.is_prep == false) {
        Serial.printf("ESP_GATTS_WRITE_EVT : handle = %d\n", res);
        if(res == FIG_IDX_PLAYBACK_STATE_CFG){
          if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_state_ntf = true;
          }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_state_ntf = false;
          }
        }
        if(res == FIG_IDX_PLAYBACK_TITLE_CFG){
          if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_title_ntf = true;
          }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_title_ntf = false;
          }
        }
        if(res == FIG_IDX_PLAYBACK_ARTIST_CFG){
          if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_artist_ntf = true;
          }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_artist_ntf = false;
          }
        }
        if(res == FIG_IDX_PLAYBACK_ALBUM_CFG){
          if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_album_ntf = true;
          }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
            enable_ble_playback_album_ntf = false;
          }
        }
        if(res == FIG_IDX_BEATS_CFG){
          if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
            enable_ble_beats_ntf = true;
          }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
            enable_ble_beats_ntf = false;
          }
        }
      }
      break;
    }
    case ESP_GATTS_CONNECT_EVT:
      current_ble_conn_id = p_data->connect.conn_id;
      current_ble_gatts_if = gatts_if;
      is_ble_connected = true;
      break;
    case ESP_GATTS_DISCONNECT_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      is_ble_connected = false;
      break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
      Serial.printf("The number handle =%x\n",param->add_attr_tab.num_handle);
      if (param->add_attr_tab.status != ESP_GATT_OK){
        Serial.printf("Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
      }
      else if (param->add_attr_tab.num_handle != FIG_IDX_NB){
        Serial.printf("Create attribute table abnormally, num_handle (%d) doesn't equal to FIG_IDX_NB(%d)", param->add_attr_tab.num_handle, FIG_IDX_NB);
      }
      else {
        memcpy(fig_handle_table, param->add_attr_tab.handles, sizeof(fig_handle_table));
        esp_ble_gatts_start_service(fig_handle_table[FIG_IDX_SVC]);
      }
      break;
    }
    default:
      break;
  }
}

static void gap_event_handler_(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      //advertising start complete event to indicate advertising start successfully or failed
      if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
        Serial.printf("Advertising start failed: %s\n", esp_err_to_name(err));
      }
      break;
    default:
      break;
    }
}

void setup_ble() {
  is_ble_connected = false;
  esp_ble_gatts_register_callback(gatts_event_handler_);
  esp_ble_gap_register_callback(gap_event_handler_);
  esp_ble_gatts_app_register(ESP_FIG_APP_ID);
}

void send_ble_beats(size_t len, uint8_t* data) {
  if (!is_ble_connected) {
    return;
  }
  if (!enable_ble_beats_ntf) {
    return;
  }
  esp_err_t ret = esp_ble_gatts_send_indicate(current_ble_gatts_if, current_ble_conn_id, fig_handle_table[FIG_IDX_BEATS_VAL], len, data, false);
  if (ret){
    Serial.printf("notify beats data failed, error code = %x", ret);
    return;
  }
}

struct stream_entry
{
  int16_t* buffer;
  int filled;
  struct stream_entry* next;
};

struct stream_entry* head_stream_entry = NULL;
struct stream_entry* last_stream_entry = NULL;
struct stream_entry* filling_stream_entry = NULL;

void free_stream_entry(struct stream_entry* p)
{
  if (p->buffer) {
    free(p->buffer);
    p->buffer = NULL;
  }
  free(p);
}

struct stream_entry* alloc_stream_entry()
{
  struct stream_entry* r = (struct stream_entry*) malloc(sizeof(struct stream_entry));
  r->buffer = (int16_t*) malloc(sizeof(int16_t) * STREAM_ENTRY_SIZE);
  if (r->buffer == NULL) {
    return NULL;
  }
  r->filled = 0;
  r->next = NULL;
  return r;
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  if (!filling_stream_entry) {
    filling_stream_entry = alloc_stream_entry();
    if (filling_stream_entry == NULL) {
      Serial.printf("Memory cannot be allocated.\n");
      return;
    }
  }
  int16_t *values = (int16_t*) data;
  int count = length / DATA_MOVING_SIZE / 2 /*int16*/;
  if (filling_stream_entry->filled + count > STREAM_ENTRY_SIZE) {
    count = STREAM_ENTRY_SIZE - filling_stream_entry->filled;
  }
  for (int i = 0; i < count; i ++) {
    int src_offset = i * DATA_MOVING_SIZE;
    int dest_offset = filling_stream_entry->filled + i;
    int16_t* src = values + src_offset;
    long sum = src[0];
    for (int j = 1; j < DATA_MOVING_SIZE; j ++) {
      sum += src[j];
    }
    filling_stream_entry->buffer[dest_offset] = sum / DATA_MOVING_SIZE;
  }
  filling_stream_entry->filled += count;
  if (filling_stream_entry->filled < STREAM_ENTRY_SIZE) {
    //Serial.printf("Buffer processing: millis=%ld\n", millis());
    return;
  }
  //Serial.printf("Buffer filled: millis=%ld\n", millis());
  if (!head_stream_entry) {
    head_stream_entry = last_stream_entry = filling_stream_entry;
    filling_stream_entry = NULL;
    return;
  }
  last_stream_entry->next = filling_stream_entry;
  last_stream_entry = filling_stream_entry;  
  filling_stream_entry = NULL;
  // process all data
  // Serial.printf("Data received: millis=%ld, rate=%d, length=%d\n", millis(), current_rate, length);
}

void avrc_sample_rate_callback(uint16_t rate)
{
  current_rate = rate;
  Serial.printf("Rate changed: rate=%d\n", rate);
}

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2)
{
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
  bool enable_notify = false;
  int index = resolve_playback_char_index(data1, &enable_notify);
  if (index < 0) {
    return;
  }
  esp_err_t ret = esp_ble_gatts_set_attr_value(fig_handle_table[index], strlen((const char*) data2), data2);
  if (ret){
    Serial.printf("set attr data failed, error code = %x", ret);
    return;
  }
  if (!is_ble_connected) {
    return;
  }
  if (!enable_notify) {
    return;
  }
  uint8_t notify_data = 0;
  ret = esp_ble_gatts_send_indicate(current_ble_gatts_if, current_ble_conn_id, fig_handle_table[index], sizeof(notify_data), (uint8_t*) &notify_data, false);
  if (ret){
    Serial.printf("notify attr data failed, error code = %x", ret);
    return;
  }
}

void avrc_audio_state_callback(esp_a2d_audio_state_t state, void *ptr)
{
  Serial.printf("State changed: millis=%ld, state=%d\n", millis(), state);
  uint8_t state_value = state;
  esp_err_t ret = esp_ble_gatts_set_attr_value(fig_handle_table[FIG_IDX_PLAYBACK_STATE_VAL], sizeof(state_value), &state_value);
  if (ret){
    Serial.printf("set attr data failed, error code = %x", ret);
    return;
  }
  if (!is_ble_connected) {
    return;
  }
  if (!enable_ble_playback_state_ntf) {
    return;
  }
  ret = esp_ble_gatts_send_indicate(current_ble_gatts_if, current_ble_conn_id, fig_handle_table[FIG_IDX_PLAYBACK_STATE_VAL], sizeof(state_value), &state_value, false);
  if (ret){
    Serial.printf("notify attr data failed, error code = %x", ret);
    return;
  }
}

#define DEFAULT_VOLUME 3

void setup()
{
  M5.begin(true, false, true);
  
  M5.dis.fillpix(0xffffff);
  M5.dis.setBrightness(10);
  
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);

  i2s_pin_config_t my_pin_config = {
    .bck_io_num = I2S_BCK_IO_NUM,
    .ws_io_num = I2S_WS_IO_NUM,
    .data_out_num = I2S_DATA_OUT_NUM,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_stream_reader(read_data_stream);  
  a2dp_sink.set_sample_rate_callback(avrc_sample_rate_callback);
  a2dp_sink.set_on_audio_state_changed (avrc_audio_state_callback);
  a2dp_sink.start("M5Atom Speaker");
  Serial.printf("A2DP started!\n");
  a2dp_sink.set_volume(DEFAULT_VOLUME);
  
  setup_ble();

  float boot[BAR_SIZE];
  for (int i = 0; i < BAR_SIZE; i ++) {
    boot[i] = 0;
  }
  display(boot);
}

void loop()
{
  if (!current_rate) {
    delay(100); // do nothing
    return;
  }
  int wait = (1000000 / current_rate * STREAM_ENTRY_SIZE * DATA_MOVING_SIZE / CHANNEL_NUMS) / 1000;
  struct stream_entry* entry = head_stream_entry;
  if (!entry) {
    DEBUGF("No fulfilled buffer: millis=%ld, wait=%d\n", millis(), wait);
    delay(wait);
    return;
  }
  head_stream_entry = entry->next;
  if (head_stream_entry == NULL) {
    last_stream_entry = NULL;
  }
  // Serial.printf("Buffer processing started: millis=%ld, wait=%d, size=%d\n", millis(), wait, entry->filled);
  unsigned long st = millis();
  FFT(entry->buffer, STREAM_ENTRY_SIZE, ((float)current_rate) * CHANNEL_NUMS / DATA_MOVING_SIZE);
  unsigned long et = millis();
  int overhead = et - st;
  DEBUGF("Buffer processing finished: millis=%ld, wait=%d, size=%d\n", millis(), wait, entry->filled);
  free_stream_entry(entry);
  //Serial.printf("Overhead = %d\n", overhead);
  if (overhead > wait) {
    overhead = 0;
  }
  delay(wait - overhead);
}

void setLED(uint8_t led, float value)
{
  if (value > 0.5) {
    digitalWrite(led, HIGH);    
  } else {
    digitalWrite(led, LOW);
  }
}

void display(float* values /* length=5 */)
{
  float rgb[3];
  for (int x = 0; x < BAR_SIZE; x ++) {
    hsv2rgb(((float)x) / BAR_SIZE * 0.3, 1.0, 1.0, rgb);
    CRGB disprgb = (((unsigned int) (rgb[0] * 255)) << 16) | (((unsigned int) (rgb[1] * 255)) << 8) | ((unsigned int) (rgb[2] * 255));
    for (int y = 0; y < BAR_SIZE; y++) {
      if (values[x] * BAR_SIZE <= y) {
        M5.dis.drawpix(x, BAR_SIZE - y - 1, 0x000000);
        continue;
      }
      M5.dis.drawpix(x, BAR_SIZE - y - 1, disprgb);
    }
  }
  setLED(LED_A, values[0]);
  setLED(LED_B, values[1]);
  uint8_t beats[BAR_SIZE];
  for (int i = 0; i < BAR_SIZE; i ++) {
    beats[i] = (uint8_t)(values[i] * 255);
  }
  send_ble_beats(sizeof(uint8_t) * BAR_SIZE, beats);
}

void FFT(int16_t* input, int samples, float Fs)
{
  uint32_t mag[samples] = {0};
  uint16_t startIndex = 0;
  uint16_t endIndex = 0;
  KickFFT<int16_t>::fft(Fs, 0, Fs/2, samples, input, mag, startIndex, endIndex);

  int bar_width = (int)(((float)samples) * MAX_MAG_RANGE / 5);
  float bars[BAR_SIZE];
  for (int i = 0; i < BAR_SIZE; i ++) {
    int16_t max_value = 0;
    int end = (i + 1) * bar_width;
    if (i == BAR_SIZE - 1) {
      end = samples;
    }
    for (int j = i * bar_width; j < end; j ++) {
      if (j < startIndex) {
        continue;
      }
      if (j >= endIndex) {
        continue;
      }
      if (j >= samples) {
        continue;
      }
      uint32_t mag_value = mag[j];
      if (max_value >= mag_value) {
        continue;
      }
      max_value = mag_value;
    }
    bars[i] = max_value > MAX_MAG ? 1.0 : (((float)max_value) / MAX_MAG);
  }
  display(bars);
}

// https://gist.github.com/postspectacular/2a4a8db092011c6743a7

// HSV->RGB conversion based on GLSL version
// expects hsv channels defined in 0.0 .. 1.0 interval
float fract(float x) { return x - int(x); }

float mix(float a, float b, float t) { return a + (b - a) * t; }

float step(float e, float x) { return x < e ? 0.0 : 1.0; }

float* hsv2rgb(float h, float s, float b, float* rgb) {
  rgb[0] = b * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[1] = b * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[2] = b * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  return rgb;
}

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}
