#ifndef ESP32CAM_H
#define ESP32CAM_H

#include <esp_camera.h>

//const framesize_t frameSizes[] = {
//  FRAMESIZE_QQVGA,  //  160x120 0
//  FRAMESIZE_QVGA,   //  320x240 1
//  FRAMESIZE_CIF,    //  400x296 2
//  FRAMESIZE_VGA,    //  640x480 3
//  FRAMESIZE_SVGA,   //  800x600 4
//  FRAMESIZE_XGA,    // 1024x768 5
//  FRAMESIZE_SXGA,   //1280x1024 6
//  FRAMESIZE_UXGA,   //1600x1200 7
//  FRAMESIZE_QXGA,   //2048x1536 8
//  FRAMESIZE_QSXGA,  //2560x2048 9
//};

int ledPin = 4; // Set the pin that the LED is connected to
int ledcChannel = 4; // Set the LEDC channel for the LED
int ledcFrequency = 5000; // Set the LEDC frequency to 5000 Hz
int led_bit = 13; // how many bit the pwm has

long last_frame_time = 0;
float FPS = 5.0; //frames per second

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapExponent(double x, double in_min, double in_max, double out_min, double out_max, double exponent) {
  // Use mapDouble() to scale the input value to the range [0,1]
  double scaledValue = mapDouble(x, in_min, in_max, 0.0, 1.0);
  // Raise the scaled value to the power of the exponent
  double exponentiatedValue = pow(scaledValue, exponent);
  // Use mapDouble() to scale the exponentiated value back to the output range
  double mappedValue = mapDouble(exponentiatedValue, 0.0, 1.0, out_min, out_max);
  return mappedValue;
}

void setLEDBrightness(double brightness) {
  // check for legal input
  if (brightness < 0.5) {
    brightness = 0;
  } else if (brightness > 100) {
    brightness = 1 << led_bit;
  } else {
    /*
     * a linear mapping is too bright
     * i needed more sensitivity in the lower brightness levels
     */
    //make exponential
    double inputMin = 0.0;     // minimum value of input range
    double inputMax = 100.0;   // maximum value of input range
    double outputMin = 1.0;    // minimum value of output range
    double outputMax = double((1 << led_bit) - 1); // maximum value of output range
    double exponent = 3;     // exponent value

    // Use the mapExponent() function to map the input value
    brightness = mapExponent(brightness, inputMin, inputMax, outputMin, outputMax, exponent);
  }
  //check for legal output
  if (brightness < 0 ) {
    brightness = 0;
  }
  if (brightness > 1 << led_bit ) {
    brightness = 1 << led_bit;
  }
  ledcWrite(ledcChannel, brightness);  // Set the LED brightness to the specified value
}

//0 - 63
void setQuality(int level) {
  level = min(level, 63);
  level = max(0, level);
  sensor_t * s = esp_camera_sensor_get();
  s->set_quality(s, level);
}

void setResolution(String resolution) {
  //parse the String to get the number of pixels
  int width = 0;
  int height = 0;
  int numPixels = 0;
  for (int i = 0; i < resolution.length(); i++) {
    if (resolution[i] == 'x') {
      width = resolution.substring(0, i).toInt();
      height = resolution.substring(i + 1).toInt();
      break;
    }
  }

  //set the resolution
  sensor_t * s = esp_camera_sensor_get();
  numPixels = width * height;
  numPixels = min(numPixels , 5308416);
  if (numPixels <= 19200) {
    s->set_framesize(s, FRAMESIZE_QQVGA);
    Serial.println("Resolution set to 160x120");
  } else if (numPixels <= 76800) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    Serial.println("Resolution set to 320x240");
  } else if (numPixels <= 115200) {
    s->set_framesize(s, FRAMESIZE_CIF);
    Serial.println("Resolution set to 400x296");
  } else if (numPixels <= 307200) {
    s->set_framesize(s, FRAMESIZE_VGA);
    Serial.println("Resolution set to 640x480");
  } else if (numPixels <= 480000) {
    s->set_framesize(s, FRAMESIZE_SVGA);
    Serial.println("Resolution set to 800x600");
  } else if (numPixels <= 786432) {
    s->set_framesize(s, FRAMESIZE_XGA);
    Serial.println("Resolution set to 1024x768");
  } else if (numPixels <= 1310720) {
    s->set_framesize(s, FRAMESIZE_SXGA);
    Serial.println("Resolution set to 1280x1024");
  } else if (numPixels <= 1920000) {
    s->set_framesize(s, FRAMESIZE_UXGA);
    Serial.println("Resolution set to 1600x1200");
  } else if (numPixels <= 3145728) {
    s->set_framesize(s, FRAMESIZE_QXGA);
    Serial.println("Resolution set to 2048x1536");
  } else {
    s->set_framesize(s, FRAMESIZE_QSXGA);
    Serial.println("Resolution set to 2560x2048");
  }
}

//default biggest possible to allocate all ram
void cam_setup(framesize_t framesize = FRAMESIZE_QSXGA) {
  const int PWDN_GPIO_NUM     = 32;
  const int RESET_GPIO_NUM    = -1;
  const int XCLK_GPIO_NUM      = 0;
  const int SIOD_GPIO_NUM     = 26;
  const int SIOC_GPIO_NUM     = 27;
  const int Y9_GPIO_NUM       = 35;
  const int Y8_GPIO_NUM       = 34;
  const int Y7_GPIO_NUM       = 39;
  const int Y6_GPIO_NUM       = 36;
  const int Y5_GPIO_NUM       = 21;
  const int Y4_GPIO_NUM       = 19;
  const int Y3_GPIO_NUM       = 18;
  const int Y2_GPIO_NUM        = 5;
  const int VSYNC_GPIO_NUM    = 25;
  const int HREF_GPIO_NUM     = 23;
  const int PCLK_GPIO_NUM     = 22;

  // Initialize the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;//20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = framesize;
  //  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x, restarting", err);
    ESP.restart();
    return;
  }
}

void print_settings() {
  static char output_string[1024];
  sensor_t * s = esp_camera_sensor_get();
  char * p = output_string;
  *p++ = '{';
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
  *p++ = '}';
  *p++ = 0;
  Serial.println(output_string);
}


void sendMqttFrame(PubSubClient *mqttClient) {

  // change camera clock
  // this will unfuck jpeg
  sensor_t * s = esp_camera_sensor_get();
  s->set_reg(s, 0xff, 0xff, 0x00); //banksel
  s->set_reg(s, 0xd3, 0xff, 5); //clock

  // limit framerate
  long current_time = millis();
  float elapsed_time = current_time - last_frame_time;
  float remaining_time = 1000.0 / FPS - elapsed_time;
  delay(max(1, int(remaining_time)));
  last_frame_time = current_time;

  // Capture a frame from the camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb) { // If the frame couldn't be captured, print an error message
    Serial.println("ERROR failed to capture frame from camera");
    esp_camera_fb_return(fb); // Return the frame buffer to the camera driver
    return;
  }

  // change the current buffer size
  // default 128 bytes
  int new_buffer_size = 1 << 13;
  if (mqttClient->getBufferSize() < new_buffer_size ) {
    // Set the new buffer size
    if (mqttClient->setBufferSize(new_buffer_size )) {
      Serial.println("Successfully set new buffer size");
    } else {
      Serial.println("Failed to set new buffer size");
      esp_camera_fb_return(fb); // Return the frame buffer to the camera driver
      return;
    }
    Serial.print("buffer size is now:");
    Serial.println(mqttClient->getBufferSize());
  }

  // Publish the image data to the MQTT topic
  int overhead = 192; // extra space for message overhead
  int image_size = fb->len;
  int buffer_size = mqttClient->getBufferSize() - overhead;

  int num_parts = image_size / buffer_size + 1;  // Calculate the number of parts needed to send the image
  int remaining = image_size;  // Keep track of how much data is left to send

  for (int i = 0; i <= num_parts - 1; i++) {
    //      s->set_reg(s, 0xff, 0xff, 0x00); //banksel
    //  s->set_reg(s, 0xd3, 0xff, 5); //clock
    int part_size = min(buffer_size, remaining);  // Calculate the size of the current part

    // Publish the current part of the image data to the MQTT topic
    int result = mqttClient->publish((String("esp32cam/50/image/part/") + String(i)).c_str(), (const uint8_t*)fb->buf + (image_size - remaining), part_size, true);
    if (result == 1) {
      //Serial.print("SUCCESS published image part data to MQTT topic");
      //Serial.println(i);
    } else {
      Serial.printf("ERROR Failed to publish image part data %d to MQTT topic with error 0x%x \n", i, result);
      esp_camera_fb_return(fb); // Return the frame buffer to the camera driver
      //      ESP.restart();
      return;
    }
    remaining -= part_size;  // Decrement the remaining data by the size of the current part
  }

  // Publish the 'esp32cam/done' topic with the number of parts
  mqttClient->publish("esp32cam/50/image_complete", String(num_parts).c_str(), false);
  esp_camera_fb_return(fb); // Return the frame buffer to the camera driver
}

#endif
