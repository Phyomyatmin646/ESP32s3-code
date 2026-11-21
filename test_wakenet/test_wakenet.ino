#include <Arduino.h>
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"

extern "C" {
    extern const esp_wn_iface_t esp_sr_wakenet9_quantized;
}

void setup() {
    Serial.begin(115200);
    srmodel_list_t* models = esp_srmodel_init("model");
    if (models) {
        char* model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp");
        const esp_wn_iface_t* wakenet = esp_wn_handle_from_name(model_name ? model_name : "wn9_hiesp");
        if (!wakenet) wakenet = &esp_sr_wakenet9_quantized;
        if (wakenet) {
            model_iface_data_t* model_data = wakenet->create("wn9_hiesp", DET_MODE_90);
            if (model_data) {
                int chunk = wakenet->get_samp_chunksize(model_data);
                Serial.printf("WakeNet9 chunk size: %d\n", chunk);
                wakenet->destroy(model_data);
            }
        }
    }
}

void loop() {
    delay(1000);
}
