#include <iostream>
#include <fstream>
#include "constants.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bme68x.h"
#include <chrono>
#include <cstring>
#include <bcm2835.h>

#define BSEC_CHECK_INPUT(x, shift)		(x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR             UINT16_C(140)
#define NUM_USED_OUTPUTS 7

typedef void (*output_ready_fct)(int64_t timestamp, float gas_estimate_1, float gas_estimate_2, float gas_estimate_3, float gas_estimate_4,
    float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, uint8_t raw_gas_index, bsec_library_return_t bsec_status, float temp, float hum, float iaq, float co2);

/* Global sensor APIs data structure */
static struct bme68x_dev bme68x_g;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heatr_conf;
static struct bme68x_data sensor_data[3];

uint8_t dev_addr = BME68X_I2C_ADDR_LOW;

/* State change and temporary data place holders */
uint8_t lastOpMode = BME68X_SLEEP_MODE;
float extTempOffset = 0.0f;
uint8_t opMode;
uint8_t nFields, iFields;

int64_t get_timestamp_us()
{
    auto currentTime = std::chrono::high_resolution_clock::now();
    return  std::chrono::time_point_cast<std::chrono::microseconds>(currentTime).time_since_epoch().count();;
}

uint32_t getMeasDur(uint8_t mode)
{
    if (mode == BME68X_SLEEP_MODE)
        mode = lastOpMode;

    return bme68x_get_meas_dur(mode, &conf, &bme68x_g);
}

uint8_t getData(struct bme68x_data* data)
{
    if (lastOpMode == BME68X_FORCED_MODE)
    {
        *data = sensor_data[0];
    }
    else
    {
        if (nFields)
        {
            /* iFields spans from 0-2 while nFields spans from
             * 0-3, where 0 means that there is no new data
             */
            *data = sensor_data[iFields];
            iFields++;

            /* Limit reading continuously to the last fields read */
            if (iFields >= nFields)
            {
                iFields = nFields - 1;
                return 0;
            }

            /* Indicate if there is something left to read */
            return nFields - iFields;
        }
    }

    return 0;
}

static bsec_library_return_t bme68x_bsec_process_data(bsec_input_t* bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready)
{
    /* Output buffer set to the maximum virtual sensor outputs supported */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = 0;
    uint8_t index = 0;

    bsec_library_return_t bsec_status = BSEC_OK;

    int64_t timestamp = 0;
    float gas_estimate_1 = 0.0f;
    float gas_estimate_2 = 0.0f;
    float gas_estimate_3 = 0.0f;
    float gas_estimate_4 = 0.0f;
    float raw_pressure = 0.0f;
    float raw_temp = 0.0f;
    float raw_humidity = 0.0f;
    float raw_gas = 0.0f;
    uint8_t raw_gas_index = 0;
    float temp = 0.0f;
    float humidity = 0.0f;
    float co2 = 0.0f;
    float iaq = 0.0f;

    /* Check if something should be processed by BSEC */
    if (num_bsec_inputs > 0)
    {
        /* Set number of outputs to the size of the allocated buffer */
        /* BSEC_NUMBER_OUTPUTS to be defined */
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;

        /* Perform processing of the data by BSEC
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme68x_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps(bsec_inputs, num_bsec_inputs, bsec_outputs, &num_bsec_outputs);

        /* Iterate through the outputs and extract the relevant ones. */
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
            case BSEC_OUTPUT_GAS_ESTIMATE_1:
                gas_estimate_1 = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_2:
                gas_estimate_2 = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_3:
                gas_estimate_3 = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_4:
                gas_estimate_4 = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                raw_pressure = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                raw_temp = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                raw_humidity = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_RAW_GAS:
                raw_gas = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_RAW_GAS_INDEX:
                raw_gas_index = (uint8_t)bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                co2 = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_IAQ:
                iaq = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                humidity = bsec_outputs[index].signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                temp = bsec_outputs[index].signal;
                break;
            default:
                continue;
            }

            /* Assume that all the returned timestamps are the same */
            timestamp = bsec_outputs[index].time_stamp;
        }

        /* Pass the extracted outputs to the user provided output_ready() function. */
        output_ready(timestamp, gas_estimate_1, gas_estimate_2, gas_estimate_3, gas_estimate_4, raw_pressure, raw_temp, raw_humidity, raw_gas, raw_gas_index, bsec_status, temp, humidity, iaq, co2);
    }
    return bsec_status;
}

uint8_t processData(int64_t currTimeNs, struct bme68x_data data, int32_t bsec_process_data, output_ready_fct output_ready)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
    bsec_library_return_t bsec_status = BSEC_OK;
    uint8_t nInputs = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[nInputs].signal = extTempOffset;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_TEMPERATURE))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE / 100.0f;
#endif
        inputs[nInputs].signal = data.temperature;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HUMIDITY))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY / 1000.0f;
#endif
        inputs[nInputs].signal = data.humidity;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[nInputs].signal = data.pressure;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_GASRESISTOR) &&
        (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[nInputs].signal = data.gas_resistance;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PROFILE_PART) &&
        (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[nInputs].signal = (opMode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }

    if (nInputs > 0)
    {
        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
        bsec_status = bme68x_bsec_process_data(inputs, nInputs, output_ready);

        if (bsec_status != BSEC_OK)
            return 0;
    }
    return 1;
}

void setBme68xConfigForced(bsec_bme_settings_t* sensor_settings)
{
    int8_t status;

    /* Set the filter, odr, temperature, pressure and humidity settings */
    status = bme68x_get_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
    {
        std::cout << "Set forced mode failed. Get conf returned: " << (int)status << std::endl;
        return;
    }

    conf.os_hum = sensor_settings->humidity_oversampling;
    conf.os_temp = sensor_settings->temperature_oversampling;
    conf.os_pres = sensor_settings->pressure_oversampling;
    status = bme68x_set_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
    {
        std::cout << "Set forced mode failed. Set conf returned: " << (int)status << std::endl;
        return;
    }

    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = sensor_settings->heater_temperature;
    heatr_conf.heatr_dur = sensor_settings->heater_duration;
    status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x_g);
    if (status != BME68X_OK)
    {
        std::cout << "Set forced mode failed. Set heatr conf returned: " << (int)status << std::endl;
        return;
    }

    status = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_g);
    if (status != BME68X_OK)
    {
        std::cout << "Set forced mode failed. Set op mode returned: " << (int)status << std::endl;
        return;
    }

    lastOpMode = BME68X_FORCED_MODE;
    opMode = BME68X_FORCED_MODE;
}

void setBme68xConfigParallel(bsec_bme_settings_t* sensor_settings)
{
    uint16_t sharedHeaterDur = 0;
    int8_t status;

    /* Set the filter, odr, temperature, pressure and humidity settings */
    status = bme68x_get_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
        return;

    conf.os_hum = sensor_settings->humidity_oversampling;
    conf.os_temp = sensor_settings->temperature_oversampling;
    conf.os_pres = sensor_settings->pressure_oversampling;
    status = bme68x_set_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
        return;


    sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = sensor_settings->heater_temperature_profile;
    heatr_conf.heatr_dur_prof = sensor_settings->heater_duration_profile;
    heatr_conf.shared_heatr_dur = sharedHeaterDur;
    heatr_conf.profile_len = sensor_settings->heater_profile_len;
    status = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme68x_g);
    if (status != BME68X_OK)
        return;

    status = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme68x_g);
    if (status != BME68X_OK)
        return;

    lastOpMode = BME68X_PARALLEL_MODE;
    opMode = BME68X_PARALLEL_MODE;
}

void output_ready(int64_t timestamp, float gas_estimate_1, float gas_estimate_2, float gas_estimate_3, float gas_estimate_4,
    float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, uint8_t raw_gas_index, bsec_library_return_t bsec_status, float temp, float humidity, float iaq, float co2)
{
    std::cout << "DATA. Raw Pressure: " << raw_pressure << ". Raw Temp: " << raw_temp << ". Raw Humidity: " << raw_humidity << ". Raw Gas: " << raw_gas << ". Raw Gas Index: " << raw_gas_index << ". Temp: " << temp << ". Humidity: " << humidity << ". IAQ: " << iaq << ". CO2: " << co2 << std::endl;
}

static bsec_library_return_t bme68x_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_library_return_t status = BSEC_OK;

    /* note: Virtual sensors as desired to be added here */
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    //requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    //requested_virtual_sensors[0].sample_rate = sample_rate;
    //requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    //requested_virtual_sensors[1].sample_rate = sample_rate;
    //requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_3;
    //requested_virtual_sensors[2].sample_rate = sample_rate;
    //requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_4;
    //requested_virtual_sensors[3].sample_rate = sample_rate;
    //requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    //requested_virtual_sensors[7].sample_rate = sample_rate;
    //requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_RAW_GAS_INDEX;
    //requested_virtual_sensors[8].sample_rate = sample_rate;

    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);

    return status;
}

typedef struct {
    /*! Result of API execution status */
    int8_t bme68x_status;
    /*! Result of BSEC library */
    bsec_library_return_t bsec_status;
}return_values_init;

int main()
{
    return_values_init ret = { BME68X_OK, BSEC_OK };
    memset(&bme68x_g, 0, sizeof(bme68x_g));
    if (!bcm2835_init()) {
        std::cerr << "bcm2835_init failed. Are you running as root?" << std::endl;
        return 1;
    }

    if (!bcm2835_i2c_begin()) {
        printf("bcm2835_i2c_begin failed. Are you running as root?\n");
        return 1;
    }

    bcm2835_i2c_set_baudrate(100000);

    bme68x_g.intf = BME68X_I2C_INTF;
    bme68x_g.read = [](uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* stuff) -> int8_t {
        bcm2835_i2c_setSlaveAddress(0x77);
        bcm2835_i2c_read_register_rs(reinterpret_cast<char*>(&reg_addr), reinterpret_cast<char*>(reg_data), len);
        return 0;
        };
    bme68x_g.write = [](uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* stuff) -> int8_t {
        bcm2835_i2c_setSlaveAddress(0x77);
        char buf[len + 1];
        buf[0] = reg_addr;
        for (uint16_t i = 0; i < len; ++i) {
            buf[i + 1] = reg_data[i];
        }
        bcm2835_i2c_write(buf, len + 1);
        return 0;
        };
    bme68x_g.delay_us = [](uint32_t period, void* stuff) {
        bcm2835_delayMicroseconds(period);
        };
    bme68x_g.amb_temp = 20;

    ret.bme68x_status = bme68x_init(&bme68x_g);
    if (ret.bme68x_status != BME68X_OK)
    {
        std::cout << "Initialize of bme68x sensor failed with status: " << (int)ret.bme68x_status << std::endl;
        return -7;
    }

    auto x = bsec_init();
    if (x == bsec_library_return_t::BSEC_OK)
    {
        std::cout << "Init success!" << std::endl;
    }
    else
    {
        std::cout << "Init failed: " << x << std::endl;
    }

    uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = { 0 };
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = { 0 };


    auto bsec_status = bsec_set_configuration(bsec_config_selectivity, sizeof(bsec_config_selectivity), work_buffer, sizeof(work_buffer));
    if (bsec_status != BSEC_OK)
    {
        return -1;
    }

    auto sample_rate = BSEC_SAMPLE_RATE_LP;
    auto bsec_status2 = bme68x_bsec_update_subscription(sample_rate);

    if (bsec_status2 != BSEC_OK)
    {
        std::cout << "Subscription failed: " << bsec_status2 << std::endl;
        return -2;
    }

    /* Timestamp variables */
    int64_t time_stamp = 0;

    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
    memset(&sensor_settings, 0, sizeof(sensor_settings));

    /* BSEC sensor data */
    struct bme68x_data data;

    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t nFieldsLeft = 0;
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    int8_t ret_val;

    opMode = sensor_settings.op_mode;

    bsec_status = BSEC_OK;
    sensor_settings.next_call = 0;

    while (1)
    {
        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = get_timestamp_us() * 1000;
        if (time_stamp >= sensor_settings.next_call)
        {
            /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
            bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
            if (bsec_status != BSEC_OK)
            {
                if (bsec_status < BSEC_OK)
                {
                    printf("ERROR: bsec_sensor_control: %d\n", bsec_status);
                    break;
                }
                else
                {
                    printf("WARNING: bsec_sensor_control: %d\n", bsec_status);
                }
            }
            switch (sensor_settings.op_mode)
            {
            case BME68X_FORCED_MODE:
                std::cout << "FORCED MODE" << std::endl;
                setBme68xConfigForced(&sensor_settings);
                break;
            case BME68X_PARALLEL_MODE:
                std::cout << "PARALLEL MODE" << std::endl;
                if (opMode != sensor_settings.op_mode)
                {
                    setBme68xConfigParallel(&sensor_settings);
                }
                break;
            case BME68X_SLEEP_MODE:
                std::cout << "SLEEP MODE" << std::endl;
                if (opMode != sensor_settings.op_mode)
                {
                    ret_val = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme68x_g);
                    if ((ret_val == BME68X_OK) && (opMode != BME68X_SLEEP_MODE))
                    {
                        opMode = BME68X_SLEEP_MODE;
                    }
                }
                break;
            }

            if (sensor_settings.trigger_measurement && sensor_settings.op_mode != BME68X_SLEEP_MODE)
            {
                nFields = 0;
                auto getResult = bme68x_get_data(lastOpMode, &sensor_data[0], &nFields, &bme68x_g);
                iFields = 0;
                if (nFields)
                {
                    std::cout << "Got data: " << nFields << std::endl;
                    do
                    {
                        nFieldsLeft = getData(&data);
                        /* check for valid gas data */
                        if (data.status & BME68X_GASM_VALID_MSK)
                        {
                            if (!processData(time_stamp, data, sensor_settings.process_data, output_ready))
                            {
                                return 7;
                            }
                        }
                    } while (nFieldsLeft);
                }
                else
                {
                    std::cout << "Got no data. Result: " << (int)getResult << std::endl;
                }
            }

            /* Increment sample counter */
            n_samples++;
        }
    }

    bcm2835_i2c_end();
    bcm2835_close();

    return 0;
}
