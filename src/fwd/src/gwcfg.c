/*!>
 *  ____  ____      _    ____ ___ _   _  ___  
 *  |  _ \|  _ \    / \  / ___|_ _| \ | |/ _ \ 
 *  | | | | |_) |  / _ \| |  _ | ||  \| | | | |
 *  | |_| |  _ <  / ___ \ |_| || || |\  | |_| |
 *  |____/|_| \_\/_/   \_\____|___|_| \_|\___/ 
 *
 * Dragino_gw_fwd -- An opensource lora gateway forward 
 *
 * See http://www.dragino.com for more information about
 * the lora gateway project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2. See the LICENSE file
 * at the top of the source tree.
 *
 * Maintainer: skerlan
 *
 */

/*!>!
 * \file
 * \brief gateway configure, parse json file
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <arpa/inet.h>

#include "fwd.h"
#include "parson.h"
#include "loragw_aux.h"
#include "loragw_hal.h"

DECLARE_GW;

#ifdef SX1302MOD  /*!> ####################### defined SX1302  */

static int parse_SX130x_configuration(const char* conf_file) {
    int i, j, number;
    uint32_t rf_freq_hz[LGW_RF_CHAIN_NB];
    char param_name[32];  /*!> used to generate variable parameter names */
    char param_value[48]; /*!> used to convert variable parameter to string */
    const char *str; /*!> used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";

    JSON_Value *root_val = NULL;
    JSON_Value *val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_txgain_obj;
    JSON_Object *conf_ts_obj;
    JSON_Array *conf_txlut_array;
    JSON_Object *conf_sx1261_obj = NULL;
    JSON_Object *conf_scan_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Array *conf_lbtchan_array = NULL;
    JSON_Array *conf_demod_array = NULL;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_conf_demod_s demodconf;
    struct lgw_conf_ftime_s tsconf;
    struct lgw_conf_sx1261_s sx1261conf;
    bool sx1250_tx_lut;
    uint32_t sf, bw, fdev;
	size_t size;

    /*!> try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        lgw_log(LOG_INFO, "%s[SETTING] %s is not a valid JSON file\n", ERRMSG, conf_file);
        return -1;
    }

    /*!> point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does contain a JSON object named %s, parsing SX130x parameters\n", conf_file, conf_obj_name);
    }

    /*!> set board configuration */
    memset(&boardconf, 0, sizeof(boardconf)); /*!> initialize configuration structure */
	boardconf.com_type = LGW_COM_SPI;        
    str = json_object_get_string(conf_obj, "spidev_path");
    if (str != NULL) {
        strncpy(boardconf.com_path, str, sizeof(boardconf.com_path));
        strncpy(GW.hal.spidev_path, str, sizeof(GW.hal.spidev_path));
        boardconf.com_path[sizeof(boardconf.com_path) - 1] = '\0'; 
    } else {
        lgw_log(LOG_INFO, "%s[SETTING] spidev path must be configured in %s\n", ERRMSG, conf_file);
        return -1;
    }

    val = json_object_get_value(conf_obj, "lorawan_public"); 
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        lgw_log(LOG_INFO, "%s[SETTING] Data type for lorawan_public seems wrong, please check\n", WARNMSG);
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); 
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        lgw_log(LOG_INFO, "%s[SETTING] Data type for clksrc seems wrong, please check\n", WARNMSG);
        boardconf.clksrc = 0;
    }
    val = json_object_get_value(conf_obj, "full_duplex"); 
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.full_duplex = (bool)json_value_get_boolean(val);
    } else {
        lgw_log(LOG_INFO, "%s[SETTING] Data type for full_duplex seems wrong, please check\n", WARNMSG);
        boardconf.full_duplex = false;
    }
    lgw_log(LOG_INFO, "[INFO~][SETTING] spidev_path %s, lorawan_public %d, clksrc %d, full_duplex %d\n", boardconf.com_path, boardconf.lorawan_public, boardconf.clksrc, boardconf.full_duplex);
    /*!> all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        lgw_log(LOG_INFO, "%s[SETTING] Failed to configure board\n", ERRMSG);
        return -1;
    }

    /*!> set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); 
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            GW.hal.antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_INFO, "%s[SETTING] Data type for antenna_gain seems wrong, please check\n", WARNMSG);
            GW.hal.antenna_gain = 0;
        }
    }
    lgw_log(LOG_INFO, "[INFO~][SETTING] antenna_gain %d dBi\n", GW.hal.antenna_gain);

    /*!> set timestamp configuration */
    conf_ts_obj = json_object_get_object(conf_obj, "fine_timestamp");
    if (conf_ts_obj == NULL) {
        MSG("[INFO~][SETTING] %s does not contain a JSON object for fine timestamp\n", conf_file);
    } else {
        val = json_object_get_value(conf_ts_obj, "enable"); 
        if (json_value_get_type(val) == JSONBoolean) {
            tsconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG("%s[SETTING] Data type for fine_timestamp.enable seems wrong, please check\n", WARNMSG);
            tsconf.enable = false;
        }
        if (tsconf.enable == true) {
            str = json_object_get_string(conf_ts_obj, "mode");
            if (str == NULL) {
                MSG("%s[SETTING] fine_timestamp.mode must be configured in %s\n", ERRMSG, conf_file);
                return -1;
            } else if (!strncmp(str, "high_capacity", 13) || !strncmp(str, "HIGH_CAPACITY", 13)) {
                tsconf.mode = LGW_FTIME_MODE_HIGH_CAPACITY;
            } else if (!strncmp(str, "all_sf", 6) || !strncmp(str, "ALL_SF", 6)) {
                tsconf.mode = LGW_FTIME_MODE_ALL_SF;
            } else {
                MSG("%s[SETTING] invalid fine timestamp mode: %s (should be high_capacity or all_sf)\n", ERRMSG, str);
                return -1;
            }
            MSG("[INFO~][SETTING] Configuring precision timestamp with %s mode\n", str);
            
            /*!> all parameters parsed, submitting configuration to the HAL */
            if (lgw_ftime_setconf(&tsconf) != LGW_HAL_SUCCESS) {
                MSG("%s[SETTING] Failed to configure fine timestamp\n", ERRMSG);
                return -1;
            }
        } else {
            MSG("[INFO~][SETTING] Configuring legacy timestamp\n");
        }
    }

    /*!> set SX1261 configuration */
    memset(&sx1261conf, 0, sizeof sx1261conf); /*!> initialize configuration structure */
    conf_sx1261_obj = json_object_get_object(conf_obj, "sx1261_conf"); 
    if (conf_sx1261_obj == NULL) {
        MSG("[INFO~][SETTING] no configuration for SX1261\n");
    } else {
        /*!> Global SX1261 configuration */
        str = json_object_get_string(conf_sx1261_obj, "spi_path");
        if (str != NULL) {
            strncpy(sx1261conf.spi_path, str, sizeof sx1261conf.spi_path);
            sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; 
        } else {
            MSG("[INFO~][SETTING] SX1261 spi_path is not configured in %s\n", conf_file);
        }
        val = json_object_get_value(conf_sx1261_obj, "rssi_offset"); 
        if (json_value_get_type(val) == JSONNumber) {
            sx1261conf.rssi_offset = (int8_t)json_value_get_number(val);
        } else {
            MSG("%s[SETTING] Data type for sx1261_conf.rssi_offset seems wrong, please check\n", WARNMSG);
            sx1261conf.rssi_offset = 0;
        }

        /*!> Spectral Scan configuration */
        conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); 
        if (conf_scan_obj == NULL) {
            MSG("[INFO~][SETTING] no configuration for Spectral Scan\n");
        } else {
            val = json_object_get_value(conf_scan_obj, "enable"); 
            if (json_value_get_type(val) == JSONBoolean) {
                /*!> Enable background spectral scan thread in packet forwarder */
                GW.spectral_scan_params.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("%s[SETTING] Data type for spectral_scan.enable seems wrong, please check\n", WARNMSG);
            }
            if (GW.spectral_scan_params.enable == true) {
                /*!> Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("[INFO~][SETTING] Spectral Scan with SX1261 is enabled\n");
                /*!> Get Spectral Scan Parameters */
                val = json_object_get_value(conf_scan_obj, "freq_start"); 
                if (json_value_get_type(val) == JSONNumber) {
                    GW.spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("%s[SETTING] Data type for spectral_scan.freq_start seems wrong, please check\n", WARNMSG);
                }
                val = json_object_get_value(conf_scan_obj, "nb_chan"); 
                if (json_value_get_type(val) == JSONNumber) {
                    GW.spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("%s[SETTING] Data type for spectral_scan.nb_chan seems wrong, please check\n", WARNMSG);
                }
                val = json_object_get_value(conf_scan_obj, "nb_scan"); 
                if (json_value_get_type(val) == JSONNumber) {
                    GW.spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
                } else {
                    MSG("%s[SETTING] Data type for spectral_scan.nb_scan seems wrong, please check\n", WARNMSG);
                }
                val = json_object_get_value(conf_scan_obj, "pace_s"); 
                if (json_value_get_type(val) == JSONNumber) {
                    GW.spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("%s[SETTING] Data type for spectral_scan.pace_s seems wrong, please check\n", WARNMSG);
                }
            }
        }

        /*!> LBT configuration */
        conf_lbt_obj = json_object_get_object(conf_sx1261_obj, "lbt"); 
        if (conf_lbt_obj == NULL) {
            MSG("[INFO~][SETTING] no configuration for LBT\n");
        } else {
            val = json_object_get_value(conf_lbt_obj, "enable"); 
            if (json_value_get_type(val) == JSONBoolean) {
                sx1261conf.lbt_conf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("%s[SETTING] Data type for lbt.enable seems wrong, please check\n", WARNMSG);
            }
            if (sx1261conf.lbt_conf.enable == true) {
                /*!> Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("[INFO~][SETTING] Listen-Before-Talk with SX1261 is enabled\n");

                val = json_object_get_value(conf_lbt_obj, "rssi_target"); 
                if (json_value_get_type(val) == JSONNumber) {
                    sx1261conf.lbt_conf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG("%s[SETTING] Data type for lbt.rssi_target seems wrong, please check\n", WARNMSG);
                    sx1261conf.lbt_conf.rssi_target = 0;
                }
                /*!> set LBT channels configuration */
                conf_lbtchan_array = json_object_get_array(conf_lbt_obj, "channels");
                if (conf_lbtchan_array != NULL) {
                    sx1261conf.lbt_conf.nb_channel = json_array_get_count(conf_lbtchan_array);
                    MSG("[INFO~][SETTING] %u LBT channels configured\n", sx1261conf.lbt_conf.nb_channel);
                }
                for (i = 0; i < (int)sx1261conf.lbt_conf.nb_channel; i++) {
                    /*!> Sanity check */
                    if (i >= LGW_LBT_CHANNEL_NB_MAX) {
                        MSG("%s[SETTING] LBT channel %d not supported, skip it\n", ERRMSG, i);
                        break;
                    }
                    /*!> Get LBT channel configuration object from array */
                    conf_lbtchan_obj = json_array_get_object(conf_lbtchan_array, i);
                    /*!> Channel frequency */
                    val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); 
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                        } else {
                            MSG("%s[SETTING] Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", WARNMSG, i);
                            sx1261conf.lbt_conf.channels[i].freq_hz = 0;
                        }
                    } else {
                        MSG("%s[SETTING] no frequency defined for LBT channel %d\n", ERRMSG, i);
                        return -1;
                    }

                    /*!> Channel bandiwdth */
                    val = json_object_dotget_value(conf_lbtchan_obj, "bandwidth"); 
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            bw = (uint32_t)json_value_get_number(val);
                            switch(bw) {
                                case 500000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_500KHZ; break;
                                case 250000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_250KHZ; break;
                                case 125000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_125KHZ; break;
                                default: sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                            }
                        } else {
                            MSG("%s[SETTING] Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", WARNMSG, i);
                            sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                        }
                    } else {
                        MSG("%s[SETTING] no bandiwdth defined for LBT channel %d\n", ERRMSG, i);
                        return -1;
                    }

                    /*!> Channel scan time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); 
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            if ((uint16_t)json_value_get_number(val) == 128) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_128_US;
                            } else if ((uint16_t)json_value_get_number(val) == 5000) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_5000_US;
                            } else {
                                MSG("%s[SETTING] scan time not supported for LBT channel %d, must be 128 or 5000\n", ERRMSG, i);
                                return -1;
                            }
                        } else {
                            MSG("%s[SETTING] Data type for lbt.channels[%d].scan_time_us seems wrong, please check\n", WARNMSG, i);
                            sx1261conf.lbt_conf.channels[i].scan_time_us = 0;
                        }
                    } else {
                        MSG("%s[SETTING] no scan_time_us defined for LBT channel %d\n", ERRMSG, i);
                        return -1;
                    }

                    /*!> Channel transmit time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "transmit_time_ms"); 
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = (uint16_t)json_value_get_number(val);
                        } else {
                            MSG("%s[SETTING] Data type for lbt.channels[%d].transmit_time_ms seems wrong, please check\n", WARNMSG, i);
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = 0;
                        }
                    } else {
                        MSG("%s[SETTING] no transmit_time_ms defined for LBT channel %d\n", ERRMSG, i);
                        return -1;
                    }
                }
            }
        }

        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG("%s[SETTING] Failed to configure the SX1261 radio\n", ERRMSG);
            return -1;
        }
    }

    /*!> set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /*!> initialize configuration structure */
        GW.tx.tx_enable[i] = false;
        snprintf(param_name, sizeof param_name, "radio_%i", i); /*!> compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); 
        if (json_value_get_type(val) != JSONObject) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for radio %i\n", i);
            continue;
        }
        /*!> there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            rfconf.enable = (bool)json_value_get_boolean(val);
        } else {
            rfconf.enable = false;
        }
        if (rfconf.enable == false) { /*!> radio disabled, nothing else to parse */
            lgw_log(LOG_INFO, "[INFO~][SETTING] radio %i disabled\n", i);
        } else  { /*!> radio enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            rf_freq_hz[i] = rfconf.freq_hz;
            /*!> put the value radio freq_hz on database */
            snprintf(param_value, sizeof(param_value), "%.3lfMHz", rfconf.freq_hz / 1e6);
            lgw_db_put("loaradio", param_name, param_value);  

            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_a", i);
            rfconf.rssi_tcomp.coeff_a = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_b", i);
            rfconf.rssi_tcomp.coeff_b = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_c", i);
            rfconf.rssi_tcomp.coeff_c = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_d", i);
            rfconf.rssi_tcomp.coeff_d = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_e", i);
            rfconf.rssi_tcomp.coeff_e = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else if (!strncmp(str, "SX1250", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1250;
            } else {
                lgw_log(LOG_INFO, "%s[SETTING] invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", WARNMSG, str);
            }
            snprintf(param_name, sizeof param_name, "radio_%i.single_input_mode", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.single_input_mode = (bool)json_value_get_boolean(val);
            } else {
                rfconf.single_input_mode = false;
            }

            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.tx_enable = (bool)json_value_get_boolean(val);
                GW.tx.tx_enable[i] = rfconf.tx_enable;
                if (rfconf.tx_enable == true) {
                    /*!> tx is enabled on this rf chain, we need its frequency range */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                    GW.tx.tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                    GW.tx.tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    if ((GW.tx.tx_freq_min[i] == 0) || (GW.tx.tx_freq_max[i] == 0)) {
                        lgw_log(LOG_INFO, "%s[SETTING] no frequency range specified for TX rf chain %d\n", WARNMSG, i);
                    }

                    /*!> set configuration for tx gains */
                    memset(&GW.tx.txlut[i], 0, sizeof GW.tx.txlut[i]); /*!> initialize configuration structure */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_gain_lut", i);
                    conf_txlut_array = json_object_dotget_array(conf_obj, param_name);
                    if (conf_txlut_array != NULL) {
                        GW.tx.txlut[i].size = json_array_get_count(conf_txlut_array);
                        /*!> Detect if we have a sx125x or sx1250 configuration */
                        conf_txgain_obj = json_array_get_object(conf_txlut_array, 0);
                        val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                        if (val != NULL) {
                            lgw_log(LOG_INFO, "[INFO~][SETTING] Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx1250\n", i, GW.tx.txlut[i].size);
                            sx1250_tx_lut = true;
                        } else {
                            lgw_log(LOG_INFO, "[INFO~][SETTING] Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx125x\n", i, GW.tx.txlut[i].size);
                            sx1250_tx_lut = false;
                        }
                        /*!> Parse the table */
                        for (j = 0; j < (int)GW.tx.txlut[i].size; j++) {
                             /*!> Sanity check */
                            if (j >= TX_GAIN_LUT_SIZE_MAX) {
                                lgw_log(LOG_INFO, "%s[SETTING] TX Gain LUT [%u] index %d not supported, skip it\n", ERRMSG, i, j);
                                break;
                            }
                            /*!> Get TX gain object from LUT */
                            conf_txgain_obj = json_array_get_object(conf_txlut_array, j);
                            /*!> rf power */
                            val = json_object_dotget_value(conf_txgain_obj, "rf_power");
                            if (json_value_get_type(val) == JSONNumber) {
                                GW.tx.txlut[i].lut[j].rf_power = (int8_t)json_value_get_number(val);
                            } else {
                                lgw_log(LOG_WARNING, "%s[SETTING] Data type for rf_power[%d] seems wrong, please check\n", WARNMSG, j);
                                GW.tx.txlut[i].lut[j].rf_power = 0;
                            }
                            /*!> PA gain */
                            val = json_object_dotget_value(conf_txgain_obj, "pa_gain");
                            if (json_value_get_type(val) == JSONNumber) {
                                GW.tx.txlut[i].lut[j].pa_gain = (uint8_t)json_value_get_number(val);
                            } else {
                                lgw_log(LOG_WARNING, "%s[SETTING] Data type for pa_gain [%d] seems wrong, please check\n", WARNMSG, j);
                                GW.tx.txlut[i].lut[j].pa_gain = 0;
                            }
                            if (sx1250_tx_lut == false) {
                                /*!> DIG gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dig_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    GW.tx.txlut[i].lut[j].dig_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    lgw_log(LOG_WARNING, "%s[SETTING] Data type for dig_gain[%d] seems wrong, please check\n", WARNMSG, j);
                                    GW.tx.txlut[i].lut[j].dig_gain = 0;
                                }
                                /*!> DAC gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dac_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    GW.tx.txlut[i].lut[j].dac_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    //lgw_log(LOG_INFO, "%s[SETTING] Data type for %s[%d] seems wrong, please check\n", "dac_gain", j);
                                    GW.tx.txlut[i].lut[j].dac_gain = 3; /*!> This is the only dac_gain supported for now */
                                }
                                /*!> MIX gain */
                                val = json_object_dotget_value(conf_txgain_obj, "mix_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    GW.tx.txlut[i].lut[j].mix_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    lgw_log(LOG_WARNING, "%s[SETTING] Data type for mix_gain[%d] seems wrong, please check\n", WARNMSG, j);
                                    GW.tx.txlut[i].lut[j].mix_gain = 0;
                                }
                            } else {
                                /*!> TODO: rework this, should not be needed for sx1250 */
                                GW.tx.txlut[i].lut[j].mix_gain = 5;

                                /*!> power index */
                                val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                                if (json_value_get_type(val) == JSONNumber) {
                                    GW.tx.txlut[i].lut[j].pwr_idx = (uint8_t)json_value_get_number(val);
                                } else {
                                    lgw_log(LOG_WARNING, "%s[SETTING] Data type for pwr_idx[%d] seems wrong, please check\n", WARNMSG, j);
                                    GW.tx.txlut[i].lut[j].pwr_idx = 0;
                                }
                            }
                        }
                        /*!> all parameters parsed, submitting configuration to the HAL */
                        if (GW.tx.txlut[i].size > 0) {
                            if (lgw_txgain_setconf(i, &GW.tx.txlut[i]) != LGW_HAL_SUCCESS) {
                                lgw_log(LOG_INFO, "%s[SETTING] Failed to configure concentrator TX Gain LUT for rf_chain %u\n", ERRMSG, i);
                                return -1;
                            }
                        } else {
                            lgw_log(LOG_INFO, "%s[SETTING] No TX gain LUT defined for rf_chain %u\n", WARNMSG, i);
                        }
                    } else {
                        lgw_log(LOG_INFO, "%s[SETTING] No TX gain LUT defined for rf_chain %u\n", WARNMSG, i);
                    }
                }
            } else {
                rfconf.tx_enable = false;
            }
            lgw_log(LOG_INFO, "[INFO~][SETTING] radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, single input mode %d\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.single_input_mode);
        }
        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, &rfconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_INFO, "%s[SETTING] invalid configuration for radio %i\n", ERRMSG, i);
            return -1;
        }
    }

    /*!> set configuration for demodulators */
    memset(&demodconf, 0, sizeof demodconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_multiSF_All"); 
    if (json_value_get_type(val) != JSONObject) {
        MSG("[INFO~][SETTING] no configuration for LoRa multi-SF spreading factors enabling\n");
    } else {
        conf_demod_array = json_object_dotget_array(conf_obj, "chan_multiSF_All.spreading_factor_enable");
        if ((conf_demod_array != NULL) && ((size = json_array_get_count(conf_demod_array)) <= LGW_MULTI_NB)) {
            for (i = 0; i < (int)size; i++) {
                number = json_array_get_number(conf_demod_array, i);
                if (number < 5 || number > 12) {
                    MSG("%s[SETTING] failed to parse chan_multiSF_All.spreading_factor_enable (wrong value at idx %d)\n", WARNMSG, i);
                    demodconf.multisf_datarate = 0xFF; /*!> enable all SFs */
                    break;
                } else {
                    /*!> set corresponding bit in the bitmask SF5 is LSB -> SF12 is MSB */
                    demodconf.multisf_datarate |= (1 << (number - 5));
                }
            }
        } else {
            MSG("%s[SETTING] failed to parse chan_multiSF_All.spreading_factor_enable\n", WARNMSG);
            demodconf.multisf_datarate = 0xFF; /*!> enable all SFs */
        }
        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_demod_setconf(&demodconf) != LGW_HAL_SUCCESS) {
            MSG("%s[SETTING] invalid configuration for demodulation parameters\n", ERRMSG);
            return -1;
        }
    }

    /*!> set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /*!> compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); 
        if (json_value_get_type(val) != JSONObject) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /*!> there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) { /*!> Lora multi-SF channel disabled, nothing else to parse */
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora multi-SF channel %i disabled\n", i);
        } else  { /*!> Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);

            /*!> put frequency on database :  chan_multisf_index frequency*/
            snprintf(param_value, sizeof(param_value), "%.3lfMHz", (ifconf.freq_hz + rf_freq_hz[ifconf.rf_chain]) / 1e6);  
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.freq", i);
            lgw_db_put("loaradio", param_name, param_value);  

            // TODO: handle individual SF enabling and disabling (spread_factor)
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 5 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_INFO, "%s[SETTING] invalid configuration for Lora multi-SF channel %i\n", ERRMSG, i);
            return -1;
        }
    }

    /*!> set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); 
    if (json_value_get_type(val) != JSONObject) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora standard channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: 
                    ifconf.bandwidth = BW_500KHZ; 
                    GW.relay.bw = 2;
                    break;
                case 250000: 
                    ifconf.bandwidth = BW_250KHZ; 
                    GW.relay.bw = 1;
                    break;
                case 125000: 
                    ifconf.bandwidth = BW_125KHZ; 
                    GW.relay.bw = 0;
                    break;
                default: 
                    ifconf.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  5: ifconf.datarate = DR_LORA_SF5;  break;
                case  6: ifconf.datarate = DR_LORA_SF6;  break;
                case  7: ifconf.datarate = DR_LORA_SF7;  break;
                case  8: ifconf.datarate = DR_LORA_SF8;  break;
                case  9: ifconf.datarate = DR_LORA_SF9;  break;
                case 10: ifconf.datarate = DR_LORA_SF10; break;
                case 11: ifconf.datarate = DR_LORA_SF11; break;
                case 12: ifconf.datarate = DR_LORA_SF12; break;
                default: ifconf.datarate = DR_UNDEFINED;
            }
            GW.relay.sf = sf;
            val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_hdr");
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.implicit_hdr = (bool)json_value_get_boolean(val);
            } else {
                ifconf.implicit_hdr = false;
            }
            if (ifconf.implicit_hdr == true) {
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_payload_length");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_payload_length = (uint8_t)json_value_get_number(val);
                } else {
                    lgw_log(LOG_INFO, "%s[SETTING] payload length setting is mandatory for implicit header mode\n", ERRMSG);
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_crc_en");
                if (json_value_get_type(val) == JSONBoolean) {
                    ifconf.implicit_crc_en = (bool)json_value_get_boolean(val);
                } else {
                    lgw_log(LOG_INFO, "%s[SETTING] CRC enable setting is mandatory for implicit header mode\n", ERRMSG);
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_coderate");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_coderate = (uint8_t)json_value_get_number(val);
                } else {
                    lgw_log(LOG_INFO, "%s[SETTING] coding rate setting is mandatory for implicit header mode\n", ERRMSG);
                    return -1;
                }
            }

            /*!> put frequency on database :  chan_multisf_index frequency*/
            snprintf(param_value, sizeof(param_value), "%.3lfMHz, %uHz bw, SF %u", (ifconf.freq_hz + rf_freq_hz[ifconf.rf_chain]) / 1e6, bw, sf);  
            lgw_db_put("loaradio", "chan_Lora_std.freq", param_value);  

            GW.relay.freq_hz = rf_freq_hz[ifconf.rf_chain] + ifconf.freq_hz;
            lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY channel> IF %iHz, %uHz bw, SF%u\n", GW.relay.freq_hz, bw, GW.relay.sf);

            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u, %s\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf, (ifconf.implicit_hdr == true) ? "Implicit header" : "Explicit header");
        }
        if (lgw_rxif_setconf(8, &ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_INFO, "%s[SETTING] invalid configuration for Lora standard channel\n", ERRMSG);
            return -1;
        }
    }

    /*!> set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); 
    if (json_value_get_type(val) != JSONObject) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] FSK channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /*!> if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + ifconf.datarate;
            }
            if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
#if 0 /*!> TODO */
            else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
#endif
            else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
            else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
            else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
            else ifconf.bandwidth = BW_UNDEFINED;

            snprintf(param_value, sizeof(param_value), "%fMHz, %uHz bw, %u bps datarate", (ifconf.freq_hz + rf_freq_hz[ifconf.rf_chain]) / 1e6, bw, ifconf.datarate);  
            lgw_db_put("loaradio", "chan_FSK.freq", param_value);  

            lgw_log(LOG_INFO, "[INFO~][SETTING] FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, &ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_INFO, "%s[SETTING] invalid configuration for FSK channel\n", ERRMSG);
            return -1;
        }
    }
    json_value_free(root_val);

    return 0;
}

#else     /*!> **************** else sx1301 ***************** */

static int parse_SX130x_configuration(const char* conf_file) {
    int i;
    char param_name[32]; /*!> used to generate variable parameter names */
    const char *str; /*!> used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";
    JSON_Value *root_val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Value *val = NULL;
    JSON_Array *conf_array = NULL;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_lbt_s lbtconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    uint32_t sf, bw, fdev;

    uint32_t rf_freq_hz[LGW_RF_CHAIN_NB];

    /*!> try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        lgw_log(LOG_ERROR, "%s[SETTING] %s is not a valid JSON file\n", ERRMSG, conf_file);
        exit(EXIT_FAILURE);
    }

    /*!> point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj_name);
    }

    /*!> set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "lorawan_public"); 
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        lgw_log(LOG_WARNING, "%s[SETTING] Data type for lorawan_public seems wrong, please check\n", WARNMSG);
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); 
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        lgw_log(LOG_WARNING, "%s[SETTING] Data type for clksrc seems wrong, please check\n", WARNMSG);
        boardconf.clksrc = 0;
    }
    lgw_log(LOG_INFO, "[INFO~][SETTING] lorawan_public %d, clksrc %d\n", boardconf.lorawan_public, boardconf.clksrc);
    /*!> all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(boardconf) != LGW_HAL_SUCCESS) {
        lgw_log(LOG_ERROR, "%s[SETTING] Failed to configure board\n", ERRMSG);
        return -1;
    }

    /*!> set LBT configuration */
    memset(&lbtconf, 0, sizeof lbtconf); /*!> initialize configuration structure */
    conf_lbt_obj = json_object_get_object(conf_obj, "lbt_cfg"); 
    if (conf_lbt_obj == NULL) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for LBT\n");
    } else {
        val = json_object_get_value(conf_lbt_obj, "enable"); 
        if (json_value_get_type(val) == JSONBoolean) {
            lbtconf.enable = (bool)json_value_get_boolean(val);
        } else {
            lgw_log(LOG_WARNING, "%s[SETTING] Data type for lbt_cfg.enable seems wrong, please check\n", WARNMSG);
        }
        if (lbtconf.enable == true) {
            val = json_object_get_value(conf_lbt_obj, "isftdi"); 
            if (json_value_get_type(val) == JSONBoolean) {
                lbtconf.isftdi = (bool)json_value_get_boolean(val);
                lgw_log(LOG_WARNING, "[INFO~][SETTING] using FTDI device for lbt scan\n");
            } else {
                lgw_log(LOG_WARNING, "[INFO~][SETTING] using local device for lbt scan\n");
            }
            val = json_object_get_value(conf_lbt_obj, "rssi_target"); 
            if (json_value_get_type(val) == JSONNumber) {
                lbtconf.rssi_target = (int8_t)json_value_get_number(val);
            } else {
                lgw_log(LOG_WARNING, "%s[SETTING] Data type for lbt_cfg.rssi_target seems wrong, please check\n", WARNMSG);
                lbtconf.rssi_target = 0;
            }
            val = json_object_get_value(conf_lbt_obj, "sx127x_rssi_offset"); 
            if (json_value_get_type(val) == JSONNumber) {
                lbtconf.rssi_offset = (int8_t)json_value_get_number(val);
            } else {
                lgw_log(LOG_WARNING, "%s[SETTING] Data type for lbt_cfg.sx127x_rssi_offset seems wrong, please check\n", WARNMSG);
                lbtconf.rssi_offset = 0;
            }
            /*!> set LBT channels configuration */
            conf_array = json_object_get_array(conf_lbt_obj, "chan_cfg");
            if (conf_array != NULL) {
                lbtconf.nb_channel = json_array_get_count( conf_array );
                lgw_log(LOG_INFO, "[INFO~][SETTING] %u LBT channels configured\n", lbtconf.nb_channel);
            }
            for (i = 0; i < (int)lbtconf.nb_channel; i++) {
                /*!> Sanity check */
                if (i >= LBT_CHANNEL_FREQ_NB)
                {
                    lgw_log(LOG_ERROR, "%s[SETTING] LBT channel %d not supported, skip it\n", ERRMSG, i );
                    break;
                }
                /*!> Get LBT channel configuration object from array */
                conf_lbtchan_obj = json_array_get_object(conf_array, i);

                /*!> Channel frequency */
                val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); 
                if (json_value_get_type(val) == JSONNumber) {
                    lbtconf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                } else {
                    lgw_log(LOG_WARNING, "%s[SETTING] Data type for lbt_cfg.channels[%d].freq_hz seems wrong, please check\n", WARNMSG, i);
                    lbtconf.channels[i].freq_hz = 0;
                }

                /*!> Channel scan time */
                val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); 
                if (json_value_get_type(val) == JSONNumber) {
                    lbtconf.channels[i].scan_time_us = (uint16_t)json_value_get_number(val);
                } else {
                    lgw_log(LOG_WARNING, "%s[SETTING] Data type for lbt_cfg.channels[%d].scan_time_us seems wrong, please check\n", WARNMSG, i);
                    lbtconf.channels[i].scan_time_us = 0;
                }
            }

            /*!> all parameters parsed, submitting configuration to the HAL */
            if (lgw_lbt_setconf(lbtconf) != LGW_HAL_SUCCESS) {
                lgw_log(LOG_ERROR, "%s[SETTING] Failed to configure LBT\n", ERRMSG);
                return -1;
            }
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] LBT is disabled\n");
        }
    }

    /*!> set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); 
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            GW.hal.antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_INFO, "%s[SETTING] Data type for antenna_gain seems wrong, please check\n", WARNMSG);
            GW.hal.antenna_gain = 0;
        }
    }
    lgw_log(LOG_INFO, "[INFO~][SETTING] antenna_gain %d dBi\n", GW.hal.antenna_gain);

    /*!> set configuration for tx gains */
    memset(&GW.tx.txlut[0], 0, sizeof GW.tx.txlut[0]); /*!> initialize configuration structure */
    for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
        snprintf(param_name, sizeof param_name, "tx_lut_%i", i); /*!> compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); 
        if (json_value_get_type(val) != JSONObject) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for tx gain lut %i\n", i);
            continue;
        }
        GW.tx.txlut[0].size++; /*!> update TX LUT size based on JSON object found in configuration file */
        /*!> there is an object to configure that TX gain index, let's parse it */
        snprintf(param_name, sizeof param_name, "tx_lut_%i.pa_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            GW.tx.txlut[0].lut[i].pa_gain = (uint8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_WARNING, "%s[SETTING] Data type for %s[%d] seems wrong, please check\n", WARNMSG, param_name, i);
            GW.tx.txlut[0].lut[i].pa_gain = 0;
        }
        snprintf(param_name, sizeof param_name, "tx_lut_%i.dac_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            GW.tx.txlut[0].lut[i].dac_gain = (uint8_t)json_value_get_number(val);
        } else {
            GW.tx.txlut[0].lut[i].dac_gain = 3; /*!> This is the only dac_gain supported for now */
        }
        snprintf(param_name, sizeof param_name, "tx_lut_%i.dig_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            GW.tx.txlut[0].lut[i].dig_gain = (uint8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_WARNING, "%s[SETTING] Data type for %s[%d] seems wrong, please check\n", WARNMSG, param_name, i);
            GW.tx.txlut[0].lut[i].dig_gain = 0;
        }
        snprintf(param_name, sizeof param_name, "tx_lut_%i.mix_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            GW.tx.txlut[0].lut[i].mix_gain = (uint8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_WARNING, "%s[SETTING] Data type for %s[%d] seems wrong, please check\n", WARNMSG, param_name, i);
            GW.tx.txlut[0].lut[i].mix_gain = 0;
        }
        snprintf(param_name, sizeof param_name, "tx_lut_%i.rf_power", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            GW.tx.txlut[0].lut[i].rf_power = (int8_t)json_value_get_number(val);
        } else {
            lgw_log(LOG_WARNING, "%s[SETTING] Data type for %s[%d] seems wrong, please check\n", WARNMSG, param_name, i);
            GW.tx.txlut[0].lut[i].rf_power = 0;
        }
    }
    /*!> all parameters parsed, submitting configuration to the HAL */
    if (GW.tx.txlut[0].size > 0) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] Configuring TX LUT with %u indexes\n", GW.tx.txlut[0].size);
        if (lgw_txgain_setconf(&GW.tx.txlut[0]) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_ERROR, "%s[SETTING] Failed to configure concentrator TX Gain LUT\n", ERRMSG);
            return -1;
        }
    } else {
        lgw_log(LOG_WARNING, "%s[SETTING] No TX gain LUT defined\n", WARNMSG);
    }

    /*!> set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /*!> initialize configuration structure */
        snprintf(param_name, sizeof param_name, "radio_%i", i); /*!> compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); 
        if (json_value_get_type(val) != JSONObject) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for radio %i\n", i);
            continue;
        }
        /*!> there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            rfconf.enable = (bool)json_value_get_boolean(val);
        } else {
            rfconf.enable = false;
        }
        if (rfconf.enable == false) { /*!> radio disabled, nothing else to parse */
            lgw_log(LOG_INFO, "[INFO~][SETTING] radio %i disabled\n", i);
        } else  { /*!> radio enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            rf_freq_hz[i] = rfconf.freq_hz;
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else {
                lgw_log(LOG_WARNING, "%s[SETTING] invalid radio type: %s (should be SX1255 or SX1257)\n", WARNMSG, str);
            }
            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.tx_enable = (bool)json_value_get_boolean(val);
                if (rfconf.tx_enable == true) {
                    /*!> tx is enabled on this rf chain, we need its frequency range */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                    GW.tx.tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                    GW.tx.tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    if ((GW.tx.tx_freq_min[i] == 0) || (GW.tx.tx_freq_max[i] == 0)) {
                        lgw_log(LOG_WARNING, "%sno frequency range specified for TX rf chain %d\n", WARNMSG, i);
                    }
                    /*!> ... and the notch filter frequency to be set */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_notch_freq", i);
                    rfconf.tx_notch_freq = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                }
            } else {
                rfconf.tx_enable = false;
            }
            lgw_log(LOG_INFO, "[INFO~][SETTING] radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, tx_notch_freq %u\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.tx_notch_freq);
        }
        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_ERROR, "%s[SETTING] invalid configuration for radio %i\n", ERRMSG, i);
            return -1;
        }
    }

    /*!> set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /*!> compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); 
        if (json_value_get_type(val) != JSONObject) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /*!> there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) { /*!> Lora multi-SF channel disabled, nothing else to parse */
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora multi-SF channel %i disabled\n", i);
        } else  { /*!> Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /*!> all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_ERROR, "%s[SETTING] invalid configuration for Lora multi-SF channel %i\n", ERRMSG, i);
            return -1;
        }
    }

    /*!> set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); 
    if (json_value_get_type(val) != JSONObject) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora standard channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: 
                    ifconf.bandwidth = BW_500KHZ; 
                    GW.relay.bw = 2;
                    break;
                case 250000: 
                    ifconf.bandwidth = BW_250KHZ; 
                    GW.relay.bw = 1;
                    break;
                case 125000: 
                    ifconf.bandwidth = BW_125KHZ; 
                    GW.relay.bw = 0;
                    break;
                default: ifconf.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  7: ifconf.datarate = DR_LORA_SF7;  break;
                case  8: ifconf.datarate = DR_LORA_SF8;  break;
                case  9: ifconf.datarate = DR_LORA_SF9;  break;
                case 10: ifconf.datarate = DR_LORA_SF10; break;
                case 11: ifconf.datarate = DR_LORA_SF11; break;
                case 12: ifconf.datarate = DR_LORA_SF12; break;
                default: ifconf.datarate = DR_UNDEFINED;
            }
            GW.relay.sf = sf;
            lgw_log(LOG_INFO, "[INFO~][SETTING] Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
        }

        /*!>!> GW has_realay confiure */ 
        GW.relay.freq_hz = rf_freq_hz[ifconf.rf_chain] + ifconf.freq_hz;
        lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY channel> IF %i Hz, %u Hz bw, SF %u\n", GW.relay.freq_hz, GW.relay.bw, GW.relay.sf);

        if (lgw_rxif_setconf(8, ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_ERROR, "%s[SETTING] invalid configuration for Lora standard channel\n", ERRMSG);
            return -1;
        }
    }

    /*!> set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /*!> initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); 
    if (json_value_get_type(val) != JSONObject) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] FSK channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /*!> if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + ifconf.datarate;
            }
            if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
            else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
            else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
            else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
            else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
            else ifconf.bandwidth = BW_UNDEFINED;

            lgw_log(LOG_INFO, "[INFO~][SETTING] FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, ifconf) != LGW_HAL_SUCCESS) {
            lgw_log(LOG_ERROR, "%s[SETTING] invalid configuration for FSK channel\n", ERRMSG);
            return -1;
        }
    }
    json_value_free(root_val);

    return 0;
}

#endif      // defined sx1301 model

static int parse_gateway_configuration(const char* conf_file) {
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Object *serv_obj = NULL;
    JSON_Array *serv_arry = NULL;
    JSON_Value *val = NULL; /*!> needed to detect the absence of some fields */
    const char *str; /*!> pointer to sub-strings in the JSON data */
    const char *strr; /*!> pointer to minor-strings in the JSON data */
    unsigned long long ull = 0;

    char logmask[11] = {'\0'};  

    serv_s* serv_entry = NULL;

    /*!> try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        lgw_log(LOG_INFO, "%s[SETTING] %s is not a valid JSON file\n", ERRMSG, conf_file);
        return -1;
    }

    /*!> point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /*!> gateway unique identifier (aka MAC address) (optional) */
    str = json_object_get_string(conf_obj, "gateway_ID");
    if (str != NULL) {
        strncpy(GW.info.gateway_id, str, sizeof(GW.info.gateway_id));
        sscanf(str, "%llx", &ull);
        GW.info.lgwm = ull;
        GW.info.net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (ull>>32)));
        GW.info.net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  ull  ));
        lgw_log(LOG_INFO, "[INFO~][SETTING] gateway MAC address is configured to %s\n", GW.info.gateway_id);
    }

    val = json_object_get_value(conf_obj, "ghoststream_enable"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.ghoststream_enabled = (bool)json_value_get_boolean(val);
        if (GW.cfg.ghoststream_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] ghoststream_enabled is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] ghoststream_enabled is disabled\n");
        }
    } 

    val = json_object_get_value(conf_obj, "radiostream_enable"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.radiostream_enabled = (bool)json_value_get_boolean(val);
        if (GW.cfg.radiostream_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] radiostream_enable is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] radiostream_enable is disabled\n");
        }
    } 

    val = json_object_get_value(conf_obj, "wd_enable"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.wd_enabled = (bool)json_value_get_boolean(val);
        if (GW.cfg.wd_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] wd_enable is enable\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] wd_enable is disable\n");
        }
    } 

    val = json_object_get_value(conf_obj, "delay_enabled"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.delay_enabled = (bool)json_value_get_boolean(val);
        if (GW.cfg.delay_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] delay_enable is enable\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] delay_enable is disable\n");
        }
    } 

    str = json_object_get_string(conf_obj, "delay_db_path");
    if (str != NULL) {
        strncpy(GW.cfg.delay_db_path, str, sizeof GW.cfg.delay_db_path);
        GW.cfg.delay_db_path[sizeof GW.cfg.delay_db_path - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] Storeage path is configured to \"%s\"\n", GW.cfg.delay_db_path);
    }

    val = json_object_get_value(conf_obj, "td_enable"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.td_enabled = (bool)json_value_get_boolean(val);
        if (GW.cfg.td_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] td_enable is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] td_enable is disabled\n");
        }
    } else
        lgw_log(LOG_INFO, "[INFO~][SETTING] td_enable is disabled\n");

    val = json_object_get_value(conf_obj, "logger_enable"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.log.logger_enabled = (bool)json_value_get_boolean(val);
        if (GW.log.logger_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] logger_enabled is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] logger_enabled is disenabled\n");
        }
    } 

    val = json_object_get_value(conf_obj, "mac_decode"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.mac_decode = (bool)json_value_get_boolean(val);
        if (GW.cfg.mac_decode == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac_decode is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac_decode is disabled\n");
        }
    } else 
        lgw_log(LOG_INFO, "[INFO~][SETTING] mac_decode is disabled\n");

    val = json_object_get_value(conf_obj, "mac2db"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.mac2db = (bool)json_value_get_boolean(val);
        if (GW.cfg.mac2db == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac2db is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac2db is disabled\n");
        }
    } 
    
    val = json_object_get_value(conf_obj, "mac2file"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.mac2file = (bool)json_value_get_boolean(val);
        if (GW.cfg.mac2file == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac2file is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] mac2file is disabled\n");
        }
    } 

    val = json_object_get_value(conf_obj, "custom_downlink"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.cfg.custom_downlink = (bool)json_value_get_boolean(val);
        if (GW.cfg.custom_downlink == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] custom_downlink is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] custom_downlink is disabled\n");
        }
    } else
        lgw_log(LOG_INFO, "[INFO~][SETTING] custom_downlink is disabled\n");

    str = json_object_get_string(conf_obj, "regional");
    if (str != NULL) {
        if (!strcmp(str, "EU")) {
            GW.cfg.region = EU;
        } else if (!strncmp(str, "EU433", 5)) {
            GW.cfg.region = EU433;
        } else if(!strncmp(str, "US", 2)) {
            GW.cfg.region = US;
        } else if(!strcmp(str, "CN470")) {
            GW.cfg.region = CN470;
        } else if(!strcmp(str, "CN779")) {
            GW.cfg.region = CN779;
        } else if(!strcmp(str, "AS1")) {
            GW.cfg.region = AS1;
        } else if(!strcmp(str, "AS2")) {
            GW.cfg.region = AS2;
        } else if(!strcmp(str, "AS3")) {
            GW.cfg.region = AS3;
        } else if(!strncmp(str, "KR", 2)) {
            GW.cfg.region = KR;
        } else if(!strncmp(str, "IN", 2)) {
            GW.cfg.region = IN;
        } else if(!strncmp(str, "RU", 2)) {
            GW.cfg.region = RU;
        } else if(!strncmp(str, "AU", 2)) {
            GW.cfg.region = AU;
        } else if(!strncmp(str, "KZ", 2)) {
            GW.cfg.region = KZ;
        } else  {
            GW.cfg.region = EU;
        }
        lgw_log(LOG_INFO, "[INFO~][SETTING] GW regional is configured to \"%s\"\n", str);
    }

    str = json_object_get_string(conf_obj, "log_mask");
    if (str != NULL) 
        strncpy(logmask, str, sizeof(logmask));
    if (logmask[0] == '1') { 
        GW.log.debug_mask |= LOG_INFO;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_INFO \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_INFO \n");
    }
    if (logmask[1] == '1') { 
        GW.log.debug_mask |= LOG_PKT;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_PKT \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_PKT \n");
    }
    if (logmask[2] == '1') {
        GW.log.debug_mask |= LOG_WARNING;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_WARNING \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_WARNING \n");
    }
    if (logmask[3] == '1') {
        GW.log.debug_mask |= LOG_ERROR;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_ERROR \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_ERROR \n");
    }
    if (logmask[4] == '1') {
        GW.log.debug_mask |= LOG_REPORT;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_REPORT \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_REPORT \n");
    }
    if (logmask[5] == '1') {
        GW.log.debug_mask |= LOG_DEBUG;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_DEBUG \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_DEBUG \n");
    }
    if (logmask[6] == '1') { 
        GW.log.debug_mask |= LOG_JIT;
        GW.log.debug_mask |= LOG_ERROR;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_JIT \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_JIT \n");
    }
    if (logmask[7] == '1') {
        GW.log.debug_mask |= LOG_BEACON;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_BEACON \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_BEACON \n");
    }
    if (logmask[8] == '1') {
        GW.log.debug_mask |= LOG_TIMERSYNC;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_TIMERSYNC \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_TIMERSYNC \n");
    }
    if (logmask[9] == '1') {
        GW.log.debug_mask |= LOG_MEM;
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG open LOG_MEM \n");
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] LOG close LOG_MEM \n");
    }

    str = json_object_get_string(conf_obj, "ghost_host");
    if (str != NULL) {
        strncpy(GW.cfg.ghost_host, str, sizeof GW.cfg.ghost_host);
        GW.cfg.ghost_host[sizeof GW.cfg.ghost_host - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] ghost_host is configured to \"%s\"\n", GW.cfg.ghost_host);
    }

    val = json_object_get_value(conf_obj, "ghost_port");
    if (val != NULL) {
        snprintf(GW.cfg.ghost_port, sizeof GW.cfg.ghost_port, "%u", (uint16_t)json_value_get_number(val));
        GW.cfg.ghost_port[sizeof GW.cfg.ghost_port - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] ghost_port is configured to \"%s\"\n", GW.cfg.ghost_port);
    }

    str = json_object_get_string(conf_obj, "platform");
    if (str != NULL) {
        strncpy(GW.info.platform, str, sizeof GW.info.platform);
        GW.info.platform[sizeof GW.info.platform - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] Platform is configured to \"%s\"\n", GW.info.platform);
    }

    str = json_object_get_string(conf_obj, "email");
    if (str != NULL) {
        strncpy(GW.info.email, str, sizeof GW.info.email);
        GW.info.email[sizeof GW.info.email - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] email is configured to \"%s\"\n", GW.info.email);
    }

    str = json_object_get_string(conf_obj, "description");
    if (str != NULL) {
        strncpy(GW.info.description, str, sizeof GW.info.description);
        GW.info.description[sizeof GW.info.description - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] GW description: \"%s\"\n", GW.info.description);
    }

    val = json_object_get_value(conf_obj, "fcnt_gap");
    if (val != NULL) {
        GW.cfg.fcnt_gap = (uint8_t)json_value_get_number(val);
        if (GW.cfg.fcnt_gap > 32 || GW.cfg.fcnt_gap < 8)
            GW.cfg.fcnt_gap = 12;
    }
    lgw_log(LOG_INFO, "[INFO~][SETTING] FCNT_GAP is configured to %u, largest fcnt is %u \n", GW.cfg.fcnt_gap, 65536 * GW.cfg.fcnt_gap);

    /*
    val = json_object_get_value(conf_obj, "status_index");
    if (json_value_get_type(val) == JSONNumber) {
        GW.info.status_index = (uint8_t)json_value_get_number(val);
    } else {
        GW.info.status_index = 15;
    }

    lgw_log(LOG_INFO, "[INFO~][SETTING] GW status index set to: \"%i\"\n", GW.info.status_index);
    */

    /*!> GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(GW.gps.gps_tty_path, str, sizeof GW.gps.gps_tty_path);
        GW.gps.gps_tty_path[sizeof GW.gps.gps_tty_path - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] GPS serial port path is configured to \"%s\"\n", GW.gps.gps_tty_path);
    }

    val = json_object_get_value(conf_obj, "autoquit_threshold");
    if (val != NULL) {
        GW.cfg.autoquit_threshold = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] autoquit_threshold is configured to %u \n", GW.cfg.autoquit_threshold);
    }

    val = json_object_get_value(conf_obj, "time_interval");
    if (val != NULL) {
        GW.cfg.time_interval = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] time_interval is configured to %u \n", GW.cfg.time_interval);
    }

    /*!> RELAY configure (optional) */
    str = json_object_get_string(conf_obj, "relay_tty_path");
    if (str != NULL) {
        strncpy(GW.relay.tty_path, str, sizeof(GW.relay.tty_path));
        GW.relay.tty_path[sizeof(GW.relay.tty_path) - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY serial port path is configured to \"%s\"\n", GW.relay.tty_path);
    }

    val = json_object_get_value(conf_obj, "as_relay"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.relay.as_relay = (bool)json_value_get_boolean(val);
        if (GW.relay.as_relay == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] as_relay is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] as_relay is disabled\n");
        }
    } else
        lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] as_relay is disabled\n");

    val = json_object_get_value(conf_obj, "has_relay"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.relay.has_relay = (bool)json_value_get_boolean(val);
        if (GW.relay.has_relay == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] has_relay is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] has_relay is disabled\n");
        }
    } else
        lgw_log(LOG_INFO, "[INFO~][SETTING][RELAY] has_relay is disabled\n");

    val = json_object_get_value(conf_obj, "relay_invert_pol"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.relay.invert_pol = (bool)json_value_get_boolean(val);
        if (GW.relay.invert_pol == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY invert_pol\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY no invert_pol\n");
        }
    } else
        lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY no invert_pol\n");

    val = json_object_get_value(conf_obj, "relay_tty_baude");
    if (val != NULL)
        GW.relay.tty_baude = (uint32_t)json_value_get_number(val);
    lgw_log(LOG_INFO, "[INFO~][SETTING] RELAY tty bauderate is configured to \"%u\"\n", GW.relay.tty_baude);

    /*!> time diff of UTC: string */
    str = json_object_get_string(conf_obj, "time_diff");
    if (str != NULL) {
        strncpy(GW.cfg.time_diff, str, sizeof(GW.cfg.time_diff));
        GW.cfg.time_diff[sizeof(GW.cfg.time_diff) - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] GW time_diff of UTC: \"%s\"\n", GW.cfg.time_diff);
    }

    /*!> get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        GW.gps.reference_coord.lat = (double)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Reference latitude is configured to %f deg\n", GW.gps.reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        GW.gps.reference_coord.lon = (double)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Reference longitude is configured to %f deg\n", GW.gps.reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        GW.gps.reference_coord.alt = (short)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Reference altitude is configured to %i meters\n", GW.gps.reference_coord.alt);
    }

    /*!> Gateway GPS coordinates hardcoding (aka. faking) option */
    val = json_object_get_value(conf_obj, "fake_gps");
    if (json_value_get_type(val) == JSONBoolean) {
        GW.gps.gps_fake_enable = (bool)json_value_get_boolean(val);
        if (GW.gps.gps_fake_enable == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] fake GPS is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] fake GPS is disabled\n");
        }
    }

    val = json_object_get_value(conf_obj, "time_ref");
    if (json_value_get_type(val) == JSONBoolean) {
        GW.gps.time_ref = (bool)json_value_get_boolean(val);
        if (GW.gps.time_ref == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] time ref from GPS is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] time ref from GPS is disabled\n");
        }
    }

    /*!> LBT module TTY configure(optional) */
    val = json_object_get_value(conf_obj, "lbt_tty_enabled"); 
    if (json_value_get_type(val) == JSONBoolean) {
        GW.lbt.lbt_tty_enabled = (bool)json_value_get_boolean(val);
        if (GW.lbt.lbt_tty_enabled == true) {
            lgw_log(LOG_INFO, "[INFO~][SETTING] lbt_tty_enabled is enabled\n");
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] lbt_tty_enabled is disabled\n");
        }
    } 

    /*!> LBT module TTY enabled */
    if (GW.lbt.lbt_tty_enabled) {
        str = json_object_get_string(conf_obj, "lbt_tty_path");
        if (str != NULL) {
            strncpy(GW.lbt.lbt_tty_path, str, sizeof(GW.lbt.lbt_tty_path));
            GW.lbt.lbt_tty_path[sizeof(GW.lbt.lbt_tty_path) - 1] = '\0'; 
        }
        lgw_log(LOG_INFO, "[INFO~][SETTING] LBT serial port path is configured to \"%s\"\n", GW.lbt.lbt_tty_path);

        val = json_object_get_value(conf_obj, "lbt_tty_baude");
        if (val != NULL)
            GW.lbt.lbt_tty_baude = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] LBT tty bauderate  is configured to \"%u\"\n", GW.lbt.lbt_tty_baude);

        val = json_object_get_value(conf_obj, "lbt_rssi_target");
        if (val != NULL)
            GW.lbt.lbt_rssi_target = (int8_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] LBT rssi threshold is configured to \"%d\"\n", GW.lbt.lbt_rssi_target);

        val = json_object_get_value(conf_obj, "lbt_scan_time_ms");
        if (val != NULL)
            GW.lbt.lbt_scan_time_ms = (uint16_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] LBT rssi scan time is configured to \"%u\"\n", GW.lbt.lbt_scan_time_ms);
    }

    /*!> Beacon signal period (optional) */
    val = json_object_get_value(conf_obj, "beacon_period");
    if (val != NULL) {
        GW.beacon.beacon_period = (uint32_t)json_value_get_number(val);
        if ((GW.beacon.beacon_period > 0) && (GW.beacon.beacon_period < 6)) {
            lgw_log(LOG_INFO, "%s[SETTING] invalid configuration for Beacon period, must be >= 6s\n", ERRMSG);
            return -1;
        } else {
            lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing period is configured to %u seconds\n", GW.beacon.beacon_period);
        }
    }

    /*!> Beacon TX frequency (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_hz");
    if (val != NULL) {
        GW.beacon.beacon_freq_hz = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing signal will be emitted at %u Hz\n", GW.beacon.beacon_freq_hz);
    }

    /*!> Number of beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_nb");
    if (val != NULL) {
        GW.beacon.beacon_freq_nb = (uint8_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing channel number is set to %u\n", GW.beacon.beacon_freq_nb);
    }

    /*!> Frequency step between beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_step");
    if (val != NULL) {
        GW.beacon.beacon_freq_step = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing channel frequency step is set to %uHz\n", GW.beacon.beacon_freq_step);
    }

    /*!> Beacon datarate (optional) */
    val = json_object_get_value(conf_obj, "beacon_datarate");
    if (val != NULL) {
        GW.beacon.beacon_datarate = (uint8_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing datarate is set to SF%d\n", GW.beacon.beacon_datarate);
    }

    /*!> Beacon modulation bandwidth (optional) */
    val = json_object_get_value(conf_obj, "beacon_bw_hz");
    if (val != NULL) {
        GW.beacon.beacon_bw_hz = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing modulation bandwidth is set to %dHz\n", GW.beacon.beacon_bw_hz);
    }

    /*!> Beacon TX power (optional) */
    val = json_object_get_value(conf_obj, "beacon_power");
    if (val != NULL) {
        GW.beacon.beacon_power = (int8_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing TX power is set to %ddBm\n", GW.beacon.beacon_power);
    }

    /*!> Beacon information descriptor (optional) */
    val = json_object_get_value(conf_obj, "beacon_infodesc");
    if (val != NULL) {
        GW.beacon.beacon_infodesc = (uint8_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Beaconing information descriptor is set to %u\n", GW.beacon.beacon_infodesc);
    }

    /*!> Auto-quit threshold (optional) */
    val = json_object_get_value(conf_obj, "autoquit_threshold");
    if (val != NULL) {
        GW.cfg.autoquit_threshold = (uint32_t)json_value_get_number(val);
        lgw_log(LOG_INFO, "[INFO~][SETTING] Auto-quit after %u non-acknowledged PULL_DATA\n", GW.cfg.autoquit_threshold);
    }

    /*!> servers configure */
	serv_arry = json_object_get_array(conf_obj, "servers");
	if ( NULL != serv_arry) {
		/*!> serv_count represents the maximal number of servers to be read. */
        int count = 0, i = 0, try = 0;
		count = json_array_get_count(serv_arry);  /*!> number of services should be less than 8 */
		lgw_log(LOG_INFO, "[INFO~][SETTING] Found %i servers in array.\n", count);
		for (i = 0; i < count; i++) {
            serv_entry = (serv_s*)lgw_malloc(sizeof(serv_s));

            serv_entry->list.next = NULL;

            serv_entry->info.stamp = 1 << (i+1);  //PKT is the first service

            /*!> service network information */
            serv_entry->net = (serv_net_s*)lgw_malloc(sizeof(serv_net_s));
            serv_entry->net->sock_up = -1;
            serv_entry->net->sock_down = -1;
            serv_entry->net->push_timeout_half.tv_sec = 0;
            serv_entry->net->push_timeout_half.tv_usec = DEFAULT_PUSH_TIMEOUT_MS * 500;
            serv_entry->net->pull_timeout.tv_sec = 0;
            serv_entry->net->pull_timeout.tv_usec = DEFAULT_PULL_TIMEOUT_MS * 1000;
            serv_entry->net->pull_interval = DEFAULT_PULL_INTERVAL;
            
            /*!> about service filter information */
            serv_entry->filter.fwd_valid_pkt = true;
            serv_entry->filter.fwd_error_pkt = false;
            serv_entry->filter.fwd_nocrc_pkt = false;
            serv_entry->filter.fport = 0;
            serv_entry->filter.devaddr = 0;
            serv_entry->filter.nwkid = 0;
            serv_entry->filter.deveui = 0;

            serv_entry->report = NULL;

            /*!> about service status information */
            serv_entry->state.live = false;
            serv_entry->state.contact = 0;

            try = 0;

            do {
                if (sem_init(&serv_entry->thread.sema, 0, 0) != 0) {
                    try++;
                } else
                    break;
            } while (try < 3);

            if (try == 3) { /*!> 等于3时，sem的初始化已经失败了3次 */
                lgw_log(LOG_WARNING, "%s[SETTING] Can't initializes the unnamed semaphore of service, ignore this element.\n", WARNMSG);
                lgw_free(serv_entry->net);
                lgw_free(serv_entry);
                continue;
            }

            serv_entry->thread.stop_sig = false;

			serv_obj = json_array_get_object(serv_arry, i);

			str = json_object_get_string(serv_obj, "server_name");  // MQTT id
            if (str != NULL) {
                strncpy(serv_entry->info.name, str, sizeof(serv_entry->info.name));
                serv_entry->info.name[sizeof(serv_entry->info.name) - 1] = '\0'; 
                lgw_log(LOG_INFO, "[INFO~][SETTING] Found a server configure, name is configure to \"%s\"\n", str);
            } else {
                lgw_gen_str((char*)&serv_entry->info.name, sizeof(serv_entry->info.name));
                lgw_log(LOG_INFO, "[INFO~][SETTING] The server name mustbe configure, generate a random name: \"%s\"\n", serv_entry->info.name);
                continue;
            }

			str = json_object_get_string(serv_obj, "server_key");  // MQTT key
            if (str != NULL) {
                serv_entry->info.key = lgw_malloc(PATH_LEN);
                strncpy(serv_entry->info.key, str, PATH_LEN);
                serv_entry->info.key[PATH_LEN - 1] = '\0'; 
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] server key is configure to \"%s\"\n", serv_entry->info.name, str);
            } 

			str = json_object_get_string(serv_obj, "server_type");
            if (str != NULL) {
				if (!strncmp(str, "semtech", 7)) {
					serv_entry->info.type = semtech;
				} else if (!strncmp(str, "ttn", 3)) {
					serv_entry->info.type = ttn;
				} else if (!strncmp(str, "pkt", 3)) {
					serv_entry->info.type = pkt;
				} else if (!strncmp(str, "relay", 5)) {
					serv_entry->info.type = relay;
				} else if (!strncmp(str, "delay", 5)) {
					serv_entry->info.type = delay;
				} else if (!strncmp(str, "mqtt", 4)) {
					serv_entry->info.type = mqtt;
                    serv_entry->net->mqtt = (mqttinfo_s*)lgw_malloc(sizeof(mqttinfo_s));
                    strr = json_object_get_string(serv_obj, "uptopic");  // MQTT key
                    if (strr != NULL) {
                        strncpy(serv_entry->net->mqtt->uptopic, strr, sizeof(serv_entry->net->mqtt->uptopic));
                        serv_entry->net->mqtt->uptopic[sizeof(serv_entry->net->mqtt->uptopic) - 1] = '\0'; 
                        lgw_log(LOG_INFO, "[INFO~][SETTING][%s] Found a mqtt uptopic is \"%s\"\n", serv_entry->info.name, strr);
                    } else {
                        strcpy(serv_entry->net->mqtt->uptopic, "test");
                        lgw_log(LOG_WARNING, "%s[SETTING][%s] Need a uptopic for mqtt publish, set to default value \"test\"\n", WARNMSG, serv_entry->info.name);
                    }
                    strr = json_object_get_string(serv_obj, "dntopic");  // MQTT key
                    if (strr != NULL) {
                        strncpy(serv_entry->net->mqtt->dntopic, strr, sizeof(serv_entry->net->mqtt->dntopic));
                        serv_entry->net->mqtt->dntopic[sizeof(serv_entry->net->mqtt->dntopic) - 1] = '\0'; 
                        lgw_log(LOG_INFO, "[INFO~][SETTING][%s] Found a mqtt dntopic is \"%s\"\n", serv_entry->info.name, strr);
                    } else {
                        strcpy(serv_entry->net->mqtt->dntopic, "test");
                        lgw_log(LOG_WARNING, "%s[SETTING][%s] Need a dntopic for mqtt publish, set to default value \"test\"\n", WARNMSG, serv_entry->info.name);
                    }
				} else if (!strncmp(str, "gwtraf", 6)) {
					serv_entry->info.type = gwtraf;
				} else 
					serv_entry->info.type = semtech;
            } else {
					serv_entry->info.type = semtech;  // 默认的服务是semtech
            }

			val = json_object_get_value(serv_obj, "enabled");
            if ( val != NULL) {
                if (json_value_get_type(val) == JSONBoolean) 
                    serv_entry->info.enabled = json_value_get_boolean(val);
                else
                    serv_entry->info.enabled = true;   // 默认是开启的
            } else 
                serv_entry->info.enabled = true;   // 默认是开启的

			if (serv_entry->info.type == semtech) {

                serv_entry->report = (report_s*)lgw_malloc(sizeof(report_s));
                serv_entry->report->report_ready = false;
                strcpy(serv_entry->report->stat_format, "semtech");
                serv_entry->report->stat_interval = DEFAULT_STAT_INTERVAL;
                pthread_mutex_init(&serv_entry->report->mx_report, NULL);

                str = json_object_get_string(serv_obj, "server_address");
                if (str != NULL) {
                    strncpy(serv_entry->net->addr, str, sizeof(serv_entry->net->addr));
                    serv_entry->net->addr[sizeof(serv_entry->net->addr) - 1] = '\0'; 
                    lgw_log(LOG_INFO, "[INFO~][SETTING][%s] server address is configure to \"%s\"\n", serv_entry->info.name, str);
                } else {  //如果没有设置服务地址，就释放内存，读入下一条记录
                    lgw_free(serv_entry->net);
                    lgw_free(serv_entry->report);
                    lgw_free(serv_entry);
                    continue;
                }

                val = json_object_get_value(serv_obj, "serv_port_up");
                if (val != NULL) {
                    snprintf(serv_entry->net->port_up, sizeof(serv_entry->net->port_up), "%u", (uint16_t)json_value_get_number(val));
                    serv_entry->net->port_up[sizeof(serv_entry->net->port_up) - 1] = '\0'; 
                } else {
                    strcpy(serv_entry->net->port_up, "1700");
                }
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] serv_port_up is configure to \"%s\"\n", serv_entry->info.name, serv_entry->net->port_up);

                val = json_object_get_value(serv_obj, "serv_port_down");
                if (val != NULL) {
                    snprintf(serv_entry->net->port_down, sizeof(serv_entry->net->port_down), "%u", (uint16_t)json_value_get_number(val));
                    serv_entry->net->port_down[sizeof(serv_entry->net->port_down) - 1] = '\0'; 
                } else {
                    strcpy(serv_entry->net->port_down, "1700");
                }
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] serv_port_down is configure to \"%s\"\n", serv_entry->info.name, serv_entry->net->port_down);

                val = json_object_get_value(serv_obj, "push_timeout_ms");
                if (val != NULL) {
                    serv_entry->net->push_timeout_half.tv_usec = 500 * (long int)json_value_get_number(val);
                }

                val = json_object_get_value(serv_obj, "pull_timeout_ms");
                if (val != NULL) {
                    serv_entry->net->pull_timeout.tv_usec = 1000 * (long int)json_value_get_number(val);
                }

                val = json_object_get_value(serv_obj, "pull_interval");
                if (val != NULL) {
                    serv_entry->net->pull_interval = (int)json_value_get_number(val);
                }

                val = json_object_get_value(serv_obj, "stat_interval");
                if (val != NULL) {
                    serv_entry->report->stat_interval = (int)json_value_get_number(val);
                    lgw_log(LOG_INFO, "[INFO~][SETTING][%s] stat_interval is configure to \"%d\"\n", serv_entry->info.name, serv_entry->report->stat_interval);
                }

            } //end of not as pkt type
            serv_entry->filter.fwd_valid_pkt = true;
            serv_entry->filter.fwd_error_pkt = true;
            serv_entry->filter.fwd_nocrc_pkt = true;
            serv_entry->filter.fport = NOFILTER;
            serv_entry->filter.devaddr = NOFILTER;
            serv_entry->filter.nwkid = NOFILTER;
            serv_entry->filter.deveui = NOFILTER;

            val = json_object_get_value(serv_obj, "forward_crc_valid");
            if (val != NULL) {
                if (json_value_get_type(val) == JSONBoolean) {
                    serv_entry->filter.fwd_valid_pkt = (bool)json_value_get_boolean(val);
                }
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a valid CRC will%s be forwarded\n", serv_entry->info.name, (serv_entry->filter.fwd_valid_pkt ? "" : " NOT"));
            }

            val = json_object_get_value(serv_obj, "forward_crc_error");
            if (val != NULL) {
                if (json_value_get_type(val) == JSONBoolean) {
                    serv_entry->filter.fwd_error_pkt = (bool)json_value_get_boolean(val);
                }
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a CRC error will%s be forwarded\n", serv_entry->info.name, (serv_entry->filter.fwd_error_pkt ? "" : " NOT"));
            }

            val = json_object_get_value(serv_obj, "forward_crc_disabled");
            if (val != NULL) {
                if (json_value_get_type(val) == JSONBoolean) {
                    serv_entry->filter.fwd_nocrc_pkt = (bool)json_value_get_boolean(val);
                }
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a no CRC will%s be forwarded\n", serv_entry->info.name, (serv_entry->filter.fwd_nocrc_pkt ? "" : " NOT"));
            }
            
            val = json_object_get_value(serv_obj, "fport_filter");
            if (val != NULL) {
                try = (uint8_t)json_value_get_number(val);
                if (try == 1)
                    serv_entry->filter.fport= INCLUDE;
                else if (try == 2)
                    serv_entry->filter.fport = EXCLUDE;
                else
                    serv_entry->filter.fport = NOFILTER;
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a fport filter, level(%d)\n", serv_entry->info.name, serv_entry->filter.fport);
            } 

            val = json_object_get_value(serv_obj, "devaddr_filter");
            if (val != NULL) {
                try = (uint8_t)json_value_get_number(val);
                if (try == 1)
                    serv_entry->filter.devaddr = INCLUDE;
                else if (try == 2)
                    serv_entry->filter.devaddr = EXCLUDE;
                else
                    serv_entry->filter.devaddr = NOFILTER;
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a devaddr filter, level(%d)\n", serv_entry->info.name, serv_entry->filter.devaddr);
            } 

            val = json_object_get_value(serv_obj, "nwkid_filter");
            if (val != NULL) {
                try = (uint8_t)json_value_get_number(val);
                if (try == 1)
                    serv_entry->filter.nwkid = INCLUDE;
                else if (try == 2)
                    serv_entry->filter.nwkid = EXCLUDE;
                else
                    serv_entry->filter.nwkid = NOFILTER;
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a nwkid filter, level(%d)\n", serv_entry->info.name, serv_entry->filter.nwkid);
            }

			val = json_object_get_value(serv_obj, "deveui_filter");
            if (val != NULL) {
                try = (uint8_t)json_value_get_number(val);
                if (try == 1)
                    serv_entry->filter.deveui = INCLUDE;
                else if (try == 2)
                    serv_entry->filter.deveui = EXCLUDE;
                else
                    serv_entry->filter.deveui = NOFILTER;
                lgw_log(LOG_INFO, "[INFO~][SETTING][%s] packets received with a deveui filter, level(%d)\n", serv_entry->info.name, serv_entry->filter.deveui);
            }

            LGW_LIST_INSERT_TAIL(&GW.serv_list, serv_entry, list);
        }
    } else 
        lgw_log(LOG_INFO, "%s[SETTING] None service offer.\n", WARNMSG);

    /*!> free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

#ifdef SX1302MOD

static int parse_debug_configuration(const char * conf_file) {
    int i;
    const char conf_obj_name[] = "debug_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Array *conf_array = NULL;
    JSON_Object *conf_obj_array = NULL;
    const char *str; /*!> pointer to sub-strings in the JSON data */

    struct lgw_conf_debug_s debugconf;

    /*!> Initialize structure */
    memset(&debugconf, 0, sizeof(debugconf));

    /*!> try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        lgw_log(LOG_INFO, "%s[SETTING] %s is not a valid JSON file\n", ERRMSG, conf_file);
        return -1;
    }

    /*!> point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        lgw_log(LOG_INFO, "[INFO~][SETTING] %s does contain a JSON object named %s, parsing debug parameters\n", conf_file, conf_obj_name);
    }

    /*!> Get reference payload configuration */
    conf_array = json_object_get_array (conf_obj, "ref_payload");
    if (conf_array != NULL) {
        debugconf.nb_ref_payload = json_array_get_count(conf_array);
        lgw_log(LOG_INFO, "[INFO~][SETTING] got %u debug reference payload\n", debugconf.nb_ref_payload);

        for (i = 0; i < (int)debugconf.nb_ref_payload; i++) {
            conf_obj_array = json_array_get_object(conf_array, i);
            /*!> id */
            str = json_object_get_string(conf_obj_array, "id");
            if (str != NULL) {
                sscanf(str, "0x%08X", &(debugconf.ref_payload[i].id));
                lgw_log(LOG_INFO, "[INFO~][SETTING] reference payload ID %d is 0x%08X\n", i, debugconf.ref_payload[i].id);
            }

            /*!> global count */
            GW.log.nb_pkt_received_ref[i] = 0;
        }
    }

    /*!> Get log file configuration */
    str = json_object_get_string(conf_obj, "log_file");
    if (str != NULL) {
        strncpy(debugconf.log_file_name, str, sizeof(debugconf.log_file_name));
        debugconf.log_file_name[sizeof(debugconf.log_file_name) - 1] = '\0'; 
        lgw_log(LOG_INFO, "[INFO~][SETTING] setting debug log file name to %s\n", debugconf.log_file_name);
    }

    /*!> Commit configuration */
    if (lgw_debug_setconf(&debugconf) != LGW_HAL_SUCCESS) {
        lgw_log(LOG_INFO, "%s[SETTING] Failed to configure debug\n", ERRMSG);
        json_value_free(root_val);
        return -1;
    }

    /*!> free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

#endif

int parsecfg() {
    int ret = 0;
    ret = parse_SX130x_configuration(GW.hal.confs.sxcfg);
    ret |= parse_gateway_configuration(GW.hal.confs.gwcfg);
#ifdef SX1302MOD
    parse_debug_configuration(GW.hal.confs.gwcfg);
#endif
    return ret;
}

