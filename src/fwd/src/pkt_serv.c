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
 * \brief 
 *  Description:
*/

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include <semaphore.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>

#include "db.h"
#include "fwd.h"
#include "jitqueue.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "timersync.h"

#include "service.h"
#include "pkt_service.h"
#include "loramac-crypto.h"
#include "mac-header-decode.h"

DECLARE_GW;

typedef struct _dn_pkt {
	LGW_LIST_ENTRY(_dn_pkt) list;
    char devaddr[16];
    char txmode[8];
    char pdformat[8];
    uint8_t relay;     
    uint8_t payload[512];
    uint32_t txfreq;
    uint8_t *fopt;
    uint8_t ftype;
    uint8_t mode;
    uint8_t psize;
    uint8_t txdr;
    uint8_t txbw;
    uint8_t txpw;
    uint8_t rxwindow;
    uint8_t txport;
    uint8_t optlen;
} dn_pkt_s;

//extern struct pthread_list pkt_pthread_list;

static uint8_t rx2bw;
static uint8_t rx2dr;
static uint32_t rx2freq;

static char db_family[32] = {'\0'};
static char db_key[32] = {'\0'};
static char tmpstr[16] = {'\0'};

LGW_LIST_HEAD_STATIC(dn_list, _dn_pkt); // downlink for customer

static int pthread_pkt_count = 0;
pthread_mutex_t  mx_pthread_pkt_count = PTHREAD_MUTEX_INITIALIZER;

static uint32_t current_concentrator_time;

static void pkt_prepare_downlink(void* arg);
static void pkt_deal_up(void* arg);
static void thread_pkt_deal_up(void* arg);
static void prepare_frame(dn_pkt_s*, devinfo_s*, uint32_t, uint8_t*, int*);
static int strcpypt(char* dest, const char* src, int* start, int size, int len);

static enum jit_error_e custom_rx2dn(dn_pkt_s* dnelem, devinfo_s *devinfo, uint32_t us, uint8_t txmode) {
    int i, fsize = 0;

    uint32_t dwfcnt = 1;

    uint8_t payload_en[PKT_PAYLOAD_SIZE] = {'\0'};  /*!> data which have decrypted */
    struct lgw_pkt_tx_s txpkt;

    enum jit_error_e jit_result = JIT_ERROR_OK;
    enum jit_pkt_type_e downlink_type;

    memset(&txpkt, 0, sizeof(txpkt));

    if (dnelem->mode > 0)
        txpkt.modulation = dnelem->mode;
    else
        txpkt.modulation = MOD_LORA;

    txpkt.no_crc = true;

    if (dnelem->txfreq > 0)
        txpkt.freq_hz = dnelem->txfreq; 
    else
        txpkt.freq_hz = rx2freq; 

    txpkt.rf_chain = 0;

    if (dnelem->txpw > 0)
        txpkt.rf_power = dnelem->txpw;
    else
        txpkt.rf_power = 20;

    if (dnelem->txdr > 0)
        txpkt.datarate = dnelem->txdr;
    else
        txpkt.datarate = rx2dr;

    if (dnelem->txbw > 0)
        txpkt.bandwidth = dnelem->txbw;
    else
        txpkt.bandwidth = rx2bw;

    txpkt.coderate = CR_LORA_4_5;

    txpkt.invert_pol = true;

    txpkt.preamble = STD_LORA_PREAMB;

    txpkt.tx_mode = txmode;

    if (txmode)
        downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_A;
    else
        downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;

    if (!dnelem->relay) {
        if (dnelem->rxwindow > 1)
            txpkt.count_us = us + 2000000UL; /*!> rx2 window plus 2s */
        else
            txpkt.count_us = us + 1000000UL; /*!> rx1 window plus 1s */
        /*!> 这个key重启将会删除, 下发的计数器 */
        sprintf(db_family, "/downlink/%08X", devinfo->devaddr);
        if (lgw_db_get(db_family, "fcnt", tmpstr, sizeof(tmpstr)) == -1) {
            lgw_db_put(db_family, "fcnt", "1");
        } else { 
            dwfcnt = atol(tmpstr);
            sprintf(tmpstr, "%u", dwfcnt + 1);
            lgw_db_put(db_family, "fcnt", tmpstr);
        }

        /*!> prepare MAC message */
        lgw_memset(payload_en, '\0', sizeof(payload_en));

        prepare_frame(dnelem, devinfo, dwfcnt++, payload_en, &fsize);

        lgw_memcpy(txpkt.payload, payload_en, fsize);

        txpkt.size = fsize;

    } else { // G2G testing  TODO! ////
        txpkt.count_us = us + 1000000UL; /*!> change to rx2 window */
        lgw_memcpy(txpkt.payload, dnelem->payload, dnelem->psize);
        txpkt.size = dnelem->psize;
    }

#ifdef SX1302MOD
    pthread_mutex_lock(&GW.hal.mx_concent);
    lgw_get_instcnt(&current_concentrator_time);
    pthread_mutex_unlock(&GW.hal.mx_concent);
#else
    get_concentrator_time(&current_concentrator_time);
#endif
    jit_result = jit_enqueue(&GW.tx.jit_queue[txpkt.rf_chain], current_concentrator_time, &txpkt, downlink_type);
    lgw_log(LOG_DEBUG, "%s[DNLK] DNRX2-> tmst=%u, freq=%u, size=%u, BW%uSF%u, ipol=%s\n", DEBUGMSG, txpkt.count_us, txpkt.freq_hz, txpkt.size, txpkt.bandwidth, txpkt.datarate, txpkt.invert_pol ? "true" : "false");
    lgw_log(LOG_DEBUG, "%s[DNLK][PAYLOAD]####################################################\n", DEBUGMSG);
    for (i = 0; i < txpkt.size; i++) {
        lgw_log(LOG_DEBUG, "%02x", txpkt.payload[i]);
    }
    lgw_log(LOG_DEBUG, "\n%s[DNLK][PAYLOAD]####################################################\n", DEBUGMSG);
    return jit_result;
}

static void prepare_frame(dn_pkt_s* dnelem, devinfo_s* devinfo, uint32_t downcnt, uint8_t* frame, int* frame_size) {
	uint32_t mic;
	uint8_t index = 0;
	uint8_t* encrypt_payload;

	LoRaMacHeader_t hdr;
	LoRaMacFrameCtrl_t fctrl;

	/*!>MHDR*/
	hdr.Value = 0;
	hdr.Bits.MType = dnelem->ftype;
	frame[index] = hdr.Value;

	/*!>DevAddr*/
	frame[++index] = devinfo->devaddr&0xFF;
	frame[++index] = (devinfo->devaddr>>8)&0xFF;
	frame[++index] = (devinfo->devaddr>>16)&0xFF;
	frame[++index] = (devinfo->devaddr>>24)&0xFF;

	/*!>FCtrl*/
	fctrl.Value = 0;
	if (dnelem->ftype == FRAME_TYPE_DATA_UNCONFIRMED_DOWN) {
		fctrl.Bits.Ack = 1;
	}
	fctrl.Bits.Adr = 1;

	/*!>FOptsLen*/
    if (dnelem->optlen && (NULL != dnelem->fopt)) 
	    fctrl.Bits.FOptsLen = dnelem->optlen;

	frame[++index] = fctrl.Value;

	/*!>FCnt*/
	frame[++index] = (downcnt)&0xFF;
	frame[++index] = (downcnt>>8)&0xFF;

	/*!>FOpts*/
    if (dnelem->optlen && (NULL != dnelem->fopt)) {
	    ++index;
        lgw_memcpy(frame + index, dnelem->fopt, dnelem->optlen);
        lgw_free(dnelem->fopt); 
	    index = index + dnelem->optlen - 1;
    }

	/*!>Fport*/
	frame[++index] = dnelem->txport&0xFF;

	/*!>encrypt the payload*/
	encrypt_payload = lgw_malloc(sizeof(uint8_t) * dnelem->psize);
	LoRaMacPayloadEncrypt(dnelem->payload, dnelem->psize, (dnelem->txport == 0) ? devinfo->nwkskey : devinfo->appskey, devinfo->devaddr, DOWN, downcnt, encrypt_payload);
	++index;
	memcpy(frame + index, encrypt_payload, dnelem->psize);
	lgw_free(encrypt_payload);
	index += dnelem->psize;

	/*!>calculate the mic*/
	LoRaMacComputeMic(frame, index, devinfo->nwkskey, devinfo->devaddr, DOWN, downcnt, &mic);
    //printf("%s[MIC] %08X\n", INFOMSG, mic);
	frame[index] = mic&0xFF;
	frame[++index] = (mic>>8)&0xFF;
	frame[++index] = (mic>>16)&0xFF;
	frame[++index] = (mic>>24)&0xFF;
	*frame_size = index + 1;
}

static inline dn_pkt_s* search_dn_list(const char* addr) {
    dn_pkt_s* entry = NULL;

    LGW_LIST_LOCK(&dn_list);
    LGW_LIST_TRAVERSE(&dn_list, entry, list) {
        if (!strcasecmp(entry->devaddr, addr))
            break;
    }
    LGW_LIST_UNLOCK(&dn_list);

    return entry;
}


int pkt_start(serv_s* serv) {
    if (lgw_pthread_create_background(&serv->thread.t_up, NULL, (void *(*)(void *))pkt_deal_up, serv)) {
        lgw_log(LOG_WARNING, "%s[THREAD][%s] Can't create packages deal pthread.\n", WARNMSG, serv->info.name);
        return -1;
    }

    switch (GW.cfg.region) {
        case EU: 
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 869525000UL;
            break;
        case EU433: 
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 434665000UL;
            break;
        case US:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_500KHZ;
            rx2freq = 923300000UL;
            break;
        case CN470:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 505300000UL;
            break;
        case CN779:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 786000000UL;
            break;
        case AU:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_500KHZ;
            rx2freq = 923300000UL;
            break;
        case AS1:
            rx2dr = DR_LORA_SF10;
            rx2bw = BW_125KHZ;
            rx2freq = 923200000UL;
            break;
        case AS2: // 923.2MHz + AS923_FREQ_OFFSET_HZ ( -1.8MHz )
            rx2dr = DR_LORA_SF10;
            rx2bw = BW_125KHZ;
            rx2freq = 923020000UL;
            break;
        case AS3: // ( -6.6MHz )
            rx2dr = DR_LORA_SF10;
            rx2bw = BW_125KHZ;
            rx2freq = 922540000UL;
            break;
        case KR:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 921900000UL;
            break;
        case IN:
            rx2dr = DR_LORA_SF10;
            rx2bw = BW_125KHZ;
            rx2freq = 866550000UL;
            break;
        case RU:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 869100000UL;
            break;
        case KZ:  //KZ865 
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 866700000UL;
            break;
        default:
            rx2dr = DR_LORA_SF12;
            rx2bw = BW_125KHZ;
            rx2freq = 869525000UL;
            break;
    }

    if (GW.cfg.custom_downlink) {
        if (lgw_pthread_create_background(&serv->thread.t_down, NULL, (void *(*)(void *))pkt_prepare_downlink, (void*)serv)) {
            lgw_log(LOG_WARNING, "%s[THREAD][%s] Can't create pthread for custom downlonk.\n", WARNMSG, serv->info.name);
            return -1;
        }
    }

    LGW_LIST_LOCK(&GW.rxpkts_list);
    GW.info.service_count++;
    LGW_LIST_UNLOCK(&GW.rxpkts_list);

    lgw_db_put("service/pkt", serv->info.name, "runing");
    lgw_db_put("thread", serv->info.name, "runing");

    return 0;
}

void pkt_stop(serv_s* serv) {
    serv->thread.stop_sig = true;
	sem_post(&serv->thread.sema);
	pthread_join(serv->thread.t_up, NULL);
    if (GW.cfg.custom_downlink) {
	    pthread_join(serv->thread.t_down, NULL);
    }
    LGW_LIST_LOCK(&GW.rxpkts_list);
    GW.info.service_count--;
    LGW_LIST_UNLOCK(&GW.rxpkts_list);
    lgw_db_del("service/pkt", serv->info.name);
    lgw_db_del("thread", serv->info.name);
}

static uint32_t timeval_sub(struct timeval *a, struct timeval *b) {
    uint32_t t = 0;
    if (b->tv_sec > a->tv_sec) {
        t = b->tv_sec - a->tv_sec;
        t = t * 1000000UL;
    }

    t = t + b->tv_usec;

    if (t > a->tv_usec) {
        t = t - a->tv_usec;
    } else
        t = 0;

    return t;
}

static void thread_pkt_deal_up(void* arg) {
    serv_ct_s* serv_ct = (serv_ct_s*) arg;
    serv_s* serv = serv_ct->serv;
    //pthread_t tid = pthread_self();
    
	int i, j;					/*!> loop variables */

    int fsize = 0;              /* FRMpayload size */

	struct lgw_pkt_rx_s *p;	/*!> pointer on a RX packet */

    enum jit_error_e jit_result = JIT_ERROR_OK;
    
    uint8_t payload_encrypt[PKT_PAYLOAD_SIZE] = {'\0'};
    uint8_t payload_txt[PKT_PAYLOAD_SIZE] = {'\0'};

    LoRaMacMessageData_t macmsg;

    for (i = 0; i < serv_ct->nb_pkt; i++) {
        p = &serv_ct->rxpkt[i];

        if (p->if_chain == IF_DELAY) {
            lgw_log(LOG_DEBUG, "%s[%s-UP] skip from Storage\n", DEBUGMSG, serv->info.name);
            continue;
        }

        switch(p->status) {
            case STAT_CRC_OK:
                if (!serv->filter.fwd_valid_pkt) {
                    continue; /*!> skip that packet */
                }
                break;
            case STAT_CRC_BAD:
                if (!serv->filter.fwd_error_pkt) {
                    continue; /*!> skip that packet */
                }
                break;
            case STAT_NO_CRC:
                if (!serv->filter.fwd_nocrc_pkt) {
                    continue; /*!> skip that packet */
                }
                break;
            default:
                continue; /*!> skip that packet */
        }
        
        /* MHDR: 1byte; FHDR: 7byte ; FPORT: 1byte;  MIC: 4byte , total: 13 */
        fsize = p->size - 13; 

        if (fsize < 1) {
            lgw_log(LOG_DEBUG, "%s[DECODE] frmpayload (fsize=%d) not match!\n", DEBUGMSG, fsize);
            continue;
        }

        /*!>for G2G(relay) 
         ** Receive downlink from other GW **
         **/

        if (GW.relay.as_relay && p->if_chain == 8 && ((p->payload[0] & RELAY_DN) == RELAY_DN)) {  
            uint32_t count_us = 0;
            count_us = (uint32_t)p->payload[1];
            count_us |= (uint32_t)p->payload[2]<<8;
            count_us |= (uint32_t)p->payload[3]<<16;
            count_us |= (uint32_t)p->payload[4]<<24;
            dn_pkt_s* entry = NULL;
            entry = (dn_pkt_s*) lgw_malloc(sizeof(dn_pkt_s));
            entry->optlen = 0;
            entry->fopt = NULL;
            entry->txpw = 0;
            entry->txbw = 0;
            entry->txdr = 0;
            entry->txfreq = 0;
            entry->relay = 1;
            entry->rxwindow = 2;
            entry->txport = DEFAULT_DOWN_FPORT;
            entry->ftype = FRAME_TYPE_DATA_UNCONFIRMED_DOWN;
            strcpy(entry->txmode, "time");
            lgw_memcpy(entry->payload, &(p->payload[5]), p->size - 5);
            entry->psize = p->size - 5;
            jit_result = custom_rx2dn(entry, NULL, count_us, TIMESTAMPED);
            if (jit_result != JIT_ERROR_OK)
                lgw_log(LOG_ERROR, "%s[RELAY]REJECTED time:%u (jit error=%d)\n", ERRMSG, count_us, jit_result);
            else {
                lgw_log(LOG_DEBUG, "%s[RELAY]customer downlink time:%u, size:%d\n", DEBUGMSG, count_us, entry->psize);
            }
            lgw_free(entry);

            continue;   /*!> next packet */
        }

        memset(&macmsg, 0, sizeof(macmsg));
        macmsg.Buffer = p->payload;
        macmsg.BufSize = p->size;

        if (LORAMAC_PARSER_SUCCESS != LoRaMacParserData(&macmsg))    /*!> MAC decode */
            continue;

        if (serv->filter.fport != NOFILTER || serv->filter.devaddr != NOFILTER || 
			serv->filter.nwkid != NOFILTER || serv->filter.deveui != NOFILTER) {
            if (pkt_basic_filter(serv, macmsg.FHDR.DevAddr, macmsg.FPort, macmsg.DevEUI)) {
                lgw_log(LOG_DEBUG, "%s[%s-UP] Drop a packet has fport(%u) of %08X.\n", DEBUGMSG, serv->info.name, macmsg.FPort, macmsg.FHDR.DevAddr);
                continue;
            }
        }

        decode_mac_pkt_up(&macmsg, p);

        if (GW.cfg.mac_decode || GW.cfg.custom_downlink) {
            devinfo_s devinfo = { .devaddr = macmsg.FHDR.DevAddr, 
                                  .devaddr_str = {0},
                                  .appskey_str = {0},
                                  .nwkskey_str = {0}
                                };
            sprintf(db_family, "devinfo/%08X", devinfo.devaddr);
            if ((lgw_db_get(db_family, "appskey", devinfo.appskey_str, sizeof(devinfo.appskey_str)) == -1) || 
                (lgw_db_get(db_family, "nwkskey", devinfo.nwkskey_str, sizeof(devinfo.nwkskey_str)) == -1)) {
                continue;
            }

            str2hex(devinfo.appskey, devinfo.appskey_str, sizeof(devinfo.appskey));
            str2hex(devinfo.nwkskey, devinfo.nwkskey_str, sizeof(devinfo.nwkskey));

            /*!> Debug message of appskey */

            /*
            lgw_log(LOG_DEBUG, "\n%s[MAC-Decode]appskey:", DEBUGMSG);
            for (j = 0; j < (int)sizeof(devinfo.appskey); ++j) {
                lgw_log(LOG_DEBUG, "%02X", devinfo.appskey[j]);
            }
            lgw_log(LOG_DEBUG, "\n");

            lgw_log(LOG_DEBUG, "\n%s[MAC-Decode]nwkskey:", DEBUGMSG);
            for (j = 0; j < (int)sizeof(devinfo.nwkskey); ++j) {
                lgw_log(LOG_DEBUG, "%02X", devinfo.nwkskey[j]);
            }
            lgw_log(LOG_DEBUG, "\n");

            */

            lgw_log(LOG_DEBUG, "%s[DECODE][PAYLOAD][size:%d]#############################################\n", DEBUGMSG, p->size);
            for (j = 0; j < p->size; j++) {
                lgw_log(LOG_DEBUG, "%02x", p->payload[j]);
            }
            lgw_log(LOG_DEBUG, "\n%s[DECODE][PAYLOAD]####################################################\n", DEBUGMSG);

            if (GW.cfg.mac_decode) {
                uint32_t fcnt, mic;
				bool fcnt_valid = false;
                lgw_memcpy(payload_encrypt, p->payload + 9 + macmsg.FHDR.FCtrl.Bits.FOptsLen, fsize);
                for (j = 0; j < GW.cfg.fcnt_gap; j++) {   
                    fcnt = macmsg.FHDR.FCnt | (j * 0x10000);
                    /* msglen = p-size - len(MIC) */
                    LoRaMacComputeMic(p->payload, p->size - 4, devinfo.nwkskey, devinfo.devaddr, UP, fcnt, &mic);
                    if (mic == macmsg.MIC) {
                        fcnt_valid = true;
                        lgw_log(LOG_DEBUG, "%s[DECODE] Found a match MIC, fcnt=(%u)\n", DEBUGMSG, fcnt);
                        break;
                    }
                }

                if (!fcnt_valid) {
                    fcnt = macmsg.FHDR.FCnt;
                }

                if (macmsg.FPort == 0)
                    LoRaMacPayloadDecrypt(payload_encrypt, fsize, devinfo.nwkskey, devinfo.devaddr, UP, fcnt, payload_txt);
                else
                    LoRaMacPayloadDecrypt(payload_encrypt, fsize, devinfo.appskey, devinfo.devaddr, UP, fcnt, payload_txt);

                /*!> Debug message of decoded payload */
                /*!>*/
                lgw_log(LOG_DEBUG, "\n%s[DECODED][DECRYPT][SIZE:%d]##########################################\n", DEBUGMSG, fsize);
                for (j = 0; j < fsize; j++) {
                    lgw_log(LOG_DEBUG, "%02X", payload_txt[j]);
                }
                lgw_log(LOG_DEBUG, "\n%s[DECODE][DECRYPT]####################################################\n", DEBUGMSG);

                if (GW.cfg.mac2file) {
                    FILE *fp;
                    char pushpath[128];
                    char rssi_snr[32] = {'\0'};
                    snprintf(pushpath, sizeof(pushpath), "/var/iot/channels/%08X-%u", devinfo.devaddr, fcnt % 5);
                    fp = fopen(pushpath, "w+");
                    if (NULL == fp)
                        lgw_log(LOG_WARNING, "%s[DECODE] Fail to open path: %s\n", WARNMSG, pushpath);
                    else { 
                        sprintf(rssi_snr, "%08X%08X", (short)p->rssic, (short)(p->snr*10));
                        fwrite(rssi_snr, sizeof(char), 16, fp);
                        fwrite(payload_txt, sizeof(uint8_t), fsize, fp);
                        fflush(fp); 
                        fclose(fp);
                    }
                
                }

                if (GW.cfg.mac2db) { /*!> 每个devaddr最多保存10个payload */
                    sprintf(db_family, "/payload/%08X", devinfo.devaddr);
                    if (lgw_db_get(db_family, "index", tmpstr, sizeof(tmpstr)) == -1) {
                        lgw_db_put(db_family, "index", "0");
                    } else 
                        j = atoi(tmpstr) % 9;
                    sprintf(db_family, "/payload/%08X/%d", devinfo.devaddr, j);
                    sprintf(db_key, "%u", p->count_us);
                    lgw_db_put(db_family, db_key, (char*)payload_txt);
                }
            }

            if (GW.cfg.custom_downlink) {
                /*!> Customer downlink process */
                dn_pkt_s* dnelem = NULL;
                sprintf(tmpstr, "%08X", devinfo.devaddr);
                dnelem = search_dn_list(tmpstr);
                if (dnelem != NULL) {
                    lgw_log(LOG_DEBUG, "%s[DNLK]Found a match devaddr: %s, prepare a downlink!\n", DEBUGMSG, tmpstr);
                    jit_result = custom_rx2dn(dnelem, &devinfo, p->count_us, TIMESTAMPED);
                    if (jit_result == JIT_ERROR_OK) { /*!> Next upmsg willbe indicate if received by note */
                        LGW_LIST_LOCK(&dn_list);
                        LGW_LIST_REMOVE(&dn_list, dnelem, list);
                        lgw_free(dnelem);
                        LGW_LIST_UNLOCK(&dn_list);
                    } else {
                        lgw_log(LOG_ERROR, "%s[DNLK]Packet REJECTED (jit error=%d)\n", ERRMSG, jit_result);
                    }

                } else
                    lgw_log(LOG_DEBUG, "%s[DNLK]NO custom downlink command for Dev %08X\n", DEBUGMSG, devinfo.devaddr);
            }
        }
    } // for nb_pkt loop

    lgw_free(serv_ct);

    pthread_mutex_lock(&mx_pthread_pkt_count);
    pthread_pkt_count--;
    pthread_mutex_unlock(&mx_pthread_pkt_count);

    //gettimeofday(&end_us, NULL);

    //lgw_log(LOG_DEBUG, "THREAD~>> [tid=%ld, end=%u:%u, start=%u:%u, sub=%u] (R=%d)\n", tid, end_us.tv_sec, end_us.tv_usec, start_us.tv_sec, start_us.tv_usec, timeval_sub(&start_us, &end_us), pthread_pkt_count);
}

static void pkt_prepare_downlink(void* arg) {
    serv_s* serv = (serv_s*) arg;
    lgw_log(LOG_INFO, "%s[THREAD][%s] Staring pkt_prepare_downlink thread...\n", INFOMSG, serv->info.name);
    
    int i, j, start; /*!> loop variables, start of cpychar */
    uint8_t psize = 0, size = 0; /*!> sizeof payload, file char lenth */

    uint32_t uaddr;

    DIR *dir;
    FILE *fp;
    struct dirent *ptr;
    struct stat statbuf;
    char dn_file[256]; 

    /*!> data buffers */
    uint8_t buff_down[512]; /*!> buffer to receive downstream packets */
    uint8_t dnpld[256];
    uint8_t hexpld[256];
    uint8_t hexopt[16];
    char swap_str[32];

    uint32_t bw_display;
    
    dn_pkt_s* entry = NULL;
    dn_pkt_s* element = NULL;

    enum jit_error_e jit_result = JIT_ERROR_OK;

    while (!serv->thread.stop_sig) {
        
        /*!> lookup file */
        if ((dir = opendir(DNPATH)) == NULL) {
            //lgw_log(LOG_ERROR, "%s[push]open sending path error\n", ERRMSG);
            wait_ms(100); 
            continue;
        }

	    while ((ptr = readdir(dir)) != NULL) {
            if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) /*!> current dir OR parrent dir */
                continue;

            lgw_log(LOG_INFO, "%s[DNLK]Looking file : %s\n", INFOMSG, ptr->d_name);

            snprintf(dn_file, sizeof(dn_file), "%s/%s", DNPATH, ptr->d_name);

            if (stat(dn_file, &statbuf) < 0) {
                lgw_log(LOG_ERROR, "%s[DNLK]Canot stat %s!\n", ERRMSG, ptr->d_name);
                continue;
            }

            if ((statbuf.st_mode & S_IFMT) == S_IFREG) {
                if ((fp = fopen(dn_file, "r")) == NULL) {
                    lgw_log(LOG_ERROR, "%s[DNLK]Cannot open %s\n", ERRMSG, ptr->d_name);
                    continue;
                }

                lgw_memset(buff_down, '\0', sizeof(buff_down));

                size = fread(buff_down, sizeof(char), sizeof(buff_down), fp); /*!> the size less than buff_down return EOF */
                fclose(fp);

                unlink(dn_file); /*!> delete the file */

                for (i = 0, j = 0; i < size; i++) {
                    if (buff_down[i] == ',')
                        j++;
                }

                if (j < 3) { /*!> Error Format, ',' must be greater than or equal to 3*/
                    lgw_log(LOG_INFO, "%s[DNLK]Format error: %s\n", INFOMSG, buff_down);
                    continue;
                }

                start = 0;

                /*!>*******************************************************/
                /*!> constrator dn_pkt_s, first fill default value */
                /*!>*******************************************************/
                entry = (dn_pkt_s*) lgw_malloc(sizeof(dn_pkt_s));
                entry->optlen = 0; 
                entry->fopt = NULL;
                entry->txpw = 0;
                entry->txbw = 0; 
                entry->txdr = 0; 
                entry->txfreq = 0; 
                entry->relay = 0; 
                entry->rxwindow = 2; 
                entry->txport = DEFAULT_DOWN_FPORT; 
                entry->ftype = FRAME_TYPE_DATA_UNCONFIRMED_DOWN;        

                /*!>* TODO: should be rewrite the function process **/

                /*!>* 1. addr **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) < 1) { 
                    lgw_free(entry);
                    continue;
                } else
                    strcpy(entry->devaddr, swap_str);


                /*!>* 2. txmode **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) < 1)
                    strcpy(entry->txmode, "time");
                else
                    strcpy(entry->txmode, swap_str); 

                /*!>* 3. payload format **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) < 1)
                    strcpy(entry->pdformat, "txt"); 
                else
                    strcpy(entry->pdformat, swap_str); 

                /*!>* 4. payload **/
                psize = strcpypt((char*)dnpld, (char*)buff_down, &start, size, sizeof(dnpld)); 
                if (psize < 1) {
                    lgw_free(entry);
                    continue;
                }

                /*!>* 5. tx power **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    entry->txpw = atoi(swap_str);
                } 

                /*!>* 6. tx bandwith **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    entry->txbw = atoi(swap_str);
                } 

                /*!>* 7. tx sf **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    if (!strncmp(swap_str, "SF7", 3))
                        entry->txdr = DR_LORA_SF7; 
                    else if (!strncmp(swap_str, "SF8", 3))
                        entry->txdr = DR_LORA_SF8; 
                    else if (!strncmp(swap_str, "SF9", 3))
                        entry->txdr = DR_LORA_SF9; 
                    else if (!strncmp(swap_str, "SF10", 4))
                        entry->txdr = DR_LORA_SF10; 
                    else if (!strncmp(swap_str, "SF11", 4))
                        entry->txdr = DR_LORA_SF11; 
                    else if (!strncmp(swap_str, "SF12", 4))
                        entry->txdr = DR_LORA_SF12; 
                    else if (!strncmp(swap_str, "SF5", 3))
                        entry->txdr = DR_LORA_SF5; 
                    else if (!strncmp(swap_str, "SF6", 3))
                        entry->txdr = DR_LORA_SF6; 
                    else 
                        entry->txdr = 0; 
                } 
                /*!>* 8. tx frequency **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    i = sscanf(swap_str, "%u", &entry->txfreq);
                    if (i != 1)
                        entry->txfreq = 0;
                } 

                /*!>* 9. tx window **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    entry->rxwindow = atoi(swap_str);
                    if (entry->rxwindow > 2 || entry->rxwindow < 1)
                        entry->rxwindow = 0;
                } 

                /*!>* 10. tx fport **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    entry->txport = atoi(swap_str);
                    if (entry->txport > 255 || entry->txport < 0)
                        entry->txport = DEFAULT_DOWN_FPORT;
                } 

                /*!>* 11. tx optlen **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    entry->optlen = atoi(swap_str);
                    if (entry->optlen > 32 || entry->optlen < 0)
                        entry->optlen = 0;
                } 

                /*!>* 12. tx opts **/
                if ((j = strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str))) > 0) {
                    if (entry->optlen > 0 && (j < 32)) {
                        hex2str((uint8_t*)swap_str, hexopt, j);
                        entry->fopt = lgw_malloc(sizeof(uint8_t) * j/2);
                        lgw_memcpy(entry->fopt, hexopt, j/2);
                        entry->optlen = j/2;  // fix optlen 
                        lgw_log(LOG_DEBUG, "%s[DNLK] mac-command optlen = %i.\n", DEBUGMSG, entry->optlen);
                    } else {
                        entry->optlen = 0;
                        entry->fopt = NULL;
                    }
                } 

                /*!>* 13. FTYPE **/
                if (strcpypt(swap_str, (char*)buff_down, &start, size, sizeof(swap_str)) > 0) {
                    switch (atoi(swap_str)) {
                        case 1:
                            entry->ftype = FRAME_TYPE_DATA_CONFIRMED_DOWN;
                            break;
                        default:
                            entry->ftype = FRAME_TYPE_DATA_UNCONFIRMED_DOWN;
                            break;
                    }
                }

                /*!>******************************************************************/
                /*!>* End of prepare customer donwlink constractor **/
                /*!>******************************************************************/


                if (strstr(entry->pdformat, "hex") != NULL) { 
                    if (psize % 2) {
                        lgw_log(LOG_INFO, "%s[DNLK] Size of hex payload invalid.\n", INFOMSG);
                        if (entry->fopt)
                            lgw_free(entry->fopt);
                        lgw_free(entry);
                        continue;
                    }
                    hex2str(dnpld, hexpld, psize);
                    psize = psize/2;
                    lgw_memcpy(entry->payload, hexpld, psize + 1);
                } else
                    lgw_memcpy(entry->payload, dnpld, psize + 1);

                entry->psize = psize;
#ifdef SX1302MOD
                switch(entry->txbw) {
                    case 0x1:
                        entry->txbw = 0x6;
                        bw_display = 500000;
                        break;
                    case 0x2:
                        entry->txbw = 0x5;
                        bw_display = 250000;
                        break;
                    case 0x3:
                        entry->txbw = 0x4;
                        bw_display = 125000;
                        break;
                    default:
                        bw_display = 0;
                        break;
                }
#else
                switch(entry->txbw) {
                    case 0x1:
                        bw_display = 500000;
                        break;
                    case 0x2:
                        bw_display = 250000;
                        break;
                    case 0x3:
                        bw_display = 125000;
                        break;
                    case 0x4:
                        bw_display = 62000;
                        break;
                    case 0x5:
                        bw_display = 31200;
                        break;
                    case 0x6:
                        bw_display = 15600;
                        break;
                    case 0x7:
                        bw_display = 7800;
                        break;
                    default:
                        bw_display = 0;
                        break;
                }
#endif

                lgw_log(LOG_DEBUG, "%s[DNLK]devaddr:%s, txmode:%s, pdfm:%s, size:%d, freq:%u, bw:%u, dr:%u, ftype:%s\n", DEBUGMSG, entry->devaddr, entry->txmode, entry->pdformat, entry->psize, entry->txfreq, bw_display, entry->txdr, entry->ftype == FRAME_TYPE_DATA_CONFIRMED_DOWN ? "CONFIMED" : "UNCONF");

                lgw_log(LOG_DEBUG, "%s[DNLK]payload:\"%s\"\n", DEBUGMSG, entry->payload); 

                if (strstr(entry->txmode, "imme") != NULL) {
                    lgw_log(LOG_INFO, "%s[DNLK] Pending IMMEDIATE downlink for %s\n", INFOMSG, entry->devaddr);

                    uaddr = strtoul(entry->devaddr, NULL, 16);

                    devinfo_s devinfo = { .devaddr = uaddr, 
                                          .devaddr_str = {0},
                                          .appskey_str = {0},
                                          .nwkskey_str = {0}
                                        };

                    sprintf(db_family, "devinfo/%08X", devinfo.devaddr);

                    if ((lgw_db_get(db_family, "appskey", devinfo.appskey_str, sizeof(devinfo.appskey_str)) == -1) || 
                        (lgw_db_get(db_family, "nwkskey", devinfo.nwkskey_str, sizeof(devinfo.nwkskey_str)) == -1)) {
                        if (entry->fopt)
                            lgw_free(entry->fopt);
                        lgw_free(entry);
                        continue;
                    }


                    str2hex(devinfo.appskey, devinfo.appskey_str, sizeof(devinfo.appskey));
                    str2hex(devinfo.nwkskey, devinfo.nwkskey_str, sizeof(devinfo.nwkskey));

                    lgw_log(LOG_DEBUG, "\n%s[DNLK][DECODE]devaddr: %08X, appSkey:", DEBUGMSG, devinfo.devaddr);
                    for (j = 0; j < (int)sizeof(devinfo.appskey); ++j) {
                        lgw_log(LOG_DEBUG, "%02X", devinfo.appskey[j]);
                    }
                    lgw_log(LOG_DEBUG, "\n");

                    jit_result = custom_rx2dn(entry, &devinfo, 0, IMMEDIATE);

                    if (jit_result != JIT_ERROR_OK)  
                        lgw_log(LOG_ERROR, "%s[DNLK]Packet REJECTED (jit error=%d)\n", ERRMSG, jit_result);
                    else
                        lgw_log(LOG_INFO, "%s[DNLK]customer immediate downlink for %s ready\n", INFOMSG, entry->devaddr);

                    if (entry->fopt)
                        lgw_free(entry->fopt);
                    lgw_free(entry);
                    continue;
                }

                LGW_LIST_LOCK(&dn_list);
                if (dn_list.size > 16) { 
                    element = LGW_LIST_REMOVE_HEAD(&dn_list, list);
                    if (element != NULL) {
                        dn_list.size--;
                        if (element->fopt)
                            lgw_free(element->fopt);
                        lgw_free(element);
                    }
                }

                LGW_LIST_TRAVERSE_SAFE_BEGIN(&dn_list, element, list) { 
                    if (!strcmp(element->devaddr, entry->devaddr)) {
                        LGW_LIST_REMOVE_CURRENT(list);
                        dn_list.size--;
                        if (element->fopt)
                            lgw_free(element->fopt);
                        lgw_free(element);
                    }
                }
                LGW_LIST_TRAVERSE_SAFE_END;

                LGW_LIST_INSERT_TAIL(&dn_list, entry, list);
                LGW_LIST_UNLOCK(&dn_list);
            }
            wait_ms(100); /*!> wait for HAT send or other process */
        }
        if (closedir(dir) < 0)
            lgw_log(LOG_INFO, "%s[DNLK]Cannot close DIR: %s\n", INFOMSG, DNPATH);
        wait_ms(100);
    }
    lgw_log(LOG_INFO, "%s[THREAD][%s] END of pkt_prepare_downlink thread\n", INFOMSG, serv->info.name);
}

/*!>! 
 * \brief copies the string pointed to by src, 
 * \param start the point of src 
 * \param size of the src length 
 * \param len of dest length
 * \retval return the number of characters copyed
 *
 */

static int strcpypt(char* dest, const char* src, int* start, int size, int len)
{
    int i, j;

    i = *start;
    
    while (src[i] == ' ' && i < size) {
        i++;
    }

    if ( i >= size ) return 0;

    for (j = 0; j < len; j++) {
        dest[j] = '\0';
    }

    for (j = 0; i < size; i++) {
        if (src[i] == ',') {
            i++; // skip ','
            break;
        }

        if (j == len - 1) 
            continue;

		if(src[i] != 0 && src[i] != 10 )
            dest[j++] = src[i];
    }

    *start = i;

    return j;
}

static void pkt_deal_up(void* arg) {
    serv_s* serv = (serv_s*) arg;
    lgw_log(LOG_INFO, "%s[THREAD][%s] Staring...\n", INFOMSG, serv->info.name);

	while (!serv->thread.stop_sig) {

		sem_wait(&serv->thread.sema);

        if (pthread_pkt_count > MAX_PKT_PTHREADS) {
            lgw_log(LOG_DEBUG, "THREAD~ [%s] WAR! More decode threads(=%d), skip current pacakege!\n", serv->info.name, pthread_pkt_count);
            continue;   //wait for , next trigger, next time, will miss some package
        }

    //do {
        serv_ct_s *serv_ct = lgw_malloc(sizeof(serv_ct_s));
        serv_ct->serv = serv;
        serv_ct->nb_pkt = get_rxpkt(serv_ct);     //only get the first rxpkt of list

        if (serv_ct->nb_pkt == 0) {
            lgw_free(serv_ct);
            continue;
        }

        pthread_t ntid;

        if (lgw_pthread_create(&ntid, NULL, (void *(*)(void *))thread_pkt_deal_up, (void *)serv_ct)) {
            lgw_free(serv_ct);
            lgw_log(LOG_WARNING, "%s[THREAD][%s] Can't create push_up pthread.\n", WARNMSG, serv->info.name);
        } else {
            pthread_detach(ntid);
            pthread_mutex_lock(&mx_pthread_pkt_count);
            pthread_pkt_count++;
            pthread_mutex_unlock(&mx_pthread_pkt_count);
        }

        lgw_log(LOG_DEBUG, "%s[THREAD][%s] pkt_push_up(count=%d) fetch %d %s.\n", DEBUGMSG, serv->info.name, pthread_pkt_count, serv_ct->nb_pkt, serv_ct->nb_pkt < 2 ? "packet" : "packets");

    //} while (GW.rxpkts_list.size > 1 && (!serv->thread.stop_sig));  
    }

    lgw_log(LOG_INFO, "%s[THREAD][%s] ENDED!\n", INFOMSG, serv->info.name);
}

/*!> --- EOF ------------------------------------------------------------------ */

