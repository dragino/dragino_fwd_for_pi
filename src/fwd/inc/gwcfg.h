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
 * \brief gateway forward data struct define
 */

#ifndef _GW_H
#define _GW_H

#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>

#include "linkedlists.h"
#include "jitqueue.h"
#include "stats.h"

#include "loragw_gps.h"

#define DEFAULT_BEACON_FREQ_HZ      869525000
#define DEFAULT_BEACON_FREQ_NB      1
#define DEFAULT_BEACON_FREQ_STEP    0
#define DEFAULT_BEACON_DATARATE     9
#define DEFAULT_BEACON_BW_HZ        125000
#define DEFAULT_BEACON_POWER        14
#define DEFAULT_BEACON_INFODESC     0

#ifdef SX1301MOD
#define NB_PKT_MAX                  16            /*!> max number of packets per fetch/send cycle */
#else
#define NB_PKT_MAX                  32            /*!> max number of packets per fetch/send cycle */
#endif

typedef enum {
	semtech,
	ttn,
    mqtt,
    pkt,
    relay,
    delay,
	gwtraf
} serv_type;

typedef enum {   /*!> Regional parameters */
    EU,
    EU433,
    US,
    CN470,
    CN779,
    AS1,
    AS2,
    AS3,
    KR,
    IN,
    RU,
    KZ,
    AU
} region_s;

typedef enum {     /*!> thread type for control thread head */
    rxpkts,
    stats,
    semtech_up,
    semtech_down,
    ttn_up,
    ttn_down,
    pkttraf,
    gps,
    jit,
    timersync,
    watchdog
} thread_type;

typedef struct _rxpkts {   /*!> rx packages receive from radio or socke */
    //bool deal;
    uint32_t entry_us;     //插入添加时间
    uint8_t stamps;        //这个指示当前有什么服务对这个包打上了印记
    uint8_t nb_pkt;
    uint8_t bind;          //当前还剩多少连接，当连接为0时，将会移除这个包
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX];
    LGW_LIST_ENTRY(_rxpkts) list;
} rxpkts_s;

typedef enum {
    NOFILTER,
    INCLUDE,
    EXCLUDE
} filter_e;

typedef struct {               // Configuration File: sx130x (global_conf.json) and gw configure (local_conf.json)
    char gwcfg[32];
    char sxcfg[32];
} confs_s;

/*!> spectral scan */
typedef struct spectral_scan_s {
    bool enable;				/*!> enable spectral scan thread */
    uint32_t freq_hz_start;		/*!> first channel frequency, in Hz */
    uint8_t nb_chan;			/*!> number of channels to scan (200kHz between each channel) */
    uint16_t nb_scan;			/*!> number of scan points for each frequency scan */
    uint32_t pace_s;			/*!> number of seconds between 2 scans in the thread */
} spectral_scan_t;

/*!> mqtt service information */
typedef struct {
    void* session;
    char uptopic[64];
    char dntopic[64];
} mqttinfo_s;

typedef struct {
    char addr[256];				// server address
    char port_up[8];			// uplink port 
    char port_down[8];			// downlink port 
    int  sock_up;				// up socket
    int  sock_down;				// down socket
    int  pull_interval;	        // send a PULL_DATA request every X seconds 
    struct timeval push_timeout_half;       /*!> time-out value (in ms) for upstream datagrams */
    struct timeval pull_timeout;
    mqttinfo_s *mqtt;
} serv_net_s;

LGW_LIST_HEAD(rxpkts_list, _rxpkts);     //定义一个数据链头，用来保存接收到的数据包         

/*!>!
 * \brief server是一个描述什么样服务的数据结构
 * 
 */
typedef struct _server {
    struct {
	    serv_type type;		        // type of server
        uint8_t stamp;              // 用来标记service，当有服务印上这个标记时，代表和当前的service有关联
        bool enabled;
        char name[32];              // identify of server
        char *key;			        // gateway key to connect to service
    } info;

    struct {
        filter_e fport;             /*!> 0/1/2, 0不处理，1如果过滤匹配数据库的，2转发匹配数据库的 */
        filter_e devaddr;           /*!> 和fport相同 */
        filter_e nwkid;             /*!> 和fport相同 */
        filter_e deveui;             /*!> 和fport相同 */
        bool fwd_valid_pkt;         /*!> packets with PAYLOAD CRC OK are forwarded */
        bool fwd_error_pkt;         /*!> packets with PAYLOAD CRC ERROR are NOT forwarded */
        bool fwd_nocrc_pkt;         /*!> packets with NO PAYLOAD CRC are NOT forwarded */
    } filter;

    struct {
        bool live;					// Server is life?
        bool connecting;			// Connection setup in progress
        int  max_stall;				// max number of missed responses
        int  stall_time;
        time_t contact;				// time of last contact
        time_t startup_time;		// time of server starting
    } state;
    
    struct {
        pthread_t t_down;			// downstream thread
        pthread_t t_up;				// upstream thread
        sem_t sema;				    // semaphore for sending data
        bool stop_sig;
    } thread;

    serv_net_s* net;

    report_s* report;

    LGW_LIST_ENTRY(_server) list;

} serv_s;

LGW_LIST_HEAD_NOLOCK(serv_list, _server);  // pkts list head of rxpkts for server 

typedef struct {
    int nb_pkt;
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX];
    serv_s* serv;
} serv_ct_s;

typedef struct _thread_info {
    pthread_t  tid;
    thread_type type;
    bool        runing;
    time_t      dog;
    uint32_t start_us;
    uint32_t end_us;
    serv_ct_s* serv_ct;  //记录一个服务内容，检测它是否已经销毁
    LGW_LIST_ENTRY(_thread_info) list;
} thread_info;

LGW_LIST_HEAD(pthread_list, _thread_info);     //定义一个数据链头，用来控制线程数量         

struct lbt_chan_stat {
    uint32_t freq_hz;
    uint32_t count_us;  // 时间
    bool chan_is_free;
};

typedef struct {
    struct {
        char gateway_id[17];	/*!> string form of gateway mac address */
        char platform[24];	    /*!> platform definition */
        char email[40];			/*!> used for contact email */
        char description[64];	/*!> used for free form description */
        uint64_t  lgwm;			/*!> Lora gateway MAC address */
        uint32_t net_mac_h;
        uint32_t net_mac_l;
        uint8_t service_count;  /*!> count of service */
        bool network_status;    /*!> If Gateway networking */
    } info;

    struct {
        char board[16];         // SX1301(LG301) / SX1308(LG308) / SX1302(LG302) / SX1276 (LG02)
        char spidev_path[32];
        bool xtal_correct_ok;
        double xtal_correct;
        uint8_t antenna_gain;
        confs_s confs;
        pthread_mutex_t mx_xcorr;
        pthread_mutex_t mx_concent;
    } hal;

    struct {
        bool     radiostream_enabled;	
        bool     ghoststream_enabled;	
        bool     delay_enabled;	
        bool     td_enabled;              /*!> if enable time diff form UTC */	
        bool     wd_enabled;		      /*!> if watchdog enabled   */
        bool     mac_decode;              /*!> if mac header decode for abp */
        bool     mac2file;                /*!> if payload text save to file */
        bool     mac2db;                  /*!> if payload text save to database */
        bool     custom_downlink;         /*!> if make a custome downlink to node */
        time_t   last_loop;               /*!> timestamp for watchdog */
        uint32_t time_interval;           /*!> time interval for send status(seconds) */
        uint8_t  fcnt_gap;
        char   time_diff[8];              /*!> time diff of UTC, UTC + diff = TZ */
        char   ghost_host[32];
        char   ghost_port[16];
        char   delay_db_path[64];
        region_s   region;
        uint32_t autoquit_threshold;/*!> enable auto-quit after a number of non-acknowledged PULL_DATA (0 = disabled) */
    } cfg;

    struct {
        bool        as_relay;                /*!> relay gateway */
        bool        has_relay;               /*!> nomal gateway will receive data from relay gateway */
        char        tty_path[64];            /*!> tty port for relay device (sx126x) */
        int         tty_fd;                  /*!> uart open fd  for relay device */
        uint32_t    tty_baude;               /*!> bauderate */
        uint32_t    freq_hz;                 /*!> relay channel equal to if_chain_8 (loar service channel) */
        bool        invert_pol;
        uint8_t     bw;      
        uint8_t     sf;       
    } relay;

    /*!> GPS configuration and synchronization */
    struct {
        char   gps_tty_path[64];        /*!> path of the TTY port GPS is connected on */
        int    gps_tty_fd;
        bool   gps_enabled;		        /*!> controls the use of the GPS                   */
        bool   time_ref;		        /*!> controls the time refer from gps              */
        bool   gps_ref_valid;           /*!> is GPS reference acceptable (ie. not too old) */
        bool   gps_fake_enable;
        struct tref time_reference_gps; /*!> time reference used for UTC <-> timestamp conversion */
        struct coord_s reference_coord; /*!> Reference coordinates, for broadcasting (beacon) */
        bool   gps_coord_valid;         /*!> could we get valid GPS coordinates? */
        struct coord_s meas_gps_coord;  /*!> GPS position of the gateway */
        struct coord_s meas_gps_err;    /*!> GPS position of the gateway */
        /*!> GPS time reference */
        pthread_mutex_t mx_timeref;	    /*!> control access to GPS time reference */
        pthread_mutex_t mx_meas_gps;	/*!> control access to the GPS statistics */
    } gps;

    struct {
        bool   lbt_tty_enabled;         /*!> enable LBT */
        char   lbt_tty_path[64];        /*!> path of the TTY port LBT is connected on */
        int    lbt_tty_fd;              /*!> LBT fd */
        int8_t lbt_rssi_target;         /*!> RSSI threshold to detect if channel is busy or not (dBm) */
        uint32_t lbt_tty_baude;         /*!> bauderate */
        uint32_t lbt_freq_hz;               
        uint16_t lbt_scan_time_ms;      /*!> scan time for LBT */
        struct lbt_chan_stat lbt_stat[16];
    } lbt;

    struct {
        struct lgw_tx_gain_lut_s txlut[LGW_RF_CHAIN_NB];
        uint32_t tx_freq_min[LGW_RF_CHAIN_NB];
        uint32_t tx_freq_max[LGW_RF_CHAIN_NB];
        bool tx_enable[LGW_RF_CHAIN_NB];
        struct jit_queue_s jit_queue[LGW_RF_CHAIN_NB];
    } tx;

    struct {
        uint32_t beacon_period;    /*!> set beaconing period, must be a sub-multiple of 86400, the nb of sec in a day */
        uint32_t beacon_freq_hz;   /*!> set beacon TX frequency, in Hz */
        uint8_t  beacon_freq_nb;   /*!> set number of beaconing channels beacon */
        uint32_t beacon_freq_step; /*!> set frequency step between beacon channels, in Hz */
        uint8_t  beacon_datarate;  /*!> set beacon datarate (SF) */
        uint32_t beacon_bw_hz;     /*!> set beacon bandwidth, in Hz */
        int8_t   beacon_power;     /*!> set beacon TX power, in dBm */
        uint8_t  beacon_infodesc;  /*!> set beacon information descriptor */
        uint32_t meas_nb_beacon_queued;
        uint32_t meas_nb_beacon_sent;
        uint32_t meas_nb_beacon_rejected;
    } beacon;

    struct {
        bool logger_enabled;            /*!> controls the activation of more logging */
        uint16_t  debug_mask;           /*!> enabled debugging options */
        char *logfile;                  /*!> path to logfile */
        uint32_t nb_pkt_log[LGW_IF_CHAIN_NB];  
        uint32_t nb_pkt_received_lora;
        uint32_t nb_pkt_received_fsk;
        uint32_t nb_pkt_received_ref[16];
        stat_dw_s stat_dw;
        pthread_mutex_t mx_report;	      // control access to the queue for each server
    } log;

#ifdef SX1302MOD
	spectral_scan_t spectral_scan_params; //Spectral Scan
#endif

    struct rxpkts_list rxpkts_list;

    struct serv_list serv_list;
} gw_s;

#define INIT_GW gw_s GW = {   .info.lgwm = 0,                                        \
                              .info.service_count = 0,                               \
                              .hal.board = "sx1302",                                 \
                              .hal.confs = { .gwcfg = "/etc/lora/local_conf.json",   \
                                             .sxcfg = "/etc/lora/global_conf.json"}, \
                              .hal.mx_concent = PTHREAD_MUTEX_INITIALIZER,           \
                              .hal.mx_xcorr   = PTHREAD_MUTEX_INITIALIZER,           \
                              .hal.xtal_correct_ok = false,                          \
                              .hal.xtal_correct = 1.0,                               \
                              .info.network_status = false,                          \
                              .cfg.wd_enabled = false,                               \
                              .cfg.td_enabled = false,                               \
                              .cfg.radiostream_enabled = true,                       \
                              .cfg.ghoststream_enabled = false,                      \
                              .cfg.delay_enabled = false,                            \
                              .cfg.fcnt_gap = 12,                                    \
                              .cfg.autoquit_threshold = 0,                           \
                              .cfg.mac_decode = false,                               \
                              .cfg.mac2file = false,                                 \
                              .cfg.mac2db = false,                                   \
                              .cfg.custom_downlink = false,                          \
                              .cfg.time_interval = 30,                               \
                              .cfg.time_diff = "8",                                  \
                              .relay.as_relay = false,                               \
                              .relay.has_relay = false,                              \
                              .relay.tty_baude = 9600,                               \
                              .relay.invert_pol = true,                              \
                              .relay.freq_hz = 868300000,                            \
                              .relay.bw = 0,                                         \
                              .relay.sf = 8,                                         \
                              .gps.gps_tty_path[0] = 0,                              \
                              .gps.time_ref = false,                                 \
                              .gps.mx_timeref  = PTHREAD_MUTEX_INITIALIZER,          \
                              .gps.mx_meas_gps = PTHREAD_MUTEX_INITIALIZER,          \
                              .lbt.lbt_tty_enabled = false,                          \
                              .lbt.lbt_tty_path[0] = 0,                              \
                              .lbt.lbt_tty_fd = -1,                                  \
                              .lbt.lbt_tty_baude = 9600,                             \
                              .lbt.lbt_rssi_target = -85,                            \
                              .lbt.lbt_scan_time_ms = 6,                             \
                              .beacon.beacon_period    = 0,                          \
                              .beacon.beacon_freq_hz   = DEFAULT_BEACON_FREQ_HZ,     \
                              .beacon.beacon_freq_nb   = DEFAULT_BEACON_FREQ_NB,     \
                              .beacon.beacon_freq_step = DEFAULT_BEACON_FREQ_STEP,   \
                              .beacon.beacon_datarate  = DEFAULT_BEACON_DATARATE,    \
                              .beacon.beacon_bw_hz     = DEFAULT_BEACON_FREQ_HZ,     \
                              .beacon.beacon_power     = DEFAULT_BEACON_POWER,       \
                              .beacon.beacon_infodesc  = DEFAULT_BEACON_INFODESC,    \
                              .beacon.meas_nb_beacon_queued   = 0,                   \
                              .beacon.meas_nb_beacon_sent     = 0,                   \
                              .beacon.meas_nb_beacon_rejected = 0,                   \
                              .log.logger_enabled  = 0,                              \
                              .log.debug_mask  = 0,                                  \
                              .log.nb_pkt_received_lora  = 0,                        \
                              .log.nb_pkt_received_fsk   = 0,                        \
                              .log.mx_report = PTHREAD_MUTEX_INITIALIZER,            \
                              .serv_list = LGW_LIST_HEAD_NOLOCK_INIT_VALUE,          \
                              .rxpkts_list = LGW_LIST_HEAD_INIT_VALUE,               \
                          }

#define DECLARE_GW extern gw_s GW

/*!>
 *
 */
int parsecfg();

#endif							// _GW_H
