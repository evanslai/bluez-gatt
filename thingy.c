#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <limits.h>
#include <errno.h>

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"

#include "src/shared/mainloop.h"
#include "src/shared/util.h"
#include "src/shared/att.h"
#include "src/shared/queue.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-client.h"

#define ATT_CID 4

#define THINGY_SENSOR_TEMPERATURE    0
#define THINGY_SENSOR_PRESSURE      1
#define THINGY_SENSOR_HUMIDITY      2
#define THINGY_SENSOR_GAS           3

#define PRLOG(...) printf(__VA_ARGS__);

#define COLOR_OFF       "\x1B[0m"
#define COLOR_RED       "\x1B[0;91m"
#define COLOR_GREEN     "\x1B[0;92m"
#define COLOR_YELLOW    "\x1B[0;93m"
#define COLOR_BLUE      "\x1B[0;94m"
#define COLOR_MAGENTA   "\x1B[0;95m"
#define COLOR_BOLDGRAY  "\x1B[1;30m"
#define COLOR_BOLDWHITE "\x1B[1;37m"

static int thingy_sensor = THINGY_SENSOR_TEMPERATURE;
static bool verbose = false;

struct client {
    int fd;
    struct bt_att *att;
    struct gatt_db *db;
    struct bt_gatt_client *gatt;

    unsigned int reliable_session_id;
};


static bool parse_args(char *str, int expected_argc,  char **argv, int *argc)
{
    char **ap;

    for (ap = argv; (*ap = strsep(&str, " \t")) != NULL;) {
        if (**ap == '\0')
            continue;

        (*argc)++;
        ap++;

        if (*argc > expected_argc)
            return false;
    }

    return true;
}

/* functions for register notification */
static void notify_cb(uint16_t value_handle, const uint8_t *value,
        uint16_t length, void *user_data)
{
    if (value_handle == 0x001f) /* temperature */
        printf("Notification: Temp received:  %d.%d degCelcius\n", (int)value[0], (int)value[1]);
    else if (value_handle == 0x0022) { /* pressure */
        int hpa = ((int)value[3] << 24) | ((int)value[2] << 16) | ((int)value[1] << 8) | ((int)value[0]);
        printf("Notification: Pressure received: %d.%d hPa\n", hpa, (int)value[4]);
    }
    else if (value_handle == 0x0025) /* humidity */
        printf("Notification: Humidity received: %d %\n", (int)value[0]);
    else if (value_handle == 0x0028) { /* gas */
        int eco2 = ((int)value[1] << 8) | ((int)value[0]);
        int tvoc = ((int)value[3] << 8) | ((int)value[2]);
        printf("Notification: Gas received: eCO2 ppm: %d, TVOC ppb: %d %\n", eco2, tvoc);
    }
    else {
        int i;

        printf("\n\tHandle Value Not/Ind: 0x%04x - ", value_handle);

        if (length == 0) {
            PRLOG("(0 bytes)\n");
            return;
        }

        printf("(%u bytes): ", length);

        for (i = 0; i < length; i++)
            printf("%02x ", value[i]);

        PRLOG("\n");
    }
}

static void register_notify_cb(uint16_t att_ecode, void *user_data)
{
    if (att_ecode) {
        PRLOG("Failed to register notify handler "
                "- error code: 0x%02x\n", att_ecode);
        return;
    }
}

static void cmd_register_notify(struct client *cli, char *cmd_str)
{
    char *argv[2];
    int argc = 0;
    uint16_t value_handle;
    unsigned int id;
    char *endptr = NULL;

    if (!bt_gatt_client_is_ready(cli->gatt)) {
        printf("GATT client not initialized\n");
        return;
    }

    if (!parse_args(cmd_str, 1, argv, &argc) || argc != 1) {
        printf("parse error: %s\n", cmd_str);
        return;
    }

    value_handle = strtol(argv[0], &endptr, 0);
    if (!endptr || *endptr != '\0' || !value_handle) {
        printf("Invalid value handle: %s\n", argv[0]);
        return;
    }

    id = bt_gatt_client_register_notify(cli->gatt, value_handle,
            register_notify_cb,
            notify_cb, NULL, NULL);
    if (!id) {
        printf("Failed to register notify handler\n");
        return;
    }
}

/* END of functions for register notification */

/* functions used in client_create() */
static void att_disconnect_cb(int err, void *user_data)
{
    printf("Device disconnected: %s\n", strerror(err));

    mainloop_quit();
}

static void log_service_event(struct gatt_db_attribute *attr, const char *str)
{
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid;
    uint16_t start, end;

    gatt_db_attribute_get_service_uuid(attr, &uuid);
    bt_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));

    gatt_db_attribute_get_service_handles(attr, &start, &end);

    PRLOG("%s - UUID: %s start: 0x%04x end: 0x%04x\n", str, uuid_str,
            start, end);
}

static void service_added_cb(struct gatt_db_attribute *attr, void *user_data)
{
    log_service_event(attr, "Service Added");
}

static void service_removed_cb(struct gatt_db_attribute *attr, void *user_data)
{
    log_service_event(attr, "Service Removed");
}

static void att_debug_cb(const char *str, void *user_data)
{
    const char *prefix = user_data;

    PRLOG(COLOR_BOLDGRAY "%s" COLOR_BOLDWHITE "%s\n" COLOR_OFF, prefix, str);
}

static void gatt_debug_cb(const char *str, void *user_data)
{
    const char *prefix = user_data;

    PRLOG(COLOR_GREEN "%s%s\n" COLOR_OFF, prefix, str);
}

static void print_uuid(const bt_uuid_t *uuid)
{
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid128;

    bt_uuid_to_uuid128(uuid, &uuid128);
    bt_uuid_to_string(&uuid128, uuid_str, sizeof(uuid_str));

    printf("%s\n", uuid_str);
}

static void print_incl(struct gatt_db_attribute *attr, void *user_data)
{
    struct client *cli = user_data;
    uint16_t handle, start, end;
    struct gatt_db_attribute *service;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_incl_data(attr, &handle, &start, &end))
        return;

    service = gatt_db_get_attribute(cli->db, start);
    if (!service)
        return;

    gatt_db_attribute_get_service_uuid(service, &uuid);

    printf("\t  " COLOR_GREEN "include" COLOR_OFF " - handle: "
            "0x%04x, - start: 0x%04x, end: 0x%04x,"
            "uuid: ", handle, start, end);
    print_uuid(&uuid);
}

static void print_desc(struct gatt_db_attribute *attr, void *user_data)
{
    printf("\t\t  " COLOR_MAGENTA "descr" COLOR_OFF
            " - handle: 0x%04x, uuid: ",
            gatt_db_attribute_get_handle(attr));
    print_uuid(gatt_db_attribute_get_type(attr));
}

static void print_chrc(struct gatt_db_attribute *attr, void *user_data)
{
    uint16_t handle, value_handle;
    uint8_t properties;
    uint16_t ext_prop;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_char_data(attr, &handle,
                &value_handle,
                &properties,
                &ext_prop,
                &uuid))
        return;

    printf("\t  " COLOR_YELLOW "charac" COLOR_OFF
            " - start: 0x%04x, value: 0x%04x, "
            "props: 0x%02x, ext_props: 0x%04x, uuid: ",
            handle, value_handle, properties, ext_prop);
    print_uuid(&uuid);

    gatt_db_service_foreach_desc(attr, print_desc, NULL);
}

static void print_service(struct gatt_db_attribute *attr, void *user_data)
{
    struct client *cli = user_data;
    uint16_t start, end;
    bool primary;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_service_data(attr, &start, &end, &primary,
                &uuid))
        return;

    printf(COLOR_RED "service" COLOR_OFF " - start: 0x%04x, "
            "end: 0x%04x, type: %s, uuid: ",
            start, end, primary ? "primary" : "secondary");
    print_uuid(&uuid);

    gatt_db_service_foreach_incl(attr, print_incl, cli);
    gatt_db_service_foreach_char(attr, print_chrc, NULL);

    printf("\n");
}

static void print_services(struct client *cli)
{
    printf("\n");

    gatt_db_foreach_service(cli->db, NULL, print_service, cli);
}

static void ready_cb(bool success, uint8_t att_ecode, void *user_data)
{
    struct client *cli = user_data;
    char temp_chrc_hnd[32] = "0x001f\0";
    char pressure_chrc_hnd[32] = "0x0022\0";
    char humidity_chrc_hnd[32] = "0x0025\0";
    char gas_chrc_hnd[32] = "0x0028\0";

    if (!success) {
        PRLOG("GATT discovery procedures failed - error code: 0x%02x\n",
                att_ecode);
        return;
    }

    PRLOG("GATT discovery procedures complete\n");


    //print_services(cli);
    if (thingy_sensor == THINGY_SENSOR_TEMPERATURE)
        cmd_register_notify(cli, temp_chrc_hnd);
    else if (thingy_sensor == THINGY_SENSOR_PRESSURE)
        cmd_register_notify(cli, pressure_chrc_hnd);
    else if (thingy_sensor == THINGY_SENSOR_HUMIDITY)
        cmd_register_notify(cli, humidity_chrc_hnd);
    else if (thingy_sensor == THINGY_SENSOR_GAS)
        cmd_register_notify(cli, gas_chrc_hnd);
    else
        cmd_register_notify(cli, temp_chrc_hnd);
}


static void service_changed_cb(uint16_t start_handle, uint16_t end_handle,
        void *user_data)
{
    struct client *cli = user_data;

    printf("\nService Changed handled - start: 0x%04x end: 0x%04x\n",
            start_handle, end_handle);

    gatt_db_foreach_service_in_range(cli->db, NULL, print_service, cli,
            start_handle, end_handle);
}
/* END of functions used in client_create() */

static struct client *client_create(int fd, uint16_t mtu)
{
    struct client *cli;

    cli = new0(struct client, 1);
    if (!cli) {
        fprintf(stderr, "Failed to allocate memory for client\n");
        return NULL;
    }

    cli->att = bt_att_new(fd, false);
    if (!cli->att) {
        fprintf(stderr, "Failed to initialze ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_set_close_on_unref(cli->att, true)) {
        fprintf(stderr, "Failed to set up ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_register_disconnect(cli->att, att_disconnect_cb, NULL,
                NULL)) {
        fprintf(stderr, "Failed to set ATT disconnect handler\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->fd = fd;
    cli->db = gatt_db_new();
    if (!cli->db) {
        fprintf(stderr, "Failed to create GATT database\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->gatt = bt_gatt_client_new(cli->db, cli->att, mtu);
    if (!cli->gatt) {
        fprintf(stderr, "Failed to create GATT client\n");
        gatt_db_unref(cli->db);
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    gatt_db_register(cli->db, service_added_cb, service_removed_cb,
            NULL, NULL);

    if (verbose) {
        bt_att_set_debug(cli->att, att_debug_cb, "att: ", NULL);
        bt_gatt_client_set_debug(cli->gatt, gatt_debug_cb, "gatt: ",
                NULL);
    }

    bt_gatt_client_ready_register(cli->gatt, ready_cb, cli, NULL);
    bt_gatt_client_set_service_changed(cli->gatt, service_changed_cb, cli,
            NULL);

    /* bt_gatt_client already holds a reference */
    gatt_db_unref(cli->db);

    return cli;
}

static void client_destroy(struct client *cli)
{
    bt_gatt_client_unref(cli->gatt);
    bt_att_unref(cli->att);
    free(cli);
}




static void signal_cb(int signum, void *user_data)
{
    switch (signum) {
        case SIGINT:
        case SIGTERM:
            mainloop_quit();
            break;
        default:
            break;
    }
}


static int l2cap_le_att_connect(bdaddr_t *src, bdaddr_t *dst, uint8_t dst_type,
        int sec)
{
    int sock;
    struct sockaddr_l2 srcaddr, dstaddr;
    struct bt_security btsec;

    if (verbose) {
        char srcaddr_str[18], dstaddr_str[18];

        ba2str(src, srcaddr_str);
        ba2str(dst, dstaddr_str);

        printf("btgatt-client: Opening L2CAP LE connection on ATT "
                "channel:\n\t src: %s\n\tdest: %s\n",
                srcaddr_str, dstaddr_str);
    }

    sock = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) {
        perror("Failed to create L2CAP socket");
        return -1;
    }

    /* Set up source address */
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.l2_family = AF_BLUETOOTH;
    srcaddr.l2_cid = htobs(ATT_CID);
    srcaddr.l2_bdaddr_type = 0;
    bacpy(&srcaddr.l2_bdaddr, src);

    if (bind(sock, (struct sockaddr *)&srcaddr, sizeof(srcaddr)) < 0) {
        perror("Failed to bind L2CAP socket");
        close(sock);
        return -1;
    }

    /* Set the security level */
    memset(&btsec, 0, sizeof(btsec));
    btsec.level = sec;
    if (setsockopt(sock, SOL_BLUETOOTH, BT_SECURITY, &btsec,
                sizeof(btsec)) != 0) {
        fprintf(stderr, "Failed to set L2CAP security level\n");
        close(sock);
        return -1;
    }

    /* Set up destination address */
    memset(&dstaddr, 0, sizeof(dstaddr));
    dstaddr.l2_family = AF_BLUETOOTH;
    dstaddr.l2_cid = htobs(ATT_CID);
    dstaddr.l2_bdaddr_type = dst_type;
    bacpy(&dstaddr.l2_bdaddr, dst);

    printf("Connecting to device...");
    fflush(stdout);

    if (connect(sock, (struct sockaddr *) &dstaddr, sizeof(dstaddr)) < 0) {
        perror(" Failed to connect");
        close(sock);
        return -1;
    }

    printf(" Done\n");

    return sock;
}

static void usage(void)
{
    printf("thingy\n");
    printf("Usage:\n\tthingy [options]\n");

    printf("Options:\n"
            "\t-i, --index <id>\t\tSpecify adapter index, e.g. hci0\n"
            "\t-d, --dest <addr>\t\tSpecify the destination address\n"
            "\t-s, --sensor <type>\t\tSpecify sensor type (temperature|"
            "pressure|humidity|gas)\n"
            "\t-v, --verbose\t\t\tEnable extra logging\n"
            "\t-h, --help\t\t\tDisplay help\n");
}


static struct option main_options[] = {
    { "index",		1, 0, 'i' },
    { "dest",		1, 0, 'd' },
    { "sensor",     1, 0, 's' },
    { "verbose",	0, 0, 'v' },
    { "help",		0, 0, 'h' },
    { }
};


int main(int argc, char *argv[])
{
    int opt;
    int sec = BT_SECURITY_LOW;
    uint16_t mtu = 0;
    uint8_t dst_type = BDADDR_LE_RANDOM;
    bool dst_addr_given = false;
    bdaddr_t src_addr, dst_addr;
    int dev_id = -1;
    int fd;
    sigset_t mask;
    struct client *cli;

    while ((opt = getopt_long(argc, argv, "+hvs:d:i:",
                    main_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                usage();
                return EXIT_SUCCESS;
            case 'v':
                verbose = true;
                break;
            case 's':
                if (strcmp(optarg, "temperture") == 0)
                    thingy_sensor = THINGY_SENSOR_TEMPERATURE;
                else if (strcmp(optarg, "pressure") == 0)
                    thingy_sensor = THINGY_SENSOR_PRESSURE;
                else if (strcmp(optarg, "humidity") == 0)
                    thingy_sensor = THINGY_SENSOR_HUMIDITY;
                else if (strcmp(optarg, "gas") == 0)
                    thingy_sensor = THINGY_SENSOR_GAS;
                else {
                    fprintf(stderr, "Invalid thingy sensor type\n");
                    return EXIT_FAILURE;
                }
                break;
            case 'd':
                if (str2ba(optarg, &dst_addr) < 0) {
                    fprintf(stderr, "Invalid remote address: %s\n",
                            optarg);
                    return EXIT_FAILURE;
                }

                dst_addr_given = true;
                break;

            case 'i':
                dev_id = hci_devid(optarg);
                if (dev_id < 0) {
                    perror("Invalid adapter");
                    return EXIT_FAILURE;
                }

                break;
            default:
                fprintf(stderr, "Invalid option: %c\n", opt);
                return EXIT_FAILURE;
        }
    }

    if (!argc) {
        usage();
        return EXIT_SUCCESS;
    }

    argc -= optind;
    argv += optind;
    optind = 0;

    if (argc) {
        usage();
        return EXIT_SUCCESS;
    }

    if (dev_id == -1)
        bacpy(&src_addr, BDADDR_ANY);
    else if (hci_devba(dev_id, &src_addr) < 0) {
        perror("Adapter not available");
        return EXIT_FAILURE;
    }

    if (!dst_addr_given) {
        fprintf(stderr, "Destination address required!\n");
        return EXIT_FAILURE;
    }

    mainloop_init();

    fd = l2cap_le_att_connect(&src_addr, &dst_addr, dst_type, sec);
    if (fd < 0)
        return EXIT_FAILURE;

    cli = client_create(fd, mtu);
    if (!cli) {
        close(fd);
        return EXIT_FAILURE;
    }


    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);

    mainloop_set_signal(&mask, signal_cb, NULL, NULL);

    mainloop_run();

    printf("\n\nShutting down...\n");

    client_destroy(cli);

    return EXIT_SUCCESS;
}

